#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_stm32::can::bxcan::filter::Mask32;
use embassy_stm32::can::bxcan::Frame;
use embassy_stm32::can::{bxcan::StandardId, Can, Rx0InterruptHandler, TxInterruptHandler};
use embassy_stm32::gpio::{Input, Pull};
use embassy_stm32::peripherals::{CAN1, TIM2, TIM3, TIM4};
use embassy_stm32::time::khz;
use embassy_stm32::timer::qei::{Qei, QeiDir};
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::{bind_interrupts, Config};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Instant, Ticker, Timer};

// 上下モーターの目標RPMとシステム状態の共有
static TARGET_RPM_TOP: Mutex<CriticalSectionRawMutex, f32> = Mutex::new(0.0);
static TARGET_RPM_BOTTOM: Mutex<CriticalSectionRawMutex, f32> = Mutex::new(0.0);
static ESTOP_FLAG: Mutex<CriticalSectionRawMutex, bool> = Mutex::new(false);
static LAST_CMD_TIME: Mutex<CriticalSectionRawMutex, Option<Instant>> = Mutex::new(None);

bind_interrupts!(struct Irqs {
    CAN1_RX0 => Rx0InterruptHandler<CAN1>;
    CAN1_TX => TxInterruptHandler<CAN1>;
});

#[embassy_executor::task]
async fn can_rx_task(mut rx: embassy_stm32::can::Rx0<'static, CAN1>) {
    info!("CAN RX Task Started");
    loop {
        if let Ok(envelope) = rx.read().await {
            let frame = envelope.frame;
            if let Some(id) = frame.id() {
                // ID: 0x201 (System Control)
                if id == StandardId::new(0x201).unwrap() {
                    if let Some(data) = frame.data() {
                        if data.len() >= 8 {
                            // Byte 0-1: Top RPM
                            let target_top = i16::from_le_bytes([data[0], data[1]]) as f32;

                            // Byte 2-3: Bottom RPM
                            let target_bottom = i16::from_le_bytes([data[2], data[3]]) as f32;

                            // Byte 4-5: Dribbler (Unused for now)

                            // Byte 6: E-Stop
                            let estop = data[6] != 0;

                            *TARGET_RPM_TOP.lock().await = target_top;
                            *TARGET_RPM_BOTTOM.lock().await = target_bottom;
                            *ESTOP_FLAG.lock().await = estop;
                            *LAST_CMD_TIME.lock().await = Some(Instant::now());
                        }
                    }
                }
            }
        }
    }
}

#[embassy_executor::task]
async fn telemetry_task(
    mut tx: embassy_stm32::can::Tx<'static, CAN1>,
    mut limit_sw1: Input<'static>,
) {
    info!("Telemetry Task Started (Switch & IMU)");
    let mut ticker = Ticker::every(Duration::from_millis(10)); // 100Hz

    loop {
        ticker.next().await;

        // 1. スイッチ送信 (ID: 0x200)
        // Bit 0: SW1, Bit 1: SW2(dummy), Bit 2: SW3(dummy)
        let mut switches = 0u8;
        if limit_sw1.is_low() {
            switches |= 0b001;
        }
        // if limit_sw2.is_low() { switches |= 0b010; }
        // if limit_sw3.is_low() { switches |= 0b100; }

        let frame_sw = Frame::new_data(StandardId::new(0x200).unwrap(), [switches]);
        let _ = tx.write(&frame_sw).await;

        // 2. IMU 送信 (ID: 0x202) -> 10000倍圧縮
        let w = (1.0 * 10000.0) as i16;
        let x = (0.0 * 10000.0) as i16;
        let y = (0.0 * 10000.0) as i16;
        let z = (0.0 * 10000.0) as i16;

        let mut imu_data = [0u8; 8];
        imu_data[0..2].copy_from_slice(&w.to_le_bytes());
        imu_data[2..4].copy_from_slice(&x.to_le_bytes());
        imu_data[4..6].copy_from_slice(&y.to_le_bytes());
        imu_data[6..8].copy_from_slice(&z.to_le_bytes());

        let frame_imu = Frame::new_data(StandardId::new(0x202).unwrap(), imu_data);
        let _ = tx.write(&frame_imu).await;
    }
}

// ESC用PWM出力 (1000us ~ 2000us)
fn set_esc_pwm(pwm: &mut SimplePwm<'_, TIM3>, ch: embassy_stm32::timer::Channel, pulse_us: f32) {
    let max_duty = pwm.get_max_duty() as f32;
    let duty = (pulse_us / 20000.0) * max_duty;
    pwm.set_duty(ch, duty as u16);
}

#[embassy_executor::task]
async fn pid_control_task(
    mut qei_top: Qei<'static, TIM4>,
    mut qei_bottom: Qei<'static, TIM2>,
    mut pwm: SimplePwm<'static, TIM3>,
) {
    info!("PID & ESC PWM Task Started");

    let dt = 0.01;
    let mut ticker = Ticker::every(Duration::from_millis(10));

    // PIDゲイン
    let kp = 0.5;
    let ki = 0.05;
    let kd = 0.01;

    let mut integral_top = 0.0;
    let mut prev_error_top = 0.0;
    let mut prev_count_top = qei_top.count();

    let mut integral_bottom = 0.0;
    let mut prev_error_bottom = 0.0;
    let mut prev_count_bottom = qei_bottom.count();

    let cpr = 8192.0;

    loop {
        ticker.next().await;

        let mut target_top = *TARGET_RPM_TOP.lock().await;
        let mut target_bottom = *TARGET_RPM_BOTTOM.lock().await;
        let estop = *ESTOP_FLAG.lock().await;
        let last_cmd = *LAST_CMD_TIME.lock().await;

        // ウォッチドッグ判定 (500ms以上指令がなければ停止)
        let is_timeout = match last_cmd {
            Some(t) => t.elapsed().as_millis() > 500,
            None => true,
        };

        // 安全装置
        if estop || is_timeout {
            target_top = 0.0;
            target_bottom = 0.0;
        }

        // --- RPM計算 ---
        let count_top = qei_top.count();
        let delta_top = count_top.wrapping_sub(prev_count_top) as i16 as f32;
        prev_count_top = count_top;
        let actual_rpm_top = (delta_top / cpr) * (60.0 / dt);

        let count_bottom = qei_bottom.count();
        let delta_bottom = count_bottom.wrapping_sub(prev_count_bottom) as i16 as f32;
        prev_count_bottom = count_bottom;
        let actual_rpm_bottom = (delta_bottom / cpr) * (60.0 / dt);

        // --- PID計算 ---
        let err_top = target_top - actual_rpm_top;
        integral_top += err_top * dt;
        if integral_top > 500.0 {
            integral_top = 500.0;
        } else if integral_top < -500.0 {
            integral_top = -500.0;
        }
        let out_top = (kp * err_top) + (ki * integral_top) + (kd * (err_top - prev_error_top) / dt);
        prev_error_top = err_top;

        let err_bottom = target_bottom - actual_rpm_bottom;
        integral_bottom += err_bottom * dt;
        if integral_bottom > 500.0 {
            integral_bottom = 500.0;
        } else if integral_bottom < -500.0 {
            integral_bottom = -500.0;
        }
        let out_bottom = (kp * err_bottom)
            + (ki * integral_bottom)
            + (kd * (err_bottom - prev_error_bottom) / dt);
        prev_error_bottom = err_bottom;

        // --- PWM出力 ---
        let mut pulse_top = 1500.0 + out_top;
        let mut pulse_bottom = 1500.0 + out_bottom;

        if pulse_top > 2000.0 {
            pulse_top = 2000.0;
        } else if pulse_top < 1000.0 {
            pulse_top = 1000.0;
        }
        if pulse_bottom > 2000.0 {
            pulse_bottom = 2000.0;
        } else if pulse_bottom < 1000.0 {
            pulse_bottom = 1000.0;
        }

        if target_top == 0.0 {
            pulse_top = 1500.0;
            integral_top = 0.0;
        } // 1500が停止(車用ESC)
        if target_bottom == 0.0 {
            pulse_bottom = 1500.0;
            integral_bottom = 0.0;
        }

        set_esc_pwm(&mut pwm, embassy_stm32::timer::Channel::Ch1, pulse_top);
        set_esc_pwm(&mut pwm, embassy_stm32::timer::Channel::Ch2, pulse_bottom);
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let config = Config::default();
    let p = embassy_stm32::init(config);
    info!("Ultimate Shooter Firmware Booted.");

    let limit_sw1 = Input::new(p.PC13, Pull::Up);

    let mut can = Can::new(p.CAN1, p.PD0, p.PD1, Irqs);
    can.as_mut().modify_config().set_bit_timing(0x001c0000);

    // 最強の構成: ハードウェアCANフィルタ設定 (0x201と0x202のみ受信し、他は捨てる)
    can.as_mut().modify_filters().enable_bank(
        0,
        Mask32::frames_with_std_id(
            StandardId::new(0x201).unwrap(),
            StandardId::new(0x202).unwrap(),
        ),
    );

    let (tx, rx0, _rx1) = can.split();

    let qei_top = Qei::new(p.TIM4, p.PB6, p.PB7);
    let qei_bottom = Qei::new(p.TIM2, p.PA0, p.PA1);

    let ch1 = PwmPin::new_ch1(p.PC6, embassy_stm32::gpio::OutputType::PushPull);
    let ch2 = PwmPin::new_ch2(p.PC7, embassy_stm32::gpio::OutputType::PushPull);
    let mut pwm = SimplePwm::new(
        p.TIM3,
        Some(ch1),
        Some(ch2),
        None,
        None,
        khz(50),
        Default::default(),
    );
    pwm.enable(embassy_stm32::timer::Channel::Ch1);
    pwm.enable(embassy_stm32::timer::Channel::Ch2);

    set_esc_pwm(&mut pwm, embassy_stm32::timer::Channel::Ch1, 1500.0);
    set_esc_pwm(&mut pwm, embassy_stm32::timer::Channel::Ch2, 1500.0);
    Timer::after_millis(2000).await;

    spawner.spawn(can_rx_task(rx0)).unwrap();
    spawner.spawn(telemetry_task(tx, limit_sw1)).unwrap();
    spawner
        .spawn(pid_control_task(qei_top, qei_bottom, pwm))
        .unwrap();
}
