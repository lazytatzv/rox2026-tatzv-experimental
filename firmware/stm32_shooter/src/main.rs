#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_stm32::can::bxcan::Frame;
use embassy_stm32::can::{bxcan::StandardId, Can, Rx0InterruptHandler, TxInterruptHandler};
use embassy_stm32::gpio::{Input, Pull};
use embassy_stm32::peripherals::{CAN1, TIM2, TIM3, TIM4};
use embassy_stm32::time::khz;
use embassy_stm32::timer::qei::{Qei, QeiDir};
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::{bind_interrupts, Config};
use embassy_time::{Duration, Ticker, Timer};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;

// 左右のモーターの目標RPMを共有
static TARGET_RPM_L: Mutex<CriticalSectionRawMutex, f32> = Mutex::new(0.0);
static TARGET_RPM_R: Mutex<CriticalSectionRawMutex, f32> = Mutex::new(0.0);

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
                // ID: 0x201 (Left Motor), 0x202 (Right Motor)
                if id == StandardId::new(0x201).unwrap() || id == StandardId::new(0x202).unwrap() {
                    if let Some(data) = frame.data() {
                        if data.len() >= 4 {
                            let bytes: [u8; 4] = [data[0], data[1], data[2], data[3]];
                            let target = f32::from_le_bytes(bytes);
                            
                            if id == StandardId::new(0x201).unwrap() {
                                let mut locked = TARGET_RPM_L.lock().await;
                                *locked = target;
                            } else {
                                let mut locked = TARGET_RPM_R.lock().await;
                                *locked = target;
                            }
                        }
                    }
                }
            }
        }
    }
}

#[embassy_executor::task]
async fn limit_switch_task(
    mut tx: embassy_stm32::can::Tx<'static, CAN1>,
    mut limit_sw: Input<'static>
) {
    info!("Limit Switch Task Started");
    let mut ticker = Ticker::every(Duration::from_millis(20)); // 50Hz

    loop {
        ticker.next().await;

        // スイッチが押された(LOW)なら1、離された(HIGH)なら0
        let is_pressed = if limit_sw.is_low() { 1u8 } else { 0u8 };

        // CAN ID 0x200 で送信 (DLC = 1)
        let frame = Frame::new_data(StandardId::new(0x200).unwrap(), [is_pressed]);
        
        // CAN送信 (メールボックスが空くまで非同期で待機)
        let _ = tx.write(&frame).await;
    }
}

// ヘルパー: ESC用PWM出力 (1000us ~ 2000us)
fn set_esc_pwm(pwm: &mut SimplePwm<'_, TIM3>, ch: embassy_stm32::timer::Channel, pulse_us: f32) {
    let max_duty = pwm.get_max_duty() as f32;
    // 50Hz (20000us周期)
    let duty = (pulse_us / 20000.0) * max_duty;
    pwm.set_duty(ch, duty as u16);
}

#[embassy_executor::task]
async fn pid_control_task(
    mut qei_l: Qei<'static, TIM4>,
    mut qei_r: Qei<'static, TIM2>,
    mut pwm: SimplePwm<'static, TIM3>
) {
    info!("PID & ESC PWM Task Started");
    
    // 100Hz PID loop (10ms) - ESCの応答速度に合わせて調整
    let dt = 0.01;
    let mut ticker = Ticker::every(Duration::from_millis(10));
    
    // PIDゲイン (実機で調整必須)
    let kp = 0.5;
    let ki = 0.05;
    let kd = 0.01;
    
    let mut integral_l = 0.0;
    let mut prev_error_l = 0.0;
    let mut prev_count_l = qei_l.count();

    let mut integral_r = 0.0;
    let mut prev_error_r = 0.0;
    let mut prev_count_r = qei_r.count();

    // エンコーダの分解能 (PPR * 4) -> 例: 2048 * 4 = 8192
    let cpr = 8192.0;

    loop {
        ticker.next().await;
        
        let target_l = *TARGET_RPM_L.lock().await;
        let target_r = *TARGET_RPM_R.lock().await;
        
        // --- 実際のRPM計算 (Left) ---
        let current_count_l = qei_l.count();
        let delta_count_l = current_count_l.wrapping_sub(prev_count_l) as i16 as f32;
        prev_count_l = current_count_l;
        let actual_rpm_l = (delta_count_l / cpr) * (60.0 / dt);

        // --- 実際のRPM計算 (Right) ---
        let current_count_r = qei_r.count();
        let delta_count_r = current_count_r.wrapping_sub(prev_count_r) as i16 as f32;
        prev_count_r = current_count_r;
        let actual_rpm_r = (delta_count_r / cpr) * (60.0 / dt);

        // --- PID計算 (Left) ---
        let error_l = target_l - actual_rpm_l;
        integral_l += error_l * dt;
        if integral_l > 500.0 { integral_l = 500.0; } else if integral_l < -500.0 { integral_l = -500.0; }
        let derivative_l = (error_l - prev_error_l) / dt;
        let output_l = (kp * error_l) + (ki * integral_l) + (kd * derivative_l);
        prev_error_l = error_l;

        // --- PID計算 (Right) ---
        let error_r = target_r - actual_rpm_r;
        integral_r += error_r * dt;
        if integral_r > 500.0 { integral_r = 500.0; } else if integral_r < -500.0 { integral_r = -500.0; }
        let derivative_r = (error_r - prev_error_r) / dt;
        let output_r = (kp * error_r) + (ki * integral_r) + (kd * derivative_r);
        prev_error_r = error_r;

        // --- ESCへのPWM出力計算 (1000us ~ 2000us) ---
        let mut pulse_l = 1500.0 + output_l;
        let mut pulse_r = 1500.0 + output_r;

        if pulse_l > 2000.0 { pulse_l = 2000.0; } else if pulse_l < 1000.0 { pulse_l = 1000.0; }
        if pulse_r > 2000.0 { pulse_r = 2000.0; } else if pulse_r < 1000.0 { pulse_r = 1000.0; }

        if target_l == 0.0 { pulse_l = 1000.0; integral_l = 0.0; }
        if target_r == 0.0 { pulse_r = 1000.0; integral_r = 0.0; }

        set_esc_pwm(&mut pwm, embassy_stm32::timer::Channel::Ch1, pulse_l);
        set_esc_pwm(&mut pwm, embassy_stm32::timer::Channel::Ch2, pulse_r);
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let config = Config::default();
    let p = embassy_stm32::init(config);
    info!("Ultimate Shooter Firmware Booted.");

    // --- 1. リミットスイッチ初期化 (例: PC13, プルアップ) ---
    let limit_sw = Input::new(p.PC13, Pull::Up);

    // --- 2. CAN 初期化 (PD0, PD1) ---
    let mut can = Can::new(p.CAN1, p.PD0, p.PD1, Irqs);
    can.as_mut().modify_config().set_bit_timing(0x001c0000); 
    can.as_mut().modify_filters().clear();
    
    // CANを送信・受信用に分割
    let (tx, rx0, _rx1) = can.split();

    // --- 3. エンコーダ (QEI) 初期化 ---
    let qei_l = Qei::new(p.TIM4, p.PB6, p.PB7);
    let qei_r = Qei::new(p.TIM2, p.PA0, p.PA1);

    // --- 4. ESC向け PWM 初期化 ---
    let ch1 = PwmPin::new_ch1(p.PC6, embassy_stm32::gpio::OutputType::PushPull);
    let ch2 = PwmPin::new_ch2(p.PC7, embassy_stm32::gpio::OutputType::PushPull);
    let mut pwm = SimplePwm::new(p.TIM3, Some(ch1), Some(ch2), None, None, khz(50), Default::default());
    pwm.enable(embassy_stm32::timer::Channel::Ch1);
    pwm.enable(embassy_stm32::timer::Channel::Ch2);

    set_esc_pwm(&mut pwm, embassy_stm32::timer::Channel::Ch1, 1000.0);
    set_esc_pwm(&mut pwm, embassy_stm32::timer::Channel::Ch2, 1000.0);
    Timer::after_millis(2000).await;

    // タスク起動
    spawner.spawn(can_rx_task(rx0)).unwrap();
    spawner.spawn(limit_switch_task(tx, limit_sw)).unwrap();
    spawner.spawn(pid_control_task(qei_l, qei_r, pwm)).unwrap();
}
