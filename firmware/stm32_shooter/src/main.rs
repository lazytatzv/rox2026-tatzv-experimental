#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_stm32::can::{bxcan::Frame, bxcan::StandardId, Can, Rx0InterruptHandler, TxInterruptHandler};
use embassy_stm32::peripherals::CAN1;
use embassy_stm32::{bind_interrupts, Config};
use embassy_time::{Duration, Ticker};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;

// A shared mutex to pass target RPM from CAN thread to PID thread
static TARGET_RPM: Mutex<CriticalSectionRawMutex, f32> = Mutex::new(0.0);

bind_interrupts!(struct Irqs {
    CAN1_RX0 => Rx0InterruptHandler<CAN1>;
    CAN1_TX => TxInterruptHandler<CAN1>;
});

#[embassy_executor::task]
async fn can_rx_task(mut can: Can<'static, CAN1>) {
    info!("CAN RX Task Started");
    loop {
        // Wait asynchronously for a CAN frame
        if let Ok(envelope) = can.read().await {
            let frame = envelope.frame;
            if let Some(id) = frame.id() {
                // We expect ID 0x201 from ROS2 mad_motor_driver_node
                if id == StandardId::new(0x201).unwrap() {
                    if let Some(data) = frame.data() {
                        // Assuming 4 bytes f32 for target RPM (Little Endian)
                        if data.len() >= 4 {
                            let bytes: [u8; 4] = [data[0], data[1], data[2], data[3]];
                            let target = f32::from_le_bytes(bytes);
                            
                            // Safely update the shared target RPM
                            let mut locked = TARGET_RPM.lock().await;
                            *locked = target;
                            
                            debug!("Received Target RPM: {}", target);
                        }
                    }
                }
            }
        }
    }
}

#[embassy_executor::task]
async fn pid_control_task() {
    info!("PID Control Task Started");
    
    // 1000Hz PID loop (1ms)
    let mut ticker = Ticker::every(Duration::from_millis(1));
    
    // Simple PID constants (Tune these on the real robot!)
    let kp = 0.5;
    let ki = 0.01;
    let kd = 0.05;
    
    let mut integral = 0.0;
    let mut prev_error = 0.0;

    loop {
        ticker.next().await;
        
        // 1. Get current target RPM safely
        let target_rpm = *TARGET_RPM.lock().await;
        
        // 2. Read actual RPM from MAD Motor Encoder (Simulated here)
        // let actual_rpm = read_encoder_rpm().await;
        let actual_rpm = 0.0; // TODO: Implement encoder read
        
        // 3. Compute PID
        let error = target_rpm - actual_rpm;
        integral += error;
        
        // Anti-windup
        if integral > 1000.0 { integral = 1000.0; }
        if integral < -1000.0 { integral = -1000.0; }
        
        let derivative = error - prev_error;
        let mut pwm_out = (kp * error) + (ki * integral) + (kd * derivative);
        
        // Clamp PWM to MAD motor bounds (-255 to 255, or similar)
        if pwm_out > 255.0 { pwm_out = 255.0; }
        if pwm_out < -255.0 { pwm_out = -255.0; }
        
        prev_error = error;
        
        // 4. Send to PWM Hardware
        // set_pwm_duty_cycle(pwm_out as i32); // TODO: Implement TIM PWM output
        
        // Safely stop if target is 0
        if target_rpm == 0.0 {
            // set_pwm_duty_cycle(0);
            integral = 0.0;
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = Config::default();
    // Nucleo-F767ZI typically runs at 216 MHz. Configure clocks here.
    
    let p = embassy_stm32::init(config);
    info!("STM32 F767ZI Booted. Initializing Ultimate Shooter Firmware.");

    // Initialize CAN1 (Pins PD0 / PD1 are common for CAN on Nucleo F767ZI)
    let can = Can::new(p.CAN1, p.PD0, p.PD1, Irqs);
    // Note: Configure CAN bitrate via split() and bxcan in a real scenario
    
    // Spawn tasks
    spawner.spawn(can_rx_task(can)).unwrap();
    spawner.spawn(pid_control_task()).unwrap();
}
