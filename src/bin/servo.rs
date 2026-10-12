#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_stm32::gpio::OutputType;
use embassy_stm32::time::hz;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_time::{Duration, Instant, Timer};
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    // Use PA8 / TIM1_CH1 on the U545RE; connect the servo signal wire to PA8.
    // IMPORTANT: SG90 servo requires 4.8-6V power supply (not 3.3V from board).
    // Connect servo VCC to external 5V, GND to board GND, signal to PA8.
    let ch1_pin = PwmPin::new(p.PA8, OutputType::PushPull);

    let mut pwm = SimplePwm::new(
        p.TIM1,
        Some(ch1_pin),
        None,
        None,
        None,
        hz(50), // 50 Hz servo PWM
        Default::default(),
    );

    let mut ch1 = pwm.ch1();
    ch1.enable();

    let max_duty = ch1.max_duty_cycle();

    const MIN_PULSE_US: u32 = 500; // 0°
    const MAX_PULSE_US: u32 = 1500; // 90°
    const PERIOD_US: u32 = 20_000; // 20 ms

    let min_duty = max_duty * MIN_PULSE_US / PERIOD_US;
    let max_duty_servo = max_duty * MAX_PULSE_US / PERIOD_US;

    let start = Instant::now();

    loop {
        let elapsed_ms = start.elapsed().as_millis() as u32;
        let completed_cycles = elapsed_ms / 3000;

        let duty = if completed_cycles < 4 {
            let cycle_ms = elapsed_ms % 3000;
            if cycle_ms < 1000 {
                // 0° -> 90° in 1 second
                let progress = cycle_ms;
                min_duty + (progress * (max_duty_servo - min_duty)) / 1000
            } else if cycle_ms < 2000 {
                // 90° -> 0° in 1 second
                let progress = cycle_ms - 1000;
                max_duty_servo - (progress * (max_duty_servo - min_duty)) / 1000
            } else {
                // pause for 1 second at 0° after two 90° sweeps
                min_duty
            }
        } else {
            // Stop completely after 5 full cycles.
            min_duty
        };

        ch1.set_duty_cycle(duty);

        if completed_cycles >= 5 {
            // Keep holding the stopped position without motion.
            loop {
                Timer::after(Duration::from_secs(1)).await;
            }
        }

        Timer::after(Duration::from_millis(10)).await;
    }
}
