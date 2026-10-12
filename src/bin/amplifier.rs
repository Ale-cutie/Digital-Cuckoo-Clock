#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::dac::{DacChannel, Value};
use embassy_stm32::rcc::mux;
use embassy_stm32::Config;
use embassy_time::{Duration, Instant, Timer};
use micromath::F32Ext;
use {defmt_rtt as _, panic_probe as _};

const TONE_HZ: f32 = 1000.0;
const SAMPLE_RATE_HZ: f32 = 32_000.0;
const DAC_CENTER: i32 = 2048;
const AMPLITUDE: i32 = 300;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = Config::default();
    config.rcc.mux.dac1sel = mux::Dacsel::LSI;
    let p = embassy_stm32::init(config);

    info!("PAM8403 mono test: PA4 tone, feed both LIN and RIN through resistors");

    // PA4 = DAC1_OUT1 (CN7-32).
    //
    // Wiring:
    //   PA4/CN7-32 -> 1k -> PAM8403 LIN
    //   PA4/CN7-32 -> 1k -> PAM8403 RIN
    //   STM32 GND  -> PAM8403 input GND
    let mut ch = DacChannel::new_blocking(p.DAC1, p.PA4);

    let phase_step: f32 = 2.0 * core::f32::consts::PI * TONE_HZ / SAMPLE_RATE_HZ;
    let sample_period = Duration::from_micros((1_000_000.0 / SAMPLE_RATE_HZ) as u64);

    let mut phase: f32 = 0.0;
    let mut next = Instant::now();
    let mut last_log = Instant::now();
    let two_pi = 2.0 * core::f32::consts::PI;

    loop {
        let v = (DAC_CENTER + (phase.sin() * AMPLITUDE as f32) as i32).clamp(0, 4095) as u16;
        ch.set(Value::Bit12Right(v));

        phase += phase_step;
        if phase >= two_pi {
            phase -= two_pi;
        }

        let now = Instant::now();
        if now.duration_since(last_log) >= Duration::from_secs(2) {
            info!("still playing");
            last_log = now;
        }

        next += sample_period;
        Timer::at(next).await;
    }
}
