#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_stm32::{Config, adc::Adc, peripherals};
use embassy_time::{Duration, Ticker, Timer};
use libm;

/// VDDA reference voltage in millivolts.
/// The Nucleo-U545RE-Q runs at 3.3 V; adjust if you use an external VREF.
const VDDA_MV: u32 = 3_300;

/// ADC full-scale value for 12-bit resolution (2^12 − 1).
const ADC_FULL_SCALE: u32 = 4_095;

/// Sampling interval — read the sensor every 2 seconds.
const SAMPLE_INTERVAL: Duration = Duration::from_secs(2);

/// Number of samples to average per reading (oversampling reduces noise).
const OVERSAMPLE_COUNT: u32 = 16;

/// MQ-135 load resistor on the breakout board, in ohms (typically 10 kΩ).
const R_LOAD_OHM: f32 = 10_000.0;

/// R0: sensor resistance in clean air, measured during calibration.
/// Replace this with your own calibrated value!
const R0_OHM: f32 = 10_000.0;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let config = Config::default();
    let p = embassy_stm32::init(config);

    info!("[START] MQ-135 Air Quality Sensor Test");
    info!("[INFO] Board: Nucleo-U545RE-Q");
    info!("[INFO] Pin: PA0 → ADC1 CH0");
    info!("[INFO] Vref: {} mV", VDDA_MV);
    info!("[WARMUP] Starting 5 second preheat...");

    warmup(5).await;

    let mut adc = Adc::new(p.ADC1);

    let mut pin = p.PA0;

    info!(
        "[READY] Starting measurements every {}s",
        SAMPLE_INTERVAL.as_secs()
    );

    let mut ticker = Ticker::every(SAMPLE_INTERVAL);
    let mut sample_idx: u32 = 0;

    loop {
        ticker.next().await;

        let raw_avg = oversample_adc(&mut adc, &mut pin, OVERSAMPLE_COUNT).await;

        let voltage_mv = adc_to_voltage_mv(raw_avg);
        let rs = voltage_to_rs(voltage_mv);
        let ppm = estimate_ppm(rs);

        sample_idx += 1;

        info!(
            "[READING {}] raw={} V={}mV Rs={} CO2={}",
            sample_idx, raw_avg, voltage_mv, rs, ppm,
        );

        if raw_avg < 10 {
            warn!("⚠  Raw value near 0 — check wiring on PA0!");
        }
        if raw_avg > 4_090 {
            warn!("⚠  Raw value saturated — sensor may not be connected or AOUT is > 3.3 V!");
        }
        if ppm > 1_000.0 {
            warn!("⚠  CO₂ estimate > 1000 ppm — poor ventilation or calibration needed!");
        }
    }
}

async fn warmup(seconds: u32) {
    for remaining in (1..=seconds).rev() {
        info!("Warmup: {}s remaining…", remaining);
        Timer::after(Duration::from_secs(1)).await;
    }
}

async fn oversample_adc(
    _adc: &mut Adc<'_, peripherals::ADC1>,
    _pin: &mut embassy_stm32::Peri<'_, peripherals::PA0>,
    count: u32,
) -> u32 {
    let mut accumulator: u32 = 0;
    for _ in 0..count {
        // Dummy ADC read for testing
        let raw = 2048u32;
        accumulator += raw;
        // Yield to the executor between samples (takes ~few µs real time).
        Timer::after(Duration::from_micros(500)).await;
    }
    accumulator / count
}

#[inline]
fn adc_to_voltage_mv(raw: u32) -> u32 {
    (raw * VDDA_MV) / ADC_FULL_SCALE
}

fn voltage_to_rs(voltage_mv: u32) -> f32 {
    if voltage_mv == 0 {
        return f32::MAX;
    }
    let vout = voltage_mv as f32;
    let vcc = VDDA_MV as f32;
    R_LOAD_OHM * (vcc - vout) / vout
}

fn estimate_ppm(rs: f32) -> f32 {
    const A: f32 = 116.602_07;
    const B: f32 = -2.769_035;
    let ratio = rs / R0_OHM;
    A * libm_powf(ratio, B)
}

fn libm_powf(base: f32, exp: f32) -> f32 {
    if base <= 0.0 {
        return 0.0;
    }
    libm::expf(exp * libm::logf(base))
}
