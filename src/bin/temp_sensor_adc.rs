#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_time::Timer;

use embassy_stm32::adc::{Adc, SampleTime};

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_stm32::init(Default::default());

    info!("ADC + temperature simulation started");

    let mut adc = Adc::new(p.ADC1);

    let mut adc_pin = p.PA3;

    loop {
        let value: u16 = adc.blocking_read(&mut adc_pin, SampleTime::CYCLES7_5);

        let voltage = (value as f32) * 3.3 / 4095.0;
        let temperature = voltage * 100.0;

        info!(
            "ADC: {} | Voltage: {} V | Temp: {} °C",
            value, voltage, temperature
        );

        Timer::after_millis(2000).await;
    }
}
