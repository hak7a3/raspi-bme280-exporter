use std::{error::Error, net::SocketAddr, str::FromStr, thread, time::{Duration, SystemTime}};

use bme280::CalibrationData;
use clap::Parser;
use embedded_hal::i2c;

mod raspi;
mod bme280;

#[derive(Debug, Parser)]
#[command(version, about, long_about = None)]
struct Args {
    #[arg(short, long, default_value_t = String::from("0.0.0.0:9000"))]
    server: String,
    #[arg(short, long, default_value_t = 0x76)]
    adress: u8,
}

fn main() {
    env_logger::init();
    let args = Args::parse();

    log::info!("start bme280 exporter");

    init_prometheus(&args.server).expect("failed to install prometheus exporter");
    log::info!("start prometheus server at {:}", args.server);

    let mut i2c = raspi::init_raspi().expect("failed to init i2c");

    let calib = init_bme280(&mut i2c, args.adress).expect("failed to init bme280");
    log::info!("calibration data: {:?}", calib);

    let press = metrics::gauge!("pressure_pascal");
    let temp = metrics::gauge!("temperature_celsius");
    let hum = metrics::gauge!("humidity_percent");
    let last_measured = metrics::gauge!("last_measured_timestamp_ms");

    loop {
        thread::sleep(Duration::from_secs(1));

        let timestamp = SystemTime::now().duration_since(SystemTime::UNIX_EPOCH).expect("failed to get timestamp");

        let measurement = bme280::read_measurement(&mut i2c, args.adress, &calib).expect("failed to read measurement");

        press.set(measurement.pressure);
        temp.set(measurement.temperature);
        hum.set(measurement.humidity);
        last_measured.set(timestamp.as_millis() as f64);
    }
}

fn init_prometheus(addr: &str) -> Result<(), Box<dyn Error>> {
    let socket = SocketAddr::from_str(addr)?;

    let builder = metrics_exporter_prometheus::PrometheusBuilder::new();
    builder.with_http_listener(socket).install()?;

    return Ok(());
}

fn init_bme280<I: i2c::I2c>(i2c: &mut I, address: u8) -> Result<CalibrationData, I::Error> {
    let id = bme280::read_id(i2c, address)?;
    log::info!("chip id: {:}", id);
    bme280::reset(i2c, address)?;
    let calib = bme280::get_calib(i2c, address)?;
    bme280::overwrite_config(i2c, address, &bme280::Config {
        oversampling_pressure: bme280::Oversampling::X16,
        oversampling_temperature: bme280::Oversampling::X2,
        oversampling_humidity: bme280::Oversampling::X1,
        standby_time: bme280::StandbyTime::Ms0_5,
        filter: bme280::Filter::Coefficient16,
    })?;
    bme280::set_mode(i2c, address, bme280::Mode::Normal)?;
    return Ok(calib);
}
