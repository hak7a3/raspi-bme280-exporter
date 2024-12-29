use std::{thread, time::Duration};

use embedded_hal::i2c;

const REG_ID: u8 = 0xD0;
const REG_RESET: u8 = 0xE0;
const COMMAND_RESET: u8 = 0xB6;
const REG_TEMP_PRESS_CALIB_DATA: u8 = 0x88;
const LEN_TEMP_PRESS_CALIB_DATA: usize = 26;
const REG_HUMIDITY_CALIB_DATA: u8 = 0xE1;
const LEN_HUMIDITY_CALIB_DATA: usize = 7;
const REG_CTRL_HUM: u8 = 0xF2;
const REG_CTRL_MEAS: u8 = 0xF4;
const REG_CONFIG: u8 = 0xF5;
const REG_MEASUREMENT: u8 = 0xF7;
const LEN_MEASUREMENT: usize = 8;

const DELAY_STARTUP: Duration = Duration::from_millis(2);

#[derive(Debug)]
pub(crate) struct CalibrationData {
    dig_t1: u16,
    dig_t2: i16,
    dig_t3: i16,
    dig_p1: u16,
    dig_p2: i16,
    dig_p3: i16,
    dig_p4: i16,
    dig_p5: i16,
    dig_p6: i16,
    dig_p7: i16,
    dig_p8: i16,
    dig_p9: i16,
    dig_h1: u8,
    dig_h2: i16,
    dig_h3: u8,
    dig_h4: i16,
    dig_h5: i16,
    dig_h6: i8,
}

#[allow(dead_code)]
#[derive(Debug, Clone, Copy)]
pub(crate) enum Oversampling {
    None = 0x00,
    X1 = 0x01,
    X2 = 0x02,
    X4 = 0x03,
    X8 = 0x04,
    X16 = 0x05,
}

#[allow(dead_code)]
#[derive(Debug, Clone, Copy)]
pub(crate) enum StandbyTime {
    Ms0_5 = 0x00,
    Ms62_5 = 0x01,
    Ms125 = 0x02,
    Ms250 = 0x03,
    Ms500 = 0x04,
    Ms1000 = 0x05,
    Ms10 = 0x06,
    Ms20 = 0x07,
}

#[allow(dead_code)]
#[derive(Debug, Clone, Copy)]
pub(crate) enum Filter {
    Off = 0x00,
    Coefficient2 = 0x01,
    Coefficient4 = 0x02,
    Coefficient8 = 0x03,
    Coefficient16 = 0x04,
}

pub(crate) struct Config {
    pub(crate) oversampling_pressure: Oversampling,
    pub(crate) oversampling_temperature: Oversampling,
    pub(crate) oversampling_humidity: Oversampling,
    pub(crate) standby_time: StandbyTime,
    pub(crate) filter: Filter,
}
struct RawCconfig {
    ctrl_hum: u8,
    ctrl_meas: u8,
    config: u8,
}

#[allow(dead_code)]
#[derive(Debug, Clone, Copy)]
pub(crate) enum Mode {
    Sleep = 0x00,
    Forced = 0x01,
    Normal = 0x03,
}

pub(crate) struct Measurement {
    pub(crate) temperature: f64,
    pub(crate) pressure: f64,
    pub(crate) humidity: f64,
}

struct RawMeasurement {
    temperature: u32,
    pressure: u32,
    humidity: u32,
}


pub(crate) fn read_id<I: i2c::I2c>(i2c: &mut I, address: u8) -> Result<u8, I::Error> {
    return _read_reg(i2c, address, REG_ID);
}

pub(crate) fn reset<I: i2c::I2c>(i2c: &mut I, address: u8) -> Result<(), I::Error> {
    let _ = i2c.write(address, &[REG_RESET, COMMAND_RESET]);

    // TODO: read status register to check if reset is done
    thread::sleep(DELAY_STARTUP);

    return Ok(());
}

pub(crate) fn get_calib<I: i2c::I2c>(i2c: &mut I, address: u8) -> Result<CalibrationData, I::Error> {
    let mut raw_temp_press_carib: [u8; LEN_TEMP_PRESS_CALIB_DATA] = [0; LEN_TEMP_PRESS_CALIB_DATA];
    i2c.write_read(address, &[REG_TEMP_PRESS_CALIB_DATA], &mut raw_temp_press_carib)?;

    let mut raw_humidity_carib: [u8; LEN_HUMIDITY_CALIB_DATA] = [0; LEN_HUMIDITY_CALIB_DATA];
    i2c.write_read(address, &[REG_HUMIDITY_CALIB_DATA], &mut raw_humidity_carib)?;

    return Ok(CalibrationData {
        dig_t1: ((raw_temp_press_carib[1] as u16) << 8) | (raw_temp_press_carib[0] as u16),
        dig_t2: ((raw_temp_press_carib[3] as i16) << 8) | (raw_temp_press_carib[2] as i16),
        dig_t3: ((raw_temp_press_carib[5] as i16) << 8) | (raw_temp_press_carib[4] as i16),
        dig_p1: ((raw_temp_press_carib[7] as u16) << 8) | (raw_temp_press_carib[6] as u16),
        dig_p2: ((raw_temp_press_carib[9] as i16) << 8) | (raw_temp_press_carib[8] as i16),
        dig_p3: ((raw_temp_press_carib[11] as i16) << 8) | (raw_temp_press_carib[10] as i16),
        dig_p4: ((raw_temp_press_carib[13] as i16) << 8) | (raw_temp_press_carib[12] as i16),
        dig_p5: ((raw_temp_press_carib[15] as i16) << 8) | (raw_temp_press_carib[14] as i16),
        dig_p6: ((raw_temp_press_carib[17] as i16) << 8) | (raw_temp_press_carib[16] as i16),
        dig_p7: ((raw_temp_press_carib[19] as i16) << 8) | (raw_temp_press_carib[18] as i16),
        dig_p8: ((raw_temp_press_carib[21] as i16) << 8) | (raw_temp_press_carib[20] as i16),
        dig_p9: ((raw_temp_press_carib[23] as i16) << 8) | (raw_temp_press_carib[22] as i16),
        dig_h1: raw_temp_press_carib[25],
        dig_h2: ((raw_humidity_carib[1] as i16) << 8) | (raw_humidity_carib[0] as i16),
        dig_h3: raw_humidity_carib[2],
        dig_h4: ((raw_humidity_carib[3] as i16) << 4) | ((raw_humidity_carib[4] as i16) & 0x0F),
        dig_h5: ((raw_humidity_carib[5] as i16) << 4) | ((raw_humidity_carib[4] as i16) >> 4),
        dig_h6: raw_humidity_carib[6] as i8,
    });
}

pub(crate) fn overwrite_config<I: i2c::I2c>(i2c: &mut I, address: u8, config: &Config) -> Result<(), I::Error> {
    let raw_config = _get_raw_config(i2c, address)?;

    let _ = i2c.write(address, &[REG_CTRL_HUM,
        (raw_config.ctrl_hum & 0b11111100) | (config.oversampling_humidity as u8)]);
    let _ = i2c.write(address, &[REG_CTRL_MEAS, 
        (raw_config.ctrl_meas & 0b00000011) | (config.oversampling_temperature as u8) << 5 | (config.oversampling_pressure as u8) << 2]);
    let _ = i2c.write(address, &[REG_CONFIG,
        (raw_config.config & 0b00000011) | (config.standby_time as u8) << 5 | (config.filter as u8) << 2]);
    
    return Ok(());
}


fn _get_raw_config<I: i2c::I2c>(i2c: &mut I, address: u8) -> Result<RawCconfig, I::Error> {
    return Ok(RawCconfig {
        ctrl_hum: _read_reg(i2c, address, REG_CTRL_HUM)?,
        ctrl_meas: _read_reg(i2c, address, REG_CTRL_MEAS)?,
        config: _read_reg(i2c, address, REG_CONFIG)?,
    });
}


pub(crate) fn set_mode<I: i2c::I2c>(i2c: &mut I, address: u8, mode: Mode) -> Result<(), I::Error> {
    let raw_config = _get_raw_config(i2c, address)?;
    let _ = i2c.write(address, &[REG_CTRL_MEAS, (raw_config.ctrl_meas & 0b11111100) | (mode as u8)]);
    return Ok(());
}


pub(crate) fn read_measurement<I: i2c::I2c>(i2c: &mut I, address: u8, calib: &CalibrationData) -> Result<Measurement, I::Error> {
    let raw_measurement = _read_raw_measurement(i2c, address)?;
    let t_fine = _calc_tfine(raw_measurement.temperature, calib);
    return Ok(Measurement {
        temperature: _calc_temperature(raw_measurement.temperature, calib),
        pressure: _calc_pressure(raw_measurement.pressure, t_fine, calib),
        humidity: _calc_humidity(raw_measurement.humidity, t_fine, calib),
    });
}

fn _read_raw_measurement<I: i2c::I2c>(i2c: &mut I, address: u8) -> Result<RawMeasurement, I::Error> {
    let mut buf:[u8; LEN_MEASUREMENT] = [0; LEN_MEASUREMENT];
    let _ = i2c.write_read(address, &[REG_MEASUREMENT], &mut buf);
    
    return Ok(RawMeasurement {
        pressure: ((buf[0] as u32) << 12) | ((buf[1] as u32) << 4) | (buf[2] as u32 >> 4),
        temperature: ((buf[3] as u32) << 12) | ((buf[4] as u32) << 4) | (buf[5] as u32 >> 4),
        humidity: ((buf[6] as u32) << 8) | (buf[7] as u32),
    });
}

fn _calc_tfine(raw: u32, calib: &CalibrationData) -> f64 {
    let var1 = ((raw as f64) / 16384.0 - (calib.dig_t1 as f64) / 1024.0) * (calib.dig_t2 as f64);
    let var2 = (raw as f64) / 131072.0 - (calib.dig_t1 as f64) / 8192.0;
    let var2 = var2 * var2 * (calib.dig_t3 as f64);
    // NOTE: データシートの式はなぜかi32に変換しているが，おそらく不要なのでf64に変更
    return var1 + var2;
}

fn _calc_temperature(raw: u32, calib: &CalibrationData) -> f64 {
    let var1 = ((raw as f64) / 16384.0 - (calib.dig_t1 as f64) / 1024.0) * (calib.dig_t2 as f64);
    let var2 = (raw as f64) / 131072.0 - (calib.dig_t1 as f64) / 8192.0;
    let var2 = var2 * var2 * (calib.dig_t3 as f64);
    return (var1 + var2) / 5120.0;
}

fn _calc_pressure(raw: u32, t_fine: f64, calib: &CalibrationData) -> f64 {
    let var1 = t_fine / 2.0 - 64000.0;
    let var2 = var1 * var1 * (calib.dig_p6 as f64) / 32768.0;
    let var2 = var2 + var1 * (calib.dig_p5 as f64) * 2.0;
    let var2 = var2 / 4.0 + (calib.dig_p4 as f64) * 65536.0;
    let var3 = (calib.dig_p3 as f64) * var1 * var1 / 524288.0;
    let var1 = (var3 + (calib.dig_p2 as f64) * var1) / 524288.0;
    let var1 = (1.0 + var1 / 32768.0) * (calib.dig_p1 as f64);

    // NOTE: div0チェックは省略
    let pressure = 1048576.0 - raw as f64;
    let pressure = (pressure - var2 / 4096.0) * 6250.0 / var1;
    let var1 = (calib.dig_p9 as f64) * pressure * pressure / 2147483648.0;
    let var2 = pressure * (calib.dig_p8 as f64) / 32768.0;
    return pressure + (var1 + var2 + (calib.dig_p7 as f64)) / 16.0;
}

fn _calc_humidity(raw: u32, t_fine: f64, calib: &CalibrationData) -> f64 {
    let var1 = t_fine - 76800.0;
    let var2 = (calib.dig_h4 as f64) * 64.0 + ((calib.dig_h5 as f64) / 16384.0) * var1;
    let var3 = raw as f64 - var2;
    let var4 = (calib.dig_h2 as f64) / 65536.0;
    let var5 = 1.0 + ((calib.dig_h3 as f64) / 67108864.0) * var1;
    let var6 = 1.0 + (calib.dig_h6 as f64) / 67108864.0 * var1 * var5;
    let var6 = var3 * var4 * (var5 * var6);
    return var6 * (1.0 - (calib.dig_h1 as f64) * var6 / 524288.0);
}


fn _read_reg<I: i2c::I2c>(i2c: &mut I, address: u8, register: u8) -> Result<u8, I::Error> {
    let mut buf = [0];
    let _ = i2c.write_read(address, &[register], &mut buf);
    return Ok(buf[0]);
}