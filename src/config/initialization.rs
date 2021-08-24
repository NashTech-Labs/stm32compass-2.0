//! Initialization code

#![no_std]

extern crate panic_itm; // panic handler
pub use cortex_m::{asm::bkpt, iprint, iprintln, peripheral::ITM};
pub use cortex_m_rt::entry;
pub use m;
use m::Float;
use stm32f3_discovery::stm32f3xx_hal::{
    gpio::gpiob::{PB6, PB7},
    gpio::AF4,
    i2c::I2c,
    prelude::*,
    stm32::{self, I2C1},
};
pub use stm32f3_discovery::{
    leds::Leds,
    stm32f3xx_hal::{delay::Delay, prelude, stm32::i2c1},
    switch_hal,
};

use core::f32::consts::PI;
use lsm303agr::UnscaledMeasurement;
pub use lsm303agr::{interface::I2cInterface, mode, Lsm303agr, MagOutputDataRate};

/// Cardinal directions. Each one matches one of the user LEDs.
#[derive(Clone, Copy)]
pub enum Direction {
    /// North / LD3
    North,
    /// Northeast / LD5
    Northeast,
    /// East / LD7
    East,
    /// Southeast / LD9
    Southeast,
    /// South / LD10
    South,
    /// Southwest / LD8
    Southwest,
    /// West / LD6
    West,
    /// Northwest / LD4
    Northwest,
}
/// Function init() provide the implementation to access leds, lsm303agr package and itm.
///
///  #Arguments
/// -> None
///
/// #Returns
/// function returns a tuple of (led, lsm303agr, delay, itm)
/// Leds -> return f3 board led.
/// Lsm303agr -> package return magnetometer sensor.
/// Delay -> return the delay function to pause the code.
/// ITM -> return the itm console to print data.
pub fn init() -> (
    Leds,
    Lsm303agr<I2cInterface<I2c<I2C1, (PB6<AF4>, PB7<AF4>)>>, mode::MagContinuous>,
    Delay,
    ITM,
) {
    let cp = match cortex_m::Peripherals::take() {
        Some(peripheral) => peripheral,
        None => panic!("Error"),
    };
    let dp = stm32::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();

    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let mut gpioe = dp.GPIOE.split(&mut rcc.ahb);
    let leds = Leds::new(
        gpioe.pe8,
        gpioe.pe9,
        gpioe.pe10,
        gpioe.pe11,
        gpioe.pe12,
        gpioe.pe13,
        gpioe.pe14,
        gpioe.pe15,
        &mut gpioe.moder,
        &mut gpioe.otyper,
    );

    let mut gpiob = dp.GPIOB.split(&mut rcc.ahb);
    let scl = gpiob.pb6.into_af4(&mut gpiob.moder, &mut gpiob.afrl);
    let sda = gpiob.pb7.into_af4(&mut gpiob.moder, &mut gpiob.afrl);

    let i2c = I2c::new(dp.I2C1, (scl, sda), 400.khz(), clocks, &mut rcc.apb1);

    let mut lsm = Lsm303agr::new_with_i2c(i2c);
    lsm.init().expect("Failed to access lsm303agr package");
    lsm.set_mag_odr(MagOutputDataRate::Hz10).unwrap();
    let mut lsm303agr = lsm
        .into_mag_continuous()
        .ok()
        .expect("Failed to read in continous mode ");

    let delay = Delay::new(cp.SYST, clocks);

    (leds, lsm303agr, delay, cp.ITM)
}
/// This function calculates the Direction of the Magnetic Field using x, y, z-axis readings.
///
/// #Input
/// Function magnitude takes a single input lsm303agr as package("Lsm303agr") type.
///
/// #Return
/// magnitude function return enum of (take_readings, direction, theta)
/// take_readings -> this return the readings of the Magnetometer sensor.
/// direction -> return the direction of Magnetic Field based on theta value.
/// theta -> return the value of theta based on "atan2" function.
pub fn direction(
    lsm303agr: &mut Lsm303agr<I2cInterface<I2c<I2C1, (PB6<AF4>, PB7<AF4>)>>, mode::MagContinuous>,
) -> (UnscaledMeasurement, Direction, f32) {
    let take_readings = lsm303agr.mag_data().expect("Error in readings package.");
    let theta = (take_readings.y as f32).atan2(take_readings.x as f32);
    let direction = if theta < -7. * PI / 8. {
        Direction::North
    } else if theta < -5. * PI / 8. {
        Direction::Northwest
    } else if theta < -3. * PI / 8. {
        Direction::West
    } else if theta < -PI / 8. {
        Direction::Southwest
    } else if theta < PI / 8. {
        Direction::South
    } else if theta < 3. * PI / 8. {
        Direction::Southeast
    } else if theta < 5. * PI / 8. {
        Direction::East
    } else if theta < 7. * PI / 8. {
        Direction::Northeast
    } else {
        Direction::North
    };
    (take_readings, direction, theta)
}
/// This function calculates the Magnitude of the Magnetic Field using x, y, z-axis readings.
///
/// #Input
/// Function magnitude takes a single input lsm303agr as package("Lsm303agr") type.
///
/// #Return
/// magnitude function return the calculated magnitude of the Magnetic Field.
pub fn magnitude(
    lsm303agr: &mut Lsm303agr<I2cInterface<I2c<I2C1, (PB6<AF4>, PB7<AF4>)>>, mode::MagContinuous>,
) -> f32 {
    let take_readings = lsm303agr.mag_data().expect("Failed to read the values.");
    const XY_GAIN: f32 = 1100.;
    const Z_GAIN: f32 = 980.;
    let x = f32::from(take_readings.x) / XY_GAIN;
    let y = f32::from(take_readings.y) / XY_GAIN;
    let z = f32::from(take_readings.z) / Z_GAIN;
    let magnitude = (x * x + y * y + z * z).sqrt();
    magnitude
}
