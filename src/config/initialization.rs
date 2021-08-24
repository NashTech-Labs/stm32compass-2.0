//! Initialization code

#![no_std]

extern crate panic_itm; // panic handler
pub use cortex_m::{asm::bkpt, iprint, iprintln, peripheral::ITM};
pub use cortex_m_rt::entry;

use lsm303agr::UnscaledMeasurement;
pub use lsm303agr::{interface::I2cInterface, mode, Lsm303agr, MagOutputDataRate};
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
