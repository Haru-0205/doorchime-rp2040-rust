//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use bsp::{
    entry,
    hal::{
        // dma::Channel,
        pwm::{FreeRunning, Slices},
    },
};
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::{InputPin, OutputPin};
use embedded_hal::PwmPin;
use panic_probe as _;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // This is the correct pin on the Raspberry Pico board. On other boards, even if they have an
    // on-board LED, it might need to be changed.
    //
    // Notably, on the Pico W, the LED is not connected to any of the RP2040 GPIOs but to the cyw43 module instead.
    // One way to do that is by using [embassy](https://github.com/embassy-rs/embassy/blob/main/examples/rp/src/bin/wifi_blinky.rs)
    //
    // If you have a Pico W and want to toggle a LED with a simple GPIO output pin, you can connect an external
    // LED to one of the GPIO pins, and reference that pin here. Don't forget adding an appropriate resistor
    // in series with the LED.

    let sw_test = pins.gpio0.into_pull_down_input();
    let door_sw = pins.gpio15.into_pull_down_input();
    let mut power_led = pins.led.into_push_pull_output();
    let mut led = pins.gpio10.into_push_pull_output();
    let pwm_pin = pins.gpio16.into_push_pull_output();
    let pwm_slices = Slices::new(pac.PWM, &mut pac.RESETS);
    let mut pwm = pwm_slices.pwm0;
    pwm.set_ph_correct();
    pwm.enable();
    let channel_a = &mut pwm.channel_a;
    let _channel_pin_a = channel_a.output_to(pwm_pin);
    channel_a.set_duty(1420 / 6);
    power_led.set_high().ok().unwrap();

    loop {
        delay.delay_ms(40);
        if door_sw.is_low().ok().unwrap() || sw_test.is_high().ok().unwrap() {
            led.set_high().ok().unwrap();
            pwm.set_top(868 - 1);
            pwm.set_div_int(200);
            pwm.set_div_frac(0);

            delay.delay_ms(400);

            pwm.set_top(1064 - 1);

            delay.delay_ms(400);

            pwm.set_top(1420 - 1);

            delay.delay_ms(400);

            pwm.set_top(1064 - 1);

            delay.delay_ms(400);

            pwm.set_top(948 - 1);

            delay.delay_ms(400);

            pwm.set_top(710 - 2);

            delay.delay_ms(600);

            pwm.set_top(0);
            delay.delay_ms(200);

            pwm.set_top(1899 - 1);

            delay.delay_ms(400);

            pwm.set_top(948 - 1);

            delay.delay_ms(400);

            pwm.set_top(845 - 1);

            delay.delay_ms(400);

            pwm.set_top(948 - 1);

            delay.delay_ms(400);

            pwm.set_top(1420 - 1);

            delay.delay_ms(400);

            pwm.set_top(1064 - 1);

            delay.delay_ms(800);

            pwm.set_top(0);
            led.set_low().ok().unwrap();

            delay.delay_ms(1800)
        } else {
        }
    }
}

// End of file
