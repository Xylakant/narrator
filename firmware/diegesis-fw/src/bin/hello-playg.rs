#![no_main]
#![no_std]

use diegesis_fw; // global logger + panicking-behavior + memory layout
use diegesis_fw::groundhog_nrf52::GlobalRollingTimer;
use groundhog::RollingTimer;
use nrf52840_hal::prelude::InputPin;
use nrf52840_hal::prelude::OutputPin;
use nrf52840_hal::{
    clocks::{Clocks, ExternalOscillator, Internal, LfOscStopped},
    gpio::{
        p0::Parts as P0Parts, p1::Parts as P1Parts, Input, Level, Output, Pin, PullUp, PushPull,
    },
    pac::{Interrupt, SPIM0, SPIM1, SPIM2, SPIM3},
    spim::Frequency,
    usbd::Usbd,
    pac,
};
use nrf_smartled::pwm::Pwm;
use smart_leds::{
    RGB, RGB8, SmartLedsWrite, colors, gamma, brightness,
};

#[cortex_m_rt::entry]
fn main() -> ! {
    defmt::info!("Hello, world!");

    let board = pac::Peripherals::take().unwrap();

    // Enable instruction caches for MAXIMUM SPEED
    board.NVMC.icachecnf.write(|w| w.cacheen().set_bit());
    cortex_m::asm::isb();

    defmt::info!("Hello, world!");

    while !board
        .POWER
        .usbregstatus
        .read()
        .vbusdetect()
        .is_vbus_present()
    {}

    // wait until USB 3.3V supply is stable
    while !board
        .POWER
        .events_usbpwrrdy
        .read()
        .events_usbpwrrdy()
        .bit_is_clear()
    {}

    let clocks = Clocks::new(board.CLOCK);
    let clocks = clocks.enable_ext_hfosc();

    GlobalRollingTimer::init(board.TIMER0);
    let usbd = board.USBD;
    let gpios_p0 = P0Parts::new(board.P0);
    let gpios_p1 = P1Parts::new(board.P1);

    let start_stop_btn = gpios_p1.p1_15.into_pulldown_input().degrade();
    let mut start_stop_led = gpios_p1.p1_14.into_push_pull_output(Level::Low).degrade();

    let all_colors: &[RGB8; 10] = &[
        RGB8 { r: 0, g: 0, b: 2 },
        RGB8 { r: 0, g: 0, b: 2 },
        RGB8 { r: 0, g: 0, b: 2 },
        RGB8 { r: 0, g: 0, b: 3 },
        RGB8 { r: 0, g: 0, b: 4 },
        RGB8 { r: 1, g: 1, b: 6 },
        RGB8 { r: 0, g: 0, b: 4 },
        RGB8 { r: 0, g: 0, b: 2 },
        RGB8 { r: 0, g: 0, b: 2 },
        RGB8 { r: 0, g: 0, b: 2 },
    ];

    let _ = gpios_p0.p0_06.into_push_pull_output(Level::Low);

    let mut coloop = all_colors.iter().cloned().cycle();
    let mut data = [colors::BLACK; 10];

    let mut led = Pwm::new(board.PWM0, gpios_p0.p0_13.degrade());

    led.write(gamma(data.iter().cloned())).ok();
    let timer = GlobalRollingTimer::new();
    let mut cycler = 0;

    loop {
        let start = timer.get_ticks();


        cycler += 1;

        if cycler >= 10 {
            let _ = coloop.next();
            cycler = 0;
        }
        let mut cl = coloop.clone();
        for led in data.iter_mut() {
            *led = cl.next().unwrap();
        }
        led.write(data.iter().cloned()).ok();
        while timer.millis_since(start) < 10 { }
    }
}
