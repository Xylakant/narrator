#![no_main]
#![no_std]

use choreographer::engine::{Action, Behavior, Sequence};
use diegesis_fw; // global logger + panicking-behavior + memory layout
use diegesis_fw::groundhog_nrf52::GlobalRollingTimer;
use groundhog::RollingTimer;
use nrf52840_hal::{
    clocks::Clocks,
    gpio::{p0::Parts as P0Parts, p1::Parts as P1Parts, Level},
    pac,
};
use nrf_smartled::pwm::Pwm;
use smart_leds::{colors, gamma, brightness, SmartLedsWrite};
use choreographer::script;
use choreographer::engine::Behavior::LoopForever;
use smart_leds::colors::*;

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
    let _clocks = clocks.enable_ext_hfosc();

    GlobalRollingTimer::init(board.TIMER0);
    let gpios_p0 = P0Parts::new(board.P0);
    let gpios_p1 = P1Parts::new(board.P1);

    let _start_stop_btn = gpios_p1.p1_15.into_pulldown_input().degrade();
    let _start_stop_led = gpios_p1.p1_14.into_push_pull_output(Level::Low).degrade();

    // Enable smartled switch
    let _ = gpios_p0.p0_06.into_push_pull_output(Level::Low);

    let mut data = [colors::BLACK; 10];
    let mut led = Pwm::new(board.PWM0, gpios_p0.p0_13.degrade());

    led.write(gamma(data.iter().cloned())).ok();
    let timer = GlobalRollingTimer::new();

    let mut script: [Sequence<GlobalRollingTimer, 8>; 10] = [
        Sequence::empty(),
        Sequence::empty(),
        Sequence::empty(),
        Sequence::empty(),
        Sequence::empty(),
        Sequence::empty(),
        Sequence::empty(),
        Sequence::empty(),
        Sequence::empty(),
        Sequence::empty(),
    ];

    script[0].set(
        script!{
            | action | color | duration_ms | period_ms_f | repeat |
            |  solid | BLACK |           0 |         0.0 |   once |
            |    sin | WHITE |        1000 |      2000.0 |   once |
            |  solid | BLACK |        1000 |         0.0 |   once |
        },
        LoopForever,
    );

    script[1].set(
        script!{
            | action | color | duration_ms | period_ms_f | repeat |
            |  solid | BLACK |         100 |         0.0 |   once |
            |    sin | WHITE |        1000 |      2000.0 |   once |
            |  solid | BLACK |         900 |         0.0 |   once |
        },
        Behavior::LoopForever,
    );
    script[2].set(
        script!{
            | action | color | duration_ms | period_ms_f | repeat |
            |  solid | BLACK |         200 |         0.0 |   once |
            |    sin |   RED |        1000 |      2000.0 |   once |
            |  solid | BLACK |         800 |         0.0 |   once |
        },
        Behavior::LoopForever,
    );
    script[3].set(
        script!{
            | action |  color | duration_ms | period_ms_f | repeat |
            |  solid |  BLACK |         300 |         0.0 |   once |
            |    sin | ORANGE |        1000 |      2000.0 |   once |
            |  solid |  BLACK |         700 |         0.0 |   once |
        },
        Behavior::LoopForever,
    );
    script[4].set(
        script!{
            | action |  color | duration_ms | period_ms_f | repeat |
            |  solid |  BLACK |         400 |         0.0 |   once |
            |    sin | YELLOW |        1000 |      2000.0 |   once |
            |  solid |  BLACK |         600 |         0.0 |   once |
        },
        Behavior::LoopForever,
    );
    script[5].set(
        script!{
            | action | color | duration_ms | period_ms_f | repeat |
            |  solid | BLACK |         500 |         0.0 |   once |
            |    sin | GREEN |        1000 |      2000.0 |   once |
            |  solid | BLACK |         500 |         0.0 |   once |
        },
        Behavior::LoopForever,
    );
    script[6].set(
        script!{
            | action | color | duration_ms | period_ms_f | repeat |
            |  solid | BLACK |         600 |         0.0 |   once |
            |    sin |  BLUE |        1000 |      2000.0 |   once |
            |  solid | BLACK |         400 |         0.0 |   once |
        },
        Behavior::LoopForever,
    );
    script[7].set(
        script!{
            | action |  color | duration_ms | period_ms_f | repeat |
            |  solid |  BLACK |         700 |         0.0 |   once |
            |    sin | VIOLET |        1000 |      2000.0 |   once |
            |  solid |  BLACK |         300 |         0.0 |   once |
        },
        Behavior::LoopForever,
    );
    script[8].set(
        script!{
            | action | color | duration_ms | period_ms_f | repeat |
            |  solid | BLACK |         800 |         0.0 |   once |
            |    sin | WHITE |        1000 |      2000.0 |   once |
            |  solid | BLACK |         200 |         0.0 |   once |
        },
        Behavior::LoopForever,
    );
    script[9].set(
        script!{
            | action | color | duration_ms | period_ms_f | repeat |
            |  solid | BLACK |         900 |         0.0 |   once |
            |    sin | WHITE |        1000 |      2000.0 |   once |
            |  solid | BLACK |         100 |         0.0 |   once |
        },
        Behavior::LoopForever,
    );

    // Update the screen
    let mut any = false;
    let mut last_rendered = timer.get_ticks();
    let mut last_polled = timer.get_ticks();

    loop {
        if timer.micros_since(last_polled) < 500 {
            continue;
        }
        last_polled = timer.get_ticks();

        for (led, scr) in data.iter_mut().zip(script.iter_mut()) {
            if let Some(pix) = scr.poll() {
                any |= *led != pix;
                *led = pix;
            }
        }

        if timer.millis_since(last_rendered) >= 15 {
            if any {
                led.write(brightness(gamma(data.iter().cloned()), 32)).unwrap();
                any = false;
            }
            last_rendered = timer.get_ticks();
        }
    }
}
