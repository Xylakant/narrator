use nrf52840_hal::gpio::{
    p0::{Parts as P0Parts, P0_02, P0_03, P0_29},
    p1::Parts as P1Parts,
    Pin, Disconnected,
};

pub enum Leds {
    DiscreteLeds {
        led0: Pin<Disconnected>,
        led1: Pin<Disconnected>,
        led2: Pin<Disconnected>,
        led3: Pin<Disconnected>,
    },
    SmartLeds {
        pwm_pin: Pin<Disconnected>,
        enable_pin: Pin<Disconnected>,
    }
}

pub struct MappedPins {
    // SPIM data and clock (unused) pins
    pub spim_p0_data: Pin<Disconnected>,
    pub spim_p0_clk: Pin<Disconnected>,
    pub spim_p1_data: Pin<Disconnected>,
    pub spim_p1_clk: Pin<Disconnected>,
    pub spim_p2_data: Pin<Disconnected>,
    pub spim_p2_clk: Pin<Disconnected>,
    pub spim_p3_data: Pin<Disconnected>,
    pub spim_p3_clk: Pin<Disconnected>,

    // User buttons
    pub start_pause_btn: Pin<Disconnected>,
    pub lap_reset_btn: Pin<Disconnected>,

    // Display leds
    pub leds: Leds,

    // ADCs (todo: variable amount? variable pins? generics?)
    pub adcs: (P0_02<Disconnected>, P0_03<Disconnected>, P0_29<Disconnected>),
}

pub trait PinMap {
    fn map_pins(p0: P0Parts, p1: P1Parts) -> MappedPins;
}

pub struct Nrf52Dk;
pub struct AdafruitPlaygroundBluefruit;

impl PinMap for Nrf52Dk {
    fn map_pins(p0: P0Parts, p1: P1Parts) -> MappedPins {
        MappedPins {
            // SPIM data and clock (unused) pins
            spim_p0_data: p0.p0_11.degrade(),
            spim_p0_clk: p1.p1_01.degrade(),
            spim_p1_data: p1.p1_03.degrade(),
            spim_p1_clk: p1.p1_02.degrade(),
            spim_p2_data: p1.p1_05.degrade(),
            spim_p2_clk: p1.p1_04.degrade(),
            spim_p3_data: p1.p1_07.degrade(),
            spim_p3_clk: p1.p1_06.degrade(),

            // User buttons
            start_pause_btn: p0.p0_25.degrade(),
            lap_reset_btn: p0.p0_24.degrade(),

            // Display leds
            leds: Leds::DiscreteLeds {
                led0: p0.p0_13.degrade(),
                led1: p0.p0_14.degrade(),
                led2: p0.p0_15.degrade(),
                led3: p0.p0_16.degrade(),
            },

            adcs: (p0.p0_02, p0.p0_03, p0.p0_29),
        }
    }
}

impl PinMap for AdafruitPlaygroundBluefruit {
    fn map_pins(p0: P0Parts, p1: P1Parts) -> MappedPins {
        MappedPins {
            // SPIM data and clock (unused) pins
            spim_p0_data: p0.p0_04.degrade(), // A4/SCL
            spim_p0_clk: p0.p0_07.degrade(),  // Not Connected
            spim_p1_data: p0.p0_05.degrade(), // A5/SDA
            spim_p1_clk: p0.p0_08.degrade(),  // Not Connected
            spim_p2_data: p0.p0_30.degrade(), // D0/A6/RX
            spim_p2_clk: p0.p0_11.degrade(),  // Not Connected
            spim_p3_data: p0.p0_14.degrade(), // D1/TX
            spim_p3_clk: p0.p0_12.degrade(),  // Not Connected

            // User buttons
            start_pause_btn: p1.p1_15.degrade(), // Right Button
            lap_reset_btn: p1.p1_02.degrade(),   // Left Button

            // Display leds
            // TODO: the playground ALSO has one discrete LED, do we
            // want to use this for any of the UI stuff?
            leds: Leds::SmartLeds {
                pwm_pin: p0.p0_13.degrade(),
                enable_pin: p0.p0_06.degrade(),
            },

            adcs: (p0.p0_02, p0.p0_03, p0.p0_29),
        }
    }
}
