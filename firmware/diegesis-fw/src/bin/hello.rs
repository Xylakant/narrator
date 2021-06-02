#![no_main]
#![no_std]

use diegesis_fw as _; // global logger + panicking-behavior + memory layout

#[cortex_m_rt::entry]
fn main() -> ! {
    defmt::info!("Hello, world!");

    diegesis_fw::exit()
}
