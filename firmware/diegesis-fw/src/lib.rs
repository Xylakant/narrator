#![no_std]

use core::sync::atomic::{AtomicUsize, Ordering};

use defmt_rtt as _; // global logger
use kolben::rlercobs;
use nrf52840_hal::{
    self as _, // memory layout
    target_constants::SRAM_UPPER,
};
use panic_probe as _;

pub mod groundhog_nrf52;
pub mod patterns;
mod saadc;
pub mod saadc_src;
pub mod spim_src;

#[macro_use]
pub mod profile_ct;

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}

static COUNT: AtomicUsize = AtomicUsize::new(0);
defmt::timestamp!("{=usize}", {
    // NOTE(no-CAS) `timestamps` runs with interrupts disabled
    let n = COUNT.load(Ordering::Relaxed);
    COUNT.store(n + 1, Ordering::Relaxed);
    n
});

/// Terminates the application and makes `probe-run` exit with exit-code = 0
pub fn exit() -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}

pub type PBox<T> = heapless::pool::singleton::Box<T>;

use embedded_dma::ReadBuffer;

pub struct NopSlice;

unsafe impl ReadBuffer for NopSlice {
    type Word = u8;

    unsafe fn read_buffer(&self) -> (*const Self::Word, usize) {
        // crimes
        ((SRAM_UPPER - 1) as *const _, 0)
    }
}

use bbqueue::{consts as bbconsts, GrantW};

#[derive(Debug)]
pub struct FillBuf {
    pub buf: GrantW<'static, bbconsts::U65536>,
    pub used: usize,
}

impl FillBuf {
    pub fn content_len(&self) -> usize {
        self.used
    }
}

impl rlercobs::Write for FillBuf {
    type Error = ();

    #[inline(always)]
    fn write(&mut self, byte: u8) -> Result<(), Self::Error> {
        let buf_byte = self.buf.get_mut(self.used).ok_or(())?;
        *buf_byte = byte;
        self.used += 1;
        Ok(())
    }
}
