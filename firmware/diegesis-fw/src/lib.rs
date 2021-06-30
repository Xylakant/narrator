#![no_std]

use core::{
    fmt::Debug,
    ops::DerefMut,
};

use nrf52840_hal::{
    self as _, // memory layout
    target_constants::SRAM_UPPER,
};

use bbqueue::{consts as bbconsts, GrantW};
use defmt_rtt as _; // global logger
use diegesis_icd::{DataReport, Managed, ReportKind};
use embedded_dma::ReadBuffer;
use heapless::pool::singleton::Pool;
use kolben::rlercobs;
use panic_probe as _;

#[cfg(feature = "board-dk")]
pub use pinmap::Nrf52Dk as Board;

#[cfg(feature = "board-playground")]
pub use pinmap::AdafruitPlaygroundBluefruit as Board;

pub mod groundhog_nrf52;
pub mod patterns;
mod saadc;
pub mod saadc_src;
pub mod spim_src;
pub mod pinmap;

#[macro_use]
pub mod profile_ct;

use groundhog_nrf52::GlobalRollingTimer;
use groundhog::RollingTimer;

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}

defmt::timestamp!("{=u32}", {
    // NOTE(no-CAS) `timestamps` runs with interrupts disabled
    GlobalRollingTimer::new().get_ticks()
});

/// Terminates the application and makes `probe-run` exit with exit-code = 0
pub fn exit() -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}

pub type PBox<T> = heapless::pool::singleton::Box<T>;

pub struct NopSlice;

unsafe impl ReadBuffer for NopSlice {
    type Word = u8;

    unsafe fn read_buffer(&self) -> (*const Self::Word, usize) {
        // crimes
        ((SRAM_UPPER - 1) as *const _, 0)
    }
}

#[derive(Debug)]
pub struct FillBuf {
    pub buf: GrantW<'static, bbconsts::U32768>,
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

#[derive(Debug)]
pub struct InternalReport<PoolA, PoolB>
where
    PoolA: Pool,
    PoolB: Pool,
    PBox<PoolA>: Debug,
    PBox<PoolB>: Debug,
{
    timestamp: u32,
    kind: InternalReportKind<PoolA, PoolB>,
}

#[derive(Debug)]
pub enum InternalReportKind<PoolA, PoolB>
where
    PoolA: Pool,
    PoolB: Pool,
    PBox<PoolA>: Debug,
    PBox<PoolB>: Debug,
{
    DigitalReport {
        channel: u8,
        payload: PBox<PoolA>,
    },
    AnalogReport {
        channel_bitflag: u8,
        payload: PBox<PoolB>,
    },
}

impl<DigitalPool, AnalogPool> InternalReport<DigitalPool, AnalogPool>
where
    DigitalPool: Pool,
    AnalogPool: Pool,
    PBox<DigitalPool>: Debug + DerefMut<Target = [u8; 4096]>,
    PBox<AnalogPool>: Debug + DerefMut<Target = [i16; 2048]>,
{
    pub fn as_data_report(&mut self) -> DataReport {
        match self.kind {
            InternalReportKind::DigitalReport {
                channel,
                ref mut payload,
            } => DataReport {
                timestamp: self.timestamp,
                kind: ReportKind::DigitalPin { channel },
                payload: Managed::Borrowed(payload.deref_mut()),
            },
            InternalReportKind::AnalogReport {
                channel_bitflag,
                ref mut payload,
            } => {
                // SAFETY: We have an array of i16s we are re-interpreting to bytes
                // This is acceptable as all data is initialized (by DMA), and bytes
                // have a weaker alignment than i16s. For both u8 and i16s, all possible
                // values are valid
                let casted_slice: &mut [u8; 4096] = unsafe {
                    let i16_slice: &mut [i16; 2048] = payload.deref_mut();
                    core::mem::transmute(i16_slice)
                };

                DataReport {
                    timestamp: self.timestamp,
                    kind: ReportKind::AnalogPin { channel_bitflag },
                    payload: Managed::Borrowed(casted_slice),
                }
            }
        }
    }
}
