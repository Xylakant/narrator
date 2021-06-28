use core::sync::atomic::{AtomicBool, Ordering};

use crate::{groundhog_nrf52::GlobalRollingTimer, InternalReport, NopSlice};
use nrf52840_hal::{
    gpio::{Level, Pin},
    pac::{spim0, SPIM0, SPIM1, SPIM2, SPIM3},
    spim::{Frequency, Instance, PendingSplit, Pins, TransferSplit},
    Spim,
};

use embedded_dma::WriteBuffer;
use embedded_hal::spi::MODE_0;
use groundhog::RollingTimer;
use heapless::{mpmc::MpMcQueue, pool::Init};

type PBox<POOL> = heapless::pool::singleton::Box<POOL, Init>;

pub enum SpimPeriph<S, POOL>
where
    S: Instance + Shame + Send,
    POOL: heapless::pool::singleton::Pool + 'static,
    PBox<POOL>: WriteBuffer<Word = u8>,
{
    Idle(Spim<S>),
    OnePending(TransferSplit<S, NopSlice, PBox<POOL>>),
    TwoPending {
        transfer: TransferSplit<S, NopSlice, PBox<POOL>>,
        pending: PendingSplit<S, NopSlice, PBox<POOL>>,
    },
    Unstable,
}

//

impl<S, POOL> SpimPeriph<S, POOL>
where
    S: Instance + Shame + Send,
    POOL: heapless::pool::singleton::Pool + 'static,
    PBox<POOL>: WriteBuffer<Word = u8>,
{
    fn take(&mut self) -> Self {
        let mut new = SpimPeriph::Unstable;
        core::mem::swap(self, &mut new);
        new
    }
}

pub trait Shame {
    fn shame_ptr() -> *const spim0::RegisterBlock;
}

impl Shame for SPIM0 {
    fn shame_ptr() -> *const spim0::RegisterBlock {
        SPIM0::ptr()
    }
}

impl Shame for SPIM1 {
    fn shame_ptr() -> *const spim0::RegisterBlock {
        SPIM1::ptr()
    }
}

impl Shame for SPIM2 {
    fn shame_ptr() -> *const spim0::RegisterBlock {
        SPIM2::ptr()
    }
}

impl Shame for SPIM3 {
    fn shame_ptr() -> *const spim0::RegisterBlock {
        SPIM3::ptr()
    }
}

// (4_000_000 ticks/s) / (2_000_000 bps / 8 bit-per-byte / 4096 byte-per-box)
const EXPECTED_TICKS: u32 = 65536;

// Allow for +/- 0.2%
// (totally unscientific number)
const ACCEPTABLE_DELTA: i32 = (EXPECTED_TICKS as i32) / 500;

use core::fmt::Debug;

pub struct SpimSrc<T, POOL, OtherPool, const N: usize>
where
    T: Instance + Shame + Send,
    POOL: heapless::pool::singleton::Pool + 'static,
    OtherPool: heapless::pool::singleton::Pool + 'static,
    PBox<POOL>: WriteBuffer<Word = u8>,
    PBox<POOL>: Debug,
    PBox<OtherPool>: Debug,
{
    periph: SpimPeriph<T, POOL>,
    pool_q: &'static MpMcQueue<InternalReport<POOL, OtherPool>, N>,
    last_start: u32,
    timer: GlobalRollingTimer,
    channel: u8,
}

impl<T, POOL, OtherPool, const N: usize> SpimSrc<T, POOL, OtherPool, N>
where
    T: Instance + Shame + Send,
    POOL: heapless::pool::singleton::Pool + 'static,
    OtherPool: heapless::pool::singleton::Pool + 'static,
    PBox<POOL>: WriteBuffer<Word = u8>,
    <POOL as heapless::pool::singleton::Pool>::Data: AsRef<[u8]>,
    PBox<POOL>: Debug,
    PBox<OtherPool>: Debug,
{
    pub fn new(
        periph: SpimPeriph<T, POOL>,
        pool_q: &'static MpMcQueue<InternalReport<POOL, OtherPool>, N>,
        timer: GlobalRollingTimer,
        channel: u8,
    ) -> Self {
        Self {
            periph,
            pool_q,
            last_start: 0,
            timer,
            channel,
        }
    }

    pub fn from_parts<DATA, DISC>(
        periph: T,
        data_pin: Pin<DATA>,
        disc_pin: Pin<DISC>,
        pool_q: &'static MpMcQueue<InternalReport<POOL, OtherPool>, N>,
        freq: Frequency,
        timer: GlobalRollingTimer,
        channel: u8,
    ) -> Self {
        let pins = Pins {
            sck: disc_pin.into_push_pull_output(Level::Low),
            miso: Some(data_pin.into_floating_input()),
            mosi: None,
        };

        // TODO: This probably should be dynamic
        periph
            .intenset
            .modify(|_r, w| w.stopped().set_bit().end().set_bit().started().set_bit());

        // todo: calculate expected ticks automatically
        assert_eq!(freq, Frequency::M2, "TODO: UPDATE EXPECTED TICKS");

        let spim = Spim::new(periph, pins, freq, MODE_0, 0x00);
        let spim_p = SpimPeriph::Idle(spim);
        SpimSrc::new(spim_p, pool_q, timer, channel)
    }

    pub fn poll(&mut self, fuse: &AtomicBool) {
        // TODO: removeme
        unsafe {
            (&*T::shame_ptr())
                .events_stopped
                .write(|w| w.events_stopped().clear_bit());
        }

        let fuse_blown = fuse.load(Ordering::SeqCst);
        let new_state = match self.periph.take() {
            SpimPeriph::Idle(p) if fuse_blown => {
                // Nothing to do here
                SpimPeriph::Idle(p)
            }
            SpimPeriph::Idle(p) => {
                if let Some(pbox) = POOL::alloc() {
                    let pbox = pbox.freeze();
                    let txfr = p.dma_transfer_split(NopSlice, pbox).map_err(drop).unwrap();
                    self.last_start = self.timer.get_ticks();
                    SpimPeriph::OnePending(txfr)
                } else {
                    // No data available! Blow the fuse.
                    fuse.store(true, Ordering::SeqCst);
                    SpimPeriph::Idle(p)
                }
            }
            SpimPeriph::OnePending(mut ts) if fuse_blown => {
                // Manually clear the started event, as we won't be
                // clearing/processing it for the second queued write
                unsafe {
                    (&*T::shame_ptr())
                        .events_started
                        .write(|w| w.events_started().clear_bit());
                }

                // We shouldn't enqueue a new transfer, the fuse is blown.
                //
                // Attempt to finish any current transfer though, since we already
                // have the alloc page
                if ts.is_done() {
                    let (_txb, rxb, p) = ts.wait();

                    let elapsed = self.timer.ticks_since(self.last_start);
                    let delta = (EXPECTED_TICKS as i32) - (elapsed as i32);
                    if (delta < -ACCEPTABLE_DELTA) || (delta > ACCEPTABLE_DELTA) {
                        defmt::warn!("spi deviation: {}", delta);
                    }

                    let rpt = InternalReport {
                        timestamp: 0x01020304,
                        kind: crate::InternalReportKind::DigitalReport {
                            channel: self.channel,
                            payload: rxb,
                        },
                    };

                    if let Ok(()) = self.pool_q.enqueue(rpt) {
                        // defmt::info!("Sent box!");
                    } else {
                        defmt::warn!("Failed to send box!");
                    }
                    SpimPeriph::Idle(p)
                } else {
                    // Not ready yet
                    SpimPeriph::OnePending(ts)
                }
            }
            SpimPeriph::OnePending(mut ts) => {
                if let Some(pbox) = POOL::alloc() {
                    let pbox = pbox.freeze();

                    // Enable end-to-start shortcut
                    unsafe {
                        (&*T::shame_ptr())
                            .shorts
                            .modify(|_r, w| w.end_start().set_bit());
                    }

                    let p_txfr = ts
                        .enqueue_next_transfer(NopSlice, pbox)
                        .map_err(drop)
                        .unwrap();

                    SpimPeriph::TwoPending {
                        transfer: ts,
                        pending: p_txfr,
                    }
                } else {
                    // No data available! Blow the fuse.
                    fuse.store(true, Ordering::SeqCst);
                    SpimPeriph::OnePending(ts)
                }
            }
            SpimPeriph::TwoPending {
                mut transfer,
                pending,
            } => {
                assert!(transfer.is_done());
                let (_txb, rxb, one) = transfer.exchange_transfer_wait(pending);

                let elapsed = self.timer.ticks_since(self.last_start);
                let delta = (EXPECTED_TICKS as i32) - (elapsed as i32);
                if (delta < -ACCEPTABLE_DELTA) || (delta > ACCEPTABLE_DELTA) {
                    defmt::warn!("spi deviation: {}", delta);
                }

                self.last_start = self.timer.get_ticks();

                // Disable end-to-start shortcut
                unsafe {
                    (&*T::shame_ptr())
                        .shorts
                        .modify(|_r, w| w.end_start().clear_bit());
                }

                let rpt = InternalReport {
                    timestamp: 0x01020304,
                    kind: crate::InternalReportKind::DigitalReport {
                        channel: self.channel,
                        payload: rxb,
                    },
                };

                if let Ok(()) = self.pool_q.enqueue(rpt) {
                    // defmt::info!("Sent box!");
                } else {
                    defmt::warn!("Failed to send box!");
                }

                SpimPeriph::OnePending(one)
            }
            SpimPeriph::Unstable => {
                defmt::panic!("SPIM Error!");
            }
        };

        self.periph = new_state;
    }
}
