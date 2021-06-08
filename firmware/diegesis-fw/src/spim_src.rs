use core::sync::atomic::AtomicBool;
use core::sync::atomic::Ordering;

use embedded_dma::WriteBuffer;
use nrf52840_hal::Spim;
use nrf52840_hal::pac::{SPIM0, SPIM1, SPIM2, SPIM3};
use nrf52840_hal::pac::spim0;
use nrf52840_hal::spim::Instance;
use nrf52840_hal::spim::PendingSplit;
use nrf52840_hal::spim::TransferSplit;
use heapless::mpmc::MpMcQueue;
use crate::NopSlice;
use heapless::pool::Init;

type PBox<POOL> = heapless::pool::singleton::Box<POOL, Init>;

pub enum SpimPeriph<S, POOL>
where
    S: Instance + Shame + Send,
    POOL: heapless::pool::singleton::Pool + 'static,
    PBox<POOL>: WriteBuffer<Word = u8>
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
    PBox<POOL>: WriteBuffer<Word = u8>
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

pub struct SpimSrc<T, POOL, const N: usize>
where
    T: Instance + Shame + Send,
    POOL: heapless::pool::singleton::Pool + 'static,
    PBox<POOL>: WriteBuffer<Word = u8>
{
    periph: SpimPeriph<T, POOL>,
    pool_q: &'static MpMcQueue<PBox<POOL>, N>,
}

impl<T, POOL, const N: usize> SpimSrc<T, POOL, N>
where
    T: Instance + Shame + Send,
    POOL: heapless::pool::singleton::Pool + 'static,
    PBox<POOL>: WriteBuffer<Word = u8>,
    <POOL as heapless::pool::singleton::Pool>::Data: AsRef<[u8]>,
{
    pub fn new(
        periph: SpimPeriph<T, POOL>,
        pool_q: &'static MpMcQueue<PBox<POOL>, N>
    ) -> Self {
        Self {
            periph,
            pool_q,
        }
    }

    pub fn poll(&mut self, fuse: &AtomicBool) {
        // TODO: removeme
        unsafe {
            (&*T::shame_ptr()).events_stopped.write(|w| {
                w.events_stopped().clear_bit()
            });
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
                    (&*T::shame_ptr()).events_started.write(|w| {
                        w.events_started().clear_bit()
                    });
                }

                // We shouldn't enqueue a new transfer, the fuse is blown.
                //
                // Attempt to finish any current transfer though, since we already
                // have the alloc page
                if ts.is_done() {
                    let (_txb, rxb, p) = ts.wait();

                    if let Ok(()) = self.pool_q.enqueue(rxb) {
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
                        (&*T::shame_ptr()).shorts.modify(|_r, w| { w.end_start().set_bit() });
                    }

                    let p_txfr = ts.enqueue_next_transfer(NopSlice, pbox).map_err(drop).unwrap();

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
            SpimPeriph::TwoPending { mut transfer, pending } => {
                assert!(transfer.is_done());
                let (_txb, rxb, one) = transfer.exchange_transfer_wait(pending);

                // Disable end-to-start shortcut
                unsafe {
                    (&*T::shame_ptr()).shorts.modify(|_r, w| { w.end_start().clear_bit() });
                }

                if let Ok(()) = self.pool_q.enqueue(rxb) {
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
