use core::sync::atomic::AtomicBool;
use core::sync::atomic::Ordering;

use crate::groundhog_nrf52::GlobalRollingTimer;
use embedded_dma::WriteBuffer;
use groundhog::RollingTimer;
use heapless::mpmc::MpMcQueue;
use heapless::pool::Init;
use nrf52840_hal::gpio::Pin;
use nrf52840_hal::pac::SAADC;
use nrf52840_hal::saadc::Transfer;
use nrf52840_hal::saadc::{Saadc, SaadcConfig};

type PBox<POOL> = heapless::pool::singleton::Box<POOL, Init>;
pub enum SaadcPeriph<POOL>
where
    POOL: heapless::pool::singleton::Pool + 'static,
    PBox<POOL>: WriteBuffer<Word = u8>,
{
    Idle(Saadc),
    OnePending(Transfer<PBox<POOL>>),

    // TODO
    /* TwoPending {
        transfer: Transfer<PBox<POOL>>,
        pending: PendingTransfer<PBox<POOL>>,
    }, */
    Unstable,
}

//

impl<POOL> SaadcPeriph<POOL>
where
    POOL: heapless::pool::singleton::Pool + 'static,
    PBox<POOL>: WriteBuffer<Word = u8>,
{
    fn take(&mut self) -> Self {
        let mut new = SaadcPeriph::Unstable;
        core::mem::swap(self, &mut new);
        new
    }
}

// (8_000 ticks/s) / (2_000_000 bps / 8 bit-per-byte / 4096 byte-per-box)
const EXPECTED_TICKS: u32 = 131;

// Allow for +/- 0.2%
// (totally unscientific number, rounded up to 1)
const ACCEPTABLE_DELTA: i32 = 1;

pub struct SaadcSrc<POOL, const N: usize>
where
    POOL: heapless::pool::singleton::Pool + 'static,
    PBox<POOL>: WriteBuffer<Word = u8>,
{
    periph: SaadcPeriph<POOL>,
    pool_q: &'static MpMcQueue<PBox<POOL>, N>,
    last_start: u32,
    timer: GlobalRollingTimer,
}

impl<POOL, const N: usize> SaadcSrc<POOL, N>
where
    POOL: heapless::pool::singleton::Pool + 'static,
    PBox<POOL>: WriteBuffer<Word = u8>,
    <POOL as heapless::pool::singleton::Pool>::Data: AsRef<[u8]>,
{
    pub fn new(
        periph: SaadcPeriph<POOL>,
        pool_q: &'static MpMcQueue<PBox<POOL>, N>,
        timer: GlobalRollingTimer,
    ) -> Self {
        Self {
            periph,
            pool_q,
            last_start: 0,
            timer,
        }
    }

    pub fn from_parts<ADC>(
        periph: SAADC,
        pin: Pin<ADC>,
        pool_q: &'static MpMcQueue<PBox<POOL>, N>,
        timer: GlobalRollingTimer,
    ) -> Self {
        // TODO: This probably should be dynamic
        periph
            .intenset
            .modify(|_r, w| w.stopped().set_bit().end().set_bit().started().set_bit());

        let saadc = Saadc::new(
            periph,
            Some(pin.into_floating_input()),
            SaadcConfig::default(),
        );
        let saadc_p = SaadcPeriph::Idle(saadc);
        SaadcSrc::new(saadc_p, pool_q, timer)
    }

    pub fn poll(&mut self, fuse: &AtomicBool) {
        // TODO: removeme
        unsafe {
            (&*SAADC::ptr())
                .events_stopped
                .write(|w| w.events_stopped().clear_bit());
        }

        let fuse_blown = fuse.load(Ordering::SeqCst);
        let new_state = match self.periph.take() {
            SaadcPeriph::Idle(p) if fuse_blown => {
                // Nothing to do here
                SaadcPeriph::Idle(p)
            }
            SaadcPeriph::Idle(p) => {
                if let Some(pbox) = POOL::alloc() {
                    let pbox = pbox.freeze();
                    let txfr = p.dma_transfer(pbox).map_err(drop).unwrap();
                    self.last_start = self.timer.get_ticks();
                    SaadcPeriph::OnePending(txfr)
                } else {
                    // No data available! Blow the fuse.
                    fuse.store(true, Ordering::SeqCst);
                    SaadcPeriph::Idle(p)
                }
            }
            SaadcPeriph::OnePending(mut ts) => {
                if fuse_blown {
                    // Manually clear the started event
                    unsafe {
                        (&*SAADC::ptr())
                            .events_started
                            .write(|w| w.events_started().clear_bit());
                    }
                }

                if ts.is_done() {
                    let (rxb, p) = ts.wait();

                    let elapsed = self.timer.ticks_since(self.last_start);
                    let delta = (EXPECTED_TICKS as i32) - (elapsed as i32);
                    if (delta < -ACCEPTABLE_DELTA) || (delta > ACCEPTABLE_DELTA) {
                        defmt::warn!("saadc deviation: {}", delta);
                    }

                    if let Ok(()) = self.pool_q.enqueue(rxb) {
                        // defmt::info!("Sent box!");
                    } else {
                        defmt::warn!("Failed to send box!");
                    }
                    SaadcPeriph::Idle(p)
                } else {
                    // Not ready yet
                    SaadcPeriph::OnePending(ts)
                }
            }

            SaadcPeriph::Unstable => {
                defmt::panic!("SAADC Error!");
            }
        };

        self.periph = new_state;
    }
}
