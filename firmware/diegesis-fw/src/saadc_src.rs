use core::{
    mem,
    sync::atomic::{AtomicBool, Ordering},
};

use embedded_dma::StaticWriteBuffer;
use groundhog::RollingTimer;
use heapless::{
    mpmc::MpMcQueue,
    pool::singleton::{Box, Pool},
};
use nrf52840_hal::{
    pac::SAADC,
    ppi::ConfigurablePpi,
    saadc::{Oversample, Time},
};

use crate::{
    groundhog_nrf52::GlobalRollingTimer,
    saadc::{AsyncConversion, AsyncPendingConversion, Channels, Saadc, SaadcConfig},
};

enum State<B, C> {
    Idle(Saadc, C),
    OnePending(AsyncConversion<B, C>),
    TwoPending(AsyncPendingConversion<B, B, C>),
    Invalid,
}

impl<B, C> State<B, C> {
    fn take(&mut self) -> Self {
        mem::replace(self, State::Invalid)
    }

    fn saadc(&self) -> &Saadc {
        match self {
            State::Idle(saadc, ..) => saadc,
            State::OnePending(pend) => pend,
            State::TwoPending(pend) => pend,
            State::Invalid => unreachable!(),
        }
    }
}

pub struct SaadcSrc<C, POOL: Pool + 'static, PPI, const N: usize> {
    state: State<Box<POOL>, C>,
    pool_q: &'static MpMcQueue<Box<POOL>, N>,
    last_start: u32,
    ppi: PPI,
}

impl<B, C, POOL, PPI, const N: usize> SaadcSrc<C, POOL, PPI, N>
where
    Box<POOL>: StaticWriteBuffer<Word = i16>,
    POOL: Pool<Data = B> + 'static,
    PPI: ConfigurablePpi,
    B: AsRef<[u8]>,
    C: Channels,
{
    pub fn new(
        peripheral: SAADC,
        channels: C,
        mut ppi: PPI,
        queue: &'static MpMcQueue<Box<POOL>, N>,
    ) -> Self {
        peripheral
            .intenset
            .modify(|_r, w| w.stopped().set_bit().end().set_bit().started().set_bit());

        let saadc = Saadc::new(
            peripheral,
            SaadcConfig {
                // Configure for fastest conversion time.
                oversample: Oversample::BYPASS,
                time: Time::_3US,
                ..Default::default()
            },
        );

        ppi.set_event_endpoint(saadc.event_end());
        ppi.set_task_endpoint(saadc.task_start());

        Self {
            state: State::Idle(saadc, channels),
            pool_q: queue,
            last_start: 0,
            ppi,
        }
    }

    pub fn poll(&mut self, fuse: &AtomicBool) {
        // TODO: removeme
        self.state.saadc().event_stopped().reset();

        let fuse_blown = fuse.load(Ordering::SeqCst);
        let old_state = self.state.take();
        let new_state = match old_state {
            State::Idle(p, c) if fuse_blown => {
                // Nothing to do here
                State::Idle(p, c)
            }
            State::Idle(p, c) => {
                if let Some(pbox) = POOL::alloc() {
                    let pbox = pbox.freeze();
                    let pend = p.start_async_conversion(c, pbox);
                    self.last_start = GlobalRollingTimer.get_ticks();
                    State::OnePending(pend)
                } else {
                    // No data available! Blow the fuse.
                    fuse.store(true, Ordering::SeqCst);
                    State::Idle(p, c)
                }
            }
            State::OnePending(ts) if fuse_blown => {
                // Manually clear the started event, as we won't be
                // clearing/processing it for the second queued write
                ts.event_started().reset();

                // We shouldn't enqueue a new transfer, the fuse is blown.
                //
                // Attempt to finish any current transfer though, since we already
                // have the alloc page
                if ts.is_done() {
                    let (p, rxb, c) = ts.wait();

                    /*let elapsed = GlobalRollingTimer.ticks_since(self.last_start);
                    let delta = (EXPECTED_TICKS as i32) - (elapsed as i32);
                    if (delta < -ACCEPTABLE_DELTA) || (delta > ACCEPTABLE_DELTA) {
                        defmt::warn!("saadc deviation: {}", delta);
                    }*/

                    if let Ok(()) = self.pool_q.enqueue(rxb) {
                        // defmt::info!("Sent box!");
                    } else {
                        defmt::warn!("Failed to send box!");
                    }
                    State::Idle(p, c)
                } else {
                    // Not ready yet
                    State::OnePending(ts)
                }
            }
            State::OnePending(ts) => {
                if let Some(pbox) = POOL::alloc() {
                    let pbox = pbox.freeze();

                    // Enable end-to-start shortcut (using PPI)
                    self.ppi.enable();

                    let pend = ts.enqueue_next_transfer(pbox).map_err(drop).unwrap();

                    State::TwoPending(pend)
                } else {
                    // No data available! Blow the fuse.
                    fuse.store(true, Ordering::SeqCst);
                    State::OnePending(ts)
                }
            }
            State::TwoPending(pend) => {
                assert!(pend.is_done());
                let (conv, buffer) = pend.wait();

                /*let elapsed = GlobalRollingTimer.ticks_since(self.last_start);
                let delta = (EXPECTED_TICKS as i32) - (elapsed as i32);
                if (delta < -ACCEPTABLE_DELTA) || (delta > ACCEPTABLE_DELTA) {
                    defmt::warn!("saadc deviation: {}", delta);
                }*/

                self.last_start = GlobalRollingTimer.get_ticks();

                // Disable end-to-start shortcut (using PPI)
                self.ppi.disable();

                if let Ok(()) = self.pool_q.enqueue(buffer) {
                    // defmt::info!("Sent box!");
                } else {
                    defmt::warn!("Failed to send box!");
                }

                State::OnePending(conv)
            }
            State::Invalid => {
                defmt::panic!("SAADC: encountered invalid state");
            }
        };

        self.state = new_state;
    }
}
