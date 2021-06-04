use embedded_dma::WriteBuffer;
use nrf52840_hal::Spim;
use nrf52840_hal::spim::Instance;
use nrf52840_hal::spim::PendingSplit;
use nrf52840_hal::spim::TransferSplit;
use heapless::mpmc::MpMcQueue;
use crate::NopSlice;
use heapless::pool::Init;

type PBox<POOL> = heapless::pool::singleton::Box<POOL, Init>;

pub enum SpimPeriph<S, POOL>
where
    S: Instance + Send,
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
    S: Instance + Send,
    POOL: heapless::pool::singleton::Pool + 'static,
    PBox<POOL>: WriteBuffer<Word = u8>
{
    fn take(&mut self) -> Self {
        let mut new = SpimPeriph::Unstable;
        core::mem::swap(self, &mut new);
        new
    }
}

pub struct SpimSrc<T, POOL, const N: usize>
where
    T: Instance + Send,
    POOL: heapless::pool::singleton::Pool + 'static,
    PBox<POOL>: WriteBuffer<Word = u8>
{
    periph: SpimPeriph<T, POOL>,
    pool_q: &'static MpMcQueue<PBox<POOL>, N>,
}

impl<T, POOL, const N: usize> SpimSrc<T, POOL, N>
where
    T: Instance + Send,
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
    pub fn poll(&mut self) {
        let new_state = match self.periph.take() {
            SpimPeriph::Idle(p) => {
                let pbox = POOL::alloc().unwrap().freeze();
                let txfr = p.dma_transfer_split(NopSlice, pbox).map_err(drop).unwrap();

                SpimPeriph::OnePending(txfr)
            }
            SpimPeriph::OnePending(mut ts) => {
                let pbox = POOL::alloc().unwrap().freeze();
                let p_txfr = ts.enqueue_next_transfer(NopSlice, pbox).map_err(drop).unwrap();

                SpimPeriph::TwoPending {
                    transfer: ts,
                    pending: p_txfr,
                }
            }
            SpimPeriph::TwoPending { mut transfer, pending } => {
                assert!(transfer.is_done());
                let (_txb, rxb, one) = transfer.exchange_transfer_wait(pending);

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
