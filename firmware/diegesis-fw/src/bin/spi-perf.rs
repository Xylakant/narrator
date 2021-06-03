#![no_main]
#![no_std]

use embedded_hal::timer::CountDown;
use diegesis_fw as _;

use nrf52840_hal::pac::Interrupt;
use nrf52840_hal::target_constants::SRAM_UPPER;
use nrf52840_hal::{
    clocks::{Clocks, ExternalOscillator, Internal, LfOscStopped},
    gpio::{
        p0::Parts as P0Parts,
        p1::Parts as P1Parts,
        Input, Level, Output, Pin, PullUp, PushPull,
    },
    pac::{TIMER0, SPIM0},
    timer::{Instance as TimerInstance, Periodic, Timer},
    usbd::Usbd,
    spim::{Frequency, Pins as SpimPins, Spim, TransferSplit, PendingSplit, MODE_0, Instance},
};
use rtic::app;
use usb_device::{bus::UsbBusAllocator, class::UsbClass as _, device::UsbDeviceState, prelude::*};
use usbd_serial::{SerialPort, USB_CLASS_CDC};
use heapless::{
    pool,
    pool::Init,
    pool::singleton::{
        Box,
        Pool
    },
    spsc::{
        Queue,
        Producer,
        Consumer,
    },
};
use kolben::rlercobs;
use postcard::to_rlercobs_writer;
use serde::{Serialize, ser::Serializer};

type UsbDevice<'a> = usb_device::device::UsbDevice<'static, Usbd<'a>>;
type UsbSerial<'a> = SerialPort<'static, Usbd<'a>>;

use bbqueue::{
    consts as bbconsts,
    framed::{FrameConsumer, FrameProducer},
    BBBuffer, ConstBBBuffer,
};

#[derive(Debug, Serialize)]
pub struct PoolBox {
    #[serde(serialize_with = "slicer")]
    data: Box<A, Init>,
}

use core::ops::Deref;

// TODO: This could be done as an array, and not a slice, which
// would be more efficient on the wire (2-3 bytes/message)
fn slicer<S>(pb: &Box<A, Init>, s: S) -> Result<S::Ok, S::Error>
where
    S: Serializer,
{
    s.serialize_bytes(pb.deref())
}


pool!(
    A: [u8; 4096]
);

static ENCODED_QUEUE: BBBuffer<bbconsts::U65536> = BBBuffer(ConstBBBuffer::new());

#[app(device = nrf52840_hal::pac, peripherals = true)]
const APP: () = {
    struct Resources {
        usb_dev: UsbDevice<'static>,
        serial: UsbSerial<'static>,
        timer: Timer<TIMER0, Periodic>,

        box_prod: Producer<'static, Box<A, Init>, 64>,
        box_cons: Consumer<'static, Box<A, Init>, 64>,

        // rpt_prod: FrameProducer<'static, bbconsts::U2048>,
        // rpt_cons: FrameConsumer<'static, bbconsts::U2048>,

        spim_p0: SpimPeriph<SPIM0>,
    }

    #[init]
    fn init(ctx: init::Context) -> init::LateResources {
        static mut CLOCKS: Option<Clocks<ExternalOscillator, Internal, LfOscStopped>> = None;
        static mut USB_BUS: Option<UsbBusAllocator<Usbd<'static>>> = None;
        static mut QUEUE: Queue<Box<A, Init>, 64> = Queue::new();

        // NOTE: nrf52840 has a total of 256KiB of RAM.
        // We are allocating 192 KiB, or 48 data blocks, using
        // heapless pool.
        static mut DATA_POOL: [u8; 32 * 4096] = [0u8; 32 * 4096];
        A::grow(DATA_POOL);

        defmt::info!("Hello, world!");

        let board = ctx.device;

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
        let clocks = clocks.enable_ext_hfosc();

        let mut timer = Timer::periodic(board.TIMER0);
        let usbd = board.USBD;
        let gpios_p0 = P0Parts::new(board.P0);
        let gpios_p1 = P1Parts::new(board.P1);

        let spim_pins = SpimPins {
            sck: gpios_p1.p1_01.into_push_pull_output(Level::Low).degrade(),
            miso: Some(gpios_p0.p0_11.into_floating_input().degrade()),
            mosi: None,
        };

        // TODO: This probably should be dynamic
        board.SPIM0.shorts.modify(|_r, w| {
            w.end_start().set_bit()
        });
        board.SPIM0.intenset.modify(|_r, w| {
            w.stopped().set_bit()
             .end().set_bit()
             .started().set_bit()
        });

        let spim = Spim::new(board.SPIM0, spim_pins, Frequency::M2, MODE_0, 0x00);
        let spim_p0 = SpimPeriph::Idle(spim);

        // timer.enable_interrupt();
        timer.start(Timer::<TIMER0, Periodic>::TICKS_PER_SECOND / 200);

        *CLOCKS = Some(clocks);
        let clocks = CLOCKS.as_ref().unwrap();
        *USB_BUS = Some(Usbd::new(usbd, &clocks));
        let usb_bus = USB_BUS.as_ref().unwrap();

        let serial = SerialPort::new(usb_bus);
        let usb_dev =
            UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c0, 0x27DD))
                .manufacturer("Ferrous Systems")
                .product("diegesis")
                .serial_number("diegesis-001")
                .device_class(USB_CLASS_CDC)
                .max_packet_size_0(64) // (makes control transfers 8x faster)
                .build();

        // let (rpt_prod, rpt_cons) = REPORT_QUEUE.try_split_framed().unwrap();
        let (box_prod, box_cons) = QUEUE.split();

        init::LateResources {
            usb_dev,
            serial,
            timer,
            // rpt_prod,
            // rpt_cons,
            box_prod,
            box_cons,
            spim_p0,
        }
    }

    #[task(binds = SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0, resources = [spim_p0, box_prod])]
    fn spim_p0(mut c: spim_p0::Context) {
        // First clear and store events
        let stopped;

        {
            // SAFETY: FIXME
            let spim0 = unsafe { &*SPIM0::ptr() };

            stopped = spim0.events_stopped.read().events_stopped().bit_is_set();

            if stopped {
                spim0.events_stopped.write(|w| w.events_stopped().clear_bit());
            }
        }

        // WE TOTALLY DON'T HAVE TWO REFERENCES LIVE AT THE
        // SAME TIME. SHHHHHH
        let port = c.resources.spim_p0;
        let new_state = match port.take() {
            SpimPeriph::Idle(p) => {
                assert!(!(stopped), "blerp");

                let pbox = A::alloc().unwrap().freeze();
                let txfr = p.dma_transfer_split(NopSlice, pbox).map_err(drop).unwrap();

                SpimPeriph::OnePending(txfr)
            }
            SpimPeriph::OnePending(mut ts) => {
                let pbox = A::alloc().unwrap().freeze();
                let p_txfr = ts.enqueue_next_transfer(NopSlice, pbox).map_err(drop).unwrap();

                SpimPeriph::TwoPending {
                    transfer: ts,
                    pending: p_txfr,
                }
            }
            SpimPeriph::TwoPending { mut transfer, pending } => {
                assert!(transfer.is_done());
                let (_txb, rxb, one) = transfer.exchange_transfer_wait(pending);

                if let Ok(()) = c.resources.box_prod.enqueue(rxb) {
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

        *port = new_state;
    }

    #[idle(resources = [usb_dev, serial, box_cons])]
    fn idle(mut c: idle::Context) -> ! {
        let mut state: UsbDeviceState = UsbDeviceState::Default;
        let mut ctr: u32 = 0;
        let mut skip_flag = false;
        let (mut enc_prod, mut enc_cons) = ENCODED_QUEUE.try_split().unwrap();

        loop {
            let new_state = c.resources.usb_dev.state();
            if new_state != state {
                defmt::info!("State change!");
                state = new_state;

                if new_state == UsbDeviceState::Configured {
                    defmt::info!("Configured!");

                    // TODO: Probably do this later, AFTER we have established usb comms
                    // or gotten a "start sniff" command
                    //
                    // This starts the unstoppable SPI logging
                    rtic::pend(Interrupt::SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0);
                }
            }

            let usb_d = &mut c.resources.usb_dev;
            let box_c = &mut c.resources.box_cons;
            let serial = &mut c.resources.serial;

            usb_poll(usb_d, serial);

            if state != UsbDeviceState::Configured {
                continue;
            }

            ctr = ctr.wrapping_add(1);

            if (ctr % 1_000_000) == 0 {
                defmt::info!("tick1m - usb");
            }

            // TODO: read?

            let mut did_write = false;

            // First: drain as many items from the box queue as possible
            // into the encoded queue. This likely will save space if the
            // data can be compressed at all, and will free up pboxes for
            // the interrupt code
            while let Some(new_box) = box_c.dequeue() {
                // TODO: with a little more complexity, we could use split grants
                // for a more efficient use of the encoding buffer. For now, we may
                // end up wasting 0 <= n < 5KiB at the end of the ring, which is a
                // whole pbox worth (7.8% of capacity ATM)
                let wgr = if let Ok(wgr) = enc_prod.grant_exact(1024 + 4096) {
                    wgr
                } else {
                    break;
                };

                let fbuf = to_rlercobs_writer(
                    &PoolBox { data: new_box },
                    FillBuf { buf: wgr, used: 0 }
                ).unwrap();
                let len = fbuf.content_len();
                fbuf.buf.commit(len);
            }

            // Second: Drain as many bytes into the serial port as possible,
            // in order to free up space to encode more.
            while let Ok(rgr) = enc_cons.read() {
                match serial.write(&rgr) {
                    Ok(n) => {
                        did_write = true;
                        rgr.release(n);
                    }
                    Err(UsbError::WouldBlock) => {
                        rgr.release(0);
                        break;
                    }
                    Err(e) => {
                        rgr.release(0);
                        panic!("BAD USB WRITE - {:?}", e);
                    }
                }
            }

            // If there was no data to write, just flush the connection to avoid
            // holding on to small amounts of data
            if !did_write {
                serial.flush().ok();
            }
        }
    }
};

fn usb_poll(usb_dev: &mut UsbDevice, serial: &mut UsbSerial) {
    if usb_dev.poll(&mut [serial]) {
        serial.poll();
    }
}

type PBox<T> = heapless::pool::singleton::Box<T>;

pub enum SpimPeriph<S>
where
    S: Instance + Send,
{
    Idle(Spim<S>),
    OnePending(TransferSplit<S, NopSlice, PBox<A>>),
    TwoPending {
        transfer: TransferSplit<S, NopSlice, PBox<A>>,
        pending: PendingSplit<S, NopSlice, PBox<A>>,
    },
    Unstable,
}

impl<S> SpimPeriph<S>
where
    S: Instance + Send,
{
    fn take(&mut self) -> Self {
        let mut new = SpimPeriph::Unstable;
        core::mem::swap(self, &mut new);
        new
    }
}

use embedded_dma::ReadBuffer;

pub struct NopSlice;

unsafe impl ReadBuffer for NopSlice {
    type Word = u8;

    unsafe fn read_buffer(&self) -> (*const Self::Word, usize) {
        // crimes
        ((SRAM_UPPER - 1) as *const _, 0)
    }
}

use bbqueue::GrantW;

#[derive(Debug)]
pub struct FillBuf {
    buf: GrantW<'static, bbconsts::U65536>,
    used: usize,
}

impl FillBuf {
    fn content_len(&self) -> usize {
        self.used
    }
}

use rlercobs::Write as _;

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
