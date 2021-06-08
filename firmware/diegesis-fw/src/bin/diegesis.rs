#![no_main]
#![no_std]

use diegesis_fw::FillBuf;
use diegesis_fw::spim_src::SpimSrc;

use nrf52840_hal::pac::Interrupt;
use nrf52840_hal::{
    clocks::{Clocks, ExternalOscillator, Internal, LfOscStopped},
    gpio::{
        p0::Parts as P0Parts,
        p1::Parts as P1Parts,
        Level,
    },
    pac::{SPIM0, SPIM1, SPIM2, SPIM3},
    usbd::Usbd,
    spim::{Frequency, Pins as SpimPins, Spim, MODE_0},
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
    mpmc::MpMcQueue,
};
use postcard::to_rlercobs_writer;
use diegesis_icd::{DataReport, Managed};
use core::ops::DerefMut;
use core::sync::atomic::AtomicBool;
use core::sync::atomic::Ordering;

type UsbDevice<'a> = usb_device::device::UsbDevice<'static, Usbd<'a>>;
type UsbSerial<'a> = SerialPort<'static, Usbd<'a>>;

use bbqueue::{
    consts as bbconsts,
    BBBuffer, ConstBBBuffer,
};
use diegesis_fw::spim_src::SpimPeriph;

pool!(
    A: [u8; 4096]
);

static ENCODED_QUEUE: BBBuffer<bbconsts::U65536> = BBBuffer(ConstBBBuffer::new());
static POOL_QUEUE: MpMcQueue<Box<A, Init>, 32> = MpMcQueue::new();

use groundhog::RollingTimer;
use diegesis_fw::groundhog_nrf52::GlobalRollingTimer;

// TODO
// * Get timer up and going
// * Count each interrupts/sec w/ Atomic
// * Count idle loops/sec
// * Average time in interrupts?
// * Bench rlercobs/postcard?
// * Count bytes/stream on bench app

use diegesis_fw::profiler;

profiler!(Profiler {
    spim_p0_ints,
    spim_p1_ints,
    spim_p2_ints,
    spim_p3_ints,
    usb_writes,
    report_sers,
    bbq_push_bytes,
    bbq_pull_bytes,
    idle_loop_iters
} => ProfilerRpt);

static PROFILER: Profiler = Profiler::new();
static FUSE: AtomicBool = AtomicBool::new(true);

#[app(device = nrf52840_hal::pac, peripherals = true)]
const APP: () = {
    struct Resources {
        usb_dev: UsbDevice<'static>,
        serial: UsbSerial<'static>,
        spim_p0: SpimSrc<SPIM0, A, 32>,
        spim_p1: SpimSrc<SPIM1, A, 32>,
        spim_p2: SpimSrc<SPIM2, A, 32>,
        spim_p3: SpimSrc<SPIM3, A, 32>,
    }

    #[init]
    fn init(ctx: init::Context) -> init::LateResources {
        static mut CLOCKS: Option<Clocks<ExternalOscillator, Internal, LfOscStopped>> = None;
        static mut USB_BUS: Option<UsbBusAllocator<Usbd<'static>>> = None;
        static mut DATA_POOL: [u8; 32 * 4096] = [0u8; 32 * 4096];

        let board = ctx.device;
        board.NVMC.icachecnf.write(|w| w.cacheen().set_bit());
        cortex_m::asm::isb();

        // NOTE: nrf52840 has a total of 256KiB of RAM.
        // We are allocating 192 KiB, or 48 data blocks, using
        // heapless pool.
        A::grow(DATA_POOL);

        defmt::info!("Hello, world!");

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

        GlobalRollingTimer::init(board.TIMER0);
        let usbd = board.USBD;
        let gpios_p0 = P0Parts::new(board.P0);
        let gpios_p1 = P1Parts::new(board.P1);

        let spim_pins_0 = SpimPins {
            sck: gpios_p1.p1_01.into_push_pull_output(Level::Low).degrade(),
            miso: Some(gpios_p0.p0_11.into_floating_input().degrade()),
            mosi: None,
        };

        let spim_pins_1 = SpimPins {
            sck: gpios_p1.p1_02.into_push_pull_output(Level::Low).degrade(),
            miso: Some(gpios_p1.p1_03.into_floating_input().degrade()),
            mosi: None,
        };

        let spim_pins_2 = SpimPins {
            sck: gpios_p1.p1_04.into_push_pull_output(Level::Low).degrade(),
            miso: Some(gpios_p1.p1_05.into_floating_input().degrade()),
            mosi: None,
        };

        let spim_pins_3 = SpimPins {
            sck: gpios_p1.p1_06.into_push_pull_output(Level::Low).degrade(),
            miso: Some(gpios_p1.p1_07.into_floating_input().degrade()),
            mosi: None,
        };

        // TODO: This probably should be dynamic
        board.SPIM0.intenset.modify(|_r, w| {
            w.stopped().set_bit()
             .end().set_bit()
             .started().set_bit()
        });

        // TODO: This probably should be dynamic
        board.SPIM1.intenset.modify(|_r, w| {
            w.stopped().set_bit()
             .end().set_bit()
             .started().set_bit()
        });

        // TODO: This probably should be dynamic
        board.SPIM2.intenset.modify(|_r, w| {
            w.stopped().set_bit()
             .end().set_bit()
             .started().set_bit()
        });

        // TODO: This probably should be dynamic
        board.SPIM3.intenset.modify(|_r, w| {
            w.stopped().set_bit()
             .end().set_bit()
             .started().set_bit()
        });

        let spim0 = Spim::new(board.SPIM0, spim_pins_0, Frequency::M4, MODE_0, 0x00);
        let spim_p0 = SpimPeriph::Idle(spim0);
        let spim1 = Spim::new(board.SPIM1, spim_pins_1, Frequency::M4, MODE_0, 0x00);
        let spim_p1 = SpimPeriph::Idle(spim1);
        let spim2 = Spim::new(board.SPIM2, spim_pins_2, Frequency::M4, MODE_0, 0x00);
        let spim_p2 = SpimPeriph::Idle(spim2);
        let spim3 = Spim::new(board.SPIM3, spim_pins_3, Frequency::M4, MODE_0, 0x00);
        let spim_p3 = SpimPeriph::Idle(spim3);

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

        // TODO: Remove me once we have a "start" interface!
        rtic::pend(Interrupt::SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0);
        rtic::pend(Interrupt::SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1);
        rtic::pend(Interrupt::SPIM2_SPIS2_SPI2);
        rtic::pend(Interrupt::SPIM3);

        init::LateResources {
            usb_dev,
            serial,
            spim_p0: SpimSrc::new(spim_p0, &POOL_QUEUE),
            spim_p1: SpimSrc::new(spim_p1, &POOL_QUEUE),
            spim_p2: SpimSrc::new(spim_p2, &POOL_QUEUE),
            spim_p3: SpimSrc::new(spim_p3, &POOL_QUEUE),
        }
    }

    #[task(binds = SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0, resources = [spim_p0])]
    fn spim_p0(c: spim_p0::Context) {
        PROFILER.spim_p0_ints();
        c.resources.spim_p0.poll(&FUSE);
    }

    #[task(binds = SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1, resources = [spim_p1])]
    fn spim_p1(c: spim_p1::Context) {
        PROFILER.spim_p1_ints();
        c.resources.spim_p1.poll(&FUSE);
    }

    #[task(binds = SPIM2_SPIS2_SPI2, resources = [spim_p2])]
    fn spim_p2(c: spim_p2::Context) {
        PROFILER.spim_p2_ints();
        c.resources.spim_p2.poll(&FUSE);
    }

    #[task(binds = SPIM3, resources = [spim_p3])]
    fn spim_p3(c: spim_p3::Context) {
        PROFILER.spim_p3_ints();
        c.resources.spim_p3.poll(&FUSE);
    }

    #[idle(resources = [usb_dev, serial])]
    fn idle(mut c: idle::Context) -> ! {
        let mut state: UsbDeviceState = UsbDeviceState::Default;
        let timer = GlobalRollingTimer::new();
        let (mut enc_prod, mut enc_cons) = ENCODED_QUEUE.try_split().unwrap();

        let start = timer.get_ticks();
        let mut last_profile = start;
        let mut fuse_timeout = None;

        loop {
            PROFILER.idle_loop_iters();
            let new_state = c.resources.usb_dev.state();
            if new_state != state {
                defmt::info!("State change!");
                state = new_state;

                if new_state == UsbDeviceState::Configured {
                    defmt::info!("Configured!");
                }
            }

            let usb_d = &mut c.resources.usb_dev;
            let serial = &mut c.resources.serial;

            // TODO: In the current version of nrf-usb, we need to poll the USB once
            // per write. This is why the following code is round-robin. In the future,
            // when a fix for this is available, we may re-consider true round-robin.
            usb_poll(usb_d, serial);

            if state != UsbDeviceState::Configured {
                continue;
            }

            if fuse_timeout.is_none() && FUSE.load(Ordering::SeqCst) {
                defmt::info!("Fuse blown! Cooling down...");
                fuse_timeout = Some(timer.get_ticks());
            }

            if let Some(tick) = fuse_timeout.take() {
                if timer.millis_since(tick) > 2500 {
                    defmt::info!("Fuse restored! Clearing...");
                    FUSE.store(false, Ordering::SeqCst);
                    rtic::pend(Interrupt::SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0);
                    rtic::pend(Interrupt::SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1);
                    rtic::pend(Interrupt::SPIM2_SPIS2_SPI2);
                    rtic::pend(Interrupt::SPIM3);
                } else {
                    // Still cooling...
                    fuse_timeout = Some(tick);
                }
            }

            if timer.millis_since(last_profile) >= 1000 {
                let rpt = PROFILER.clear_and_report();
                defmt::info!("{}", rpt);
                last_profile = timer.get_ticks();
            }

            // TODO: read?

            // TODO: with a little more complexity, we could use split grants
            // for a more efficient use of the encoding buffer. For now, we may
            // end up wasting 0 <= n < 5KiB at the end of the ring, which is a
            // whole pbox worth (7.8% of 64K capacity)
            if let Ok(wgr) = enc_prod.grant_exact(1024 + 4096) {
                if let Some(mut new_box) = POOL_QUEUE.dequeue() {
                    PROFILER.report_sers();
                    let fbuf = to_rlercobs_writer(
                        // TODO(AJM): We should be sending DataReports through the queue,
                        // not just boxes, so the senders can generate the metadata
                        &DataReport { timestamp: 0x01020304, payload: Managed::Borrowed(new_box.deref_mut()) },
                        FillBuf { buf: wgr, used: 0 }
                    ).unwrap();
                    let len = fbuf.content_len();
                    fbuf.buf.commit(len);
                    PROFILER.bbq_push_bytes.fetch_add(len as u32, Ordering::SeqCst);
                }
            };

            // Second: Drain bytes into the serial port in order to
            // free up space to encode more.
            if let Ok(rgr) = enc_cons.read() {
                match serial.write(&rgr) {
                    Ok(n) => {
                        PROFILER.usb_writes();
                        PROFILER.bbq_pull_bytes.fetch_add(n as u32, Ordering::SeqCst);
                        rgr.release(n);
                    }
                    Err(UsbError::WouldBlock) => {
                        rgr.release(0);
                    }
                    Err(e) => {
                        rgr.release(0);
                        panic!("BAD USB WRITE - {:?}", e);
                    }
                }
            }
        }
    }
};

fn usb_poll(usb_dev: &mut UsbDevice, serial: &mut UsbSerial) {
    if usb_dev.poll(&mut [serial]) {
        serial.poll();
    }
}
