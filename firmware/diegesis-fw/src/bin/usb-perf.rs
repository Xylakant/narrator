#![no_main]
#![no_std]

// TODO: Remove
#![allow(unused_imports)]

use core::{
    iter::{Cloned, Cycle},
    sync::atomic::{AtomicBool, AtomicU32, AtomicU8, AtomicUsize, Ordering},
};

use embedded_hal::{
    digital::v2::{InputPin, OutputPin},
    timer::CountDown,
};
// use generic_array::typenum::U8;
use diegesis_fw as _;

#[allow(unused_imports)]
use nrf52840_hal::{
    clocks::{Clocks, ExternalOscillator, Internal, LfOscStopped},
    gpio::{
        p0::Parts as P0Parts,
        p1::Parts as P1Parts,
        Input, Level, Output, Pin, PullUp, PushPull,
    },
    pac::{TIMER0, TIMER1, TWIM0},
    timer::{Instance as TimerInstance, Periodic, Timer},
    twim::{Frequency as TwimFrequency, Pins as TwimPins},
    usbd::Usbd,
    Twim,
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

type UsbDevice<'a> = usb_device::device::UsbDevice<'static, Usbd<'a>>;
type UsbSerial<'a> = SerialPort<'static, Usbd<'a>>;

use bbqueue::{
    consts as bbconsts,
    framed::{FrameConsumer, FrameProducer},
    BBBuffer, ConstBBBuffer,
};

pool!(
    A: [u8; 4096]
);

static REPORT_QUEUE: BBBuffer<bbconsts::U2048> = BBBuffer(ConstBBBuffer::new());
static NONCE: AtomicU8 = AtomicU8::new(42);

#[app(device = nrf52840_hal::pac, peripherals = true)]
const APP: () = {
    struct Resources {
        usb_dev: UsbDevice<'static>,
        serial: UsbSerial<'static>,
        timer: Timer<TIMER0, Periodic>,

        box_prod: Producer<'static, Box<A, Init>, 64>,
        box_cons: Consumer<'static, Box<A, Init>, 64>,

        rpt_prod: FrameProducer<'static, bbconsts::U2048>,
        rpt_cons: FrameConsumer<'static, bbconsts::U2048>,
    }

    #[init]
    fn init(ctx: init::Context) -> init::LateResources {
        static mut CLOCKS: Option<Clocks<ExternalOscillator, Internal, LfOscStopped>> = None;
        static mut USB_BUS: Option<UsbBusAllocator<Usbd<'static>>> = None;
        static mut QUEUE: Queue<Box<A, Init>, 64> = Queue::new();

        // NOTE: nrf52840 has a total of 256KiB of RAM.
        // We are allocating 192 KiB, or 48 data blocks, using
        // heapless pool.
        static mut DATA_POOL: [u8; 192 * 1024] = [0u8; 192 * 1024];
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
        // let mut timer1 = Timer::periodic(board.TIMER1);
        let usbd = board.USBD;
        let gpios_p0 = P0Parts::new(board.P0);
        let gpios_p1 = P1Parts::new(board.P1);

        timer.enable_interrupt();
        timer.start(Timer::<TIMER0, Periodic>::TICKS_PER_SECOND / 140);
        // timer1.enable_interrupt();
        // timer1.start(Timer::<TIMER1, Periodic>::TICKS_PER_SECOND / 30);

        *CLOCKS = Some(clocks);
        let clocks = CLOCKS.as_ref().unwrap();
        *USB_BUS = Some(Usbd::new(usbd, &clocks));
        let usb_bus = USB_BUS.as_ref().unwrap();

        let serial = SerialPort::new(usb_bus);
        let usb_dev =
            UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c0, 0x27DD))
                .manufacturer("Fake company")
                .product("Serial port")
                .serial_number("TEST")
                .device_class(USB_CLASS_CDC)
                .build();

        let (rpt_prod, rpt_cons) = REPORT_QUEUE.try_split_framed().unwrap();
        let (box_prod, box_cons) = QUEUE.split();

        init::LateResources {
            usb_dev,
            serial,
            timer,
            rpt_prod,
            rpt_cons,
            box_prod,
            box_cons,
        }
    }

    #[task(binds = TIMER0, priority = 1, resources = [timer, rpt_prod, box_prod])]
    fn tick(mut c: tick::Context) {
        static mut CUR_CHAR: u8 = b'a';

        c.resources.timer.event_compare_cc0().write(|w| w);
        let mut pbox = if let Some(pb) = A::alloc() {
            pb.freeze()
        } else {
            defmt::warn!("No box available!");
            return;
        };

        if *CUR_CHAR >= b'z' {
            *CUR_CHAR = b'a';
        } else {
            *CUR_CHAR += 1;
        }

        pbox.chunks_mut(80).for_each(|c| {
            c.iter_mut().for_each(|b| *b = *CUR_CHAR);
            c[c.len() - 1] = b'\n';
        });

        if let Ok(()) = c.resources.box_prod.enqueue(pbox) {
            // defmt::info!("Sent box!");
        } else {
            defmt::warn!("Failed to send box!");
        }
    }

    #[idle(resources = [usb_dev, serial, box_cons])]
    fn idle(mut c: idle::Context) -> ! {
        let mut state: UsbDeviceState = UsbDeviceState::Default;
        let mut ctr: u32 = 0;
        let mut skip_flag = false;
        let mut wip: Option<(usize, Box<A, Init>)> = None;

        loop {
            let new_state = c.resources.usb_dev.state();
            if new_state != state {
                defmt::info!("State change!");
                state = new_state;

                if new_state == UsbDeviceState::Configured {
                    defmt::info!("Configured!");
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

            if let Some((offset, cur_box)) = wip.take() {
                let remaining = 4096 - offset;
                match serial.write(&cur_box[offset..]) {
                    Ok(n) if n >= remaining => {
                        // We're done! Box will be released since we took it.
                        // defmt::info!("Completed box!");
                    }
                    Ok(n) => {
                        // defmt::info!("Wrote {}/4096 bytes, {} remaining", n, remaining - n);
                        // Not done yet! Put it back so we don't drop the box.
                        wip = Some((offset + n, cur_box));
                    }
                    Err(UsbError::WouldBlock) => {
                        wip = Some((offset, cur_box));
                    }
                    Err(e) => {
                        panic!("BAD USB WRITE - {:?}", e);
                    }
                }
            } else if let Some(new_box) = box_c.dequeue() {
                // defmt::info!("Dequeued Box!");
                wip = Some((0, new_box));
            }
        }
    }
};

fn usb_poll(usb_dev: &mut UsbDevice, serial: &mut UsbSerial) {
    if usb_dev.poll(&mut [serial]) {
        serial.poll();
    }
}
