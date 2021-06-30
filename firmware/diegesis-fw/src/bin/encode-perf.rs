#![no_main]
#![no_std]

#![allow(dead_code, unused_imports)]

use core::sync::atomic::{AtomicBool, Ordering};

use diegesis_fw::{Board, InternalReport, groundhog_nrf52::GlobalRollingTimer, pinmap::{PinMap, Leds}, profiler, saadc_src::SaadcSrc, spim_src::SpimSrc, time_ticks};
use nrf52840_hal::{
    clocks::{Clocks, ExternalOscillator, Internal, LfOscStopped},
    gpio::{
        p0::{Parts as P0Parts, P0_02, P0_03, P0_29},
        p1::Parts as P1Parts,
        Disconnected, Level, Output, Pin, PushPull,
    },
    pac::{Interrupt, SPIM0, SPIM1, SPIM2, SPIM3, TIMER1, Peripherals},
    ppi::{self, Ppi0, Ppi1},
    spim::Frequency,
    usbd::Usbd,
};
use kolben::rlercobs;

use bbqueue::{consts as bbconsts, BBBuffer, ConstBBBuffer};
use embedded_hal::digital::v2::OutputPin;
use groundhog::RollingTimer;
use heapless::{mpmc::MpMcQueue, pool::singleton::Pool};
use postcard::to_rlercobs_writer;
use rtic::app;
use usb_device::{bus::UsbBusAllocator, class::UsbClass as _, device::UsbDeviceState, prelude::*};
use usbd_serial::{SerialPort, USB_CLASS_CDC};
use serde::{Serialize, Deserialize};

use diegesis_icd::{DataReport, Managed, ReportKind};

use rlercobs::Write as _;

#[cortex_m_rt::entry]
fn main() -> ! {
    // Enable instruction caches for MAXIMUM SPEED
    let board = Peripherals::take().unwrap();
    board.NVMC.icachecnf.write(|w| w.cacheen().set_bit());
    cortex_m::asm::isb();

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
    let _clocks = clocks.enable_ext_hfosc();

    GlobalRollingTimer::init(board.TIMER0);

    let mut test_buffer = [0u8; 4096];
    let mut output_buf = [0u8; 4096 + 1024];
    let mut third_buf = [0u8; 4096 + 1024];

    let fill_patterns: &[(&str, fn(&mut [u8; 4096]))] = &[
        ("all_zeroes  ", all_zeroes),
        ("all_ones    ", all_ones),
        ("rolling_data", rolling_data),
    ];

    let timer = GlobalRollingTimer::new();
        defmt::info!("|  name          | enc bytes | Tticks | Tcycbyte |");

    for (name, pattern) in fill_patterns {
        // fill
        (pattern)(&mut test_buffer);

        // encode
        let output = DataReport {
            timestamp: 0x01020304,

            kind: ReportKind::DigitalPin { channel: 23 },

            payload: Managed::Borrowed(&mut test_buffer),
        };

        let start = timer.get_ticks();
        let fbuf = to_rlercobs_writer(
            // TODO(AJM): We should be sending DataReports through the queue,
            // not just boxes, so the senders can generate the metadata
            &output,
            FillBuf { buf: &mut output_buf, used: 0 },
        )
        .unwrap();

        let len = fbuf.content_len();
        let elapsed = timer.ticks_since(start);
        let cyc_bytes = ((elapsed as f32) / 4_000_000.0 / 4096.0) * 64_000_000.0;
        defmt::info!("| {} | {}\t | {} | {} |", name, len as u32, elapsed, cyc_bytes);
        // display timing

        let now = timer.get_ticks();
        while timer.millis_since(now) < 500 { }
    }

    for (name, pattern) in fill_patterns {
        // fill
        (pattern)(&mut test_buffer);

        // encode
        let output = DataReport {
            timestamp: 0x01020304,

            kind: ReportKind::DigitalPin { channel: 23 },

            payload: Managed::Borrowed(&mut test_buffer),
        };

        // #[inline(always)]
        // fn try_push(&mut self, data: u8) -> core::result::Result<(), ()> {
        //     self.cobs.write(data).map_err(drop)
        // }

        // fn release(mut self) -> core::result::Result<Self::Output, ()> {
        //     self.cobs.end().map_err(drop)?;
        //     self.cobs.writer().write(0x00).map_err(drop)?;
        //     Ok(self.cobs.free())
        // }

        let start = timer.get_ticks();
        cortex_m::asm::nop();

        let serialized = postcard::to_slice(&output, &mut output_buf).unwrap();

        let enc_start = timer.get_ticks();

        let encoded = kolben::rlercobs::encode_all(serialized, &mut third_buf, true).unwrap();
        let len = encoded.len();

        let enc_elapsed = timer.ticks_since(enc_start);
        let elapsed = timer.ticks_since(start);
        let cyc_bytes = ((elapsed as f32) / 4_000_000.0 / 4096.0) * 64_000_000.0;
        defmt::info!("| {} | {}\t | {} | {} |", name, len as u32, elapsed, cyc_bytes);
        // display timing
        defmt::info!("ser: {}, enc: {}", elapsed - enc_elapsed, enc_elapsed);

        let now = timer.get_ticks();
        while timer.millis_since(now) < 500 { }
    }

    diegesis_fw::exit();
}

fn all_zeroes(data: &mut [u8; 4096]) {
    data.iter_mut().for_each(|b| *b = 0x00);
}

fn all_ones(data: &mut [u8; 4096]) {
    data.iter_mut().for_each(|b| *b = 0xFF);
}

fn rolling_data(data: &mut [u8; 4096]) {
    let mut i = 0u8;
    data.iter_mut().for_each(|b| {
        i = i.wrapping_add(1);
        *b = i;
    });
}


#[derive(Debug)]
pub struct FillBuf<'a> {
    pub buf: &'a mut [u8],
    pub used: usize,
}

impl<'a> FillBuf<'a> {
    pub fn content_len(&self) -> usize {
        self.used
    }
}

impl<'a> rlercobs::Write for FillBuf<'a> {
    type Error = ();

    #[inline(always)]
    fn write(&mut self, byte: u8) -> Result<(), Self::Error> {
        let buf_byte = self.buf.get_mut(self.used).ok_or(())?;
        *buf_byte = byte;
        self.used += 1;
        Ok(())
    }
}
