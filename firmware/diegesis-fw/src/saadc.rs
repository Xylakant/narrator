//! Modified SAADC driver, to be upstreamed into nrf-hal.

use nrf52840_hal::gpio;
use nrf52840_hal::pac::{saadc, SAADC};

use core::ops::Deref;
use core::sync::atomic::{compiler_fence, Ordering::SeqCst};
use embedded_dma::StaticWriteBuffer;
use embedded_hal::adc::{Channel, OneShot};

pub use saadc::{
    ch::config::{GAIN_A as Gain, REFSEL_A as Reference, RESP_A as Resistor, TACQ_A as Time},
    oversample::OVERSAMPLE_A as Oversample,
    resolution::VAL_A as Resolution,
};
use saadc::{
    EVENTS_DONE, EVENTS_END, EVENTS_RESULTDONE, EVENTS_STARTED, EVENTS_STOPPED,
    TASKS_CALIBRATEOFFSET, TASKS_SAMPLE, TASKS_START, TASKS_STOP,
};

/// Interface for the SAADC peripheral.
///
/// External analog channels supported by the SAADC implement the `Channel` trait.
/// Currently, use of only one channel is allowed.
pub struct Saadc(SAADC);

impl Saadc {
    pub fn new(saadc: SAADC, config: SaadcConfig) -> Self {
        // The write enums do not implement clone/copy/debug, only the
        // read ones, hence the need to pull out and move the values.
        let SaadcConfig {
            resolution,
            oversample,
            reference,
            gain,
            resistor,
            time,
        } = config;

        saadc.enable.write(|w| w.enable().enabled());
        saadc.resolution.write(|w| w.val().variant(resolution));
        saadc
            .oversample
            .write(|w| w.oversample().variant(oversample));
        saadc.samplerate.write(|w| w.mode().task());

        let mut this = Saadc(saadc);
        this.set_channel_config(
            0,
            ChannelConfig {
                reference,
                gain,
                time,
                resistor,
            },
        );

        // Calibrate
        this.0.events_calibratedone.reset();
        this.0.tasks_calibrateoffset.write(|w| unsafe { w.bits(1) });
        while this.0.events_calibratedone.read().bits() == 0 {}

        this
    }

    pub fn set_channel_config(&mut self, channel: u8, config: ChannelConfig) {
        let ChannelConfig {
            gain,
            reference,
            resistor,
            time,
        } = config;

        let channel = &self.0.ch[channel as usize];
        channel.config.write(|w| {
            w.refsel().variant(reference);
            w.gain().variant(gain);
            w.tacq().variant(time);
            w.mode().se();
            w.resp().variant(resistor);
            w.resn().bypass();
            w.burst().enabled();
            w
        });
    }

    /// Starts an asynchronous SAADC conversion on multiple channels.
    ///
    /// The conversion is expected to be driven via PPI, so this will *not* collect any samples on
    /// its own.
    pub fn start_async_conversion<B, C>(self, channels: C, mut buffer: B) -> AsyncConversion<B, C>
    where
        B: StaticWriteBuffer<Word = i16>,
        C: Channels,
    {
        // TODO check that buffer is in RAM

        let mut chan_buf = [0; 8];
        channels.channels(&mut chan_buf);
        for (i, val) in chan_buf.iter().enumerate() {
            let channel = &self.0.ch[i];
            channel.pselp.write(|w| unsafe { w.bits(u32::from(*val)) });
        }

        // Use one-shot mode. Continuous mode does not support using multiple channels.
        self.0.samplerate.write(|w| w.mode().task());

        let (addr, words) = unsafe { buffer.static_write_buffer() };
        assert!(words < (1 << 15));
        self.0
            .result
            .ptr
            .write(|w| unsafe { w.ptr().bits(addr as u32) });
        self.0
            .result
            .maxcnt
            .write(|w| unsafe { w.maxcnt().bits(words as u16) });

        compiler_fence(SeqCst);

        self.0.events_started.reset();
        self.0.tasks_start.write(|w| w.tasks_start().set_bit());

        AsyncConversion {
            saadc: self,
            buffer,
            channels,
        }
    }

    #[inline]
    pub fn enable_end_interrupt(&self) {
        self.0.intenset.write(|w| w.end().set_bit());
    }

    #[inline]
    pub fn disable_end_interrupt(&self) {
        self.0.intenclr.write(|w| w.end().set_bit());
    }

    #[inline]
    pub fn task_start(&self) -> &TASKS_START {
        &self.0.tasks_start
    }

    #[inline]
    pub fn task_stop(&self) -> &TASKS_STOP {
        &self.0.tasks_stop
    }

    #[inline]
    pub fn task_sample(&self) -> &TASKS_SAMPLE {
        &self.0.tasks_sample
    }

    #[inline]
    pub fn task_calibrateoffset(&self) -> &TASKS_CALIBRATEOFFSET {
        &self.0.tasks_calibrateoffset
    }

    #[inline]
    pub fn event_started(&self) -> &EVENTS_STARTED {
        &self.0.events_started
    }

    #[inline]
    pub fn event_stopped(&self) -> &EVENTS_STOPPED {
        &self.0.events_stopped
    }

    #[inline]
    pub fn event_end(&self) -> &EVENTS_END {
        &self.0.events_end
    }

    #[inline]
    pub fn event_done(&self) -> &EVENTS_DONE {
        &self.0.events_done
    }

    #[inline]
    pub fn event_resultdone(&self) -> &EVENTS_RESULTDONE {
        &self.0.events_resultdone
    }
}

/// A set of SAADC channels.
pub trait Channels {
    /// Number of channels.
    #[doc(hidden)]
    const LEN: usize;

    /// Writes the channel IDs into the first `Self::LEN` elements in `channels`.
    #[doc(hidden)]
    fn channels(&self, channels: &mut [u8; 8]);
    // NB: this doesn't return `[u8; Self::LEN]` because of limitations in current Rust
}

macro_rules! channel_tuples {
    ( $($len:literal: ( $($t:ident,)+ ),)+ ) => {
        $(
            impl<
                $( $t: Channel<Saadc, ID = u8> ),*
            > Channels for ( $($t,)+ ) {
                const LEN: usize = $len;

                fn channels(&self, channels: &mut [u8; 8]) {
                    channels[..$len].copy_from_slice(&[$($t::channel(),)+]);
                }
            }
        )+
    };
}

impl<T: Channel<Saadc, ID = u8>> Channels for T {
    const LEN: usize = 1;

    fn channels(&self, channels: &mut [u8; 8]) {
        channels[0] = T::channel();
    }
}

channel_tuples! {
    1: (T1,),
    2: (T1, T2,),
    3: (T1, T2, T3,),
    4: (T1, T2, T3, T4,),
    5: (T1, T2, T3, T4, T5,),
    6: (T1, T2, T3, T4, T5, T6,),
    7: (T1, T2, T3, T4, T5, T6, T7,),
    8: (T1, T2, T3, T4, T5, T6, T7, T8,),
}

/// An ongoing asynchronous SAADC conversion.
pub struct AsyncConversion<B, C> {
    saadc: Saadc,
    buffer: B,
    /// Channels/Pins are stored here until the conversion is done.
    channels: C,
}

impl<B: StaticWriteBuffer<Word = i16>, C> AsyncConversion<B, C> {
    pub fn wait(self) -> (Saadc, B, C) {
        while self.saadc.0.events_end.read().events_end().bit_is_clear() {}
        self.saadc.0.events_end.reset();

        compiler_fence(SeqCst);

        (self.saadc, self.buffer, self.channels)
    }

    pub fn is_done(&self) -> bool {
        self.saadc.0.events_end.read().events_end().bit_is_set()
    }

    /// Caller has to enable the end -> start shortcut via PPI.
    pub fn enqueue_next_transfer<B2>(
        self,
        mut buffer: B2,
    ) -> Result<AsyncPendingConversion<B, B2, C>, (Self, B2)>
    where
        B2: StaticWriteBuffer<Word = i16>,
    {
        // Previous transfer hasn't started yet.
        if self
            .saadc
            .0
            .events_started
            .read()
            .events_started()
            .bit_is_clear()
        {
            return Err((self, buffer));
        }

        self.saadc.0.events_started.reset();

        // TODO check that buffer is in RAM

        let (addr, words) = unsafe { buffer.static_write_buffer() };
        assert!(words < (1 << 15));
        self.saadc
            .0
            .result
            .ptr
            .write(|w| unsafe { w.ptr().bits(addr as u32) });
        self.saadc
            .0
            .result
            .maxcnt
            .write(|w| unsafe { w.maxcnt().bits(words as u16) });

        compiler_fence(SeqCst);

        Ok(AsyncPendingConversion {
            saadc: self.saadc,
            current_buffer: self.buffer,
            pending_buffer: buffer,
            channels: self.channels,
        })
    }
}

impl<B, C> Deref for AsyncConversion<B, C> {
    type Target = Saadc;

    fn deref(&self) -> &Self::Target {
        &self.saadc
    }
}

/// Indicates that an async conversion is ongoing, with another one queued up.
pub struct AsyncPendingConversion<B, B2, C> {
    saadc: Saadc,
    current_buffer: B,
    pending_buffer: B2,
    /// Channels/Pins are stored here until the conversion is done.
    channels: C,
}

impl<B: StaticWriteBuffer<Word = i16>, B2: StaticWriteBuffer<Word = i16>, C>
    AsyncPendingConversion<B, B2, C>
{
    pub fn wait(self) -> (AsyncConversion<B2, C>, B) {
        while self.saadc.0.events_end.read().events_end().bit_is_clear() {}
        self.saadc.0.events_end.reset();

        compiler_fence(SeqCst);

        let conv = AsyncConversion {
            saadc: self.saadc,
            buffer: self.pending_buffer,
            channels: self.channels,
        };
        (conv, self.current_buffer)
    }

    /// Returns whether the first conversion is done.
    pub fn is_done(&self) -> bool {
        self.saadc.0.events_end.read().events_end().bit_is_set()
    }
}

impl<B, B2, C> Deref for AsyncPendingConversion<B, B2, C> {
    type Target = Saadc;

    fn deref(&self) -> &Self::Target {
        &self.saadc
    }
}

/// Used to configure the SAADC peripheral.
///
/// See the documentation of the `Default` impl for suitable default values.
pub struct SaadcConfig {
    /// Output resolution in bits.
    pub resolution: Resolution,
    /// Average 2^`oversample` input samples before transferring the result into memory.
    pub oversample: Oversample,
    /// Reference voltage of the SAADC input.
    pub reference: Reference,
    /// Gain used to control the effective input range of the SAADC.
    pub gain: Gain,
    /// Positive channel resistor control.
    pub resistor: Resistor,
    /// Acquisition time in microseconds.
    pub time: Time,
    // FIXME: remove per-channel config from this
}

/// Default SAADC configuration. 0 volts reads as 0, VDD volts reads as `u16::MAX`.
/// The returned SaadcConfig is configured with the following values:
///
#[cfg_attr(feature = "52840", doc = "```")]
#[cfg_attr(not(feature = "52840"), doc = "```ignore")]
/// # use nrf_hal_common::saadc::SaadcConfig;
/// # use nrf_hal_common::pac::{saadc, SAADC};
/// # use saadc::{
/// #    ch::config::{GAIN_A as Gain, REFSEL_A as Reference, RESP_A as Resistor, TACQ_A as Time},
/// #    oversample::OVERSAMPLE_A as Oversample,
/// #    resolution::VAL_A as Resolution,
/// # };
/// # let saadc =
/// SaadcConfig {
///     resolution: Resolution::_14BIT,
///     oversample: Oversample::OVER8X,
///     reference: Reference::VDD1_4,
///     gain: Gain::GAIN1_4,
///     resistor: Resistor::BYPASS,
///     time: Time::_20US,
/// };
/// #
/// # // ensure default values haven't changed
/// # let test_saadc = SaadcConfig::default();
/// # assert_eq!(saadc.resolution, test_saadc.resolution);
/// # assert_eq!(saadc.oversample, test_saadc.oversample);
/// # assert_eq!(saadc.reference, test_saadc.reference);
/// # assert_eq!(saadc.gain, test_saadc.gain);
/// # assert_eq!(saadc.resistor, test_saadc.resistor);
/// # assert_eq!(saadc.time, test_saadc.time);
/// # ()
/// ```
impl Default for SaadcConfig {
    fn default() -> Self {
        // Note: do not forget to update the docs above if you change values here
        SaadcConfig {
            resolution: Resolution::_14BIT,
            oversample: Oversample::OVER8X,
            reference: Reference::VDD1_4,
            gain: Gain::GAIN1_4,
            resistor: Resistor::BYPASS,
            time: Time::_20US,
        }
    }
}

pub struct ChannelConfig {
    /// Reference voltage of the SAADC input.
    pub reference: Reference,
    /// Gain used to control the effective input range of the SAADC.
    pub gain: Gain,
    /// Acquisition time in microseconds.
    pub time: Time,
    /// Positive channel resistor control.
    pub resistor: Resistor,
}

impl Default for ChannelConfig {
    fn default() -> Self {
        // Note: do not forget to update the docs above if you change values here
        Self {
            reference: Reference::VDD1_4,
            gain: Gain::GAIN1_4,
            resistor: Resistor::BYPASS,
            time: Time::_20US,
        }
    }
}

impl<PIN> OneShot<Saadc, i16, PIN> for Saadc
where
    PIN: Channel<Saadc, ID = u8>,
{
    type Error = ();

    /// Sample channel `PIN` for the configured ADC acquisition time in differential input mode.
    /// Note that this is a blocking operation.
    fn read(&mut self, _pin: &mut PIN) -> nb::Result<i16, Self::Error> {
        self.0.ch[0]
            .pselp
            .write(|w| unsafe { w.bits(PIN::channel().into()) });

        // Use one-shot mode.
        self.0.samplerate.write(|w| w.mode().task());

        let mut val: i16 = 0;
        self.0
            .result
            .ptr
            .write(|w| unsafe { w.ptr().bits(((&mut val) as *mut _) as u32) });
        self.0
            .result
            .maxcnt
            .write(|w| unsafe { w.maxcnt().bits(1) });

        // Conservative compiler fence to prevent starting the ADC before the
        // pointer and maxcount have been set.
        compiler_fence(SeqCst);

        self.0.tasks_start.write(|w| unsafe { w.bits(1) });
        self.0.tasks_sample.write(|w| unsafe { w.bits(1) });

        while self.0.events_end.read().bits() == 0 {}
        self.0.events_end.reset();

        // Will only occur if more than one channel has been enabled.
        if self.0.result.amount.read().bits() != 1 {
            return Err(nb::Error::Other(()));
        }

        // Second fence to prevent optimizations creating issues with the EasyDMA-modified `val`.
        compiler_fence(SeqCst);

        Ok(val)
    }
}

macro_rules! channel_mappings {
    ( $($n:expr => $pin:ident,)*) => {
        $(
            impl<STATE> Channel<Saadc> for gpio::p0::$pin<STATE> {
                type ID = u8;

                fn channel() -> <Self as embedded_hal::adc::Channel<Saadc>>::ID {
                    $n
                }
            }
        )*
    };
}

#[cfg(feature = "9160")]
channel_mappings! {
    1 => P0_13,
    2 => P0_14,
    3 => P0_15,
    4 => P0_16,
    5 => P0_17,
    6 => P0_18,
    7 => P0_19,
    8 => P0_20,
}

#[cfg(not(feature = "9160"))]
channel_mappings! {
    1 => P0_02,
    2 => P0_03,
    3 => P0_04,
    4 => P0_05,
    5 => P0_28,
    6 => P0_29,
    7 => P0_30,
    8 => P0_31,
}

#[cfg(not(feature = "9160"))]
impl Channel<Saadc> for InternalVdd {
    type ID = u8;

    fn channel() -> <Self as embedded_hal::adc::Channel<Saadc>>::ID {
        9
    }
}

#[cfg(not(feature = "9160"))]
/// Channel that doesn't sample a pin, but the internal VDD voltage.
pub struct InternalVdd;

#[cfg(any(feature = "52833", feature = "52840"))]
impl Channel<Saadc> for InternalVddHdiv5 {
    type ID = u8;

    fn channel() -> <Self as embedded_hal::adc::Channel<Saadc>>::ID {
        0x0D
    }
}

#[cfg(any(feature = "52833", feature = "52840"))]
/// The voltage on the VDDH pin, divided by 5.
pub struct InternalVddHdiv5;
