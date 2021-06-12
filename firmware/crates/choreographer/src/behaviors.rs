use crate::LossyIntoF32;
use groundhog::RollingTimer;
use libm::{cosf, fabsf, sinf};
use smart_leds::RGB8;

#[derive(Clone, Debug, Default)]
pub struct StayColor<R>
where
    R: RollingTimer + Default + Clone,
    R::Tick: PartialOrd + LossyIntoF32,
{
    start_tick: R::Tick,
    pub duration_ms: R::Tick,
    pub color: RGB8,
}

impl<R> StayColor<R>
where
    R: RollingTimer + Default + Clone,
    R::Tick: PartialOrd + LossyIntoF32,
{
    pub fn new(duration_ms: R::Tick, color: RGB8) -> Self {
        Self {
            start_tick: R::default().get_ticks(),
            duration_ms,
            color,
        }
    }

    pub fn reinit(&mut self) {
        self.start_tick = R::default().get_ticks();
    }

    pub fn poll(&self) -> Option<RGB8> {
        let timer = R::default();
        if timer.millis_since(self.start_tick) >= self.duration_ms {
            None
        } else {
            Some(self.color)
        }
    }
}

#[derive(Clone)]
pub struct Cycler<R>
where
    R: RollingTimer + Default + Clone,
    R::Tick: PartialOrd + LossyIntoF32,
{
    start_tick: R::Tick,
    pub period_ms: f32,
    pub duration_ms: R::Tick,
    pub color: RGB8,
    func: fn(f32) -> f32,
}

// Methods:
//
// reinit(): reinitialize with the current time
// poll() -> Option<RGB8>: Some if updated color, None if action is complete

impl<R> Cycler<R>
where
    R: RollingTimer + Default + Clone,
    R::Tick: PartialOrd + LossyIntoF32,
{
    pub fn new(period_ms: f32, duration_ms: R::Tick, color: RGB8) -> Self {
        // Since we "rectify" the sine wave, it actually has a period that
        // looks half as long.
        let period_ms = period_ms * 2.0;

        Self {
            start_tick: R::default().get_ticks(),
            period_ms,
            duration_ms,
            color,
            func: sinf,
        }
    }

    pub fn reinit(&mut self) {
        self.start_tick = R::default().get_ticks();
    }

    pub fn poll(&self) -> Option<RGB8> {
        let timer = R::default();
        let delta = timer.millis_since(self.start_tick);

        if delta >= self.duration_ms {
            return None;
        }

        let deltaf = delta.lossy_into();
        let normalized = deltaf / self.period_ms;
        let rad_norm = normalized * 2.0 * core::f32::consts::PI;
        let out_norm = (self.func)(rad_norm);
        let abs_out = fabsf(out_norm);

        let retval = RGB8 {
            r: (abs_out * (self.color.r as f32)) as u8,
            g: (abs_out * (self.color.g as f32)) as u8,
            b: (abs_out * (self.color.b as f32)) as u8,
        };

        Some(retval)
    }

    pub fn start_high(&mut self) {
        self.func = cosf
    }

    pub fn start_low(&mut self) {
        self.func = sinf
    }
}

#[derive(Clone)]
pub struct FadeColor<R>
where
    R: RollingTimer + Default + Clone,
    R::Tick: PartialOrd + LossyIntoF32,
{
    pub cycler: Cycler<R>,
}

impl<R> FadeColor<R>
where
    R: RollingTimer + Default + Clone,
    R::Tick: PartialOrd + LossyIntoF32,
{
    pub fn new_fade_up(duration_ms: R::Tick, color: RGB8) -> Self {
        let period_ms = duration_ms.lossy_into() * 2.0;

        let mut cycler = Cycler::new(period_ms, duration_ms, color);
        cycler.start_low();

        Self { cycler }
    }

    pub fn new_fade_down(duration_ms: R::Tick, color: RGB8) -> Self {
        let period_ms = duration_ms.lossy_into() * 2.0;

        let mut cycler = Cycler::new(period_ms, duration_ms, color);
        cycler.start_high();

        Self { cycler }
    }

    pub fn reinit(&mut self) {
        self.cycler.reinit();
    }

    pub fn poll(&self) -> Option<RGB8> {
        self.cycler.poll()
    }

    pub fn inner_mut(&mut self) -> &mut Cycler<R> {
        &mut self.cycler
    }
}
