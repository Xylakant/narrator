use crate::LossyIntoF32;
use groundhog::RollingTimer;
use libm::{cosf, fabsf, sinf};
use smart_leds::RGB8;

#[derive(Clone, Debug, Default)]
pub struct StayColor<R>
where
    R: RollingTimer<Tick = u32> + Default + Clone,
{
    start_tick: R::Tick,
    pub duration_ms: R::Tick,
    pub color: RGB8,
    pub phase_offset_ms: R::Tick,
    pub auto_incr_phase: AutoIncr,
}

impl<R> StayColor<R>
where
    R: RollingTimer<Tick = u32> + Default + Clone,
{
    pub fn new(duration_ms: R::Tick, color: RGB8) -> Self {
        let timer = R::default();
        Self {
            start_tick: timer.get_ticks(),
            duration_ms,
            // TODO(AJM): This is a hack to get around no generic `.zero()` method
            phase_offset_ms: timer.ticks_since(timer.get_ticks()),
            color,
            auto_incr_phase: AutoIncr::Never,
        }
    }

    pub fn calc_end(&self) -> R::Tick {
        self.start_tick.wrapping_add(self.duration_ms * (R::TICKS_PER_SECOND / 1000))
    }

    pub fn calc_end_phase(&self) -> R::Tick {
        self.phase_offset_ms.wrapping_add(self.duration_ms)
    }

    pub fn reinit(&mut self, start: R::Tick, start_ph: R::Tick) {
        self.start_tick = start;
        match self.auto_incr_phase {
            AutoIncr::Never => {}
            AutoIncr::Once => {
                self.phase_offset_ms = start_ph;
                self.auto_incr_phase = AutoIncr::Never;
            }
            AutoIncr::Forever => {
                self.phase_offset_ms = start_ph;
            }
        }
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

#[derive(Debug, Clone, Copy)]
pub enum AutoIncr {
    Never,
    Once,
    Forever,
}

impl Default for AutoIncr {
    fn default() -> Self {
        AutoIncr::Never
    }
}

#[derive(Clone)]
pub struct Cycler<R>
where
    R: RollingTimer<Tick = u32> + Default + Clone,
{
    start_tick: R::Tick,
    pub auto_incr_phase: AutoIncr,
    pub period_ms: f32,
    pub duration_ms: R::Tick,
    pub phase_offset_ms: R::Tick,
    pub color: RGB8,
    func: fn(f32) -> f32,
}

// Methods:
//
// reinit(): reinitialize with the current time
// poll() -> Option<RGB8>: Some if updated color, None if action is complete

impl<R> Cycler<R>
where
    R: RollingTimer<Tick = u32> + Default + Clone,
{
    pub fn new(period_ms: f32, duration_ms: R::Tick, color: RGB8) -> Self {
        // Since we "rectify" the sine wave, it actually has a period that
        // looks half as long.
        let period_ms = period_ms * 2.0;

        Self {
            start_tick: R::default().get_ticks(),
            period_ms,
            duration_ms,
            phase_offset_ms: 0,
            color,
            func: sinf,
            auto_incr_phase: AutoIncr::Never,
        }
    }

    pub fn calc_end(&self) -> R::Tick {
        self.start_tick.wrapping_add(self.duration_ms * (R::TICKS_PER_SECOND / 1000))
    }

    pub fn calc_end_phase(&self) -> R::Tick {
        self.phase_offset_ms.wrapping_add(self.duration_ms)
    }

    pub fn reinit(&mut self, start: R::Tick, start_ph: R::Tick) {
        self.start_tick = start;
        match self.auto_incr_phase {
            AutoIncr::Never => { },
            AutoIncr::Once => {
                self.phase_offset_ms = start_ph;
                self.auto_incr_phase = AutoIncr::Never;
            }
            AutoIncr::Forever => {
                self.phase_offset_ms = start_ph;
            }
        }
    }

    pub fn poll(&self) -> Option<RGB8> {
        let timer = R::default();
        let delta = timer.millis_since(self.start_tick);

        if delta >= self.duration_ms {
            return None;
        }

        let deltaf = delta.wrapping_add(self.phase_offset_ms).lossy_into();
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
    R: RollingTimer<Tick = u32> + Default + Clone,
{
    pub cycler: Cycler<R>,
}

impl<R> FadeColor<R>
where
    R: RollingTimer<Tick = u32> + Default + Clone,
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

    pub fn calc_end_phase(&self) -> R::Tick {
        self.cycler.calc_end_phase()
    }

    pub fn calc_end(&self) -> R::Tick {
        self.cycler.calc_end()
    }

    pub fn reinit(&mut self, start: R::Tick, start_ph: R::Tick) {
        self.cycler.reinit(start, start_ph);
    }

    pub fn poll(&self) -> Option<RGB8> {
        self.cycler.poll()
    }

    pub fn inner_mut(&mut self) -> &mut Cycler<R> {
        &mut self.cycler
    }
}
