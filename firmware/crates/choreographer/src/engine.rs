use core::cmp::min;

use crate::behaviors::AutoIncr;
use crate::behaviors::{Cycler, FadeColor, StayColor};
use crate::LossyIntoF32;
use groundhog::RollingTimer;
use heapless::Vec;
use smart_leds::RGB8;

#[derive(Clone, Default)]
pub struct Sequence<R, const N: usize>
where
    R: RollingTimer<Tick = u32> + Default + Clone,
{
    seq: Vec<Action<R>, N>,
    position: usize,
    behavior: Behavior,
}

#[derive(Clone)]
pub struct Action<R>
where
    R: RollingTimer<Tick = u32> + Default + Clone,
{
    action: Actions<R>,
    behavior: Behavior,
}

#[derive(Clone)]
pub enum Actions<R>
where
    R: RollingTimer<Tick = u32> + Default + Clone,
{
    Sin(Cycler<R>),
    Static(StayColor<R>),
    Fade(FadeColor<R>),
}

#[derive(Clone)]
pub enum Behavior {
    OneShot,
    LoopForever,

    #[allow(dead_code)]
    LoopN {
        current: usize,
        cycles: usize,
    },
    Nop,
}

pub struct ActionBuilder<R>
where
    R: RollingTimer<Tick = u32> + Default + Clone,
{
    act: Action<R>,
}

impl Default for Behavior {
    fn default() -> Self {
        Behavior::Nop
    }
}

impl<R> Default for Actions<R>
where
    R: RollingTimer<Tick = u32> + Default + Clone,
{
    fn default() -> Self {
        let timer = R::default();
        Actions::Static(StayColor::new(
            // TODO(AJM): This is a hack to get around no generic `.zero()` method
            timer.ticks_since(timer.get_ticks()),
            RGB8 { r: 0, g: 0, b: 0 },
        ))
    }
}

impl<R, const N: usize> Sequence<R, N>
where
    R: RollingTimer<Tick = u32> + Default + Clone,
{
    pub fn empty() -> Self {
        Self {
            seq: Vec::new(),
            position: 0,
            behavior: Behavior::OneShot,
        }
    }

    pub fn clear(&mut self) {
        self.seq.clear();
        self.position = 0;
    }

    pub fn set(&mut self, actions: &[Action<R>], behavior: Behavior) {
        let amt = min(N, actions.len());
        self.clear();

        self.seq.extend_from_slice(&actions[..amt]).ok();
        self.behavior = behavior;
    }

    pub fn poll(&mut self) -> Option<RGB8> {
        if self.seq.is_empty() || (self.position >= self.seq.len()) {
            return None;
        }

        let behavior = &mut self.behavior;
        let seq = &mut self.seq;
        let position = &mut self.position;

        use Behavior::*;
        match behavior {
            OneShot => seq[*position].poll().or_else(|| {
                let end = seq[*position].calc_end();
                let end_ph = seq[*position].calc_end_phase();
                *position += 1;
                if *position < seq.len() {
                    seq[*position].reinit(end, end_ph);
                    seq[*position].poll()
                } else {
                    None
                }
            }),
            LoopForever => seq[*position].poll().or_else(|| {
                let end = seq[*position].calc_end();
                let end_ph = seq[*position].calc_end_phase();
                *position += 1;

                if *position >= seq.len() {
                    *position = 0;
                }

                seq[*position].reinit(end, end_ph);
                seq[*position].poll()
            }),
            LoopN {
                ref mut current,
                cycles,
            } => seq[*position].poll().or_else(|| {
                let end = seq[*position].calc_end();
                let end_ph = seq[*position].calc_end_phase();
                *position += 1;

                if *position >= seq.len() {
                    if *current < *cycles {
                        *position = 0;
                        *current += 1;
                        seq[*position].reinit(end, end_ph);
                        seq[*position].poll()
                    } else {
                        None
                    }
                } else {
                    seq[*position].reinit(end, end_ph);
                    seq[*position].poll()
                }
            }),
            Nop => None,
        }
    }
}

impl<R> Action<R>
where
    R: RollingTimer<Tick = u32> + Default + Clone,
{
    pub fn new(action: Actions<R>, behavior: Behavior) -> Self {
        Self { action, behavior }
    }

    pub fn build() -> ActionBuilder<R> {
        ActionBuilder::new()
    }

    pub fn calc_end(&self) -> R::Tick {
        self.action.calc_end()
    }

    pub fn calc_end_phase(&self) -> R::Tick {
        self.action.calc_end_phase()
    }

    pub fn reinit(&mut self, start: R::Tick, end_ph: R::Tick) {
        self.action.reinit(start, end_ph);

        use Behavior::*;
        match &mut self.behavior {
            OneShot => {}
            LoopForever => {}
            Nop => {}
            LoopN {
                ref mut current, ..
            } => {
                *current = 0;
            }
        }
    }

    pub fn poll(&mut self) -> Option<RGB8> {
        use Behavior::*;

        let action = &mut self.action;
        let behavior = &mut self.behavior;

        match behavior {
            OneShot => action.poll(),
            LoopForever => action.poll().or_else(|| {
                let end = action.calc_end();
                let end_ph = action.calc_end_phase();
                action.reinit(end, end_ph);
                action.poll()
            }),
            LoopN {
                ref mut current,
                cycles,
            } => action.poll().or_else(|| {
                if *current < *cycles {
                    *current += 1;
                    action.poll()
                } else {
                    None
                }
            }),
            Nop => None,
        }
    }
}

pub enum PhaseIncr {
    Millis(u32),
    AutoIncr,
    AutoIncrOnStart,
}

impl From<u32> for PhaseIncr {
    fn from(data: u32) -> Self {
        PhaseIncr::Millis(data)
    }
}

// Builder Methods
impl<R> ActionBuilder<R>
where
    R: RollingTimer<Tick = u32> + Default + Clone,
{
    #[inline(always)]
    pub fn new() -> Self {
        Self {
            act: Action {
                action: Actions::default(),
                behavior: Behavior::default(),
            },
        }
    }

    #[inline(always)]
    pub fn finish(self) -> Action<R> {
        self.act
    }

    #[inline(always)]
    pub fn times(mut self, ct: usize) -> Self {
        self.act.behavior = Behavior::LoopN {
            current: 0,
            cycles: ct,
        };
        self
    }

    #[inline(always)]
    pub fn once(mut self) -> Self {
        self.act.behavior = Behavior::OneShot;
        self
    }

    #[inline(always)]
    pub fn forever(mut self) -> Self {
        self.act.behavior = Behavior::LoopForever;
        self
    }

    #[inline(always)]
    pub fn color(mut self, color: RGB8) -> Self {
        match &mut self.act.action {
            Actions::Sin(ref mut a) => a.color = color,
            Actions::Static(ref mut a) => a.color = color,
            Actions::Fade(ref mut a) => a.inner_mut().color = color,
        }
        self
    }

    #[inline(always)]
    pub fn for_ms(mut self, duration: R::Tick) -> Self {
        match &mut self.act.action {
            Actions::Sin(ref mut a) => a.duration_ms = duration,
            Actions::Static(ref mut a) => a.duration_ms = duration,
            Actions::Fade(ref mut a) => {
                a.inner_mut().duration_ms = duration;
                a.inner_mut().period_ms = duration.lossy_into() * 4.0;
            }
        }
        self
    }

    #[inline(always)]
    pub fn phase_offset_ms(mut self, phase_offset_ms: PhaseIncr) -> Self {
        let (phase_offset_ms, incr) = match phase_offset_ms {
            PhaseIncr::Millis(ms) => (ms, AutoIncr::Never),
            PhaseIncr::AutoIncr => (0, AutoIncr::Forever),
            PhaseIncr::AutoIncrOnStart => (0, AutoIncr::Once),
        };
        match &mut self.act.action {
            Actions::Sin(ref mut a) => {
                a.phase_offset_ms = phase_offset_ms;
                a.auto_incr_phase = incr;
            },
            Actions::Static(ref mut a) => {
                a.phase_offset_ms = phase_offset_ms;
                a.auto_incr_phase = incr;
            },
            Actions::Fade(ref mut a) => {
                a.inner_mut().phase_offset_ms = phase_offset_ms;
                a.inner_mut().auto_incr_phase = incr;
            }
        }
        self
    }

    #[inline(always)]
    pub fn dur_per_ms(mut self, duration: R::Tick, period_ms: f32) -> Self {
        match &mut self.act.action {
            Actions::Sin(ref mut a) => {
                a.duration_ms = duration;
                a.period_ms = period_ms * 2.0
            }
            Actions::Static(ref mut a) => a.duration_ms = duration,
            Actions::Fade(ref mut a) => {
                a.inner_mut().duration_ms = duration;
                a.inner_mut().period_ms = duration.lossy_into() * 4.0;
            }
        }
        self
    }

    #[inline(always)]
    pub fn period_ms(mut self, duration: f32) -> Self {
        match &mut self.act.action {
            Actions::Sin(ref mut a) => a.period_ms = duration,
            Actions::Static(_) => {}
            Actions::Fade(ref mut a) => a.inner_mut().period_ms = duration,
        }
        self
    }

    #[inline(always)]
    pub fn sin(mut self) -> Self {
        self.act.action = match self.act.action {
            s @ Actions::Sin(_) => s,
            Actions::Static(StayColor {
                color, duration_ms, phase_offset_ms, ..
            }) => {
                let mut c = Cycler::new(1.0f32, duration_ms, color);
                c.phase_offset_ms = phase_offset_ms;
                Actions::Sin(c)
            }
            Actions::Fade(FadeColor { mut cycler }) => {
                cycler.start_low();
                Actions::Sin(cycler)
            }
        };
        self
    }

    #[inline(always)]
    pub fn cos(mut self) -> Self {
        self.act.action = match self.act.action {
            Actions::Sin(mut s) => {
                s.start_high();
                Actions::Sin(s)
            },
            Actions::Static(StayColor {
                color, duration_ms, phase_offset_ms, ..
            }) => {
                let mut c = Cycler::new(1.0f32, duration_ms, color);
                c.phase_offset_ms = phase_offset_ms;
                c.start_high();
                Actions::Sin(c)
            }
            Actions::Fade(FadeColor { mut cycler }) => {
                cycler.start_high();
                Actions::Sin(cycler)
            }
        };
        self
    }

    #[inline(always)]
    pub fn solid(mut self) -> Self {
        self.act.action = match self.act.action {
            Actions::Sin(cycler) => {
                Actions::Static(StayColor::new(cycler.duration_ms, cycler.color))
            }
            s @ Actions::Static(_) => s,
            Actions::Fade(FadeColor { cycler }) => {
                Actions::Static(StayColor::new(cycler.duration_ms, cycler.color))
            }
        };
        self
    }

    #[inline(always)]
    pub fn fade_up(mut self) -> Self {
        self.act.action = match self.act.action {
            Actions::Sin(cycler) => {
                Actions::Fade(FadeColor::new_fade_up(cycler.duration_ms, cycler.color))
            }
            Actions::Static(stat) => {
                Actions::Fade(FadeColor::new_fade_up(stat.duration_ms, stat.color))
            }
            Actions::Fade(FadeColor { cycler }) => {
                Actions::Fade(FadeColor::new_fade_up(cycler.duration_ms, cycler.color))
            }
        };
        self
    }

    #[inline(always)]
    pub fn fade_down(mut self) -> Self {
        self.act.action = match self.act.action {
            Actions::Sin(cycler) => {
                Actions::Fade(FadeColor::new_fade_down(cycler.duration_ms, cycler.color))
            }
            Actions::Static(stat) => {
                Actions::Fade(FadeColor::new_fade_down(stat.duration_ms, stat.color))
            }
            Actions::Fade(FadeColor { cycler }) => {
                Actions::Fade(FadeColor::new_fade_down(cycler.duration_ms, cycler.color))
            }
        };
        self
    }
}

impl<R> Actions<R>
where
    R: RollingTimer<Tick = u32> + Default + Clone,
{
    pub fn calc_end(&self) -> R::Tick {
        use Actions::*;

        match self {
            Sin(s) => s.calc_end(),
            Static(s) => s.calc_end(),
            Fade(f) => f.calc_end(),
        }
    }

    pub fn calc_end_phase(&self) -> R::Tick {
        use Actions::*;

        match self {
            Sin(s) => s.calc_end_phase(),
            Static(s) => s.calc_end_phase(),
            Fade(f) => f.calc_end_phase(),
        }
    }

    pub fn reinit(&mut self, start: R::Tick, start_ph: R::Tick) {
        use Actions::*;

        match self {
            Sin(s) => s.reinit(start, start_ph),
            Static(s) => s.reinit(start, start_ph),
            Fade(f) => f.reinit(start, start_ph),
        }
    }

    pub fn poll(&self) -> Option<RGB8> {
        use Actions::*;
        match self {
            Sin(s) => s.poll(),
            Static(s) => s.poll(),
            Fade(f) => f.poll(),
        }
    }
}
