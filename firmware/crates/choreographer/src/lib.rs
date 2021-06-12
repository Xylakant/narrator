#![cfg_attr(not(test), no_std)]

pub mod behaviors;
pub mod engine;

pub trait LossyIntoF32 {
    fn lossy_into(&self) -> f32;
}

pub mod reexports {
    pub use smart_leds::colors;
}

impl LossyIntoF32 for u64 {
    fn lossy_into(&self) -> f32 {
        // oops
        *self as f32
    }
}

impl LossyIntoF32 for u32 {
    fn lossy_into(&self) -> f32 {
        // oops
        *self as f32
    }
}

impl LossyIntoF32 for u16 {
    fn lossy_into(&self) -> f32 {
        (*self).into()
    }
}

impl LossyIntoF32 for u8 {
    fn lossy_into(&self) -> f32 {
        (*self).into()
    }
}

// script!{
//     LoopForever:
//     | action | color | duration_ms | period_ms_f | repeat |
//     |  solid | BLACK |           0 |           - |   once |
//     |    sin | WHITE |        1000 |      2000.0 |   once |
//     |  solid | BLACK |        1000 |           - |   once |
// }

#[macro_export]
macro_rules! script {
    (| action | color | duration_ms | period_ms_f | repeat | $(| $action:ident | $color:ident | $duration_ms:literal | $period_ms_f:literal | $repeat:ident |)+) => {
        &[
            $(
                $crate::engine::Action::build()
                    .$action()
                    .color($crate::reexports::colors::$color)
                    .for_ms($duration_ms)
                    .period_ms($period_ms_f)
                    .$repeat()
                    .finish(),
            )+
        ]
    };
}
