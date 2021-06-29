#[macro_export]
macro_rules! time_ticks {
    ($atomic_tick:expr, $body:expr) => {
        let timer = $crate::groundhog_nrf52::GlobalRollingTimer::new();
        let start = timer.get_ticks();
        let ret = {
            $body
        };
        $atomic_tick.fetch_add(start, core::sync::atomic::Ordering::SeqCst);
        ret
    };
}

#[macro_export]
macro_rules! profiler {
    ($instance_ty:ident { $($ctr:ident),+ } => $report_ty:ident) => (

        pub struct $instance_ty {
            $(
                pub(crate) $ctr: core::sync::atomic::AtomicU32,
            )+
        }

        #[derive(Debug, defmt::Format)]
        pub struct $report_ty {
            $(
                pub $ctr: u32,
            )+
        }

        impl $instance_ty {
            pub const fn new() -> Self {
                Self {
                    $(
                        $ctr: core::sync::atomic::AtomicU32::new(0),
                    )+
                }
            }

            pub fn clear_and_report(&self) -> $report_ty {
                // self.spim_p0_ints.swap(0, Ordering::SeqCst)
                $report_ty {
                    $(
                        $ctr: self.$ctr.swap(0, core::sync::atomic::Ordering::SeqCst),
                    )+
                }
            }

            $(
                pub fn $ctr(&self) {
                    self.$ctr.fetch_add(1, core::sync::atomic::Ordering::SeqCst);
                }
            )+
        }
    )
}
