use crate::groundhog_nrf52::GlobalRollingTimer;
use choreographer::engine::{Behavior, Sequence};
use choreographer::script;

pub fn rainbow_crawler_ccw<const LEDS: usize, const MAX_STEPS: usize>(
    script: &mut [Sequence<GlobalRollingTimer, MAX_STEPS>; LEDS],
) {
    script[0].set(
        script! {
            | action | color | duration_ms | period_ms_f | repeat |
            |  solid | BLACK |           0 |         0.0 |   once |
            |    sin | WHITE |        1000 |      2000.0 |   once |
            |  solid | BLACK |        1000 |         0.0 |   once |
        },
        Behavior::LoopForever,
    );

    script[1].set(
        script! {
            | action | color | duration_ms | period_ms_f | repeat |
            |  solid | BLACK |         100 |         0.0 |   once |
            |    sin | WHITE |        1000 |      2000.0 |   once |
            |  solid | BLACK |         900 |         0.0 |   once |
        },
        Behavior::LoopForever,
    );
    script[2].set(
        script! {
            | action | color | duration_ms | period_ms_f | repeat |
            |  solid | BLACK |         200 |         0.0 |   once |
            |    sin |   RED |        1000 |      2000.0 |   once |
            |  solid | BLACK |         800 |         0.0 |   once |
        },
        Behavior::LoopForever,
    );
    script[3].set(
        script! {
            | action |  color | duration_ms | period_ms_f | repeat |
            |  solid |  BLACK |         300 |         0.0 |   once |
            |    sin | ORANGE |        1000 |      2000.0 |   once |
            |  solid |  BLACK |         700 |         0.0 |   once |
        },
        Behavior::LoopForever,
    );
    script[4].set(
        script! {
            | action |  color | duration_ms | period_ms_f | repeat |
            |  solid |  BLACK |         400 |         0.0 |   once |
            |    sin | YELLOW |        1000 |      2000.0 |   once |
            |  solid |  BLACK |         600 |         0.0 |   once |
        },
        Behavior::LoopForever,
    );
    script[5].set(
        script! {
            | action | color | duration_ms | period_ms_f | repeat |
            |  solid | BLACK |         500 |         0.0 |   once |
            |    sin | GREEN |        1000 |      2000.0 |   once |
            |  solid | BLACK |         500 |         0.0 |   once |
        },
        Behavior::LoopForever,
    );
    script[6].set(
        script! {
            | action | color | duration_ms | period_ms_f | repeat |
            |  solid | BLACK |         600 |         0.0 |   once |
            |    sin |  BLUE |        1000 |      2000.0 |   once |
            |  solid | BLACK |         400 |         0.0 |   once |
        },
        Behavior::LoopForever,
    );
    script[7].set(
        script! {
            | action |  color | duration_ms | period_ms_f | repeat |
            |  solid |  BLACK |         700 |         0.0 |   once |
            |    sin | VIOLET |        1000 |      2000.0 |   once |
            |  solid |  BLACK |         300 |         0.0 |   once |
        },
        Behavior::LoopForever,
    );
    script[8].set(
        script! {
            | action | color | duration_ms | period_ms_f | repeat |
            |  solid | BLACK |         800 |         0.0 |   once |
            |    sin | WHITE |        1000 |      2000.0 |   once |
            |  solid | BLACK |         200 |         0.0 |   once |
        },
        Behavior::LoopForever,
    );
    script[9].set(
        script! {
            | action | color | duration_ms | period_ms_f | repeat |
            |  solid | BLACK |         900 |         0.0 |   once |
            |    sin | WHITE |        1000 |      2000.0 |   once |
            |  solid | BLACK |         100 |         0.0 |   once |
        },
        Behavior::LoopForever,
    );
}
