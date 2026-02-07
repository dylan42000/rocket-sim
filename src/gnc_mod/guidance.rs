use std::f64::consts::FRAC_PI_2;

use crate::dynamics::state::State;
use crate::vehicle::Mission;

// ---------------------------------------------------------------------------
// Guidance: desired pitch angle as a function of time/state
// ---------------------------------------------------------------------------

/// Pitch program: returns desired pitch angle (rad from horizontal).
/// - Phase 1 (0 to t_vertical): vertical ascent (90 deg)
/// - Phase 2 (t_vertical to t_pitchover_end): linear pitchover
/// - Phase 3 (after pitchover): gravity turn (follow velocity)
pub fn guidance_pitch(state: &State, mission: &Mission) -> f64 {
    let _ = mission; // available for future per-mission tuning
    let t = state.time;

    let t_vertical = 2.0;       // seconds of vertical ascent
    let t_pitchover_end = 15.0; // end of pitchover maneuver
    let target_pitch = 45.0_f64.to_radians(); // target pitch at end of pitchover

    if t < t_vertical {
        // Vertical ascent
        FRAC_PI_2
    } else if t < t_pitchover_end {
        // Linear pitchover from 90 deg to target_pitch
        let frac = (t - t_vertical) / (t_pitchover_end - t_vertical);
        FRAC_PI_2 + frac * (target_pitch - FRAC_PI_2)
    } else {
        // Gravity turn: desired pitch = flight path angle
        let speed = state.vel.norm();
        if speed > 5.0 {
            (state.vel.z / speed).asin()
        } else {
            target_pitch
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Vector3;

    #[test]
    fn guidance_vertical_at_start() {
        let state = State {
            time: 0.0,
            pos: Vector3::zeros(),
            vel: Vector3::zeros(),
            quat: nalgebra::UnitQuaternion::identity(),
            omega: Vector3::zeros(),
            mass: 30.0,
            stage_idx: 0,
        };
        let mission = Mission {
            name: "T".into(),
            stages: vec![],
        };
        let pitch = guidance_pitch(&state, &mission);
        assert!((pitch - FRAC_PI_2).abs() < 0.01, "Should be vertical at t=0");
    }

    #[test]
    fn guidance_pitchover_midpoint() {
        let state = State {
            time: 8.5, // midpoint of 2..15
            pos: Vector3::new(0.0, 0.0, 1000.0),
            vel: Vector3::new(0.0, 0.0, 200.0),
            quat: nalgebra::UnitQuaternion::identity(),
            omega: Vector3::zeros(),
            mass: 25.0,
            stage_idx: 0,
        };
        let mission = Mission {
            name: "T".into(),
            stages: vec![],
        };
        let pitch = guidance_pitch(&state, &mission);
        // Should be between 90 deg and 45 deg
        assert!(pitch < FRAC_PI_2 && pitch > 45.0_f64.to_radians());
    }
}
