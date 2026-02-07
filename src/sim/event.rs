use crate::dynamics::state::State;

// ---------------------------------------------------------------------------
// Simulation events
// ---------------------------------------------------------------------------

/// Kinds of simulation events.
#[derive(Debug, Clone, PartialEq)]
pub enum EventKind {
    Launch,
    Burnout { stage: usize },
    Staging { from: usize, to: usize },
    Apogee,
    Landing,
    Custom(String),
}

/// A discrete event that occurred during simulation.
#[derive(Debug, Clone)]
pub struct SimEvent {
    pub time: f64,
    pub kind: EventKind,
    pub state: State,
}

/// Trait for passive event detectors.
/// Implementations inspect consecutive states and report events.
pub trait EventDetector {
    fn check(&mut self, prev: &State, current: &State) -> Option<EventKind>;
}

/// Detects apogee (altitude going from increasing to decreasing).
pub struct ApogeeDetector;

impl EventDetector for ApogeeDetector {
    fn check(&mut self, prev: &State, current: &State) -> Option<EventKind> {
        if prev.vel.z > 0.0 && current.vel.z <= 0.0 && current.pos.z > 100.0 {
            Some(EventKind::Apogee)
        } else {
            None
        }
    }
}

/// Detects when altitude crosses a threshold (ascending or descending).
pub struct AltitudeDetector {
    pub altitude: f64,
    pub ascending: bool,
    fired: bool,
}

impl AltitudeDetector {
    pub fn new(altitude: f64, ascending: bool) -> Self {
        Self { altitude, ascending, fired: false }
    }
}

impl EventDetector for AltitudeDetector {
    fn check(&mut self, prev: &State, current: &State) -> Option<EventKind> {
        if self.fired {
            return None;
        }
        let crossed = if self.ascending {
            prev.pos.z < self.altitude && current.pos.z >= self.altitude
        } else {
            prev.pos.z > self.altitude && current.pos.z <= self.altitude
        };
        if crossed {
            self.fired = true;
            Some(EventKind::Custom(format!(
                "Altitude {:.0}m ({})",
                self.altitude,
                if self.ascending { "ascending" } else { "descending" }
            )))
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{UnitQuaternion, Vector3};

    fn make_state(alt: f64, vz: f64) -> State {
        State {
            time: 0.0,
            pos: Vector3::new(0.0, 0.0, alt),
            vel: Vector3::new(0.0, 0.0, vz),
            quat: UnitQuaternion::identity(),
            omega: Vector3::zeros(),
            mass: 100.0,
            stage_idx: 0,
        }
    }

    #[test]
    fn apogee_detected() {
        let mut det = ApogeeDetector;
        let prev = make_state(5000.0, 10.0);
        let curr = make_state(5005.0, -1.0);
        assert_eq!(det.check(&prev, &curr), Some(EventKind::Apogee));
    }

    #[test]
    fn altitude_detector_ascending() {
        let mut det = AltitudeDetector::new(1000.0, true);
        let prev = make_state(900.0, 100.0);
        let curr = make_state(1050.0, 100.0);
        assert!(det.check(&prev, &curr).is_some());
        // Should not fire again
        assert!(det.check(&prev, &curr).is_none());
    }
}
