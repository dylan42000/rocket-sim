use std::io::{self, Write};

use crate::dynamics::state::State;
use crate::physics::atmosphere;
use crate::vehicle::Mission;

/// Summary statistics computed from a flight trajectory.
#[derive(Debug, Clone)]
pub struct FlightSummary {
    pub apogee_m: f64,
    pub apogee_time: f64,
    pub max_speed: f64,
    pub max_mach: f64,
    pub max_accel: f64,
    pub max_accel_g: f64,
    pub flight_time: f64,
    pub impact_speed: f64,
}

impl FlightSummary {
    /// Compute summary from trajectory data.
    pub fn from_trajectory(trajectory: &[State]) -> Self {
        let apogee_state = trajectory
            .iter()
            .max_by(|a, b| a.pos.z.partial_cmp(&b.pos.z).unwrap())
            .unwrap();

        let max_speed = trajectory
            .iter()
            .map(|s| s.vel.norm())
            .fold(0.0_f64, f64::max);

        let max_mach = trajectory
            .iter()
            .map(|s| {
                let ss = atmosphere::isa(s.pos.z.max(0.0)).sound_speed;
                s.vel.norm() / ss
            })
            .fold(0.0_f64, f64::max);

        let max_accel = trajectory
            .windows(2)
            .map(|w| {
                let dt = w[1].time - w[0].time;
                if dt > 0.0 {
                    (w[1].vel - w[0].vel).norm() / dt
                } else {
                    0.0
                }
            })
            .fold(0.0_f64, f64::max);

        let last = trajectory.last().unwrap();

        FlightSummary {
            apogee_m: apogee_state.pos.z,
            apogee_time: apogee_state.time,
            max_speed,
            max_mach,
            max_accel,
            max_accel_g: max_accel / 9.80665,
            flight_time: last.time,
            impact_speed: last.vel.norm(),
        }
    }
}

/// Write flight summary as JSON to a writer.
pub fn write_summary<W: Write>(
    writer: &mut W,
    mission: &Mission,
    summary: &FlightSummary,
) -> io::Result<()> {
    writeln!(writer, "{{")?;
    writeln!(writer, "  \"mission\": {{")?;
    writeln!(writer, "    \"name\": \"{}\",", mission.name)?;
    writeln!(writer, "    \"stages\": {}", mission.stages.len())?;
    writeln!(writer, "  }},")?;
    writeln!(writer, "  \"performance\": {{")?;
    writeln!(writer, "    \"apogee_m\": {:.2},", summary.apogee_m)?;
    writeln!(writer, "    \"apogee_time_s\": {:.2},", summary.apogee_time)?;
    writeln!(writer, "    \"max_speed_ms\": {:.2},", summary.max_speed)?;
    writeln!(writer, "    \"max_mach\": {:.3},", summary.max_mach)?;
    writeln!(writer, "    \"max_accel_ms2\": {:.2},", summary.max_accel)?;
    writeln!(writer, "    \"max_accel_g\": {:.2},", summary.max_accel_g)?;
    writeln!(writer, "    \"flight_time_s\": {:.2},", summary.flight_time)?;
    writeln!(writer, "    \"impact_speed_ms\": {:.2}", summary.impact_speed)?;
    writeln!(writer, "  }}")?;
    writeln!(writer, "}}")?;
    Ok(())
}

/// Write flight summary JSON to a file.
pub fn write_summary_file(
    path: &str,
    mission: &Mission,
    summary: &FlightSummary,
) -> io::Result<()> {
    let mut file = std::fs::File::create(path)?;
    write_summary(&mut file, mission, summary)
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{UnitQuaternion, Vector3};

    fn simple_trajectory() -> Vec<State> {
        vec![
            State {
                time: 0.0,
                pos: Vector3::zeros(),
                vel: Vector3::new(0.0, 0.0, 100.0),
                quat: UnitQuaternion::identity(),
                omega: Vector3::zeros(),
                mass: 100.0,
                stage_idx: 0,
            },
            State {
                time: 10.0,
                pos: Vector3::new(0.0, 0.0, 5000.0),
                vel: Vector3::new(0.0, 0.0, 0.0),
                quat: UnitQuaternion::identity(),
                omega: Vector3::zeros(),
                mass: 80.0,
                stage_idx: 0,
            },
            State {
                time: 20.0,
                pos: Vector3::zeros(),
                vel: Vector3::new(0.0, 0.0, -50.0),
                quat: UnitQuaternion::identity(),
                omega: Vector3::zeros(),
                mass: 80.0,
                stage_idx: 0,
            },
        ]
    }

    #[test]
    fn summary_computes_apogee() {
        let traj = simple_trajectory();
        let s = FlightSummary::from_trajectory(&traj);
        assert!((s.apogee_m - 5000.0).abs() < 0.1);
        assert!((s.apogee_time - 10.0).abs() < 0.1);
    }

    #[test]
    fn json_output_is_valid() {
        let traj = simple_trajectory();
        let summary = FlightSummary::from_trajectory(&traj);
        let mission = Mission {
            name: "Test".into(),
            stages: vec![],
        };

        let mut buf = Vec::new();
        write_summary(&mut buf, &mission, &summary).unwrap();
        let json = String::from_utf8(buf).unwrap();
        assert!(json.contains("\"mission\""));
        assert!(json.contains("\"apogee_m\""));
        assert!(json.contains("\"Test\""));
    }
}
