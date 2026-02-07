use nalgebra::Vector3;

use rocket_sim::dynamics::state::{GncCommand, SimConfig, State};
use rocket_sim::gnc::Controller;
use rocket_sim::sim;
use rocket_sim::vehicle::{Mission, StageBuilder};

/// A simple bang-bang controller that kicks the rocket into a pitchover
/// after a fixed time, then holds zero gimbal.
struct BangBangController {
    pitchover_start: f64,
    pitchover_end: f64,
    gimbal_kick: f64,
}

impl Controller for BangBangController {
    fn control(&mut self, state: &State, _mission: &Mission, _dt: f64) -> GncCommand {
        let gy = if state.time > self.pitchover_start && state.time < self.pitchover_end {
            -self.gimbal_kick // negative = nose down (pitch over)
        } else {
            0.0
        };
        GncCommand { gimbal_y: gy, gimbal_z: 0.0 }
    }

    fn name(&self) -> &str {
        "BangBang"
    }
}

fn main() {
    let mission = Mission {
        name: "BangBang Demo".into(),
        stages: vec![
            StageBuilder::new("Main")
                .dry_mass(20.0)
                .propellant_mass(10.0)
                .thrust(2000.0)
                .isp(220.0)
                .cd(0.3)
                .area(0.008)
                .inertia(Vector3::new(5.0, 5.0, 0.5))
                .nozzle_offset(1.0)
                .cp_offset(0.3)
                .tvc_max(0.15)
                .build(),
        ],
    };

    let config = SimConfig { dt: 0.005, max_time: 300.0 };

    let mut controller = BangBangController {
        pitchover_start: 3.0,
        pitchover_end: 8.0,
        gimbal_kick: 0.08,
    };

    println!("Simulating with {} controller...", controller.name());
    let (trajectory, _) = sim::simulate_with(&mission, &config, &mut controller);

    let apogee = trajectory.iter().map(|s| s.pos.z).fold(0.0_f64, f64::max);
    let downrange = trajectory.last()
        .map(|s| (s.pos.x.powi(2) + s.pos.y.powi(2)).sqrt())
        .unwrap_or(0.0);

    println!("Apogee: {:.0} m ({:.2} km)", apogee, apogee / 1000.0);
    println!("Downrange: {:.0} m", downrange);
    println!("Flight time: {:.1} s", trajectory.last().unwrap().time);
    println!("Trajectory points: {}", trajectory.len());
}
