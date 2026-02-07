use nalgebra::{UnitQuaternion, Vector3};

use crate::dynamics::state::{GncCommand, SimConfig, State};
use crate::gnc::Controller;
use crate::gnc::TvcController;
use crate::vehicle::Mission;
use super::integrator::rk4_step;

// ---------------------------------------------------------------------------
// Stage separation logic
// ---------------------------------------------------------------------------

/// Check if current stage propellant is exhausted, advance to next stage.
/// Returns updated state with stage_idx incremented and dropped mass removed.
fn check_staging(state: &mut State, mission: &Mission) {
    if state.stage_idx >= mission.stages.len() {
        return;
    }
    let stage = &mission.stages[state.stage_idx];
    let upper_mass: f64 = mission.stages[state.stage_idx + 1..]
        .iter()
        .map(|s| s.total_mass())
        .sum();
    let remaining_prop = state.mass - stage.dry_mass - upper_mass;

    if remaining_prop <= 0.01 && state.stage_idx + 1 < mission.stages.len() {
        // Drop current stage dry mass, advance
        state.mass -= stage.dry_mass;
        state.stage_idx += 1;
    }
}

// ---------------------------------------------------------------------------
// Full mission simulation
// ---------------------------------------------------------------------------

/// Simulate a complete multi-stage mission with a custom controller.
/// Returns trajectory and the GNC commands at each step.
pub fn simulate_with(
    mission: &Mission,
    config: &SimConfig,
    controller: &mut dyn Controller,
) -> (Vec<State>, Vec<GncCommand>) {
    let mut state = State {
        time: 0.0,
        pos: Vector3::zeros(),
        vel: Vector3::zeros(),
        quat: UnitQuaternion::identity(),
        omega: Vector3::zeros(),
        mass: mission.total_mass(),
        stage_idx: 0,
    };

    let capacity = (config.max_time / config.dt) as usize + 1;
    let cap = capacity.min(200_000);
    let mut trajectory = Vec::with_capacity(cap);
    let mut commands = Vec::with_capacity(cap);

    trajectory.push(state.clone());
    commands.push(GncCommand::default());

    let mut launched = false;

    while state.time < config.max_time {
        // GNC update
        let cmd = controller.control(&state, mission, config.dt);

        // Integrate
        state = rk4_step(&state, mission, &cmd, config.dt);

        // Stage separation
        check_staging(&mut state, mission);

        if state.pos.z > 1.0 {
            launched = true;
        }

        // Ground impact
        if launched && state.pos.z <= 0.0 {
            state.pos.z = 0.0;
            trajectory.push(state);
            commands.push(cmd);
            break;
        }

        trajectory.push(state.clone());
        commands.push(cmd);
    }

    (trajectory, commands)
}

/// Simulate with the default TvcController (convenience wrapper).
pub fn simulate(mission: &Mission, config: &SimConfig) -> (Vec<State>, Vec<GncCommand>) {
    let mut controller = TvcController::new();
    simulate_with(mission, config, &mut controller)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::vehicle::Stage;

    fn two_stage_mission() -> Mission {
        Mission {
            name: "2-Stage Test".into(),
            stages: vec![
                Stage {
                    name: "Booster".into(),
                    dry_mass: 40.0,
                    propellant_mass: 30.0,
                    thrust: 5000.0,
                    isp: 220.0,
                    cd: 0.35,
                    area: 0.02,
                    inertia: Vector3::new(20.0, 20.0, 2.0),
                    nozzle_offset: 1.5,
                    cp_offset: 0.4,
                    tvc_max: 0.1,
                },
                Stage {
                    name: "Sustainer".into(),
                    dry_mass: 10.0,
                    propellant_mass: 8.0,
                    thrust: 1500.0,
                    isp: 250.0,
                    cd: 0.3,
                    area: 0.01,
                    inertia: Vector3::new(3.0, 3.0, 0.3),
                    nozzle_offset: 0.8,
                    cp_offset: 0.3,
                    tvc_max: 0.08,
                },
            ],
        }
    }

    fn single_stage() -> Mission {
        Mission {
            name: "1-Stage".into(),
            stages: vec![Stage {
                name: "Main".into(),
                dry_mass: 20.0,
                propellant_mass: 10.0,
                thrust: 2000.0,
                isp: 220.0,
                cd: 0.3,
                area: 0.008,
                inertia: Vector3::new(5.0, 5.0, 0.5),
                nozzle_offset: 1.0,
                cp_offset: 0.3,
                tvc_max: 0.1,
            }],
        }
    }

    #[test]
    fn single_stage_reaches_apogee() {
        let m = single_stage();
        let config = SimConfig { dt: 0.005, max_time: 300.0 };
        let (traj, _) = simulate(&m, &config);
        let apogee = traj.iter().map(|s| s.pos.z).fold(0.0_f64, f64::max);
        assert!(apogee > 1_000.0, "Single stage should reach >1 km, got {}", apogee);
    }

    #[test]
    fn two_stage_higher_than_single() {
        let m1 = single_stage();
        let m2 = two_stage_mission();
        let config = SimConfig { dt: 0.005, max_time: 300.0 };
        let (t1, _) = simulate(&m1, &config);
        let (t2, _) = simulate(&m2, &config);
        let ap1 = t1.iter().map(|s| s.pos.z).fold(0.0_f64, f64::max);
        let ap2 = t2.iter().map(|s| s.pos.z).fold(0.0_f64, f64::max);
        assert!(ap2 > ap1, "2-stage ({:.0}m) should beat 1-stage ({:.0}m)", ap2, ap1);
    }

    #[test]
    fn staging_occurs() {
        let m = two_stage_mission();
        let config = SimConfig { dt: 0.005, max_time: 300.0 };
        let (traj, _) = simulate(&m, &config);
        let max_stage = traj.iter().map(|s| s.stage_idx).max().unwrap();
        assert_eq!(max_stage, 1, "Should reach stage index 1 (sustainer)");
    }

    #[test]
    fn quaternion_stays_unit() {
        let m = single_stage();
        let config = SimConfig { dt: 0.005, max_time: 30.0 };
        let (traj, _) = simulate(&m, &config);
        for s in &traj {
            let norm = s.quat.quaternion().norm();
            assert!(
                (norm - 1.0).abs() < 1e-6,
                "Quaternion norm drifted to {} at t={:.2}",
                norm,
                s.time
            );
        }
    }

    #[test]
    fn rocket_returns_to_ground() {
        let m = single_stage();
        let config = SimConfig::default();
        let (traj, _) = simulate(&m, &config);
        let last = traj.last().unwrap();
        assert!(last.pos.z <= 0.01, "Rocket should return to ground");
    }
}
