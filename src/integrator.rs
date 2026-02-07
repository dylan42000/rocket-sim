use nalgebra::Vector3;

use crate::dynamics;
use crate::types::{SimConfig, State, Vehicle};

// ---------------------------------------------------------------------------
// Classical 4th-order Runge-Kutta integrator
// ---------------------------------------------------------------------------

/// Single RK4 step: advance state by dt.
pub fn rk4_step(state: &State, vehicle: &Vehicle, dt: f64) -> State {
    let k1 = dynamics::derivatives(state, vehicle);
    let k2 = dynamics::derivatives(&state.apply(&k1, dt * 0.5), vehicle);
    let k3 = dynamics::derivatives(&state.apply(&k2, dt * 0.5), vehicle);
    let k4 = dynamics::derivatives(&state.apply(&k3, dt), vehicle);

    State {
        time: state.time + dt,
        pos: state.pos + (k1.dpos + 2.0 * k2.dpos + 2.0 * k3.dpos + k4.dpos) * (dt / 6.0),
        vel: state.vel + (k1.dvel + 2.0 * k2.dvel + 2.0 * k3.dvel + k4.dvel) * (dt / 6.0),
        mass: (state.mass + (k1.dmass + 2.0 * k2.dmass + 2.0 * k3.dmass + k4.dmass) * (dt / 6.0))
            .max(0.0),
    }
}

// ---------------------------------------------------------------------------
// Full simulation loop
// ---------------------------------------------------------------------------

/// Run simulation from launch to ground impact (or max_time).
/// Returns the full trajectory as a Vec of state snapshots.
pub fn simulate(vehicle: &Vehicle, config: &SimConfig) -> Vec<State> {
    let mut state = State {
        time: 0.0,
        pos: Vector3::zeros(),
        vel: Vector3::zeros(),
        mass: vehicle.total_mass(),
    };

    let capacity = (config.max_time / config.dt) as usize + 1;
    let mut trajectory = Vec::with_capacity(capacity.min(100_000));
    trajectory.push(state.clone());

    let mut launched = false;

    while state.time < config.max_time {
        state = rk4_step(&state, vehicle, config.dt);

        if state.pos.z > 1.0 {
            launched = true;
        }

        // Ground collision after launch
        if launched && state.pos.z <= 0.0 {
            state.pos.z = 0.0;
            trajectory.push(state);
            break;
        }

        trajectory.push(state.clone());
    }

    trajectory
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn sounding_rocket() -> Vehicle {
        // Self-consistent parameters: burn_time = propellant / mass_flow
        let thrust = 2000.0;
        let isp = 220.0;
        let propellant_mass = 10.0;
        let mass_flow = thrust / (isp * crate::types::G0);
        Vehicle {
            name: "Test".into(),
            dry_mass: 20.0,
            propellant_mass,
            thrust,
            isp,
            burn_time: propellant_mass / mass_flow,
            cd: 0.3,
            area: 0.008,
            launch_angle: 0.0,
        }
    }

    #[test]
    fn rocket_goes_up() {
        let v = sounding_rocket();
        let config = SimConfig {
            dt: 0.01,
            max_time: 10.0,
        };
        let traj = simulate(&v, &config);
        // After burn, rocket must be well above ground
        let burnout = traj.iter().find(|s| s.time >= 5.0).unwrap();
        assert!(burnout.pos.z > 100.0, "Rocket should be >100m at burnout");
    }

    #[test]
    fn rocket_comes_back_down() {
        let v = sounding_rocket();
        let config = SimConfig::default();
        let traj = simulate(&v, &config);
        let final_state = traj.last().unwrap();
        assert!(
            final_state.pos.z <= 0.01,
            "Rocket should return to ground"
        );
        assert!(final_state.time > v.burn_time, "Flight lasts past burnout");
    }

    #[test]
    fn mass_conserved() {
        let v = sounding_rocket();
        let config = SimConfig::default();
        let traj = simulate(&v, &config);
        let final_state = traj.last().unwrap();
        assert!(
            (final_state.mass - v.dry_mass).abs() < 0.1,
            "Final mass should be ~dry_mass"
        );
    }

    #[test]
    fn apogee_is_physical() {
        let v = sounding_rocket();
        let config = SimConfig::default();
        let traj = simulate(&v, &config);
        let apogee = traj
            .iter()
            .max_by(|a, b| a.pos.z.partial_cmp(&b.pos.z).unwrap())
            .unwrap();
        // Ideal delta-v ~746 m/s, apogee in vacuum ~28 km, with drag expect less
        assert!(apogee.pos.z > 1_000.0, "Apogee should be > 1 km");
        assert!(
            apogee.pos.z < 50_000.0,
            "Apogee should be < 50 km (sanity)"
        );
    }
}
