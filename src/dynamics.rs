use nalgebra::Vector3;

use crate::atmosphere;
use crate::types::{Deriv, State, Vehicle, EARTH_RADIUS, G0};

// ---------------------------------------------------------------------------
// Equations of motion (3DOF point-mass)
// ---------------------------------------------------------------------------

/// Compute state derivatives for a given state and vehicle.
///
/// Forces modeled:
///   1. Gravity — inverse-square law, radial (down)
///   2. Thrust  — constant magnitude during burn, along velocity (gravity turn)
///   3. Drag    — quadratic, opposing velocity
pub fn derivatives(state: &State, vehicle: &Vehicle) -> Deriv {
    let alt = state.pos.z.max(0.0);

    // --- Gravity (acceleration) ---
    let g = G0 * (EARTH_RADIUS / (EARTH_RADIUS + alt)).powi(2);
    let a_gravity = Vector3::new(0.0, 0.0, -g);

    // --- Thrust (acceleration) ---
    let a_thrust = if state.time < vehicle.burn_time && state.mass > vehicle.dry_mass {
        let speed = state.vel.norm();
        let direction = if speed > 1.0 {
            // Gravity turn: thrust follows velocity vector
            state.vel.normalize()
        } else {
            // On pad / very low speed: thrust along launch direction
            let angle = vehicle.launch_angle;
            Vector3::new(angle.sin(), 0.0, angle.cos())
        };
        direction * (vehicle.thrust / state.mass)
    } else {
        Vector3::zeros()
    };

    // --- Aerodynamic drag (acceleration) ---
    let a_drag = {
        let speed = state.vel.norm();
        if speed > 1e-6 {
            let atm = atmosphere::isa(alt);
            let q = 0.5 * atm.density * speed * speed; // dynamic pressure
            let f_drag = q * vehicle.cd * vehicle.area;
            -state.vel.normalize() * (f_drag / state.mass)
        } else {
            Vector3::zeros()
        }
    };

    // --- Mass flow ---
    let dmass = if state.time < vehicle.burn_time && state.mass > vehicle.dry_mass {
        -vehicle.mass_flow()
    } else {
        0.0
    };

    Deriv {
        dpos: state.vel,
        dvel: a_gravity + a_thrust + a_drag,
        dmass,
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn test_vehicle() -> Vehicle {
        // Self-consistent: burn_time = propellant_mass / mass_flow_rate
        let thrust = 2000.0;
        let isp = 220.0;
        let propellant_mass = 10.0;
        let mass_flow = thrust / (isp * G0);
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
    fn net_upward_accel_on_pad() {
        let v = test_vehicle();
        assert!(v.twr() > 1.0, "Vehicle must have TWR > 1 to launch");
        let state = State {
            time: 0.0,
            pos: Vector3::new(0.0, 0.0, 0.0),
            vel: Vector3::zeros(),
            mass: v.total_mass(),
        };
        let d = derivatives(&state, &v);
        // TWR > 1 → net acceleration is upward
        assert!(
            d.dvel.z > 0.0,
            "Net accel should be upward, got {}",
            d.dvel.z
        );
    }

    #[test]
    fn no_thrust_after_burnout() {
        let v = test_vehicle();
        let state = State {
            time: 10.0, // well past burn_time = 5s
            pos: Vector3::new(0.0, 0.0, 5000.0),
            vel: Vector3::new(0.0, 0.0, 200.0),
            mass: v.dry_mass,
        };
        let d = derivatives(&state, &v);
        // Only gravity + drag, both act downward on ascending rocket
        assert!(d.dvel.z < 0.0);
    }

    #[test]
    fn mass_decreases_during_burn() {
        let v = test_vehicle();
        let state = State {
            time: 1.0,
            pos: Vector3::new(0.0, 0.0, 100.0),
            vel: Vector3::new(0.0, 0.0, 50.0),
            mass: 28.0,
        };
        let d = derivatives(&state, &v);
        assert!(d.dmass < 0.0);
    }

    #[test]
    fn mass_stable_after_burnout() {
        let v = test_vehicle();
        let state = State {
            time: 10.0,
            pos: Vector3::new(0.0, 0.0, 5000.0),
            vel: Vector3::new(0.0, 0.0, 100.0),
            mass: v.dry_mass,
        };
        let d = derivatives(&state, &v);
        assert!((d.dmass).abs() < 1e-10);
    }
}
