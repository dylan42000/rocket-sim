use nalgebra::{Quaternion, Vector3};

use crate::physics::atmosphere;
use crate::dynamics::state::{Deriv, GncCommand, State, EARTH_RADIUS, G0};
use crate::vehicle::Mission;

// ---------------------------------------------------------------------------
// 6DOF Equations of motion
// ---------------------------------------------------------------------------

/// Compute full 6DOF state derivatives.
///
/// Forces & moments:
///   1. Gravity (inverse-square, inertial frame)
///   2. Thrust with TVC gimbal (body frame → inertial)
///   3. Aerodynamic drag (opposing velocity)
///   4. Aerodynamic restoring moment (CP-CG offset)
///   5. Aerodynamic damping moment
pub fn derivatives(state: &State, mission: &Mission, cmd: &GncCommand) -> Deriv {
    let stage = match mission.active_stage(state.stage_idx) {
        Some(s) => s,
        None => return zero_deriv(state),
    };

    let alt = state.pos.z.max(0.0);
    let atm = atmosphere::isa(alt);

    // --- Mass properties ---
    let remaining_prop = state.mass
        - stage.dry_mass
        - upper_stages_mass(mission, state.stage_idx);
    let burning = remaining_prop > 0.01 && stage.thrust > 0.0;

    // --- Gravity (inertial) ---
    let g = G0 * (EARTH_RADIUS / (EARTH_RADIUS + alt)).powi(2);
    let f_gravity = Vector3::new(0.0, 0.0, -g * state.mass);

    // --- Thrust (body frame → inertial) ---
    let f_thrust_body = if burning {
        // TVC: deflect thrust vector from body +Z by gimbal angles
        let gy = cmd.gimbal_y.clamp(-stage.tvc_max, stage.tvc_max);
        let gz = cmd.gimbal_z.clamp(-stage.tvc_max, stage.tvc_max);
        Vector3::new(
            stage.thrust * gz.sin(),
            stage.thrust * gy.sin(),
            stage.thrust * gy.cos() * gz.cos(),
        )
    } else {
        Vector3::zeros()
    };
    let f_thrust_inertial = state.quat * f_thrust_body;

    // --- Aerodynamic drag (inertial, opposing velocity) ---
    let speed = state.vel.norm();
    let f_drag = if speed > 1e-6 {
        let q_dyn = 0.5 * atm.density * speed * speed;
        let drag_mag = q_dyn * stage.cd * stage.area;
        -state.vel.normalize() * drag_mag
    } else {
        Vector3::zeros()
    };

    // --- Total force → translational acceleration ---
    let f_total = f_gravity + f_thrust_inertial + f_drag;
    let accel = f_total / state.mass;

    // --- Torques (body frame) ---
    let mut torque_body = Vector3::zeros();

    // TVC torque: thrust offset from CG creates moment
    if burning {
        // Moment arm from CG to nozzle (body frame, nozzle at -Z)
        let arm = Vector3::new(0.0, 0.0, -stage.nozzle_offset);
        torque_body += arm.cross(&f_thrust_body);
    }

    // Aerodynamic restoring moment from CP-CG offset
    if speed > 1.0 && stage.cp_offset.abs() > 1e-6 {
        let vel_body = state.quat.inverse() * state.vel;
        let q_dyn = 0.5 * atm.density * speed * speed;
        // Normal force coefficient ~ 2.0 for slender body (C_N_alpha)
        let cn_alpha = 2.0;
        // Angle of attack components in body frame
        let alpha_y = vel_body.y.atan2(vel_body.z); // pitch AoA
        let alpha_z = vel_body.x.atan2(vel_body.z); // yaw AoA
        let normal_force = q_dyn * stage.area * cn_alpha;
        // Restoring moment: positive cp_offset = CP ahead of CG = stable
        torque_body.x += -normal_force * alpha_y * stage.cp_offset;
        torque_body.y += normal_force * alpha_z * stage.cp_offset;
    }

    // Aerodynamic damping (proportional to angular rate)
    if speed > 1.0 {
        let q_dyn = 0.5 * atm.density * speed * speed;
        let damp = q_dyn * stage.area * 0.5; // simplified damping coefficient
        torque_body -= state.omega * damp;
    }

    // --- Euler's equation: I * domega = torque - omega × (I * omega) ---
    let i_vec = stage.inertia;
    let i_omega = Vector3::new(
        i_vec.x * state.omega.x,
        i_vec.y * state.omega.y,
        i_vec.z * state.omega.z,
    );
    let domega = Vector3::new(
        (torque_body.x - (state.omega.y * i_omega.z - state.omega.z * i_omega.y)) / i_vec.x,
        (torque_body.y - (state.omega.z * i_omega.x - state.omega.x * i_omega.z)) / i_vec.y,
        (torque_body.z - (state.omega.x * i_omega.y - state.omega.y * i_omega.x)) / i_vec.z,
    );

    // --- Quaternion kinematics: dq/dt = 0.5 * q * omega_quat ---
    let omega_quat = Quaternion::new(0.0, state.omega.x, state.omega.y, state.omega.z);
    let dquat = state.quat.quaternion() * omega_quat * 0.5;

    // --- Mass flow ---
    let dmass = if burning { -stage.mass_flow() } else { 0.0 };

    Deriv {
        dpos: state.vel,
        dvel: accel,
        dquat,
        domega,
        dmass,
    }
}

pub(crate) fn upper_stages_mass(mission: &Mission, current_idx: usize) -> f64 {
    mission.stages[current_idx + 1..]
        .iter()
        .map(|s| s.total_mass())
        .sum()
}

fn zero_deriv(state: &State) -> Deriv {
    // Post-mission: only gravity
    let alt = state.pos.z.max(0.0);
    let g = G0 * (EARTH_RADIUS / (EARTH_RADIUS + alt)).powi(2);
    Deriv {
        dpos: state.vel,
        dvel: Vector3::new(0.0, 0.0, -g),
        dquat: Quaternion::new(0.0, 0.0, 0.0, 0.0),
        domega: Vector3::zeros(),
        dmass: 0.0,
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::vehicle::Stage;
    use nalgebra::UnitQuaternion;

    fn test_mission() -> Mission {
        Mission {
            name: "Test".into(),
            stages: vec![Stage {
                name: "S1".into(),
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

    fn pad_state(mission: &Mission) -> State {
        State {
            time: 0.0,
            pos: Vector3::zeros(),
            vel: Vector3::zeros(),
            quat: UnitQuaternion::identity(),
            omega: Vector3::zeros(),
            mass: mission.total_mass(),
            stage_idx: 0,
        }
    }

    #[test]
    fn net_upward_accel_on_pad() {
        let m = test_mission();
        let s = pad_state(&m);
        let d = derivatives(&s, &m, &GncCommand::default());
        assert!(d.dvel.z > 0.0, "TWR > 1 → net upward, got {}", d.dvel.z);
    }

    #[test]
    fn tvc_creates_torque() {
        let m = test_mission();
        let s = pad_state(&m);
        let cmd = GncCommand {
            gimbal_y: 0.05,
            gimbal_z: 0.0,
        };
        let d = derivatives(&s, &m, &cmd);
        assert!(d.domega.x.abs() > 1e-6, "TVC should create pitch torque");
    }

    #[test]
    fn no_thrust_after_burnout() {
        let m = test_mission();
        let s = State {
            time: 100.0,
            pos: Vector3::new(0.0, 0.0, 5000.0),
            vel: Vector3::new(0.0, 0.0, 200.0),
            quat: UnitQuaternion::identity(),
            omega: Vector3::zeros(),
            mass: m.stages[0].dry_mass,
            stage_idx: 0,
        };
        let d = derivatives(&s, &m, &GncCommand::default());
        assert!(d.dvel.z < 0.0, "Only gravity + drag after burnout");
        assert!(d.dmass.abs() < 1e-10);
    }

    #[test]
    fn quat_deriv_zero_at_rest() {
        let m = test_mission();
        let s = pad_state(&m);
        let d = derivatives(&s, &m, &GncCommand::default());
        let dq_norm = (d.dquat.w.powi(2) + d.dquat.i.powi(2)
            + d.dquat.j.powi(2) + d.dquat.k.powi(2))
        .sqrt();
        assert!(dq_norm < 1e-10, "No rotation → zero quat derivative");
    }
}
