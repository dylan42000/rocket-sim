use nalgebra::{Quaternion, UnitQuaternion, Vector3};

// ---------------------------------------------------------------------------
// Physical constants
// ---------------------------------------------------------------------------

pub const G0: f64 = 9.80665;
pub const EARTH_RADIUS: f64 = 6_371_000.0;

// ---------------------------------------------------------------------------
// 6DOF State: position, velocity, attitude, angular rate, mass
// ---------------------------------------------------------------------------

#[derive(Debug, Clone)]
pub struct State {
    pub time: f64,
    pub pos: Vector3<f64>,              // m, inertial ENU
    pub vel: Vector3<f64>,              // m/s, inertial
    pub quat: UnitQuaternion<f64>,      // body→inertial rotation
    pub omega: Vector3<f64>,            // rad/s, body frame angular velocity
    pub mass: f64,                      // kg
    pub stage_idx: usize,               // active stage index
}

impl State {
    pub fn apply(&self, d: &Deriv, dt: f64) -> State {
        // Quaternion integration: q_new = normalize(q + dq * dt)
        let q_raw = self.quat.quaternion() + d.dquat * dt;
        State {
            time: self.time + dt,
            pos: self.pos + d.dpos * dt,
            vel: self.vel + d.dvel * dt,
            quat: UnitQuaternion::new_normalize(q_raw),
            omega: self.omega + d.domega * dt,
            mass: (self.mass + d.dmass * dt).max(0.0),
            stage_idx: self.stage_idx,
        }
    }

    /// Body Z-axis (thrust direction) in inertial frame.
    pub fn body_z(&self) -> Vector3<f64> {
        self.quat * Vector3::z()
    }

    /// Pitch angle from local horizontal (rad). Positive = nose up.
    pub fn pitch(&self) -> f64 {
        self.body_z().z.asin()
    }

    /// Angle of attack (rad) — angle between velocity and body Z-axis.
    pub fn alpha(&self) -> f64 {
        let speed = self.vel.norm();
        if speed < 1.0 {
            return 0.0;
        }
        let cos_alpha = self.vel.dot(&self.body_z()) / speed;
        cos_alpha.clamp(-1.0, 1.0).acos()
    }
}

// ---------------------------------------------------------------------------
// State derivative
// ---------------------------------------------------------------------------

#[derive(Debug, Clone)]
pub struct Deriv {
    pub dpos: Vector3<f64>,
    pub dvel: Vector3<f64>,
    pub dquat: Quaternion<f64>,   // NOT unit — raw quaternion derivative
    pub domega: Vector3<f64>,     // angular acceleration, body frame
    pub dmass: f64,
}

// ---------------------------------------------------------------------------
// GNC command output
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, Default)]
pub struct GncCommand {
    pub gimbal_y: f64,   // TVC pitch gimbal, rad (positive = nose up)
    pub gimbal_z: f64,   // TVC yaw gimbal, rad (positive = nose right)
}

// ---------------------------------------------------------------------------
// Simulation config
// ---------------------------------------------------------------------------

#[derive(Debug, Clone)]
pub struct SimConfig {
    pub dt: f64,
    pub max_time: f64,
}

impl Default for SimConfig {
    fn default() -> Self {
        Self {
            dt: 0.005,        // 200 Hz (6DOF needs tighter timestep)
            max_time: 600.0,  // 10 min
        }
    }
}
