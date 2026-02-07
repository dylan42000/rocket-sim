use nalgebra::Vector3;

// ---------------------------------------------------------------------------
// Physical constants
// ---------------------------------------------------------------------------

pub const G0: f64 = 9.80665; // standard gravity, m/s^2
pub const EARTH_RADIUS: f64 = 6_371_000.0; // mean Earth radius, m

// ---------------------------------------------------------------------------
// Simulation state
// ---------------------------------------------------------------------------

/// Full state vector at a single point in time.
/// Frame: East-North-Up (ENU), origin at launch site.
#[derive(Debug, Clone)]
pub struct State {
    pub time: f64,            // s
    pub pos: Vector3<f64>,    // m   [East, North, Up]
    pub vel: Vector3<f64>,    // m/s
    pub mass: f64,            // kg  (decreases during burn)
}

impl State {
    /// Advance state by a derivative scaled by dt (used inside RK4).
    pub fn apply(&self, d: &Deriv, dt: f64) -> State {
        State {
            time: self.time + dt,
            pos: self.pos + d.dpos * dt,
            vel: self.vel + d.dvel * dt,
            mass: (self.mass + d.dmass * dt).max(0.0),
        }
    }
}

// ---------------------------------------------------------------------------
// State derivative (dp/dt, dv/dt, dm/dt)
// ---------------------------------------------------------------------------

#[derive(Debug, Clone)]
pub struct Deriv {
    pub dpos: Vector3<f64>,   // velocity
    pub dvel: Vector3<f64>,   // acceleration
    pub dmass: f64,           // mass flow rate (negative during burn)
}

// ---------------------------------------------------------------------------
// Vehicle definition
// ---------------------------------------------------------------------------

#[derive(Debug, Clone)]
pub struct Vehicle {
    pub name: String,
    pub dry_mass: f64,        // kg — structure, payload, recovery
    pub propellant_mass: f64, // kg
    pub thrust: f64,          // N  (sea-level, constant)
    pub isp: f64,             // s  (specific impulse)
    pub burn_time: f64,       // s
    pub cd: f64,              // drag coefficient (dimensionless)
    pub area: f64,            // aerodynamic reference area, m^2
    pub launch_angle: f64,    // rad from vertical (0 = straight up)
}

impl Vehicle {
    /// Propellant mass flow rate: mdot = F / (Isp * g0)
    pub fn mass_flow(&self) -> f64 {
        self.thrust / (self.isp * G0)
    }

    /// Wet mass (total mass at ignition)
    pub fn total_mass(&self) -> f64 {
        self.dry_mass + self.propellant_mass
    }

    /// Thrust-to-weight ratio at ignition
    pub fn twr(&self) -> f64 {
        self.thrust / (self.total_mass() * G0)
    }

    /// Ideal delta-v (Tsiolkovsky rocket equation)
    pub fn delta_v(&self) -> f64 {
        self.isp * G0 * (self.total_mass() / self.dry_mass).ln()
    }
}

// ---------------------------------------------------------------------------
// Simulation configuration
// ---------------------------------------------------------------------------

#[derive(Debug, Clone)]
pub struct SimConfig {
    pub dt: f64,              // integration timestep, s
    pub max_time: f64,        // hard stop, s
}

impl Default for SimConfig {
    fn default() -> Self {
        Self {
            dt: 0.01,         // 100 Hz
            max_time: 300.0,  // 5 min ceiling
        }
    }
}
