use nalgebra::Vector3;

use crate::dynamics::state::G0;

// ---------------------------------------------------------------------------
// Stage definition (one stage of a multi-stage rocket)
// ---------------------------------------------------------------------------

#[derive(Debug, Clone)]
pub struct Stage {
    pub name: String,
    pub dry_mass: f64,
    pub propellant_mass: f64,
    pub thrust: f64,              // N
    pub isp: f64,                 // s
    pub cd: f64,
    pub area: f64,                // m^2
    pub inertia: Vector3<f64>,    // [Ixx, Iyy, Izz] principal moments, kg·m^2
    pub nozzle_offset: f64,       // distance from CG to nozzle, m (positive = nozzle behind CG)
    pub cp_offset: f64,           // distance from CG to CP, m (positive = CP ahead, stable)
    pub tvc_max: f64,             // max gimbal angle, rad
}

impl Stage {
    pub fn mass_flow(&self) -> f64 {
        self.thrust / (self.isp * G0)
    }

    pub fn total_mass(&self) -> f64 {
        self.dry_mass + self.propellant_mass
    }

    /// Self-consistent burn time from propellant and mass flow.
    pub fn burn_time(&self) -> f64 {
        if self.thrust > 0.0 {
            self.propellant_mass / self.mass_flow()
        } else {
            0.0
        }
    }

    pub fn delta_v(&self, payload_mass: f64) -> f64 {
        let m0 = self.total_mass() + payload_mass;
        let mf = self.dry_mass + payload_mass;
        self.isp * G0 * (m0 / mf).ln()
    }
}

// ---------------------------------------------------------------------------
// Stage builder
// ---------------------------------------------------------------------------

pub struct StageBuilder {
    name: String,
    dry_mass: f64,
    propellant_mass: f64,
    thrust: f64,
    isp: f64,
    cd: f64,
    area: f64,
    inertia: Vector3<f64>,
    nozzle_offset: f64,
    cp_offset: f64,
    tvc_max: f64,
}

impl StageBuilder {
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            dry_mass: 10.0,
            propellant_mass: 5.0,
            thrust: 1000.0,
            isp: 220.0,
            cd: 0.3,
            area: 0.01,
            inertia: Vector3::new(5.0, 5.0, 0.5),
            nozzle_offset: 1.0,
            cp_offset: 0.3,
            tvc_max: 0.1,
        }
    }

    pub fn dry_mass(mut self, v: f64) -> Self { self.dry_mass = v; self }
    pub fn propellant_mass(mut self, v: f64) -> Self { self.propellant_mass = v; self }
    pub fn thrust(mut self, v: f64) -> Self { self.thrust = v; self }
    pub fn isp(mut self, v: f64) -> Self { self.isp = v; self }
    pub fn cd(mut self, v: f64) -> Self { self.cd = v; self }
    pub fn area(mut self, v: f64) -> Self { self.area = v; self }
    pub fn inertia(mut self, v: Vector3<f64>) -> Self { self.inertia = v; self }
    pub fn nozzle_offset(mut self, v: f64) -> Self { self.nozzle_offset = v; self }
    pub fn cp_offset(mut self, v: f64) -> Self { self.cp_offset = v; self }
    pub fn tvc_max(mut self, v: f64) -> Self { self.tvc_max = v; self }

    pub fn build(self) -> Stage {
        Stage {
            name: self.name,
            dry_mass: self.dry_mass,
            propellant_mass: self.propellant_mass,
            thrust: self.thrust,
            isp: self.isp,
            cd: self.cd,
            area: self.area,
            inertia: self.inertia,
            nozzle_offset: self.nozzle_offset,
            cp_offset: self.cp_offset,
            tvc_max: self.tvc_max,
        }
    }
}
