use nalgebra::Vector3;

use super::stage::{Stage, StageBuilder};

// ---------------------------------------------------------------------------
// Mission: ordered sequence of stages
// ---------------------------------------------------------------------------

#[derive(Debug, Clone)]
pub struct Mission {
    pub name: String,
    pub stages: Vec<Stage>,
}

impl Mission {
    /// Total wet mass of all stages combined.
    pub fn total_mass(&self) -> f64 {
        self.stages.iter().map(|s| s.total_mass()).sum()
    }

    /// Total ideal delta-v (each stage computed with upper stages as payload).
    pub fn total_delta_v(&self) -> f64 {
        let mut dv = 0.0;
        for i in 0..self.stages.len() {
            let payload: f64 = self.stages[i + 1..].iter().map(|s| s.total_mass()).sum();
            dv += self.stages[i].delta_v(payload);
        }
        dv
    }

    /// Get the currently active stage.
    pub fn active_stage(&self, idx: usize) -> Option<&Stage> {
        self.stages.get(idx)
    }
}

// ---------------------------------------------------------------------------
// Mission builder
// ---------------------------------------------------------------------------

pub struct MissionBuilder {
    name: String,
    stages: Vec<Stage>,
}

impl MissionBuilder {
    pub fn new(name: impl Into<String>) -> Self {
        Self { name: name.into(), stages: vec![] }
    }

    pub fn stage(mut self, stage: Stage) -> Self {
        self.stages.push(stage);
        self
    }

    pub fn build(self) -> Mission {
        Mission { name: self.name, stages: self.stages }
    }
}

// ---------------------------------------------------------------------------
// Preset missions
// ---------------------------------------------------------------------------

pub mod presets {
    use super::*;

    /// 2-stage sounding rocket ("Pathfinder").
    pub fn pathfinder() -> Mission {
        Mission {
            name: "Pathfinder".into(),
            stages: vec![
                StageBuilder::new("S1-Booster")
                    .dry_mass(40.0)
                    .propellant_mass(25.0)
                    .thrust(5000.0)
                    .isp(220.0)
                    .cd(0.35)
                    .area(0.02)
                    .inertia(Vector3::new(20.0, 20.0, 2.0))
                    .nozzle_offset(1.5)
                    .cp_offset(0.4)
                    .tvc_max(0.1)
                    .build(),
                StageBuilder::new("S2-Sustainer")
                    .dry_mass(8.0)
                    .propellant_mass(6.0)
                    .thrust(1200.0)
                    .isp(250.0)
                    .cd(0.28)
                    .area(0.008)
                    .inertia(Vector3::new(2.0, 2.0, 0.2))
                    .nozzle_offset(0.6)
                    .cp_offset(0.25)
                    .tvc_max(0.08)
                    .build(),
            ],
        }
    }
}
