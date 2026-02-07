pub mod physics;
pub mod dynamics;
pub mod vehicle;
mod gnc_mod;
pub mod sim;
pub mod io;
pub mod orbital;

// The gnc module: expose gnc_mod as `gnc` publicly
pub mod gnc {
    pub use crate::gnc_mod::*;
}

// Backward-compatible re-exports
pub mod atmosphere {
    pub use crate::physics::atmosphere::*;
}

pub mod integrator {
    pub use crate::sim::runner::{simulate, simulate_with};
    pub use crate::sim::integrator::rk4_step;
}

pub mod types {
    pub use crate::dynamics::state::{Deriv, GncCommand, SimConfig, State, G0, EARTH_RADIUS};
    pub use crate::vehicle::stage::Stage;
    pub use crate::vehicle::mission::Mission;
}
