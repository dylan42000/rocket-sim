# rocket-sim

A 6DOF multi-stage rocket flight simulator written in Rust. Simulates realistic sounding rocket trajectories with thrust vector control (TVC), aerodynamic forces, staging, and gravity turn guidance — all at 200 Hz with RK4 integration.

Also includes an orbital mechanics toolkit for Keplerian elements, Hohmann transfers, and J2-perturbed orbit propagation.

## Features

- **6DOF dynamics** — Quaternion-based attitude, inverse-square gravity, TVC thrust deflection, aerodynamic drag/restoring moments, damping
- **Multi-stage rockets** — Automatic stage separation on propellant depletion, dry mass jettison
- **GNC pipeline** — 3-phase pitch program (vertical ascent → pitchover → gravity turn) with PID controllers
- **Pluggable controllers** — `Controller` trait lets you drop in custom flight control logic
- **Orbital mechanics** — Keplerian elements ↔ state vectors, Hohmann transfer calculator, J2-perturbed orbit propagation (RK4)
- **Data export** — CSV trajectory and JSON flight summary, zero external dependencies
- **ISA 1976 atmosphere** — 7-layer standard atmosphere model (0–86 km)
- **Real-time visualization** — Optional egui/eframe GUI with altitude, velocity, pitch, and trajectory plots
- **36 unit tests** covering physics, dynamics, GNC, orbital mechanics, and integration

## Quick Start

```bash
cargo run
```

Output:

```
====================================================================
  6DOF FLIGHT SIMULATION — Pathfinder
====================================================================

  Stage 1 — S1-Booster
  ──────────────────────────────────────────────────────────────────
  Mass: 40+25 kg  Thrust: 5000 N  Isp: 220 s  Burn: 10.8 s

  Stage 2 — S2-Sustainer
  ──────────────────────────────────────────────────────────────────
  Mass: 8+6 kg  Thrust: 1200 N  Isp: 250 s  Burn: 12.3 s

  Total mass: 79 kg   Total dv: 2193 m/s

  Flight Events
  ──────────────────────────────────────────────────────────────────
  STAGING   t=  10.8s   alt=    3310m   vel=  628.5m/s   mass=14.0kg
  APOGEE    t= 134.3s   alt=   75124m   vel=  293.4m/s   pitch=13.3 deg
  LANDING   t= 270.9s   vel=  377.1m/s

  Performance
  ──────────────────────────────────────────────────────────────────
  Max altitude:     75124 m   (75.12 km)
  Max speed:       1336.5 m/s (Mach 4.53)
  Max accel:         88.2 m/s^2 (9.0 g)
  Flight time:      270.9 s
```

## Usage

```bash
# CLI simulation with trajectory table
cargo run

# Export trajectory CSV and flight summary JSON
cargo run -- --export

# Real-time visualization (requires system OpenGL)
cargo run --bin rocket-viz --features viz

# Run all tests
cargo test

# Examples
cargo run --example pathfinder           # 2-stage sounding rocket + CSV export
cargo run --example custom_controller    # Bang-bang controller via Controller trait
cargo run --example orbital_transfer     # Hohmann LEO→GEO + J2 orbit propagation
```

## Project Structure

```
src/
├── lib.rs                        # Module declarations + backward-compat re-exports
├── vehicle/
│   ├── stage.rs                  # Stage struct + StageBuilder
│   └── mission.rs                # Mission struct + MissionBuilder + presets
├── physics/
│   ├── atmosphere.rs             # ISA 1976 standard atmosphere (0–86 km)
│   ├── gravity.rs                # Inverse-square, J2 perturbation (ECI), point-mass
│   └── aerodynamics.rs           # Drag force, restoring moment, damping
├── dynamics/
│   ├── state.rs                  # State, Deriv, GncCommand, SimConfig, constants
│   └── sixdof.rs                 # 6DOF equations of motion
├── gnc_mod/ (exposed as `gnc`)
│   ├── controller.rs             # Controller trait
│   ├── pid.rs                    # PID controller with anti-windup
│   ├── tvc.rs                    # TvcController (guidance + PID)
│   └── guidance.rs               # 3-phase pitch program
├── sim/
│   ├── integrator.rs             # RK4 step
│   ├── runner.rs                 # simulate() / simulate_with() + staging
│   └── event.rs                  # SimEvent, EventDetector trait
├── orbital/
│   ├── elements.rs               # KeplerianElements ↔ state vector
│   ├── maneuvers.rs              # Hohmann transfer, circular velocity
│   └── propagator.rs             # 3DOF orbit propagation with optional J2
├── io/
│   ├── csv.rs                    # Trajectory CSV writer
│   └── json.rs                   # FlightSummary + JSON writer
└── bin/
    └── viz.rs                    # egui GUI visualization (feature-gated)
examples/
├── pathfinder.rs                 # Preset sounding rocket mission
├── custom_controller.rs          # Implement your own Controller
└── orbital_transfer.rs           # Hohmann transfer + J2 demo
```

## Simulation Pipeline

Each timestep (dt = 0.005 s):

1. **Guidance** computes desired pitch angle from a 3-phase program (vertical → pitchover → gravity turn)
2. **Control** (PID) drives TVC gimbal angles to track the pitch/yaw commands
3. **Dynamics** computes 6DOF state derivatives — gravity, thrust with TVC deflection, aerodynamic drag, restoring moment, and damping torque
4. **Integration** (RK4) advances position, velocity, quaternion, angular rate, and mass
5. **Staging** checks propellant depletion and jettisons spent stages

## Writing a Custom Controller

Implement the `Controller` trait to plug your own flight logic into the simulation:

```rust
use rocket_sim::dynamics::state::{GncCommand, State};
use rocket_sim::gnc::Controller;
use rocket_sim::vehicle::Mission;

struct MyController;

impl Controller for MyController {
    fn control(&mut self, state: &State, mission: &Mission, dt: f64) -> GncCommand {
        // Your control law here
        GncCommand { gimbal_y: 0.0, gimbal_z: 0.0 }
    }

    fn name(&self) -> &str { "MyController" }
}

// Use it:
// let (traj, cmds) = rocket_sim::sim::simulate_with(&mission, &config, &mut MyController);
```

See `examples/custom_controller.rs` for a complete bang-bang controller example.

## Orbital Mechanics

The `orbital` module provides tools independent of the 6DOF flight simulator:

```rust
use rocket_sim::orbital::{self, KeplerianElements, OrbitalState};
use rocket_sim::physics::gravity::R_EARTH_ECI;

// Hohmann transfer
let transfer = orbital::hohmann(R_EARTH_ECI + 200e3, 42_164_000.0);
println!("Total Δv: {:.0} m/s", transfer.total_dv); // ~3932 m/s

// Keplerian elements ↔ state vectors
let orbit = KeplerianElements::circular(400_000.0, 51.6_f64.to_radians());
let (pos, vel) = orbit.to_state_vector();

// Propagate with J2 perturbation
let initial = OrbitalState { time: 0.0, pos, vel };
let trajectory = orbital::propagate_orbit(&initial, 1.0, orbit.period(), true);
```

## Coordinate Conventions

| Frame | Convention |
|-------|-----------|
| Inertial | ENU (East-North-Up), Z = altitude |
| Body | Z-axis = thrust/nose direction |
| Attitude | Quaternion rotates body → inertial |
| Orbital | ECI (Earth-Centered Inertial) |
| Angles | Radians internally, degrees for display |

## Dependencies

| Crate | Purpose | Required |
|-------|---------|----------|
| `nalgebra` | Linear algebra (Vector3, Quaternion) | Yes |
| `eframe` | GUI framework | Optional (`viz` feature) |
| `egui_plot` | Plotting widgets | Optional (`viz` feature) |

No `serde`, no runtime dependencies beyond `nalgebra`. JSON/CSV export is hand-rolled.

## License

MIT
