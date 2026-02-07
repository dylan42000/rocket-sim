use rocket_sim::atmosphere;
use rocket_sim::integrator;
use rocket_sim::types::{SimConfig, State, Vehicle};

fn main() {
    // -----------------------------------------------------------------------
    // Vehicle: "Pathfinder-1" sounding rocket
    // -----------------------------------------------------------------------
    // Self-consistent: burn_time = propellant_mass / (thrust / (isp * g0))
    let thrust = 2000.0;
    let isp = 220.0;
    let propellant_mass = 10.0;
    let mass_flow = thrust / (isp * 9.80665);

    let vehicle = Vehicle {
        name: "Pathfinder-1".into(),
        dry_mass: 20.0,          // kg  (structure + recovery + payload)
        propellant_mass,         // kg  (solid APCP)
        thrust,                  // N   (~200 kgf)
        isp,                     // s   (typical amateur solid motor)
        burn_time: propellant_mass / mass_flow, // ~10.8 s
        cd: 0.3,                 // slender body drag coefficient
        area: 0.007_854,         // m^2 (10 cm diameter circle)
        launch_angle: 0.0,       // rad (vertical)
    };

    let config = SimConfig {
        dt: 0.01,
        max_time: 300.0,
    };

    // -----------------------------------------------------------------------
    // Run simulation
    // -----------------------------------------------------------------------
    let trajectory = integrator::simulate(&vehicle, &config);

    // -----------------------------------------------------------------------
    // Analyze trajectory
    // -----------------------------------------------------------------------
    let burnout = trajectory
        .iter()
        .find(|s| s.time >= vehicle.burn_time)
        .unwrap();

    let apogee = trajectory
        .iter()
        .max_by(|a, b| a.pos.z.partial_cmp(&b.pos.z).unwrap())
        .unwrap();

    let max_speed = trajectory
        .iter()
        .map(|s| s.vel.norm())
        .fold(0.0_f64, f64::max);

    let max_accel = max_acceleration(&trajectory, &vehicle);
    let final_state = trajectory.last().unwrap();

    // -----------------------------------------------------------------------
    // Print results
    // -----------------------------------------------------------------------
    println!();
    println!("====================================================================");
    println!("  ROCKET FLIGHT SIMULATION — {}", vehicle.name);
    println!("====================================================================");
    println!();
    println!("  Vehicle Parameters");
    println!("  ──────────────────────────────────────────────────────────────────");
    println!(
        "  Dry mass:      {:>8.1} kg    Propellant:   {:>8.1} kg",
        vehicle.dry_mass, vehicle.propellant_mass
    );
    println!(
        "  Total mass:    {:>8.1} kg    TWR:          {:>8.2}",
        vehicle.total_mass(),
        vehicle.twr()
    );
    println!(
        "  Thrust:        {:>8.0} N     Isp:          {:>8.0} s",
        vehicle.thrust, vehicle.isp
    );
    println!(
        "  Burn time:     {:>8.1} s     Delta-v:      {:>8.0} m/s",
        vehicle.burn_time,
        vehicle.delta_v()
    );
    println!(
        "  Cd:            {:>8.3}       Area:         {:>8.4} m^2",
        vehicle.cd, vehicle.area
    );
    println!();

    println!("  Flight Events");
    println!("  ──────────────────────────────────────────────────────────────────");

    let bo_mach = burnout.vel.norm() / atmosphere::isa(burnout.pos.z).sound_speed;
    println!(
        "  BURNOUT   t={:>6.1}s   alt={:>8.0}m   vel={:>7.1}m/s   Mach {:.2}",
        burnout.time,
        burnout.pos.z,
        burnout.vel.norm(),
        bo_mach
    );

    let ap_atm = atmosphere::isa(apogee.pos.z);
    println!(
        "  APOGEE    t={:>6.1}s   alt={:>8.0}m   vel={:>7.1}m/s   rho={:.4} kg/m^3",
        apogee.time, apogee.pos.z, apogee.vel.norm(), ap_atm.density,
    );

    println!(
        "  LANDING   t={:>6.1}s   vel={:>7.1}m/s",
        final_state.time,
        final_state.vel.norm()
    );
    println!();

    println!("  Performance Summary");
    println!("  ──────────────────────────────────────────────────────────────────");
    println!(
        "  Max altitude:  {:>8.0} m   ({:.2} km)",
        apogee.pos.z,
        apogee.pos.z / 1000.0
    );
    println!(
        "  Max speed:     {:>8.1} m/s (Mach {:.2})",
        max_speed,
        max_speed / atmosphere::isa(0.0).sound_speed
    );
    println!("  Max accel:     {:>8.1} m/s^2 ({:.1} g)", max_accel, max_accel / 9.80665);
    println!(
        "  Flight time:   {:>8.1} s",
        final_state.time
    );
    println!();

    // -----------------------------------------------------------------------
    // Trajectory table (sampled)
    // -----------------------------------------------------------------------
    println!("  Trajectory");
    println!("  ──────────────────────────────────────────────────────────────────");
    println!(
        "  {:>7}  {:>9}  {:>9}  {:>8}  {:>8}  {:>7}",
        "t (s)", "alt (m)", "vel (m/s)", "Mach", "mass(kg)", "phase"
    );
    println!("  {}", "─".repeat(60));

    let sample_interval = (trajectory.len() / 30).max(1);
    for (i, s) in trajectory.iter().enumerate() {
        let print = i % sample_interval == 0
            || i == 0
            || (s.time - vehicle.burn_time).abs() < config.dt * 1.5
            || i == trajectory.len() - 1;

        if !print {
            continue;
        }

        let speed = s.vel.norm();
        let mach = speed / atmosphere::isa(s.pos.z.max(0.0)).sound_speed;
        let phase = if s.time < vehicle.burn_time {
            "BURN"
        } else if s.vel.z > 0.0 {
            "COAST"
        } else {
            "DESC"
        };

        println!(
            "  {:>7.2}  {:>9.1}  {:>9.1}  {:>8.3}  {:>8.2}  {:>7}",
            s.time, s.pos.z, speed, mach, s.mass, phase
        );
    }

    println!();
    println!("  Simulation: {} steps, dt={} s", trajectory.len(), config.dt);
    println!("====================================================================");
    println!();
}

/// Estimate peak acceleration from trajectory (finite differences).
fn max_acceleration(traj: &[State], _vehicle: &Vehicle) -> f64 {
    let mut max_a = 0.0_f64;
    for pair in traj.windows(2) {
        let dt = pair[1].time - pair[0].time;
        if dt > 0.0 {
            let dv = (pair[1].vel - pair[0].vel).norm();
            max_a = max_a.max(dv / dt);
        }
    }
    max_a
}
