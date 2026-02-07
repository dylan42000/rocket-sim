use rocket_sim::dynamics::state::SimConfig;
use rocket_sim::io::json::FlightSummary;
use rocket_sim::physics::atmosphere;
use rocket_sim::sim;
use rocket_sim::vehicle::presets;

fn main() {
    let mission = presets::pathfinder();
    let config = SimConfig {
        dt: 0.005,
        max_time: 600.0,
    };

    let export = std::env::args().any(|a| a == "--export");

    // -----------------------------------------------------------------------
    // Run 6DOF simulation with GNC
    // -----------------------------------------------------------------------
    let (trajectory, _commands) = sim::simulate(&mission, &config);

    // -----------------------------------------------------------------------
    // Analyze
    // -----------------------------------------------------------------------
    let apogee = trajectory
        .iter()
        .max_by(|a, b| a.pos.z.partial_cmp(&b.pos.z).unwrap())
        .unwrap();

    let summary = FlightSummary::from_trajectory(&trajectory);
    let final_state = trajectory.last().unwrap();

    // Find staging event
    let staging_time = trajectory
        .windows(2)
        .find(|w| w[0].stage_idx != w[1].stage_idx)
        .map(|w| w[1].time);

    // -----------------------------------------------------------------------
    // Export if requested
    // -----------------------------------------------------------------------
    if export {
        let csv_path = "trajectory.csv";
        let json_path = "flight_summary.json";
        rocket_sim::io::csv::write_trajectory_file(csv_path, &trajectory)
            .expect("Failed to write CSV");
        rocket_sim::io::json::write_summary_file(json_path, &mission, &summary)
            .expect("Failed to write JSON");
        println!("Exported: {} and {}", csv_path, json_path);
    }

    // -----------------------------------------------------------------------
    // Print
    // -----------------------------------------------------------------------
    println!();
    println!("====================================================================");
    println!("  6DOF FLIGHT SIMULATION — {}", mission.name);
    println!("====================================================================");
    println!();

    for (i, stage) in mission.stages.iter().enumerate() {
        println!("  Stage {} — {}", i + 1, stage.name);
        println!("  ──────────────────────────────────────────────────────────────────");
        println!(
            "  Mass: {:.0}+{:.0} kg  Thrust: {:.0} N  Isp: {:.0} s  Burn: {:.1} s",
            stage.dry_mass,
            stage.propellant_mass,
            stage.thrust,
            stage.isp,
            stage.burn_time()
        );
        println!(
            "  Inertia: [{:.1}, {:.1}, {:.1}]  TVC max: {:.1} deg  CP offset: {:.2} m",
            stage.inertia.x,
            stage.inertia.y,
            stage.inertia.z,
            stage.tvc_max.to_degrees(),
            stage.cp_offset
        );
        println!();
    }

    println!("  Total mass: {:.0} kg   Total dv: {:.0} m/s", mission.total_mass(), mission.total_delta_v());
    println!();

    println!("  Flight Events");
    println!("  ──────────────────────────────────────────────────────────────────");
    if let Some(t) = staging_time {
        let st = trajectory.iter().find(|s| s.time >= t).unwrap();
        println!(
            "  STAGING   t={:>6.1}s   alt={:>8.0}m   vel={:>7.1}m/s   mass={:.1}kg",
            t, st.pos.z, st.vel.norm(), st.mass
        );
    }
    println!(
        "  APOGEE    t={:>6.1}s   alt={:>8.0}m   vel={:>7.1}m/s   pitch={:.1} deg",
        apogee.time,
        apogee.pos.z,
        apogee.vel.norm(),
        apogee.pitch().to_degrees()
    );
    println!(
        "  LANDING   t={:>6.1}s   vel={:>7.1}m/s",
        final_state.time,
        final_state.vel.norm()
    );
    println!();

    println!("  Performance");
    println!("  ──────────────────────────────────────────────────────────────────");
    println!(
        "  Max altitude:  {:>8.0} m   ({:.2} km)",
        summary.apogee_m,
        summary.apogee_m / 1000.0
    );
    println!(
        "  Max speed:     {:>8.1} m/s (Mach {:.2})",
        summary.max_speed,
        summary.max_mach
    );
    println!(
        "  Max accel:     {:>8.1} m/s^2 ({:.1} g)",
        summary.max_accel,
        summary.max_accel_g
    );
    println!(
        "  Max AoA:       {:>8.2} deg",
        trajectory.iter().map(|s| s.alpha().to_degrees()).fold(0.0_f64, f64::max)
    );
    println!(
        "  Flight time:   {:>8.1} s",
        summary.flight_time
    );
    println!();

    // -----------------------------------------------------------------------
    // Trajectory table
    // -----------------------------------------------------------------------
    println!("  Trajectory (6DOF)");
    println!("  ──────────────────────────────────────────────────────────────────────────────");
    println!(
        "  {:>6}  {:>9}  {:>8}  {:>7}  {:>7}  {:>7}  {:>5}  {:>5}",
        "t(s)", "alt(m)", "vel(m/s)", "Mach", "pitch", "AoA", "stg", "phase"
    );
    println!("  {}", "─".repeat(72));

    let sample_interval = (trajectory.len() / 40).max(1);
    for (i, s) in trajectory.iter().enumerate() {
        let is_staging = staging_time.map_or(false, |t| (s.time - t).abs() < config.dt * 2.0);
        let print = i % sample_interval == 0
            || i == 0
            || is_staging
            || i == trajectory.len() - 1;

        if !print {
            continue;
        }

        let speed = s.vel.norm();
        let mach = speed / atmosphere::isa(s.pos.z.max(0.0)).sound_speed;
        let phase = if s.stage_idx < mission.stages.len() {
            let st = &mission.stages[s.stage_idx];
            let upper: f64 = mission.stages[s.stage_idx + 1..].iter().map(|x| x.total_mass()).sum();
            if s.mass - st.dry_mass - upper > 0.01 {
                "BURN"
            } else if s.vel.z > 0.0 {
                "COAST"
            } else {
                "DESC"
            }
        } else if s.vel.z > 0.0 {
            "COAST"
        } else {
            "DESC"
        };

        println!(
            "  {:>6.1}  {:>9.0}  {:>8.1}  {:>7.3}  {:>6.1}\u{00b0}  {:>6.2}\u{00b0}  {:>5}  {:>5}",
            s.time,
            s.pos.z,
            speed,
            mach,
            s.pitch().to_degrees(),
            s.alpha().to_degrees(),
            s.stage_idx + 1,
            phase
        );
    }

    println!();
    println!(
        "  Simulation: {} steps, dt={} s, 6DOF + GNC",
        trajectory.len(),
        config.dt
    );
    println!("  Run `cargo run --bin rocket-viz --features viz` for real-time visualization");
    if !export {
        println!("  Run `cargo run -- --export` to save trajectory.csv and flight_summary.json");
    }
    println!("====================================================================");
    println!();
}
