use rocket_sim::dynamics::state::SimConfig;
use rocket_sim::io::csv;
use rocket_sim::io::json::{self, FlightSummary};
use rocket_sim::sim;
use rocket_sim::vehicle::presets;

fn main() {
    let mission = presets::pathfinder();
    let config = SimConfig { dt: 0.005, max_time: 600.0 };

    println!("Simulating {} ...", mission.name);
    let (trajectory, _) = sim::simulate(&mission, &config);

    let summary = FlightSummary::from_trajectory(&trajectory);
    println!("Apogee: {:.1} km", summary.apogee_m / 1000.0);
    println!("Max speed: {:.1} m/s (Mach {:.2})", summary.max_speed, summary.max_mach);
    println!("Flight time: {:.1} s", summary.flight_time);

    csv::write_trajectory_file("pathfinder_trajectory.csv", &trajectory)
        .expect("Failed to write CSV");
    json::write_summary_file("pathfinder_summary.json", &mission, &summary)
        .expect("Failed to write JSON");

    println!("Exported: pathfinder_trajectory.csv, pathfinder_summary.json");
}
