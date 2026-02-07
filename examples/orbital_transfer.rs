use rocket_sim::orbital::{self, KeplerianElements, OrbitalState};
use rocket_sim::physics::gravity::R_EARTH_ECI;

fn main() {
    println!("=== Hohmann Transfer: LEO → GEO ===\n");

    let r_leo = R_EARTH_ECI + 200_000.0;  // 200 km LEO
    let r_geo = 42_164_000.0;              // GEO radius (~35,786 km altitude)

    let transfer = orbital::hohmann(r_leo, r_geo);

    println!("LEO altitude: {:.0} km", (r_leo - R_EARTH_ECI) / 1000.0);
    println!("GEO altitude: {:.0} km", (r_geo - R_EARTH_ECI) / 1000.0);
    println!();
    println!("Delta-v 1 (raise apoapsis): {:.1} m/s", transfer.dv1);
    println!("Delta-v 2 (circularize):    {:.1} m/s", transfer.dv2);
    println!("Total delta-v:              {:.1} m/s", transfer.total_dv);
    println!("Transfer time:              {:.2} hours", transfer.transfer_time / 3600.0);
    println!();

    // Propagate a circular LEO orbit with J2 for a few orbits
    println!("=== LEO Orbit Propagation (with J2) ===\n");

    let orbit = KeplerianElements::circular(400_000.0, 51.6_f64.to_radians());
    let (pos, vel) = orbit.to_state_vector();
    let period = orbit.period();

    println!("Orbit: {:.0} km, {:.1} deg inclination", 400.0, 51.6);
    println!("Period: {:.1} min", period / 60.0);
    println!("Orbital speed: {:.1} m/s", vel.norm());
    println!();

    let initial = OrbitalState {
        time: 0.0,
        pos,
        vel,
    };

    // Propagate 3 orbits
    let duration = 3.0 * period;
    let traj = orbital::propagate_orbit(&initial, 1.0, duration, true);

    println!("Propagated {:.1} orbits ({:.0} seconds, {} steps)",
        duration / period, duration, traj.len());

    // Sample altitude at orbit boundaries
    let steps_per_orbit = (period / 1.0) as usize;
    for i in 0..=3 {
        let idx = (i * steps_per_orbit).min(traj.len() - 1);
        let s = &traj[idx];
        let elements = KeplerianElements::from_state_vector(&s.pos, &s.vel);
        println!(
            "  Orbit {}: alt={:.1} km, ecc={:.6}, inc={:.3} deg, RAAN={:.3} deg",
            i,
            s.altitude() / 1000.0,
            elements.ecc,
            elements.inc.to_degrees(),
            elements.raan.to_degrees(),
        );
    }

    println!();
    println!("Note: J2 causes RAAN regression (~-5 deg/day for ISS orbit)");
}
