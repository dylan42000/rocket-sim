use crate::types::G0;

// ---------------------------------------------------------------------------
// ISA 1976 Standard Atmosphere (sea level to 86 km)
// ---------------------------------------------------------------------------

const R_AIR: f64 = 287.052_87; // specific gas constant for dry air, J/(kg·K)
const GAMMA: f64 = 1.4;        // ratio of specific heats

const T0: f64 = 288.15;        // sea-level temperature, K
const P0: f64 = 101_325.0;     // sea-level pressure, Pa

/// Atmospheric properties at a given geometric altitude.
#[derive(Debug, Clone, Copy)]
pub struct Atmo {
    pub density: f64,      // kg/m^3
    pub pressure: f64,     // Pa
    pub temperature: f64,  // K
    pub sound_speed: f64,  // m/s
}

/// ISA 1976 standard atmosphere model.
///
/// Piecewise temperature profile with 7 layers from 0-86 km.
/// Clamps negative altitudes to sea level; returns near-vacuum above 86 km.
pub fn isa(altitude_m: f64) -> Atmo {
    let h = altitude_m.max(0.0);

    let (temperature, pressure) = if h < 11_000.0 {
        // Troposphere: lapse -6.5 K/km
        gradient_layer(h, 0.0, T0, -0.0065, P0)
    } else if h < 20_000.0 {
        // Tropopause: isothermal 216.65 K
        isothermal_layer(h, 11_000.0, 216.65, 22_632.1)
    } else if h < 32_000.0 {
        // Stratosphere I: lapse +1.0 K/km
        gradient_layer(h, 20_000.0, 216.65, 0.001, 5_474.89)
    } else if h < 47_000.0 {
        // Stratosphere II: lapse +2.8 K/km
        gradient_layer(h, 32_000.0, 228.65, 0.0028, 868.019)
    } else if h < 51_000.0 {
        // Mesosphere I: isothermal 270.65 K
        isothermal_layer(h, 47_000.0, 270.65, 110.906)
    } else if h < 71_000.0 {
        // Mesosphere II: lapse -2.8 K/km
        gradient_layer(h, 51_000.0, 270.65, -0.0028, 66.9389)
    } else if h < 86_000.0 {
        // Mesosphere III: lapse -2.0 K/km
        gradient_layer(h, 71_000.0, 214.65, -0.002, 3.956_42)
    } else {
        // Above 86 km: exponential decay approximation
        let t = 186.87;
        let p = 0.3734 * (-0.000_15 * (h - 86_000.0)).exp();
        (t, p.max(0.0))
    };

    let density = if temperature > 0.0 {
        pressure / (R_AIR * temperature)
    } else {
        0.0
    };

    Atmo {
        density,
        pressure,
        temperature,
        sound_speed: (GAMMA * R_AIR * temperature).sqrt(),
    }
}

// ---------------------------------------------------------------------------
// Layer helpers
// ---------------------------------------------------------------------------

/// Gradient layer: T = T_base + lapse * (h - h_base)
fn gradient_layer(h: f64, h_base: f64, t_base: f64, lapse: f64, p_base: f64) -> (f64, f64) {
    let t = t_base + lapse * (h - h_base);
    let p = p_base * (t / t_base).powf(-G0 / (lapse * R_AIR));
    (t, p)
}

/// Isothermal layer: T = const, pressure decays exponentially
fn isothermal_layer(h: f64, h_base: f64, t: f64, p_base: f64) -> (f64, f64) {
    let p = p_base * ((-G0 / (R_AIR * t)) * (h - h_base)).exp();
    (t, p)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sea_level_standard_values() {
        let a = isa(0.0);
        assert!((a.temperature - 288.15).abs() < 0.01);
        assert!((a.pressure - 101_325.0).abs() < 1.0);
        assert!((a.density - 1.225).abs() < 0.001);
        assert!((a.sound_speed - 340.29).abs() < 0.1);
    }

    #[test]
    fn tropopause_11km() {
        let a = isa(11_000.0);
        assert!((a.temperature - 216.65).abs() < 0.5);
        assert!((a.pressure - 22_632.0).abs() < 100.0);
    }

    #[test]
    fn density_monotonically_decreases() {
        let rho_0 = isa(0.0).density;
        let rho_10k = isa(10_000.0).density;
        let rho_50k = isa(50_000.0).density;
        assert!(rho_0 > rho_10k);
        assert!(rho_10k > rho_50k);
        assert!(rho_50k > 0.0);
    }

    #[test]
    fn negative_altitude_clamps_to_sea_level() {
        let a = isa(-500.0);
        assert!((a.temperature - 288.15).abs() < 0.01);
    }

    #[test]
    fn near_vacuum_above_86km() {
        let a = isa(100_000.0);
        assert!(a.density < 1e-5);
        assert!(a.pressure < 1.0);
    }
}
