use kiss3d::nalgebra::Vector3;
use std::f64::consts::PI;

pub struct Particle {
    phase: f64,
    velocity: f64,
}

impl Particle {
    fn new() -> Self {
        Particle {
            phase: 0.0,
            velocity: 4.0,
        }
    }

    pub fn coord(&self, time: f64) -> Vector3<f64> {
        let mean_anomaly = (self.phase + self.velocity * time).rem_euclid(2.0 * PI);

        let ecc: f64 = 0.25;
        let semi_major: f64 = 0.2;

        let true_anomaly = mean_anomaly
            + (2.0 * ecc - 0.25 * ecc.powi(3)) * mean_anomaly.sin()
            + 1.25 * ecc.powi(2) * (2.0 * mean_anomaly).sin()
            + 13.0 / 12.0 * ecc.powi(3) * (3.0 * mean_anomaly).sin();

        let radius = semi_major * (1.0 - ecc.powi(2)) / (1.0 + ecc * true_anomaly.cos());

        Vector3::new(
            radius * true_anomaly.cos(),
            radius * true_anomaly.sin(),
            0.0,
        )
    }
}

pub struct System {
    pub particles: Vec<Particle>,
}

impl System {
    pub fn new(n_particles: usize) -> Self {
        let mut particles = vec![];
        for _ in 0..n_particles {
            particles.push(Particle::new());
        }

        System { particles }
    }
}
