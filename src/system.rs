use kiss3d::nalgebra::Vector3;
use std::f64::consts::PI;

struct Particle {
    phase: f64,
    velocity: f64,
}

impl Particle {
    fn new() -> Self {
        Particle {
            phase: 0.0,
            velocity: 1.0,
        }
    }

    fn coord(&self, time: f64) -> Vector3<f64> {
        let mean_anomaly = (self.phase + self.velocity * time).rem_euclid(2.0 * PI);

        let ecc = 0_f64;

        let true_anomaly = mean_anomaly
            + (2.0 * ecc - 0.25 * ecc.powi(3)) * mean_anomaly.sin()
            + 1.25 * ecc.powi(2) * (2.0 * mean_anomaly).sin()
            + 13.0 / 12.0 * ecc.powi(3) * (3.0 * mean_anomaly).sin();

        Vector3::new(0.0, 0.0, 0.0)
    }
}

pub struct System {
    particles: Vec<Particle>,
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
