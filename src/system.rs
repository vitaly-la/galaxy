use std::f64::consts::PI;

use kiss3d::nalgebra::{Rotation3, Vector3};

use rand::Rng;

fn ecc_from_mean(mean_anomaly: f64, ecc: f64, precision: f64) -> f64 {
    let mut ecc_anomaly = mean_anomaly;
    let mut old_value;
    loop {
        old_value = ecc_anomaly;
        ecc_anomaly = mean_anomaly + ecc * ecc_anomaly.sin();
        if (ecc_anomaly - old_value).abs() < precision {
            break;
        }
    }
    ecc_anomaly
}

struct Particle {
    radius: f64,
    phase: f64,
    velocity: f64,
    ecc: f64,
    semi_major: f64,
    euler_a: f64,
    euler_b: f64,
    euler_g: f64,
}

impl Particle {
    fn new() -> Self {
        let mut rng = rand::thread_rng();
        Particle {
            radius: 0.02,
            phase: 2.0 * PI * rng.gen::<f64>(),
            velocity: 2.5 + 2.5 * rng.gen::<f64>(),
            ecc: 0.75 * rng.gen::<f64>(),
            semi_major: 0.25,
            euler_a: 2.0 * PI * rng.gen::<f64>(),
            euler_b: 2.0 * PI * rng.gen::<f64>(),
            euler_g: 2.0 * PI * rng.gen::<f64>(),
        }
    }

    fn coord(&self, time: f64) -> Vector3<f64> {
        let mean_anomaly = (self.phase + self.velocity * time).rem_euclid(2.0 * PI);

        let ecc_anomaly = ecc_from_mean(mean_anomaly, self.ecc, 0.0001);

        let beta = self.ecc / (1.0 + (1.0 - self.ecc * self.ecc).sqrt());
        let true_anomaly = ecc_anomaly
            + 2.0 * (beta * ecc_anomaly.sin() / (1.0 - beta * ecc_anomaly.cos())).atan();

        let radius =
            self.semi_major * (1.0 - self.ecc.powi(2)) / (1.0 + self.ecc * true_anomaly.cos());

        let local_coord = Vector3::new(
            -radius * true_anomaly.cos(),
            0.0,
            radius * true_anomaly.sin(),
        );

        Rotation3::from_euler_angles(self.euler_a, self.euler_b, self.euler_g) * local_coord
    }
}

pub struct System {
    pub size: usize,
    particles: Vec<Particle>,
}

impl System {
    pub fn new(size: usize) -> Self {
        let mut particles = vec![];
        for _ in 0..size {
            particles.push(Particle::new());
        }

        System { size, particles }
    }

    pub fn particle_coord(&self, i: usize, time: f64) -> Vector3<f32> {
        self.particles[i].coord(time).map(|x| x as f32)
    }

    pub fn particle_radius(&self, i: usize) -> f32 {
        self.particles[i].radius as f32
    }
}
