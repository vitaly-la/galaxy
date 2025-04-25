use std::collections::HashMap;
use std::f64::consts::PI;
use std::time::Duration;

use kiss3d::nalgebra::{Rotation3, Vector3};

use ncollide3d::bounding_volume;
use ncollide3d::broad_phase::DBVTBroadPhase;
use ncollide3d::na;
use ncollide3d::na::Isometry3;
use ncollide3d::pipeline::broad_phase::{BroadPhase, BroadPhaseInterferenceHandler};
use ncollide3d::shape::Ball;

use rand::Rng;

const RADIUS: f64 = 0.01;

fn ecc_from_mean(mean_anomaly: f64, ecc: f64) -> f64 {
    let mut ecc_anomaly = mean_anomaly;
    for _ in 0..10 {
        ecc_anomaly = mean_anomaly + ecc * ecc_anomaly.sin();
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
        let mut rng = rand::rng();
        let ecc = 0.0;
        let semi_major: f64 = 0.25;
        let velocity = semi_major.powi(-2).cbrt() * (1.0 + 0.1 * rng.random::<f64>());
        Particle {
            radius: RADIUS,
            phase: 2.0 * PI * rng.random::<f64>(),
            velocity,
            ecc,
            semi_major,
            euler_a: 2.0 * PI * rng.random::<f64>(),
            euler_b: 2.0 * PI * rng.random::<f64>(),
            euler_g: 2.0 * PI * rng.random::<f64>(),
        }
    }

    fn coord(&self, time: f64) -> Vector3<f64> {
        let mean_anomaly = (self.phase + self.velocity * time).rem_euclid(2.0 * PI);

        let ecc_anomaly = ecc_from_mean(mean_anomaly, self.ecc);

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
    particles: HashMap<usize, Particle>,
    time: Duration,
}

impl System {
    pub fn new() -> Self {
        let mut particles = HashMap::new();
        for i in 0..100 {
            particles.insert(i, Particle::new());
        }

        System {
            particles,
            time: Duration::ZERO,
        }
    }

    pub fn size(&self) -> usize {
        self.particles.len()
    }

    pub fn particle_active(&self, i: usize) -> bool {
        self.particles.contains_key(&i)
    }

    pub fn particle_coord(&self, i: usize) -> Vector3<f32> {
        self.particles[&i]
            .coord(self.time.as_secs_f64())
            .map(|x| x as f32)
    }

    pub fn particle_radius(&self, i: usize) -> f32 {
        self.particles[&i].radius as f32
    }

    pub fn run(&mut self, elapsed: Duration) {
        self.time += elapsed;

        let mut bf = DBVTBroadPhase::new(RADIUS);
        for (i, particle) in self.particles.iter() {
            let coord = particle.coord(self.time.as_secs_f64());
            bf.create_proxy(
                bounding_volume::aabb(
                    &Ball::new(particle.radius),
                    &Isometry3::new(coord, na::zero()),
                ),
                i.clone(),
            );
        }

        struct Handler<'a> {
            intersections: Vec<(usize, usize)>,
            particles: &'a HashMap<usize, Particle>,
            time: Duration,
        }

        impl<'a> BroadPhaseInterferenceHandler<usize> for Handler<'a> {
            fn is_interference_allowed(&mut self, _: &usize, _: &usize) -> bool {
                true
            }

            fn interference_started(&mut self, a: &usize, b: &usize) {
                let p1_coord = self.particles[&a].coord(self.time.as_secs_f64());
                let p2_coord = self.particles[&b].coord(self.time.as_secs_f64());
                let distance = (p1_coord - p2_coord).norm();
                if distance < self.particles[&a].radius + self.particles[&b].radius {
                    self.intersections.push((*a, *b));
                }
            }

            fn interference_stopped(&mut self, _: &usize, _: &usize) {}
        }

        let mut handler = Handler {
            intersections: vec![],
            particles: &self.particles,
            time: self.time,
        };
        bf.update(&mut handler);

        for (i, j) in handler.intersections {
            if self.particles.contains_key(&i) && self.particles.contains_key(&j) {
                self.particles.get_mut(&i).unwrap().radius =
                    (self.particles[&i].radius.powi(3) + self.particles[&j].radius.powi(3)).cbrt();
                self.particles.remove(&j);
            }
        }
    }
}
