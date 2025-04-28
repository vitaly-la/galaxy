use std::collections::HashMap;
use std::f64::consts::PI;
use std::time::Duration;

use kiss3d::nalgebra::{Rotation3, Vector3};

use ncollide3d::bounding_volume::ball_aabb;
use ncollide3d::broad_phase::DBVTBroadPhase;
use ncollide3d::na::Point;
use ncollide3d::pipeline::broad_phase::{BroadPhase, BroadPhaseInterferenceHandler};

use rand::Rng;

const GRAVITY: f64 = 0.01;
const RADIUS: f64 = 0.02;

fn ecc_from_mean(mean_anomaly: f64, ecc: f64) -> f64 {
    let mut ecc_anomaly = if ecc < 0.8 { mean_anomaly } else { PI };
    for _ in 0..50 {
        ecc_anomaly -= (ecc_anomaly - ecc * ecc_anomaly.sin() - mean_anomaly) / (1.0 - ecc * ecc_anomaly.cos());
    }
    ecc_anomaly
}

fn collide(a: &Particle, b: &Particle, time: Duration) -> Particle {
    let mass_a = a.radius.powi(3);
    let mass_b = b.radius.powi(3);
    let mass = mass_a + mass_b;
    let radius = mass.cbrt();

    let coord_a = a.coord(time.as_secs_f64());
    let coord_b = b.coord(time.as_secs_f64());
    let coord = (mass_a * coord_a + mass_b * coord_b) / mass;

    let momentum_a = mass_a * a.velocity(time.as_secs_f64());
    let momentum_b = mass_b * b.velocity(time.as_secs_f64());
    let momentum = momentum_a + momentum_b;
    let velocity = momentum / mass;

    let angle = coord.cross(&velocity).normalize();
    let almost_local_coord = Rotation3::rotation_between(&Vector3::y_axis(), &angle).unwrap().inverse() * coord;
    let almost_local_velocity = Rotation3::rotation_between(&Vector3::y_axis(), &angle).unwrap().inverse() * velocity;

    _ = (almost_local_coord, almost_local_velocity);

    Particle {
        radius,
        ecc: a.ecc,
        semi_major: a.semi_major,
        phase: a.phase,
        mean_motion: a.mean_motion,
        heading: a.heading,
        angle,
    }
}

struct Particle {
    radius: f64,
    ecc: f64,
    semi_major: f64,
    phase: f64,
    mean_motion: f64,
    heading: f64,
    angle: Vector3<f64>,
}

impl Particle {
    fn new(i: usize) -> Self {
        let mut rng = rand::rng();
        _ = rng.random::<f64>();
        let ecc = 0.0;
        let semi_major: f64 = 0.25;
        let mean_motion = (GRAVITY / semi_major.powi(3)).sqrt();
        let angle = if i == 0 {
            (Vector3::y() + Vector3::x()).normalize()
        } else {
            (Vector3::y() - Vector3::x()).normalize()
        };

        Particle {
            radius: RADIUS,
            ecc,
            semi_major,
            phase: 0.0,
            mean_motion,
            heading: 0.0,
            angle,
        }
    }

    fn coord(&self, time: f64) -> Vector3<f64> {
        let mean_anomaly = (self.phase + self.mean_motion * time).rem_euclid(2.0 * PI);

        let ecc_anomaly = ecc_from_mean(mean_anomaly, self.ecc);

        let beta = self.ecc / (1.0 + (1.0 - self.ecc * self.ecc).sqrt());
        let true_anomaly = ecc_anomaly + 2.0 * (beta * ecc_anomaly.sin() / (1.0 - beta * ecc_anomaly.cos())).atan();

        let radius = self.semi_major * (1.0 - self.ecc.powi(2)) / (1.0 + self.ecc * true_anomaly.cos());

        let local_coord = Vector3::new(-radius * true_anomaly.cos(), 0.0, radius * true_anomaly.sin());

        Rotation3::rotation_between(&Vector3::y_axis(), &self.angle).unwrap()
            * Rotation3::from_axis_angle(&Vector3::y_axis(), self.heading)
            * local_coord
    }

    fn velocity(&self, time: f64) -> Vector3<f64> {
        let mean_anomaly = (self.phase + self.mean_motion * time).rem_euclid(2.0 * PI);

        let ecc_anomaly = ecc_from_mean(mean_anomaly, self.ecc);
        let ecc_dot = self.mean_motion / (1.0 - self.ecc * ecc_anomaly.cos());

        let beta = self.ecc / (1.0 + (1.0 - self.ecc * self.ecc).sqrt());

        let subterm = 1.0 - beta * ecc_anomaly.cos();
        let subterm_dot = beta * ecc_anomaly.sin() * ecc_dot;

        let term = beta * ecc_anomaly.sin() / subterm;
        let term_dot = beta * (ecc_anomaly.cos() * ecc_dot * subterm - ecc_anomaly.sin() * subterm_dot) / subterm.powi(2);

        let true_anomaly = ecc_anomaly + 2.0 * term.atan();
        let true_anomaly_dot = ecc_dot + 2.0 / (1.0 + term.powi(2)) * term_dot;

        let radius = self.semi_major * (1.0 - self.ecc.powi(2)) / (1.0 + self.ecc * true_anomaly.cos());
        let radius_dot = self.semi_major * (1.0 - self.ecc.powi(2)) * self.ecc * true_anomaly.sin() * true_anomaly_dot
            / (1.0 + self.ecc * true_anomaly.cos()).powi(2);

        let local_velocity = Vector3::new(
            -radius_dot * true_anomaly.cos() + radius * true_anomaly.sin() * true_anomaly_dot,
            0.0,
            radius_dot * true_anomaly.sin() + radius * true_anomaly.cos() * true_anomaly_dot,
        );

        Rotation3::rotation_between(&Vector3::y_axis(), &self.angle).unwrap()
            * Rotation3::from_axis_angle(&Vector3::y_axis(), self.heading)
            * local_velocity
    }
}

pub struct System {
    particles: HashMap<usize, Particle>,
    time: Duration,
}

impl System {
    pub fn new() -> Self {
        let mut particles = HashMap::new();
        for i in 0..2 {
            particles.insert(i, Particle::new(i));
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
        self.particles[&i].coord(self.time.as_secs_f64()).map(|x| x as f32)
    }

    pub fn particle_radius(&self, i: usize) -> f32 {
        self.particles[&i].radius as f32
    }

    pub fn run(&mut self, elapsed: Duration) {
        self.time += elapsed;

        let mut bf = DBVTBroadPhase::new(0.02 * RADIUS);
        for (i, particle) in self.particles.iter() {
            let coord = particle.coord(self.time.as_secs_f64());
            bf.create_proxy(ball_aabb(&Point::from(coord), particle.radius), i.clone());
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
                let a_coord = self.particles[&a].coord(self.time.as_secs_f64());
                let b_coord = self.particles[&b].coord(self.time.as_secs_f64());
                let distance = (a_coord - b_coord).norm();
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

        for (a, b) in handler.intersections {
            if self.particle_active(a) && self.particle_active(b) {
                *self.particles.get_mut(&a).unwrap() = collide(&self.particles[&a], &self.particles[&b], self.time);
                self.particles.remove(&b);
            }
        }
    }
}
