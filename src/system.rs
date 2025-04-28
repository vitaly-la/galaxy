use std::collections::HashMap;
use std::f64::consts::PI;
use std::time::Duration;

use kiss3d::nalgebra::{Rotation3, Vector3};

use ncollide3d::bounding_volume::ball_aabb;
use ncollide3d::broad_phase::DBVTBroadPhase;
use ncollide3d::na::Point;
use ncollide3d::pipeline::broad_phase::{BroadPhase, BroadPhaseInterferenceHandler};

use rand::Rng;

use rastro::orbits::{anomaly_eccentric_to_true, anomaly_true_to_mean, mean_motion_general};

const GRAVITY: f64 = 0.01;
const RADIUS: f64 = 0.005;

fn anomaly_mean_to_eccentric(mean_anomaly: f64, ecc: f64, _degrees: bool) -> f64 {
    let mut ecc_anomaly = if ecc < 0.8 { mean_anomaly } else { PI };
    for _ in 0..20 {
        ecc_anomaly -= (ecc_anomaly - ecc * ecc_anomaly.sin() - mean_anomaly) / (1.0 - ecc * ecc_anomaly.cos());
    }
    ecc_anomaly
}

fn anomaly_mean_to_true(mean_anomaly: f64, ecc: f64, degrees: bool) -> f64 {
    let ecc_anomaly = anomaly_mean_to_eccentric(mean_anomaly, ecc, degrees);
    anomaly_eccentric_to_true(ecc_anomaly, ecc, degrees)
}

fn collide(a: &Particle, b: &Particle, duration: Duration) -> Particle {
    let time = duration.as_secs_f64();

    let mass_a = a.radius.powi(3);
    let mass_b = b.radius.powi(3);
    let mass = mass_a + mass_b;
    let radius = mass.cbrt();

    let coord_a = a.coord(time);
    let coord_b = b.coord(time);
    let coord = (mass_a * coord_a + mass_b * coord_b) / mass;

    let momentum_a = mass_a * a.velocity(time);
    let momentum_b = mass_b * b.velocity(time);
    let momentum = momentum_a + momentum_b;
    let velocity = momentum / mass;

    let angle = coord.cross(&velocity).normalize();
    let almost_local_coord = Rotation3::rotation_between(&Vector3::y_axis(), &angle).unwrap().inverse() * coord;
    let almost_local_velocity = Rotation3::rotation_between(&Vector3::y_axis(), &angle).unwrap().inverse() * velocity;

    let ecc_vec = almost_local_velocity.cross(&almost_local_coord.cross(&almost_local_velocity)) / GRAVITY - almost_local_coord.normalize();
    let ecc = ecc_vec.norm();

    let mut true_anomaly = (ecc_vec.dot(&almost_local_coord) / ecc_vec.norm() / almost_local_coord.norm()).acos();
    if almost_local_coord.dot(&almost_local_velocity) < 0.0 {
        true_anomaly = 2.0 * PI - true_anomaly;
    }

    let semi_major = almost_local_coord.norm() * (1.0 + ecc * true_anomaly.cos()) / (1.0 - ecc * ecc);
    let mean_motion = mean_motion_general(semi_major, GRAVITY, false);

    let mean_anomaly = anomaly_true_to_mean(true_anomaly, ecc, false);
    let phase = (mean_anomaly - mean_motion * time).rem_euclid(2.0 * PI);

    let distance = semi_major * (1.0 - ecc * ecc) / (1.0 + ecc * true_anomaly.cos());
    let local_coord = Vector3::new(-distance * true_anomaly.cos(), 0.0, distance * true_anomaly.sin());
    let heading = 2.0 * PI - local_coord.angle(&almost_local_coord);

    Particle {
        radius,
        ecc,
        semi_major,
        phase,
        mean_motion,
        heading,
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
    fn new() -> Self {
        let mut rng = rand::rng();
        let ecc = 0.25 * rng.random::<f64>();
        let semi_major: f64 = 0.125 + 0.25 * rng.random::<f64>();
        let mean_motion = mean_motion_general(semi_major, GRAVITY, false);
        let angle =
            (Vector3::y() + Vector3::new(rng.random::<f64>() - 0.5, rng.random::<f64>() - 0.5, rng.random::<f64>() - 0.5)).normalize();

        Particle {
            radius: RADIUS,
            ecc,
            semi_major,
            phase: 2.0 * PI * rng.random::<f64>(),
            mean_motion,
            heading: 2.0 * PI * rng.random::<f64>(),
            angle,
        }
    }

    fn coord(&self, time: f64) -> Vector3<f64> {
        let mean_anomaly = (self.phase + self.mean_motion * time).rem_euclid(2.0 * PI);

        let true_anomaly = anomaly_mean_to_true(mean_anomaly, self.ecc, false);

        let radius = self.semi_major * (1.0 - self.ecc * self.ecc) / (1.0 + self.ecc * true_anomaly.cos());

        let local_coord = Vector3::new(-radius * true_anomaly.cos(), 0.0, radius * true_anomaly.sin());

        Rotation3::rotation_between(&Vector3::y_axis(), &self.angle).unwrap()
            * Rotation3::from_axis_angle(&Vector3::y_axis(), self.heading)
            * local_coord
    }

    fn velocity(&self, time: f64) -> Vector3<f64> {
        let mean_anomaly = (self.phase + self.mean_motion * time).rem_euclid(2.0 * PI);

        let true_anomaly = anomaly_mean_to_true(mean_anomaly, self.ecc, false);

        let term = self.mean_motion * self.semi_major / (1.0 - self.ecc * self.ecc).sqrt();

        let v_r = term * self.ecc * true_anomaly.sin();
        let v_t = term * (1.0 + self.ecc * true_anomaly.cos());

        let u_r = Vector3::new(-true_anomaly.cos(), 0.0, true_anomaly.sin());
        let u_t = Vector3::new(true_anomaly.sin(), 0.0, true_anomaly.cos());

        let local_velocity = u_r * v_r + u_t * v_t;

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
        for i in 0..500 {
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
                if self.particles[&a].ecc > 0.9 || self.particles[&b].semi_major < 0.1 {
                    self.particles.remove(&a);
                }
                self.particles.remove(&b);
            }
        }
    }
}
