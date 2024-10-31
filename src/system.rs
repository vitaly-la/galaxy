use kiss3d::nalgebra::Vector3;
use rand::rngs::ThreadRng;
use rand::Rng;

struct Particle {
    pos: Vector3<f64>,
    vel: Vector3<f64>,
}

impl Particle {
    fn new(rng: &mut ThreadRng) -> Self {
        Particle {
            pos: Vector3::new(rng.gen(), rng.gen(), rng.gen()),
            vel: Vector3::new(rng.gen(), rng.gen(), rng.gen()),
        }
    }
}

pub struct System {
    particles: Vec<Particle>,
}

impl System {
    pub fn new(n_particles: usize) -> Self {
        let mut rng = rand::thread_rng();

        let mut particles = Vec::<Particle>::new();
        for _ in 0..n_particles {
            particles.push(Particle::new(&mut rng));
        }

        System { particles }
    }
}
