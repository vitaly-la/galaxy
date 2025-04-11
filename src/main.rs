mod system;

use std::time::Instant;

use kiss3d::light::Light;
use kiss3d::nalgebra::Translation3;
use kiss3d::window::Window;

use system::System;

fn main() {
    let system = System::new(10);
    let start = Instant::now();

    let mut window = Window::new("galaxy");
    window.set_light(Light::StickToCamera);

    let mut spheres = vec![];
    for i in 0..system.size {
        let mut sphere = window.add_sphere(system.particle_radius(i));
        sphere.set_color(0.0, 1.0, 0.0);
        spheres.push(sphere);
    }

    while window.render() {
        let elapsed = start.elapsed().as_secs_f64();
        for (i, sphere) in spheres.iter_mut().enumerate() {
            let position = system.particle_coord(i, elapsed);
            sphere.set_local_translation(Translation3::from(position));
        }
    }
}
