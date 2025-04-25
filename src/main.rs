use std::collections::HashMap;
use std::time::Instant;

use kiss3d::light::Light;
use kiss3d::nalgebra::Translation3;
use kiss3d::window::Window;

mod system;

fn main() {
    let mut window = Window::new("galaxy");
    window.set_light(Light::StickToCamera);

    let mut system = system::System::new();

    let mut spheres = HashMap::new();
    for i in 0..system.size() {
        let mut sphere = window.add_sphere(1.0);
        sphere.set_color(0.5, 0.8, 1.0);
        spheres.insert(i, sphere);
    }

    let mut start = Instant::now();

    while window.render() {
        spheres.retain(|i, sphere| {
            if system.particle_active(*i) {
                let position = system.particle_coord(*i);
                sphere.set_local_translation(Translation3::from(position));
                let scale = 2.0 * system.particle_radius(*i);
                sphere.set_local_scale(scale, scale, scale);
                true
            } else {
                window.remove_node(sphere);
                false
            }
        });

        let elapsed = start.elapsed();
        start += elapsed;
        system.run(elapsed);
    }
}
