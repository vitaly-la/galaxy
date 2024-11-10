extern crate kiss3d;

use kiss3d::light::Light;
use kiss3d::nalgebra::Translation3;
use kiss3d::window::Window;

use system::System;

use std::time::Instant;

mod system;

fn main() {
    let system = System::new(1);
    let start = Instant::now();

    let mut window = Window::new("Galaxy.rs");
    let mut s = window.add_sphere(0.05);

    s.set_color(0.0, 1.0, 0.0);

    window.set_light(Light::StickToCamera);

    while window.render() {
        let elapsed = start.elapsed().as_secs_f64();
        let pos = system.particles[0].coord(elapsed);
        s.set_local_translation(Translation3::new(pos.x as f32, pos.y as f32, pos.z as f32));
    }
}
