extern crate kiss3d;

use kiss3d::light::Light;
use kiss3d::nalgebra::Translation3;
use kiss3d::window::Window;

use system::System;

mod system;

fn main() {
    let mut system = System::new(1);

    let mut window = Window::new("Galaxy.rs");
    let mut s = window.add_sphere(0.1);

    s.set_color(0.0, 1.0, 0.0);

    window.set_light(Light::StickToCamera);

    while window.render() {
        s.set_local_translation(Translation3::new(0.2, 0.0, 0.0));
    }
}
