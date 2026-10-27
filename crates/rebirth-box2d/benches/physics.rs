use criterion::{black_box, criterion_group, criterion_main, Criterion};
use rebirth_box2d::{Body, Joint, Vec2, World};

fn bench_pyramid_step(c: &mut Criterion) {
    let mut group = c.benchmark_group("step");

    group.bench_function("pyramid_78_boxes", |b| {
        let mut world = World::new(Vec2::new(0.0, -10.0), 10);
        let ground = Body::new_static(Vec2::new(100.0, 20.0)).with_position(Vec2::new(0.0, -10.0));
        world.add_body(ground);

        let mut x = Vec2::new(-6.0, 0.75);
        for i in 0..12 {
            let mut y = x;
            for _ in i..12 {
                let body = Body::new(Vec2::new(1.0, 1.0), 10.0).with_position(y);
                world.add_body(body);
                y += Vec2::new(1.125, 0.0);
            }
            x += Vec2::new(0.5625, 2.0);
        }

        // Warm up slightly
        for _ in 0..10 {
            world.step(1.0 / 60.0);
        }

        b.iter(|| {
            world.step(black_box(1.0 / 60.0));
        });
    });

    group.bench_function("suspension_bridge_15_planks", |b| {
        let mut world = World::new(Vec2::new(0.0, -10.0), 10);
        let ground = Body::new_static(Vec2::new(100.0, 20.0)).with_position(Vec2::new(0.0, -10.0));
        world.add_body(ground);

        let num_planks = 15;
        let mass = 50.0;
        let time_step = 1.0 / 60.0;

        for i in 0..num_planks {
            let plank = Body::new(Vec2::new(1.0, 0.25), mass)
                .with_position(Vec2::new(-8.5 + 1.25 * i as f32, 5.0));
            world.add_body(plank);
        }

        let frequency_hz = 2.0;
        let damping_ratio = 0.7;
        let omega = 2.0 * core::f32::consts::PI * frequency_hz;
        let d = 2.0 * mass * damping_ratio * omega;
        let k = mass * omega * omega;
        let softness = 1.0 / (d + time_step * k);
        let bias_factor = time_step * k / (d + time_step * k);

        for i in 0..num_planks {
            let next_idx = if i + 1 < num_planks { i + 2 } else { 0 };
            let joint = Joint::new(
                rebirth_box2d::BodyId(i + 1),
                rebirth_box2d::BodyId(next_idx),
                Vec2::new(-9.125 + 1.25 * i as f32, 5.0),
                &world.bodies,
            )
            .with_softness(softness)
            .with_bias_factor(bias_factor);
            world.add_joint(joint);
        }

        for _ in 0..10 {
            world.step(1.0 / 60.0);
        }

        b.iter(|| {
            world.step(black_box(1.0 / 60.0));
        });
    });

    group.finish();
}

criterion_group!(benches, bench_pyramid_step);
criterion_main!(benches);
