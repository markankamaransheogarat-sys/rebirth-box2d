use rebirth_box2d::{Body, Joint, Vec2, World};

#[test]
fn test_single_box_fall_and_rest() {
    let mut world = World::new(Vec2::new(0.0, -10.0), 10);

    // Ground platform at y = -10 with height = 20 -> top surface at y = 0
    let ground = Body::new_static(Vec2::new(100.0, 20.0)).with_position(Vec2::new(0.0, -10.0));
    world.add_body(ground);

    // 1x1 crate starting at y = 4.0
    let b = Body::new(Vec2::new(1.0, 1.0), 200.0).with_position(Vec2::new(0.0, 4.0));
    let crate_id = world.add_body(b);

    // Simulate 2 seconds at 60 Hz (120 steps)
    for _ in 0..120 {
        world.step(1.0 / 60.0);
    }

    let crate_body = world.body(crate_id);
    // Resting position: top of ground is 0.0, box half-height is 0.5.
    assert!(
        (crate_body.position.y - 0.5).abs() < 0.05,
        "Box position y was {}, expected ~0.5",
        crate_body.position.y
    );
    assert!(
        crate_body.velocity.length() < 0.05,
        "Box velocity was {}, expected resting ~0",
        crate_body.velocity.length()
    );
}

#[test]
fn test_vertical_stack_stability() {
    let mut world = World::new(Vec2::new(0.0, -10.0), 10);

    // Ground: top surface at y = 0
    let ground = Body::new_static(Vec2::new(100.0, 20.0)).with_position(Vec2::new(0.0, -10.0));
    world.add_body(ground);

    let mut box_ids = Vec::new();
    for i in 0..10 {
        let b = Body::new(Vec2::new(1.0, 1.0), 1.0)
            .with_position(Vec2::new(0.0, 0.51 + 1.05 * i as f32));
        box_ids.push(world.add_body(b));
    }

    // Simulate 300 steps (5 seconds)
    for _ in 0..300 {
        world.step(1.0 / 60.0);
    }

    // All boxes must still be upright and vertically stacked without collapsing
    for (i, &id) in box_ids.iter().enumerate() {
        let b = world.body(id);
        let expected_y = 0.5 + i as f32 * 1.0;
        assert!(
            (b.position.y - expected_y).abs() < 0.15,
            "Box {} at y = {}, expected {}",
            i,
            b.position.y,
            expected_y
        );
        assert!(
            b.position.x.abs() < 0.25,
            "Box {} drift x = {}, expected < 0.25",
            i,
            b.position.x
        );
    }
}

#[test]
fn test_pendulum_joint_constraint() {
    let mut world = World::new(Vec2::new(0.0, -10.0), 10);

    // Static ceiling anchor
    let ceiling = Body::new_static(Vec2::new(10.0, 1.0)).with_position(Vec2::new(0.0, 10.0));
    let ceiling_id = world.add_body(ceiling);

    // Bob at (5.0, 10.0), initial horizontal offset of 5 units
    let bob = Body::new(Vec2::new(1.0, 1.0), 10.0).with_position(Vec2::new(5.0, 10.0));
    let bob_id = world.add_body(bob);

    // Pin joint at (0.0, 10.0)
    let joint = Joint::new(ceiling_id, bob_id, Vec2::new(0.0, 10.0), &world.bodies);
    world.add_joint(joint);

    let initial_dist = 5.0;

    // Simulate 120 steps (2 seconds of swinging)
    for _ in 0..120 {
        world.step(1.0 / 60.0);
        let pos = world.body(bob_id).position;
        let dist = (pos - Vec2::new(0.0, 10.0)).length();
        // Distance should remain approximately 5.0 throughout the swing
        assert!(
            (dist - initial_dist).abs() < 0.08,
            "Pendulum length stretched to {}, expected ~5.0",
            dist
        );
    }
}

#[test]
fn test_pyramid_stability() {
    let mut world = World::new(Vec2::new(0.0, -10.0), 10);

    let ground = Body::new_static(Vec2::new(100.0, 20.0)).with_position(Vec2::new(0.0, -10.0));
    world.add_body(ground);

    // 5-tier pyramid (15 boxes)
    let mut x = Vec2::new(-3.0, 0.75);
    let mut count = 0;

    for i in 0..5 {
        let mut y = x;
        for _ in i..5 {
            let b = Body::new(Vec2::new(1.0, 1.0), 10.0).with_position(y);
            world.add_body(b);
            count += 1;
            y += Vec2::new(1.125, 0.0);
        }
        x += Vec2::new(0.5625, 1.5);
    }

    assert_eq!(count, 15);

    // Run for 120 steps
    for _ in 0..120 {
        world.step(1.0 / 60.0);
    }

    assert_eq!(world.bodies.len(), 16);
    for b in &world.bodies[1..] {
        assert!(b.position.y > 0.45 && b.position.y < 10.0);
        assert!(b.velocity.length() < 1.0);
    }
}

#[test]
fn test_friction_ramp_sliding() {
    let mut world = World::new(Vec2::new(0.0, -10.0), 10);

    // Inclined static ramp at 20 degrees
    let ramp_angle = -0.35; // radians (~20 deg)
    let mut ramp = Body::new_static(Vec2::new(20.0, 1.0))
        .with_position(Vec2::new(0.0, 5.0))
        .with_rotation(ramp_angle);
    ramp.friction = 0.5;
    world.add_body(ramp);

    // Slick box (friction 0.0) placed on ramp
    let slick = Body::new(Vec2::new(1.0, 1.0), 10.0)
        .with_position(Vec2::new(0.0, 7.0))
        .with_friction(0.0);
    let slick_id = world.add_body(slick);

    // High-friction box (friction 0.9) placed on ramp
    let sticky = Body::new(Vec2::new(1.0, 1.0), 10.0)
        .with_position(Vec2::new(-3.0, 8.0))
        .with_friction(0.9);
    let sticky_id = world.add_body(sticky);

    for _ in 0..120 {
        world.step(1.0 / 60.0);
    }

    let slick_body = world.body(slick_id);
    let sticky_body = world.body(sticky_id);

    // Slick box must have accelerated down the ramp (x increased / y decreased)
    assert!(
        slick_body.position.x > 3.0,
        "Slick box should slide down the ramp, x was {}",
        slick_body.position.x
    );
    // Sticky box must have stayed near its starting point
    assert!(
        (sticky_body.position.x - (-3.0)).abs() < 1.5,
        "Sticky box slid too much, x was {}",
        sticky_body.position.x
    );
}
