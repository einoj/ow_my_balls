use rapier2d::prelude::*;
use raylib::prelude::*;

const LINE_WIDTH: i32 = 15;
const TILE_SIZE: i32 = 40;
const TILE_SIZE_F: f32 = TILE_SIZE as f32;

const GROUND_H: f32 = 1.0;
const GROUND_W: f32 = 100.0;

fn render_2d_player(d: &mut RaylibDrawHandle, position: Vector2) {
    d.draw_circle_v(
        position.scale_by(TILE_SIZE_F)+TILE_SIZE_F/2.0,
        0.2*TILE_SIZE as f32, Color::ORANGE);
}

fn render_world(d: &mut RaylibDrawHandle) {
    d.draw_rectangle_v(
        Vector2::new(0.0,LINE_WIDTH as f32).scale_by(TILE_SIZE_F)-TILE_SIZE_F/2.0,
        Vector2::new(GROUND_W, GROUND_H).scale_by(TILE_SIZE_F)+TILE_SIZE_F/2.0,
        Color::ORANGE);
    d.draw_rectangle_v(
        Vector2::new(0.0,0.0),
        Vector2::new(0.2, TILE_SIZE_F).scale_by(TILE_SIZE_F)+TILE_SIZE_F/2.0,
        Color::ORANGE);
    d.draw_rectangle_v(
        Vector2::new(TILE_SIZE_F-3.7,-1.0).scale_by(TILE_SIZE_F)+TILE_SIZE_F/2.0,
        Vector2::new(0.2, TILE_SIZE_F).scale_by(TILE_SIZE_F)+TILE_SIZE_F/2.0,
        Color::ORANGE);
}

fn main() {
    let mut rigid_body_set = RigidBodySet::new();
    let mut collider_set = ColliderSet::new();

    /* Create the ground. */
    let collider = ColliderBuilder::cuboid(GROUND_W, GROUND_H)
       .translation(vector![0.0, LINE_WIDTH as f32])
       .build();
    /* walls */
    let collider_wall_l = ColliderBuilder::cuboid(0.2, TILE_SIZE_F)
       .build();
    let collider_wall_r = ColliderBuilder::cuboid(0.2, TILE_SIZE_F)
       .translation(vector![TILE_SIZE_F-3.5, 0.0])
       .build();
        
    collider_set.insert(collider);
    collider_set.insert(collider_wall_l);
    collider_set.insert(collider_wall_r);

    /* Create the bouncing ball. */
    for i in 0..9001 {
        let rigid_body = RigidBodyBuilder::dynamic()
            .translation(vector![LINE_WIDTH as f32+0.0001*i as f32, 0.0-i as f32])
            .build();
        let collider = ColliderBuilder::ball(0.2).restitution(0.9).build();
        let ball_body_handle = rigid_body_set.insert(rigid_body);
        collider_set.insert_with_parent(collider, ball_body_handle, &mut rigid_body_set);
    }

    /* Create other structures necessary for the simulation. */
    let gravity = vector![0.0, 9.81];
    let integration_parameters = IntegrationParameters::default();
    let mut physics_pipeline = PhysicsPipeline::new();
    let mut island_manager = IslandManager::new();
    let mut broad_phase = BroadPhase::new();
    let mut narrow_phase = NarrowPhase::new();
    let mut impulse_joint_set = ImpulseJointSet::new();
    let mut multibody_joint_set = MultibodyJointSet::new();
    let mut ccd_solver = CCDSolver::new();
    let physics_hooks = ();
    let event_handler = ();

    /* raylib stuff */
    let window_length: i32 = LINE_WIDTH*TILE_SIZE;
    let window_width: i32 = window_length + (LINE_WIDTH * 60);
    let (mut rl, thread) = raylib::init()
        .size(window_width, window_length)
        .title("bouncy ball")
        .build();

    /* Run the game loop, stepping the simulation once per frame. */
    let mut lasttime = rl.get_time();
    while !rl.window_should_close() {
        let now = rl.get_time();
        if now - lasttime > 1.0/50.0 {
            lasttime = now;
            physics_pipeline.step(
                &gravity,
                &integration_parameters,
                &mut island_manager,
                &mut broad_phase,
                &mut narrow_phase,
                &mut rigid_body_set,
                &mut collider_set,
                &mut impulse_joint_set,
                &mut multibody_joint_set,
                &mut ccd_solver,
                None,
                &physics_hooks,
                &event_handler,
                );
            let mut d = rl.begin_drawing(&thread);
            d.clear_background(Color::BLACK);
            render_world(&mut d);
            for handle in island_manager.active_dynamic_bodies() {
                let ball_body = &rigid_body_set[*handle];
                render_2d_player(&mut d, Vector2::new(
                        ball_body.translation().x, ball_body.translation().y));
            }
        }
    }
}
