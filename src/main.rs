use rapier2d::prelude::*;
use raylib::prelude::*;
use raylib::ffi::MouseButton::MOUSE_LEFT_BUTTON;

const SCALE_FACTOR: f32 = 40.0;
const WINDOW_HEIGHT: i32 = 600;
const WINDOW_HEIGHT_F: f32 = WINDOW_HEIGHT as f32;
const WINDOW_WIDTH: i32 = 1500;
const WINDOW_WIDTH_F: f32 = WINDOW_WIDTH as f32;

const GROUND_H: f32 = 1.0;
const GROUND_W: f32 = 40.0;
const WALL_WIDTH: f32 = 15.0;

fn render_2d_player(d: &mut RaylibDrawHandle, position: Vector2, color: Color) {
    d.draw_circle_v(
        position.scale_by(WINDOW_HEIGHT_F/15.0),
        WINDOW_HEIGHT_F/75.0, color);
}

fn render_world(d: &mut RaylibDrawHandle) {
    d.draw_rectangle_v(
        Vector2::new(0.0,WINDOW_HEIGHT_F-SCALE_FACTOR),
        Vector2::new(WINDOW_WIDTH_F, GROUND_W),
        Color::ORANGE);
    d.draw_rectangle_v(
        Vector2::new(0.0,0.0),
        Vector2::new(WALL_WIDTH/2.0, WINDOW_HEIGHT_F),
        Color::ORANGE);
    d.draw_rectangle_v(
        Vector2::new(WINDOW_WIDTH_F - WALL_WIDTH/2.0,0.0),
        Vector2::new(WALL_WIDTH/2.0, WINDOW_HEIGHT_F),
        Color::ORANGE);
}

fn main() {
    let mut rigid_body_set = RigidBodySet::new();
    let mut collider_set = ColliderSet::new();

    /* Create the ground. */
    let collider = ColliderBuilder::cuboid(WINDOW_WIDTH_F, GROUND_H)
       .translation(vector![0.0, WINDOW_HEIGHT_F/SCALE_FACTOR])
       .build();
    /* walls */
    let collider_wall_l = ColliderBuilder::cuboid(0.2, 1000.0)
       .translation(vector![0.0, -1.0])
       .build();
    let collider_wall_r = ColliderBuilder::cuboid(0.2, 1000.0)
       .translation(vector![WINDOW_WIDTH_F/SCALE_FACTOR, 0.0])
       .build();
        
    collider_set.insert(collider);
    collider_set.insert(collider_wall_l);
    collider_set.insert(collider_wall_r);

    /* Create the bouncing ball. */
    for i in 0..9001 {
        let rigid_body = RigidBodyBuilder::dynamic()
            .translation(vector![WALL_WIDTH+0.0001*i as f32, 0.0-i as f32])
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

    let (mut rl, thread) = raylib::init()
        .size(WINDOW_WIDTH, WINDOW_HEIGHT)
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
            if rl.is_mouse_button_down(MOUSE_LEFT_BUTTON) {
                let rigid_body = RigidBodyBuilder::dynamic()
                    .translation(vector![rl.get_mouse_x() as f32/SCALE_FACTOR,
                                 rl.get_mouse_y() as f32/SCALE_FACTOR])
                    .linvel(vector![0.0, 80.0])
                    .user_data(42)
                    .build();
                let collider = ColliderBuilder::ball(0.2).restitution(0.9).mass(50.0).build();
                let ball_body_handle = rigid_body_set.insert(rigid_body);
                collider_set.insert_with_parent(collider, ball_body_handle, &mut rigid_body_set);
            }
            let mut d = rl.begin_drawing(&thread);
            d.clear_background(Color::BLACK);
            render_world(&mut d);
            for handle in island_manager.active_dynamic_bodies() {
                let ball_body = &rigid_body_set[*handle];
                let color: Color = match ball_body.user_data {
                    42 => Color::BLUE,
                    _ => Color::RED,
                };
                render_2d_player(&mut d, Vector2::new(
                        ball_body.translation().x, ball_body.translation().y),
                        color);
            }
        }
    }
}
