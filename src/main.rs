use rapier2d::prelude::*;
use macroquad::prelude::*;

const SCALE_FACTOR: f32 = 40.0;
const WINDOW_HEIGHT: i32 = 600;
const WINDOW_HEIGHT_F: f32 = WINDOW_HEIGHT as f32;
const WINDOW_WIDTH: i32 = 1500;
const WINDOW_WIDTH_F: f32 = WINDOW_WIDTH as f32;

const GROUND_H: f32 = 1.0;
const GROUND_W: f32 = 40.0;
const WALL_WIDTH: f32 = 15.0;

fn render_ball(x: f32, y: f32, color: Color) {
    draw_circle(
        x *(WINDOW_HEIGHT_F/15.0),
        y *(WINDOW_HEIGHT_F/15.0),
        WINDOW_HEIGHT_F/75.0, color);
}

fn render_world() {
    draw_rectangle(
        0.0,WINDOW_HEIGHT_F-SCALE_FACTOR,
        WINDOW_WIDTH_F, GROUND_W,
        ORANGE);
    draw_rectangle(
        0.0,0.0,
        WALL_WIDTH/2.0, WINDOW_HEIGHT_F,
        ORANGE);
    draw_rectangle(
        WINDOW_WIDTH_F - WALL_WIDTH/2.0,0.0,
        WALL_WIDTH/2.0, WINDOW_HEIGHT_F,
        ORANGE);
}

#[macroquad::main("Ow My Balls")]
async fn main() {
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

    /* Run the game loop, stepping the simulation once per frame. */
    loop {
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
        clear_background(BLACK);
        render_world();
        for handle in island_manager.active_dynamic_bodies() {
            let ball_body = &rigid_body_set[*handle];
            let color: Color = match ball_body.user_data {
                42 => BLUE,
                _ => RED,
            };
            render_ball(ball_body.translation().x, ball_body.translation().y, color);
        }
    }
}
