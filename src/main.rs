use rapier2d::prelude::*;
use macroquad::prelude::*;
use macroquad::miniquad::EventHandler;
use macroquad::input::utils::*;

const LEVEL_HEIGHT: f32 = 60.0;
const LEVEL_WIDTH: f32 = 100.0;

const WALL_WIDTH: f32 = 2.0;
pub struct TDtire {
    body: RigidBody,
    collider: Collider,
}

impl TDtire {
    const W: f32 = 2.0;
    const H: f32 = 4.5;

    pub fn new(x: f32, y: f32) -> Self {
       Self {
           body: RigidBodyBuilder::dynamic()
                .translation(vector![x, y])
                .linvel(vector![-20.0, -0.0])
                .can_sleep(false)
                .build(),
           collider: ColliderBuilder::cuboid(Self::H, Self::W/2.0)
               .restitution(0.9)
               .build(),
       }
    }
}

struct World {
    island_manager: IslandManager,
    rigid_body_set: RigidBodySet,
    collider_set: ColliderSet,
    step_physics_fn: Box<dyn FnMut(&mut IslandManager, &mut RigidBodySet, &mut ColliderSet) -> ()>,
    camera: Camera2D,
}

impl EventHandler for World {
    fn update(&mut self) {
        (self.step_physics_fn)(&mut self.island_manager, &mut self.rigid_body_set, &mut self.collider_set);
    }

    fn draw(&mut self) {
        clear_background(BLACK);
        
        // ground
        draw_rectangle(0.0, 0.0, LEVEL_WIDTH, WALL_WIDTH, ORANGE);
        // left wall
        draw_rectangle(0.0, 0.0, WALL_WIDTH, LEVEL_HEIGHT, ORANGE);
        // right wall
        draw_rectangle(LEVEL_WIDTH - WALL_WIDTH, 0.0, WALL_WIDTH, LEVEL_HEIGHT, ORANGE);
        // roof
        draw_rectangle(0.0, LEVEL_HEIGHT - WALL_WIDTH, LEVEL_WIDTH, WALL_WIDTH, ORANGE);

        for handle in self.island_manager.active_dynamic_bodies() {
            let tire_body = &self.rigid_body_set[*handle];
            draw_rectangle_lines(
                tire_body.translation().x, 
                tire_body.translation().y,
                2.0, 4.5, 1.0, RED);
        }
    }

    fn mouse_button_down_event(
        &mut self,
        button: MouseButton,
        x: f32,
        y: f32
    ) {
        match button {
            miniquad::MouseButton::Left => {
                let click_pos = self.camera.screen_to_world(Vec2 { x, y });
                let tire = TDtire::new(click_pos.x, click_pos.y);
                let ball_body_handle = self.rigid_body_set.insert(tire.body);
                self.collider_set.insert_with_parent(tire.collider, ball_body_handle, &mut self.rigid_body_set);
            },
            _ => {},
        }
    }
}

fn init_world(camera: Camera2D) -> World {
    let rigid_body_set = RigidBodySet::new();
    let mut collider_set = ColliderSet::new();

    let collider_ground = ColliderBuilder::cuboid(LEVEL_WIDTH * 0.5, WALL_WIDTH * 0.5)
        .translation(vector![LEVEL_WIDTH * 0.5, -WALL_WIDTH * 0.5])
        .build();
    let collider_wall_l = ColliderBuilder::cuboid(WALL_WIDTH * 0.5, LEVEL_HEIGHT * 0.5)
        .translation(vector![WALL_WIDTH * 0.5, LEVEL_HEIGHT * 0.5])
        .build();
    let collider_wall_r = ColliderBuilder::cuboid(WALL_WIDTH * 0.5, LEVEL_HEIGHT * 0.5)
        .translation(vector![LEVEL_WIDTH - WALL_WIDTH * 0.5, LEVEL_HEIGHT * 0.5])
        .build();
    let collider_roof = ColliderBuilder::cuboid(LEVEL_WIDTH * 0.5, WALL_WIDTH * 0.5)
        .translation(vector![LEVEL_WIDTH * 0.5, LEVEL_HEIGHT - WALL_WIDTH* 0.5])
        .build();

    collider_set.insert(collider_ground);
    collider_set.insert(collider_wall_l);
    collider_set.insert(collider_wall_r);
    collider_set.insert(collider_roof);


    /* Create other structures necessary for the simulation. */
    let gravity = vector![0.0, 0.0];
    let integration_parameters = IntegrationParameters::default();
    let mut physics_pipeline = PhysicsPipeline::new();
    let island_manager = IslandManager::new();
    let mut broad_phase = BroadPhase::new();
    let mut narrow_phase = NarrowPhase::new();
    let mut impulse_joint_set = ImpulseJointSet::new();
    let mut multibody_joint_set = MultibodyJointSet::new();
    let mut ccd_solver = CCDSolver::new();
    let physics_hooks = ();
    let event_handler = ();

    let step_physics_fn = Box::new(move |island_manager: &mut IslandManager, rigid_body_set: &mut RigidBodySet, collider_set: &mut ColliderSet| {
        physics_pipeline.step(
            &gravity,
            &integration_parameters,
            island_manager,
            &mut broad_phase,
            &mut narrow_phase,
            rigid_body_set,
            collider_set,
            &mut impulse_joint_set,
            &mut multibody_joint_set,
            &mut ccd_solver,
            None,
            &physics_hooks,
            &event_handler,
        );
    });

    World {
        island_manager,
        rigid_body_set,
        collider_set,
        step_physics_fn,
        camera,
    }
}

#[macroquad::main("Ow My Balls")]
async fn main() {
    let camera = Camera2D::from_display_rect(Rect::new(0., 0., LEVEL_WIDTH, LEVEL_HEIGHT));
    set_camera(&camera);
    let mut world = init_world(camera);
    let input_subscription = register_input_subscriber();
    /* Run the game loop, stepping the simulation once per frame. */
    loop {
        repeat_all_miniquad_input(&mut world, input_subscription);
        world.update();
        world.draw();
        next_frame().await
    }
}
