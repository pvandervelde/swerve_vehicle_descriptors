use std::f64::consts::PI;

use criterion::{black_box, criterion_group, criterion_main, Criterion};
use crossbeam_channel::{Receiver, Sender};
use nalgebra::{Matrix3, Matrix6, Translation3, UnitQuaternion, Vector3};
use swerve_vehicle_descriptors::{
    change_notification_processing::{ChangeID, HardwareChangeProcessor},
    hardware::{
        actuator_interface::{ActuatorAvailableRatesOfChange, HardwareActuator},
        joint_state::{JointState, JointStateRange},
    },
    model_elements::{
        frame_elements::{Actuator, FrameDofType, FrameID, JointConstraint},
        model::MotionModel,
    },
    number_space::NumberSpaceType,
    Error,
};

criterion_group! {
    name = benches;
    config = Criterion::default();
    targets =
        motion_model_get_children,
        motion_model_get_steering_frame_for_wheel,
        motion_model_is_ancestor,
        motion_model_get_homogeneous_transform_to_parent,
        motion_model_get_homogeneous_transform_to_ancestor,
        motion_model_get_homogeneous_transform_to_body,
        motion_model_get_homogeneous_transform_between_frames,
}

criterion_main!(benches);

pub fn motion_model_get_children(c: &mut Criterion) {
    let model = create_model_and_fill();
    let wheel_ids = model.get_wheels().unwrap();

    c.bench_function("MotionModel::get_children", |b| {
        b.iter(|| model.get_children(black_box(wheel_ids.first().unwrap())));
    });
}

pub fn motion_model_get_homogeneous_transform_between_frames(c: &mut Criterion) {
    let model = create_model_and_fill();
    let wheel_ids = model.get_wheels().unwrap();

    c.bench_function(
        "MotionModel::get_homogeneous_transform_between_frames",
        |b| {
            b.iter(|| {
                model.get_homogeneous_transform_between_frames(
                    black_box(wheel_ids.first().unwrap()),
                    black_box(wheel_ids.get(1).unwrap()),
                )
            });
        },
    );
}

pub fn motion_model_get_homogeneous_transform_to_ancestor(c: &mut Criterion) {
    let model = create_model_and_fill();
    let wheel_ids = model.get_wheels().unwrap();
    let body_id = model.get_body().unwrap();

    c.bench_function("MotionModel::get_homogeneous_transform_to_ancestor", |b| {
        b.iter(|| {
            model.get_homogeneous_transform_to_ancestor(
                black_box(wheel_ids.first().unwrap()),
                black_box(body_id),
            )
        });
    });
}

pub fn motion_model_get_homogeneous_transform_to_body(c: &mut Criterion) {
    let model = create_model_and_fill();
    let wheel_ids = model.get_wheels().unwrap();

    c.bench_function("MotionModel::get_homogeneous_transform_to_body", |b| {
        b.iter(|| model.get_homogeneous_transform_to_body(black_box(wheel_ids.first().unwrap())));
    });
}

pub fn motion_model_get_homogeneous_transform_to_parent(c: &mut Criterion) {
    let model = create_model_and_fill();
    let wheel_ids = model.get_wheels().unwrap();

    c.bench_function("MotionModel::get_homogeneous_transform_to_parent", |b| {
        b.iter(|| model.get_homogeneous_transform_to_parent(black_box(wheel_ids.first().unwrap())));
    });
}

pub fn motion_model_get_steering_frame_for_wheel(c: &mut Criterion) {
    let model = create_model_and_fill();
    let wheel_ids = model.get_wheels().unwrap();

    c.bench_function("MotionModel::get_steering_frame_for_wheel", |b| {
        b.iter(|| model.get_steering_frame_for_wheel(black_box(wheel_ids.first().unwrap())));
    });
}

pub fn motion_model_is_ancestor(c: &mut Criterion) {
    let model = create_model_and_fill();
    let wheel_ids = model.get_wheels().unwrap();

    c.bench_function("MotionModel::is_ancestor", |b| {
        b.iter(|| {
            model.is_ancestor(
                black_box(wheel_ids.first().unwrap()),
                black_box(wheel_ids.get(1).unwrap()),
            )
        });
    });
}

fn create_model_and_fill() -> MotionModel {
    let change_processor = Box::new(HardwareChangeProcessor::new(10));

    let mut model = MotionModel::new();

    let body_id = add_body_to_model(&mut model).unwrap();

    add_drive_module(
        &mut model,
        &body_id,
        DriveModulePosition::LeftFront,
        &change_processor,
    );
    add_drive_module(
        &mut model,
        &body_id,
        DriveModulePosition::LeftRear,
        &change_processor,
    );
    add_drive_module(
        &mut model,
        &body_id,
        DriveModulePosition::RightRear,
        &change_processor,
    );
    add_drive_module(
        &mut model,
        &body_id,
        DriveModulePosition::RightFront,
        &change_processor,
    );

    model
}

//
// HELPER METHODS
//

// The following functions assume that they are creating a robot with the following layout:
//
// body - reference frame is assumed to be in the middle of all the parts
//   active_frame-1
//     suspension-1 (left front)
//       steering-1
//         wheel-1
//   active_frame-2
//     suspension-2 (left rear)
//       steering-1
//         wheel-1
//   active_frame-3
//     suspension-3 (right rear)
//       steering-1
//         wheel-1
//   active_frame-4
//     suspension-4 (right front)
//       steering-1
//         wheel-1
//
// The relative positions and orientations are as follows
//
// - suspension left front
//   - position relative to parent: (1.0, 0.5, 0.0)
//   - orientation relative to parent: 30 degree rotation around the z-axis
// - steering left front
//   - position relative to parent: (0.25, 0.0, -0.1)
//   - orientation relative to parent: -30 degree rotation around the z-axis
// - wheel left front
//   - position relative to parent: (0.0, 0.0, -0.1)
//   - orientation relative to parent: 0 degree

#[derive(Clone, Copy, Debug, PartialEq)]
enum DriveModulePosition {
    LeftFront,
    LeftRear,
    RightRear,
    RightFront,
}

// Defines the signs for the different potions of the drive modules. e.g. the left
// rear module is on the negative x-axis of the vehicle.
fn position_multipliers(relative_position: DriveModulePosition) -> (i32, i32, i32) {
    match relative_position {
        DriveModulePosition::LeftFront => (1, 1, 1),
        DriveModulePosition::LeftRear => (-1, 1, 1),
        DriveModulePosition::RightRear => (-1, -1, 1),
        DriveModulePosition::RightFront => (1, -1, 1),
    }
}

// Defines the frame angles for the suspension and the steering reference frames
fn frame_angles_in_degrees_for(relative_position: DriveModulePosition) -> (f64, f64) {
    match relative_position {
        DriveModulePosition::LeftFront => (30.0, -30.0),
        DriveModulePosition::LeftRear => (150.0, -150.0),
        DriveModulePosition::RightRear => (210.0, -210.0),
        DriveModulePosition::RightFront => (330.0, -330.0),
    }
}

struct MockHardwareActuator {
    receiver: Receiver<(JointState, ActuatorAvailableRatesOfChange)>,
    sender: Sender<(JointState, ActuatorAvailableRatesOfChange)>,
    command_sender: Sender<JointState>,
    update_sender: Option<Sender<ChangeID>>,
    id: Option<ChangeID>,
}

impl HardwareActuator for MockHardwareActuator {
    fn get_actuator_motion_type(&self) -> NumberSpaceType {
        NumberSpaceType::LinearUnlimited
    }

    fn get_current_state_receiver(
        &self,
    ) -> Result<Receiver<(JointState, ActuatorAvailableRatesOfChange)>, Error> {
        Ok(self.receiver.clone())
    }

    fn get_command_sender(&self) -> Result<Sender<JointState>, Error> {
        Ok(self.command_sender.clone())
    }

    fn on_change(&mut self, id: ChangeID, sender: Sender<ChangeID>) {
        self.id = Some(id);
        self.update_sender = Some(sender);
    }

    fn get_actuator_range(&self) -> JointStateRange {
        todo!()
    }
}

fn add_actuated_joint_to_model<'a>(
    model: &mut MotionModel,
    parent_id: &FrameID,
    position: DriveModulePosition,
    dof: FrameDofType,
    actuator: Actuator,
) -> Result<FrameID, Error> {
    let (mul_x, mul_y, mul_z) = position_multipliers(position);
    let (angle, _) = frame_angles_in_degrees_for(position);
    let deg_to_rad = PI / 180.0;

    let name = "actuated".to_string();
    let position_relative_to_parent =
        Translation3::<f64>::new(1.0 * mul_x as f64, 0.5 * mul_y as f64, 0.0 * mul_z as f64);
    let orientation_relative_to_parent =
        UnitQuaternion::<f64>::from_euler_angles(0.0, 0.0, angle * deg_to_rad);
    let mass = 1.0;
    let center_of_mass = Vector3::<f64>::identity();
    let moment_of_inertia = Matrix3::<f64>::identity();
    let spatial_inertia = Matrix6::<f64>::identity();

    model.add_actuated_chassis_element(
        name,
        dof,
        *parent_id,
        position_relative_to_parent,
        orientation_relative_to_parent,
        mass,
        center_of_mass,
        moment_of_inertia,
        spatial_inertia,
        actuator,
    )
}

fn add_body_to_model(model: &mut MotionModel) -> Result<FrameID, Error> {
    let name = "body".to_string();
    let position_relative_to_world = Translation3::<f64>::identity();
    let orientation_relative_to_world = UnitQuaternion::<f64>::identity();
    let mass = 1.0;
    let center_of_mass = Vector3::<f64>::identity();
    let moment_of_inertia = Matrix3::<f64>::identity();
    let spatial_inertia = Matrix6::<f64>::identity();

    model.add_body(
        name,
        position_relative_to_world,
        orientation_relative_to_world,
        mass,
        center_of_mass,
        moment_of_inertia,
        spatial_inertia,
    )
}

fn add_drive_module(
    model: &mut MotionModel,
    body_id: &FrameID,
    position: DriveModulePosition,
    change_processor: &Box<HardwareChangeProcessor>,
) {
    let actuator1 = create_actuator(change_processor);
    let extension_joint_id = add_actuated_joint_to_model(
        model,
        body_id,
        position,
        FrameDofType::PrismaticZ,
        actuator1.0,
    )
    .unwrap();
    extend_linear_actuator(&actuator1.1);

    let suspension_id = add_suspension_to_model(model, &extension_joint_id, position).unwrap();

    let actuator2 = create_actuator(change_processor);
    let steering_id = add_steering_to_model(model, &suspension_id, position, actuator2.0).unwrap();
    extend_angular_actuator(&actuator2.1);

    let actuator3 = create_actuator(change_processor);
    let _ = add_wheel_to_model(model, &steering_id, actuator3.0).unwrap();
    extend_angular_actuator(&actuator3.1);
}

fn add_steering_to_model<'a>(
    model: &mut MotionModel,
    parent_id: &FrameID,
    position: DriveModulePosition,
    actuator: Actuator,
) -> Result<FrameID, Error> {
    let (mul_x, mul_y, mul_z) = position_multipliers(position);
    let (_, angle) = frame_angles_in_degrees_for(position);
    let deg_to_rad = PI / 180.0;

    let name = "steering".to_string();
    let position_relative_to_parent =
        Translation3::<f64>::new(0.25 * mul_x as f64, 0.0 * mul_y as f64, -0.1 * mul_z as f64);
    let orientation_relative_to_parent =
        UnitQuaternion::<f64>::from_euler_angles(0.0, 0.0, angle * deg_to_rad);
    let mass = 1.0;
    let center_of_mass = Vector3::<f64>::identity();
    let moment_of_inertia = Matrix3::<f64>::identity();
    let spatial_inertia = Matrix6::<f64>::identity();

    model.add_steering_element(
        name,
        *parent_id,
        position_relative_to_parent,
        orientation_relative_to_parent,
        mass,
        center_of_mass,
        moment_of_inertia,
        spatial_inertia,
        actuator,
    )
}

fn add_suspension_to_model(
    model: &mut MotionModel,
    parent_id: &FrameID,
    position: DriveModulePosition,
) -> Result<FrameID, Error> {
    let (mul_x, mul_y, mul_z) = position_multipliers(position);
    let (angle, _) = frame_angles_in_degrees_for(position);
    let deg_to_rad = PI / 180.0;

    let name: String = "suspension".to_string();
    let position_relative_to_parent =
        Translation3::<f64>::new(1.0 * mul_x as f64, 0.5 * mul_y as f64, 0.0 * mul_z as f64);
    let orientation_relative_to_parent =
        UnitQuaternion::<f64>::from_euler_angles(0.0, 0.0, angle * deg_to_rad);
    let mass = 1.0;
    let center_of_mass = Vector3::<f64>::identity();
    let moment_of_inertia = Matrix3::<f64>::identity();
    let spatial_inertia = Matrix6::<f64>::identity();

    model.add_suspension_element(
        name,
        FrameDofType::PrismaticZ,
        *parent_id,
        position_relative_to_parent,
        orientation_relative_to_parent,
        mass,
        center_of_mass,
        moment_of_inertia,
        spatial_inertia,
        JointConstraint::new(),
    )
}

fn add_wheel_to_model<'a>(
    model: &mut MotionModel,
    parent_id: &FrameID,
    actuator: Actuator,
) -> Result<FrameID, Error> {
    let name = "wheel".to_string();

    // Assume that the steering is the
    let position_relative_to_parent = Translation3::<f64>::new(0.0, 0.0, -0.1);

    // Assume that the parent is the steering and it has the same orientation
    let orientation_relative_to_parent = UnitQuaternion::<f64>::identity();
    let mass = 1.0;
    let center_of_mass = Vector3::<f64>::identity();
    let moment_of_inertia = Matrix3::<f64>::identity();
    let spatial_inertia = Matrix6::<f64>::identity();

    model.add_wheel(
        name,
        *parent_id,
        position_relative_to_parent,
        orientation_relative_to_parent,
        mass,
        center_of_mass,
        moment_of_inertia,
        spatial_inertia,
        actuator,
    )
}

fn create_actuator(
    change_processor: &Box<HardwareChangeProcessor>,
) -> (Actuator, MockHardwareActuator) {
    let (sender, receiver) = crossbeam_channel::unbounded();
    let (cmd_sender, _cmd_receiver) = crossbeam_channel::unbounded();
    let mut hardware_actuator = MockHardwareActuator {
        receiver,
        sender,
        command_sender: cmd_sender,
        update_sender: None,
        id: None,
    };

    let actuator = Actuator::new(&mut hardware_actuator, change_processor).unwrap();
    (actuator, hardware_actuator)
}

fn extend_angular_actuator(hardware_actuator: &MockHardwareActuator) {
    // Push the actuator out
    let angle_x_deg = 30.0;
    let angle_x_rad = angle_x_deg * (PI / 180.0);
    let msg = (
        JointState::new(angle_x_rad, None, None, None),
        ActuatorAvailableRatesOfChange::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    );

    hardware_actuator.sender.send(msg).unwrap();
    hardware_actuator
        .update_sender
        .as_ref()
        .unwrap()
        .send(hardware_actuator.id.unwrap())
        .unwrap();
}

fn extend_linear_actuator(hardware_actuator: &MockHardwareActuator) {
    // Push the actuator out
    let angle_x_deg = 30.0;
    let _angle_x_rad = angle_x_deg * (PI / 180.0);
    let msg = (
        JointState::new(0.5, None, None, None),
        ActuatorAvailableRatesOfChange::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    );

    hardware_actuator.sender.send(msg).unwrap();
    hardware_actuator
        .update_sender
        .as_ref()
        .unwrap()
        .send(hardware_actuator.id.unwrap())
        .unwrap();
}
