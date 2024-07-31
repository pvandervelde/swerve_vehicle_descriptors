//! Provides an abstraction for the elements of the frame of a vehicle and the way they
//! connect with other elements.
//!
//! Each frame element is a combination of a [ReferenceFrame](frame_elements::ReferenceFrame) that
//! provides a coordinate system for the element, and the physical properties of the element as
//! captured by the [ChassisElement](frame_elements::ChassisElement). A frame element may have one
//! of:
//!
//! - a [Sensor](frame_elements::JointSensor) to record and publish the current position and orientation
//!   of the frame
//! - an [Actuator](frame_elements::Actuator) to provide motion, either linear or angular, to the
//!   frame
//! - a [JointConstraint](frame_elements::JointConstraint) to provide limitations on the motion of
//!   the frame.
//!
//! The [Model](model::MotionModel) stores all the frame elements and their relationships in a
//! directed tree.
//!
//! # Examples
//!
//! Create a model of a vehicle with 4 wheels. Each wheel has a suspension arm and a steering
//! system between the wheel and the vehicle body.
//!
//! ```
//! use std::{ f64::consts::PI, time::Duration };
//! use crossbeam_channel::{ Receiver, Sender };
//! use nalgebra::{ Matrix3, Matrix4, Matrix6, Translation3, UnitQuaternion, Vector3 };
//! use swerve_vehicle_descriptors::Error;
//! use swerve_vehicle_descriptors::change_notification_processing::{ ChangeID, HardwareChangeProcessor };
//! use swerve_vehicle_descriptors::hardware::actuator_interface::{ HardwareActuator, ActuatorAvailableRatesOfChange };
//! use swerve_vehicle_descriptors::hardware::joint_state::{ JointState, JointStateRange };
//! use swerve_vehicle_descriptors::number_space::NumberSpaceType;
//! use swerve_vehicle_descriptors::model_elements::frame_elements::{ Actuator, FrameDofType, FrameID, JointConstraint };
//! use swerve_vehicle_descriptors::model_elements::model::{ ChassisElementPhysicalProperties, MotionModel };
//!
//! // The following functions assume that they are creating a robot with the following layout:
//! //
//! // body - reference frame is assumed to be in the middle of all the parts
//! //   suspension-1 (left front)
//! //     steering-1
//! //       wheel-1
//! //   suspension-2 (left rear)
//! //     steering-1
//! //       wheel-1
//! //   suspension-3 (right rear)
//! //     steering-1
//! //       wheel-1
//! //   suspension-4 (right front)
//! //     steering-1
//! //       wheel-1
//! //
//! // The relative positions and orientations are as follows
//! //
//! // - suspension left front
//! //   - position relative to parent: (1.0, 0.5, 0.0)
//! //   - orientation relative to parent: 30 degree rotation around the z-axis
//! // - steering left front
//! //   - position relative to parent: (0.25, 0.0, -0.1)
//! //   - orientation relative to parent: -30 degree rotation around the z-axis
//! // - wheel left front
//! //   - position relative to parent: (0.0, 0.0, -0.1)
//! //   - orientation relative to parent: 0 degree
//!
//! struct MockHardwareActuator {
//!     receiver: Receiver<(JointState, ActuatorAvailableRatesOfChange)>,
//!     sender: Sender<(JointState, ActuatorAvailableRatesOfChange)>,
//!     command_sender: Sender<JointState>,
//!     update_sender: Option<Sender<ChangeID>>,
//!     id: Option<ChangeID>,
//! }
//!
//! impl HardwareActuator for MockHardwareActuator {
//!     fn actuator_motion_type(&self) -> NumberSpaceType {
//!         NumberSpaceType::LinearUnlimited
//!     }
//!
//!     fn current_state_receiver(
//!         &self,
//!     ) -> Result<Receiver<(JointState, ActuatorAvailableRatesOfChange)>, Error> {
//!         Ok(self.receiver.clone())
//!     }
//!
//!     fn command_sender(&self) -> Result<Sender<JointState>, Error> {
//!         Ok(self.command_sender.clone())
//!     }
//!
//!     fn on_change(&mut self, id: ChangeID, sender: Sender<ChangeID>) {
//!         self.id = Some(id);
//!         self.update_sender = Some(sender);
//!     }
//!
//!     fn actuator_range(&self) -> JointStateRange {
//!         todo!()
//!     }
//! }
//!
//! #[derive(Clone, Copy, Debug, PartialEq)]
//! enum DriveModulePosition {
//!     LeftFront,
//!     LeftRear,
//!     RightRear,
//!     RightFront,
//! }
//!
//! fn position_multipliers(relative_position: DriveModulePosition) -> (i32, i32, i32) {
//!     match relative_position {
//!         DriveModulePosition::LeftFront => (1, 1, 1),
//!         DriveModulePosition::LeftRear => (-1, 1, 1),
//!         DriveModulePosition::RightRear => (-1, -1, 1),
//!         DriveModulePosition::RightFront => (1, -1, 1),
//!     }
//! }
//!
//! fn frame_angles_in_degrees_for(relative_position: DriveModulePosition) -> (f64, f64) {
//!     match relative_position {
//!         DriveModulePosition::LeftFront => (30.0, -30.0),
//!         DriveModulePosition::LeftRear => (150.0, -150.0),
//!         DriveModulePosition::RightRear => (210.0, -210.0),
//!         DriveModulePosition::RightFront => (330.0, -330.0),
//!     }
//! }
//!
//! fn add_steering_to_model<'a>(
//!     model: &mut MotionModel,
//!     parent_id: &FrameID,
//!     position: DriveModulePosition,
//!     actuator: Actuator,
//! ) -> Result<FrameID, Error> {
//!     let (mul_x, mul_y, mul_z) = position_multipliers(position);
//!     let (_, angle) = frame_angles_in_degrees_for(position);
//!     let deg_to_rad = PI / 180.0;
//!
//!     let name = "steering".to_string();
//!
//!     let physical_properties = ChassisElementPhysicalProperties::new(
//!         1.0,
//!         Vector3::<f64>::identity(),
//!         Matrix3::<f64>::identity(),
//!         Matrix6::<f64>::identity(),
//!     );
//!
//!     model.add_steering_element(
//!         name,
//!         parent_id.clone(),
//!         Translation3::<f64>::new(0.25 * mul_x as f64, 0.0 * mul_y as f64, -0.1 * mul_z as f64),
//!         UnitQuaternion::<f64>::from_euler_angles(0.0, 0.0, angle * deg_to_rad),
//!         physical_properties,
//!         actuator,
//!     )
//! }
//!
//! fn add_suspension_to_model(
//!     model: &mut MotionModel,
//!     parent_id: &FrameID,
//!     position: DriveModulePosition,
//! ) -> Result<FrameID, Error> {
//!     let (mul_x, mul_y, mul_z) = position_multipliers(position);
//!     let (angle, _) = frame_angles_in_degrees_for(position);
//!     let deg_to_rad = PI / 180.0;
//!
//!     let name: String = "suspension".to_string();
//!
//!     let physical_properties = ChassisElementPhysicalProperties::new(
//!         1.0,
//!         Vector3::<f64>::identity(),
//!         Matrix3::<f64>::identity(),
//!         Matrix6::<f64>::identity(),
//!     );
//!
//!     model.add_suspension_element(
//!         name,
//!         FrameDofType::PrismaticZ,
//!         parent_id.clone(),
//!         Translation3::<f64>::new(1.0 * mul_x as f64, 0.5 * mul_y as f64, 0.0 * mul_z as f64),
//!         UnitQuaternion::<f64>::from_euler_angles(0.0, 0.0, angle * deg_to_rad),
//!         physical_properties,
//!         JointConstraint::new(),
//!     )
//! }
//!
//! fn add_wheel_to_model<'a>(
//!     model: &mut MotionModel,
//!     parent_id: &FrameID,
//!     actuator: Actuator,
//! ) -> Result<FrameID, Error> {
//!     let name = "wheel".to_string();
//!
//!     let physical_properties = ChassisElementPhysicalProperties::new(
//!         1.0,
//!         Vector3::<f64>::identity(),
//!         Matrix3::<f64>::identity(),
//!         Matrix6::<f64>::identity(),
//!     );
//!
//!     model.add_wheel(
//!         name,
//!         parent_id.clone(),
//!         Translation3::<f64>::new(0.0, 0.0, -0.1),
//!         UnitQuaternion::<f64>::identity(),
//!         physical_properties,
//!         actuator,
//!     )
//! }
//!
//! pub fn create_actuator(change_processor: &Box<HardwareChangeProcessor>) -> Actuator {
//!     let (sender, receiver) = crossbeam_channel::unbounded();
//!     let (cmd_sender, _cmd_receiver) = crossbeam_channel::unbounded();
//!     let mut hardware_actuator = MockHardwareActuator {
//!         receiver,
//!         sender,
//!         command_sender: cmd_sender,
//!         update_sender: None,
//!         id: None,
//!     };
//!
//!     Actuator::new(&mut hardware_actuator, change_processor).unwrap()
//! }
//!
//! pub fn create_model() -> Result<MotionModel, Error> {
//!     let mut model = MotionModel::new();
//!
//!     let physical_properties = ChassisElementPhysicalProperties::new(
//!         10.0,
//!         Vector3::<f64>::identity(),
//!         Matrix3::<f64>::identity(),
//!         Matrix6::<f64>::identity(),
//!     );
//!
//!     // Add the body of the vehicle
//!     let body_id = model.add_body(
//!         "my-cool-vehicle".to_string(),
//!         Translation3::<f64>::new(0.0, 0.0, 0.0),
//!         UnitQuaternion::<f64>::from_euler_angles(0.0, 0.0, 0.0),
//!         physical_properties,
//!     )?;
//!
//!     let change_processor = Box::new(HardwareChangeProcessor::new(1000));
//!
//!     // left front
//!     let lf_suspension_id = add_suspension_to_model(&mut model, &body_id, DriveModulePosition::LeftFront)?;
//!     let lf_steering_id = add_steering_to_model(&mut model, &lf_suspension_id, DriveModulePosition::LeftFront, create_actuator(&change_processor))?;
//!     let _ = add_wheel_to_model(&mut model, &lf_steering_id, create_actuator(&change_processor))?;
//!
//!     // left rear
//!     let lr_suspension_id = add_suspension_to_model(&mut model, &body_id, DriveModulePosition::LeftRear)?;
//!     let lr_steering_id = add_steering_to_model(&mut model, &lr_suspension_id, DriveModulePosition::LeftRear, create_actuator(&change_processor))?;
//!     let _ = add_wheel_to_model(&mut model, &lr_steering_id, create_actuator(&change_processor))?;
//!
//!     // right rear
//!     let rr_suspension_id = add_suspension_to_model(&mut model, &body_id, DriveModulePosition::RightRear)?;
//!     let rr_steering_id = add_steering_to_model(&mut model, &rr_suspension_id, DriveModulePosition::RightRear, create_actuator(&change_processor))?;
//!     let _ = add_wheel_to_model(&mut model, &rr_steering_id, create_actuator(&change_processor))?;
//!
//!     // right front
//!     let rf_suspension_id = add_suspension_to_model(&mut model, &body_id, DriveModulePosition::RightFront)?;
//!     let rf_steering_id = add_steering_to_model(&mut model, &rf_suspension_id, DriveModulePosition::RightFront, create_actuator(&change_processor))?;
//!     let _ = add_wheel_to_model(&mut model, &rf_steering_id, create_actuator(&change_processor))?;
//!
//!     Ok(model)
//! }
//!
//! ```

pub mod frame_elements;
pub mod model;
