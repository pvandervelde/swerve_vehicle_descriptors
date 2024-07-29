#![warn(missing_docs)]
#![warn(rustdoc::missing_crate_level_docs)]
#![warn(rustdoc::missing_doc_code_examples)]

//! This library contains the different elements needed to define a swerve (N wheel steering and N
//! wheel drive) vehicle.
//!
//! This library provides constructs to describe the geometry of the vehicle for the purposes of
//! calculating the kinematics or dynamics of the vehicle.
//!
//! # Usage
//!
//! This crate is available on [crates.io]() and can be used by adding `swerve_vehicle_descriptors`
//! to your dependencies in your project's `cargo.toml`
//!
//! ```toml
//! [dependencies]
//! swerve_vehicle_descriptors = "0.1"
//! ```
//!
//! # Example: Create the vehicle geometry
//!
//! One of the main tasks for this library is to create an abstract representation of the vehicle
//! with all the relevant moving parts, i.e. the parts that contribute to the motion of the vehicle.
//! This is done through the [`crate::model_elements::model::MotionModel`] struct.
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
//!     fn get_actuator_motion_type(&self) -> NumberSpaceType {
//!         NumberSpaceType::LinearUnlimited
//!     }
//!
//!     fn get_current_state_receiver(
//!         &self,
//!     ) -> Result<Receiver<(JointState, ActuatorAvailableRatesOfChange)>, Error> {
//!         Ok(self.receiver.clone())
//!     }
//!
//!     fn get_command_sender(&self) -> Result<Sender<JointState>, Error> {
//!         Ok(self.command_sender.clone())
//!     }
//!
//!     fn on_change(&mut self, id: ChangeID, sender: Sender<ChangeID>) {
//!         self.id = Some(id);
//!         self.update_sender = Some(sender);
//!     }
//!
//!     fn get_actuator_range(&self) -> JointStateRange {
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
//! fn add_steering_to_model(
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
//!
//! ```
//! # Example: Create a hardware sensor
//!
//! When creating the model of the vehicle there will be points where sensors are attached to the
//! structural elements, for instance to measure the extension or position of a joint. The
//! [`crate::hardware::sensor_interface::HardwareSensor`] trait provides an interface to get signals from a
//! physical sensor.
//!
//! ```
//! use crossbeam_channel::{Receiver, Sender};
//! use swerve_vehicle_descriptors::Error;
//! use swerve_vehicle_descriptors::change_notification_processing::ChangeID;
//! use swerve_vehicle_descriptors::hardware::sensor_interface::HardwareSensor;
//! use swerve_vehicle_descriptors::hardware::joint_state::{JointState, JointStateRange};
//! use swerve_vehicle_descriptors::number_space::NumberSpaceType;
//!
//! pub struct MySensor {
//!      // We keep a copy of the receiver side of the channel so that we can hand it out
//!      // when requested
//!      receiver: Receiver<(JointState)>,
//!      // when ever the state of the joint changes we send the new joint state through
//!      // the sender. And then also notify that the state has changed through
//!      // the 'update_sender'
//!      sender: Sender<JointState>,
//!      update_sender: Option<Sender<ChangeID>>,
//!      id: Option<ChangeID>,
//! }
//!
//! impl MySensor {
//!     pub fn state_changed(&self) {
//!         // Send the updated state of the joint to whoever is listening
//!         self.sender.send(
//!             JointState::new(
//!                 1.5,       // position in meters for a linear joint, or in radians in a revolute joint
//!                 Some(2.3), // velocity in meters per second for a linear joint, or radians per second for a revolute joint
//!                 Some(4.2), // acceleration in meters per second squared for a linear joint, or radians per second squared for a revolute joint
//!                 Some(8.1), // jerk in meters per second cubed for a linear joint, or radians per second cubed for a revolute joint
//!             ),
//!         ).unwrap();
//!
//!         // Let the world know that there is an updated state waiting
//!         let id = self.id.unwrap().clone();
//!         self.update_sender.as_ref().unwrap().send(id).unwrap();
//!     }
//!
//!     pub fn new() -> Self {
//!         let (sender, receiver) = crossbeam_channel::unbounded();
//!         Self {
//!             receiver,
//!             sender,
//!             update_sender: None,
//!             id: None,
//!         }
//!     }
//! }
//!
//! impl HardwareSensor for MySensor {
//!     fn get_current_state_receiver(&self) -> Result<Receiver<JointState>, Error> {
//!         Ok(self.receiver.clone())
//!     }
//!
//!     fn get_joint_motion_type(&self) -> NumberSpaceType {
//!          NumberSpaceType::LinearUnlimited
//!     }
//!
//!     fn get_joint_range(&self) -> JointStateRange {
//!         // This could be a constant value, or a value depending on the current position
//!         // of the joint
//!         let minimum = JointState::new(
//!             1.0, // position in meters for a linear joint, or in radians in a revolute joint
//!             Some(2.0), // velocity in meters per second for a linear joint, or radians per second for a revolute joint
//!             Some(4.0), // acceleration in meters per second squared for a linear joint, or radians per second squared for a revolute joint
//!             Some(8.0), // jerk in meters per second cubed for a linear joint, or radians per second cubed for a revolute joint
//!         );
//!         let maximum = JointState::new(
//!             2.0,
//!             Some(4.0),
//!             Some(8.0),
//!             Some(16.0),
//!         );
//!         JointStateRange::new(minimum, maximum)
//!     }
//!
//!     fn on_change(&mut self, id: ChangeID, notifier: Sender<ChangeID>) {
//!         self.id = Some(id);
//!         self.update_sender = Some(notifier);
//!     }
//! }
//!
//! ```
//!
//! # Example: Create a hardware actuator
//!
//! When creating the model of the vehicle there will be points where actuators are attached to the
//! structural elements, for instance to change the position of the suspension, or to turn the wheel.
//! The [`crate::hardware::actuator_interface::HardwareActuator`] trait provides an interface to get signals to and from a
//! physical actuator.
//!
//! Note that actuators can be prismatic or revolute.
//!
//! ```
//! use crossbeam_channel::{Receiver, Sender};
//! use swerve_vehicle_descriptors::Error;
//! use swerve_vehicle_descriptors::change_notification_processing::ChangeID;
//! use swerve_vehicle_descriptors::hardware::actuator_interface::{ HardwareActuator, ActuatorAvailableRatesOfChange };
//! use swerve_vehicle_descriptors::hardware::joint_state::{JointState, JointStateRange};
//! use swerve_vehicle_descriptors::number_space::NumberSpaceType;
//!
//! pub struct MyActuator {
//!     // We keep a copy of the receiver side of the channel so that we can hand it out
//!     // when requested
//!     receiver: Receiver<(JointState, ActuatorAvailableRatesOfChange)>,
//!     // when ever the state of the joint changes we send the new joint state through
//!     // the sender. And then also notify that the state has changed through
//!     // the 'update_sender'
//!     sender: Sender<(JointState, ActuatorAvailableRatesOfChange)>,
//!     update_sender: Option<Sender<ChangeID>>,
//!     id: Option<ChangeID>,
//!     // The channel sender used to send the desired state to the hardware
//!     command_sender: Sender<JointState>,
//!     command_receiver: Receiver<JointState>,
//! }
//!
//! impl MyActuator {
//!     pub fn state_changed(&self) {
//!         // Send the updated state of the joint to whoever is listening
//!         self.sender.send(
//!             (
//!                 JointState::new(
//!                     1.5,       // position in meters for a linear joint, or in radians in a revolute joint
//!                     Some(2.3), // velocity in meters per second for a linear joint, or radians per second for a revolute joint
//!                     Some(4.2), // acceleration in meters per second squared for a linear joint, or radians per second squared for a revolute joint
//!                     Some(8.1), // jerk in meters per second cubed for a linear joint, or radians per second cubed for a revolute joint
//!                 ),
//!                 ActuatorAvailableRatesOfChange::new(
//!                     -10.0,
//!                     10.0,
//!                     -5.0,
//!                     5.0,
//!                     -20.0,
//!                     20.0
//!                 ),
//!             )
//!         ).unwrap();
//!
//!         // Let the world know that there is an updated state waiting
//!         let id = self.id.unwrap().clone();
//!         self.update_sender.as_ref().unwrap().send(id).unwrap();
//!     }
//!
//!     pub fn new() -> Self {
//!         let (sender, receiver) = crossbeam_channel::unbounded();
//!         let (command_sender, command_receiver) = crossbeam_channel::unbounded();
//!         Self {
//!             receiver,
//!             sender,
//!             update_sender: None,
//!             id: None,
//!             command_sender,
//!             command_receiver,
//!         }
//!     }
//! }
//!
//! impl HardwareActuator for MyActuator {
//!     fn get_actuator_motion_type(&self) -> NumberSpaceType {
//!          NumberSpaceType::LinearUnlimited
//!     }
//!
//!     fn get_actuator_range(&self) -> JointStateRange {
//!         // This could be a constant value, or a value depending on the current position
//!         // of the joint
//!         let minimum = JointState::new(
//!             1.0,       // position in meters for a linear joint, or in radians in a revolute joint
//!             Some(2.0), // velocity in meters per second for a linear joint, or radians per second for a revolute joint
//!             Some(4.0), // acceleration in meters per second squared for a linear joint, or radians per second squared for a revolute joint
//!             Some(8.0), // jerk in meters per second cubed for a linear joint, or radians per second cubed for a revolute joint
//!         );
//!         let maximum = JointState::new(
//!             2.0,
//!             Some(4.0),
//!             Some(8.0),
//!             Some(16.0),
//!         );
//!
//!         JointStateRange::new(minimum, maximum)
//!     }
//!
//!     fn get_command_sender(&self) -> Result<Sender<JointState>, Error> {
//!         Ok(self.command_sender.clone())
//!     }
//!
//!     fn get_current_state_receiver(&self) -> Result<Receiver<(JointState, ActuatorAvailableRatesOfChange)>, Error> {
//!         Ok(self.receiver.clone())
//!     }
//!
//!     fn on_change(&mut self, id: ChangeID, notifier: Sender<ChangeID>) {
//!         self.id = Some(id);
//!         self.update_sender = Some(notifier);
//!     }
//! }
//! ```

use model_elements::frame_elements::FrameID;
use thiserror::Error;

pub mod change_notification_processing;
pub mod hardware;
pub mod number_space;

pub mod model_elements;

/// Defines the different errors for the swerve model crate.
#[derive(Debug, Error, PartialEq)]
#[non_exhaustive]
pub enum Error {
    /// Indicates that we failed to compute the transformation between two reference frames.
    #[error("Failed to compute the transform between {from:?} and {to:?}")]
    FailedToComputeTransform {
        /// The starting frame ID
        from: FrameID,
        /// The destination frame ID
        to: FrameID,
    },

    /// Indicates that we failed to get a joint state from an actuator.
    #[error("Failed to read the joint state for the given actuator.")]
    FailedToReadActuatorJointState,

    /// Indicates that we failed to set a joint state for a given actuator.
    #[error("Failed to set the joint state for the given actuator.")]
    FailedToSetActuatorJointState,

    /// Indicates that a user tried to add a frame element to a model or kinematic tree that
    /// already contains a frame element with the same ID.
    ///
    /// * 'id' - The ID of the frame element that already exists.
    #[error("A frame element with id {id:?} already exists in the collection.")]
    FrameElementAlreadyExists {
        /// The ID of the frame element that already exists.
        id: FrameID,
    },

    /// Indicates that a frame element or frame ID was provided that is not valid, e.g.
    /// not stored in the collection.
    #[error("The frame element with id {id:?} is not a valid element for the operation.")]
    InvalidFrameID {
        /// The ID of the frame element.
        id: FrameID,
    },

    /// Indicates that a frame element with a given ID was expected to exist, but it did not.
    #[error("Expected a frame element with id {id:?} to be present, but it was not.")]
    MissingFrameElement {
        /// The ID of the frame element.
        id: FrameID,
    },

    /// Indicates that there already is a frame in the chain of frame elements that is
    /// a steering frame.
    ///
    /// A chain of frame elements should have at maximum one wheel and one steering frame.
    #[error("A steering frame already exists in the parent chain. Each wheel to body chain is only allowed to have one steering frame in it.")]
    MultipleSteeringFramesInChain {
        /// The ID of the next steering frame in the chain.
        id: FrameID,
    },

    /// Indicates that there is no steering frame in the chain of frame elements from the wheel
    /// frame to the body frame.
    ///
    /// A chain of frame elements from the wheel frame to the body frame should have exactly one
    /// wheel frame and one steering frame.
    #[error("No steering frame in the parent chain. Each wheel to body chain should have exactly one wheel frame and one steering frame.")]
    NoSteeringFramesInChain {
        /// The ID of the parent frame below which the frame is being added.
        id: FrameID,
    },
}
