#![warn(missing_docs)]

//! Kinematic model definition for a swerve (4 wheel steering and 4 wheel drive) robot.
//!
//! Provides abstraction of the swerve robot geometry aimed at calculating the
//! kinematics or dynamics of the robot for purposes of control.

use thiserror::Error;

/// Defines different number spaces
pub mod number_space;

/// Provides types for the (asynchronous) processing of messages from/to hardware
pub mod change_notification_processing;

/// Provides the abstraction for the hardware in the robot.
pub mod hardware;

/// Provides an abstraction for the frame elements of the robot and the way they
/// connect with other elements.
///
/// The model elements module describes provides a simplified view on how
/// robot frame parts connect to each other, through linear or rotational connections,
/// and provides structures that describe the entire robot model.
///
/// ## Examples
///
/// Create a model from a number of frames
///
/// ```rust
/// todo!()
/// ```
pub mod model_elements;

/// Defines the different errors for the swerve model crate.
#[derive(Debug, Error, PartialEq)]
#[non_exhaustive]
pub enum Error {
    /// Indicates that we failed to get a joint state from an actuator.
    #[error("Failed to read the joint state for the given actuator.")]
    FailedToReadActuatorJointState,

    /// Indicates that we failed to set a joint state for a given actuator.
    #[error("Failed to set the joint state for the given actuator.")]
    FailedToSetActuatorJointState,
}
