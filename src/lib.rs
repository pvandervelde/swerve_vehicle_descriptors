#![warn(missing_docs)]

//! Kinematic model definition for a swerve (4 wheel steering and 4 wheel drive) robot.
//!
//! Provides abstraction of the swerve robot geometry aimed at calculating the
//! kinematics or dynamics of the robot for purposes of control.

use model_elements::frame_elements::FrameID;
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
