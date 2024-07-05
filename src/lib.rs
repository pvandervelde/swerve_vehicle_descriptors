#![warn(missing_docs)]

//! Kinematic model definition for a swerve (4 wheel steering and 4 wheel drive) robot.
//!
//! Provides abstraction of the swerve robot geometry aimed at calculating the
//! kinematics or dynamics of the robot for purposes of control.

use thiserror::Error;

/// Defines different number spaces
pub mod number_space;

/// Provides types for the (asynchronous) processing of messages from/to hardware
pub mod message_processing;

/// Defines the different errors for the swerve model crate.
#[derive(Debug, Error, PartialEq)]
#[non_exhaustive]
pub enum Error {}
