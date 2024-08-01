//! Defines the interface for actuators

use crossbeam_channel::{Receiver, Sender};

use crate::{change_notification_processing::ChangeID, number_space::NumberSpaceType, Error};

use super::joint_state::{JointState, JointStateRange};

#[cfg(test)]
#[path = "actuator_interface_tests.rs"]
mod actuator_interface_tests;

/// Defines the minimum and maximum rates of change available for
/// an Actuator at its current state.
///
/// The rates of change for which the values are stored are:
///
/// * [Velocity](https://en.wikipedia.org/wiki/Velocity) - The rate of change of
///   the position with respect to time
/// * [Acceleration](https://en.wikipedia.org/wiki/Acceleration) - The rate of
///   change of the velocity with respect to time
/// * [Jerk](https://en.wikipedia.org/wiki/Jerk_(physics)) - The rate of change
///   of the acceleration with respect to time
///
/// The overall minimum and maximum values for a [JointState] are provided
/// by the [JointStateRange], however it is possible (even likely) that the
/// minimum or maximum values cannot be reached at all times. For instance
/// the maximum velocity of a joint may depend on the current position of a joint,
/// i.e. a joint at maximum linear position cannot extend any further so the
/// maximum velocity is 0.0, not the overall maximum velocity.
///
/// The maximum value stored is assumed to be the greatest value for motion
/// in positive direction, while the minimum value is assumed to be the greatest
/// value for motion in the negative direction.
#[derive(Clone, Copy, Debug)]
pub struct ActuatorAvailableRatesOfChange {
    /// The current minimum velocity
    minimum_velocity: f64,

    /// The current maximum velocity
    maximum_velocity: f64,

    /// The current minimum acceleration
    minimum_acceleration: f64,

    /// The current maximum acceleration
    maximum_acceleration: f64,

    /// The current minimum jerk
    minimum_jerk: f64,

    /// The current maximum jerk
    maximum_jerk: f64,
}

impl ActuatorAvailableRatesOfChange {
    /// Returns the current maximum acceleration.
    pub fn maximum_acceleration(&self) -> f64 {
        self.maximum_acceleration
    }

    /// Returns the current maximum jerk.
    pub fn maximum_jerk(&self) -> f64 {
        self.maximum_jerk
    }

    /// Returns the current maximum velocity.
    pub fn maximum_velocity(&self) -> f64 {
        self.maximum_velocity
    }

    /// Returns the current minimum acceleration.
    pub fn minimum_acceleration(&self) -> f64 {
        self.minimum_acceleration
    }

    /// Returns the current minimum jerk.
    pub fn minimum_jerk(&self) -> f64 {
        self.minimum_jerk
    }

    /// Returns the current minimum velocity.
    pub fn minimum_velocity(&self) -> f64 {
        self.minimum_velocity
    }

    /// Creates a new instance of [ActuatorAvailableRatesOfChange] with the given values
    /// for velocity, acceleration and jerk.
    ///
    /// ## Parameters
    ///
    /// * `minimum_velocity` - The minimum velocity for the given actuator
    /// * `maximum_velocity` - The maximum velocity for the given actuator
    /// * `minimum_acceleration` - The minimum acceleration for the given actuator
    /// * `maximum_acceleration` - The maximum acceleration for the given actuator
    /// * `minimum_jerk` - The minimum jerk for the given actuator
    /// * `maximum_jerk` - The maximum jerk for the given actuator
    ///
    /// ## Examples
    ///
    /// ```
    /// use swerve_vehicle_descriptors::hardware::actuator_interface::ActuatorAvailableRatesOfChange;
    ///
    /// let result = ActuatorAvailableRatesOfChange::new(-10.0, 10.0, -5.0, 5.0, -20.0, 20.0);
    ///
    /// assert_eq!(result.minimum_velocity(), -10.0);
    /// assert_eq!(result.maximum_velocity(), 10.0);
    ///
    /// assert_eq!(result.minimum_acceleration(), -5.0);
    /// assert_eq!(result.maximum_acceleration(), 5.0);
    ///
    /// assert_eq!(result.minimum_jerk(), -20.0);
    /// assert_eq!(result.maximum_jerk(), 20.0);
    /// ```
    pub fn new(
        minimum_velocity: f64,
        maximum_velocity: f64,
        minimum_acceleration: f64,
        maximum_acceleration: f64,
        minimum_jerk: f64,
        maximum_jerk: f64,
    ) -> Self {
        Self {
            minimum_velocity,
            maximum_velocity,
            minimum_acceleration,
            maximum_acceleration,
            minimum_jerk,
            maximum_jerk,
        }
    }
}

/// Defines the interface for hardware that moves a robot joint element.
pub trait HardwareActuator {
    /// Returns the [NumberSpaceType] that is used to describe the motion of the actuator.
    fn actuator_motion_type(&self) -> NumberSpaceType;

    /// Returns the minimum and maximum states for the actuator.
    fn actuator_range(&self) -> JointStateRange;

    /// Returns the [Sender] that can be used to send command values to the
    /// actuator implementation.
    fn command_sender(&self) -> Result<Sender<JointState>, Error>;

    /// Returns the [Receiver] that is used to receive the current [JointState]
    /// and the currently available minimum and maximum rate of change.
    fn current_state_receiver(
        &self,
    ) -> Result<Receiver<(JointState, ActuatorAvailableRatesOfChange)>, Error>;

    /// Stores the notification function for updating the software actuator
    /// and the [ChangeID] that informs the software actuator which hardware
    /// actuator has been updated.
    fn on_change(&mut self, id: ChangeID, notifier: Sender<ChangeID>);
}
