use crossbeam_channel::{Receiver, Sender};

use crate::{change_notification_processing::ChangeID, number_space::NumberSpaceType, Error};

use super::joint_state::{JointState, JointStateRange};

#[cfg(test)]
#[path = "actuator_interface_tests.rs"]
mod actuator_interface_tests;

/// Defines the minimum and maximum rates of change available for
/// an [Actuator] or [ActuatorInterface] at its current state.
///
/// The overall minimum and maximum values for a [JointState] are provided
/// by the [JointStateRange], however it is possible (even likely) that the
/// minimum or maximum values cannot be reached at all times. For instance
/// the maximum velocity of a joint may depend on the current position of a joint,
/// i.e. a joint at maximum linear position cannot extend any further so the
/// maximum velocity is 0.0, not the overall maximum velocity.
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
    pub fn get_maximum_acceleration(&self) -> f64 {
        self.maximum_acceleration
    }

    /// Returns the current maximum jerk.
    pub fn get_maximum_jerk(&self) -> f64 {
        self.maximum_jerk
    }

    /// Returns the current maximum velocity.
    pub fn get_maximum_velocity(&self) -> f64 {
        self.maximum_velocity
    }

    /// Returns the current minimum acceleration.
    pub fn get_minimum_acceleration(&self) -> f64 {
        self.minimum_acceleration
    }

    /// Returns the current minimum jerk.
    pub fn get_minimum_jerk(&self) -> f64 {
        self.minimum_jerk
    }

    /// Returns the current minimum velocity.
    pub fn get_minimum_velocity(&self) -> f64 {
        self.minimum_velocity
    }

    /// Sets the current maximum acceleration.
    pub fn set_maximum_acceleration(&mut self, maximum_acceleration: f64) {
        self.maximum_acceleration = maximum_acceleration;
    }

    /// Sets the current maximum jerk.
    pub fn set_maximum_jerk(&mut self, maximum_jerk: f64) {
        self.maximum_jerk = maximum_jerk;
    }

    /// Sets the current maximum velocity.
    pub fn set_maximum_velocity(&mut self, maximum_velocity: f64) {
        self.maximum_velocity = maximum_velocity;
    }

    /// Sets the current minimum acceleration.
    pub fn set_minimum_acceleration(&mut self, minimum_acceleration: f64) {
        self.minimum_acceleration = minimum_acceleration;
    }

    /// Sets the current minimum jerk.
    pub fn set_minimum_jerk(&mut self, minimum_jerk: f64) {
        self.minimum_jerk = minimum_jerk;
    }

    /// Sets the current minimum velocity.
    pub fn set_minimum_velocity(&mut self, minimum_velocity: f64) {
        self.minimum_velocity = minimum_velocity;
    }

    /// Creates a new instance of [ActuatorAvailableRatesOfChange] with the given values.
    ///
    /// ## Parameters
    ///
    /// * 'minimum_velocity' - The minimum velocity for the given actuator
    /// * 'maximum_velocity' - The maximum velocity for the given actuator
    /// * 'minimum_acceleration' - The minimum acceleration for the given actuator
    /// * 'maximum_acceleration' - The maximum acceleration for the given actuator
    /// * 'minimum_jerk' - The minimum jerk for the given actuator
    /// * 'maximum_jerk' - The maximum jerk for the given actuator
    ///
    /// ## Examples
    ///
    /// ```
    /// use swerve_model::hardware::actuator_interface::ActuatorAvailableRatesOfChange;
    ///
    /// let result = ActuatorAvailableRatesOfChange::new(minimum_velocity, maximum_velocity, minimum_acceleration, maximum_acceleration, minimum_jerk, maximum_jerk);
    /// assert_eq!(result, );
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
    fn get_actuator_motion_type(&self) -> NumberSpaceType;

    /// Returns the minimum and maximum states for the actuator.
    fn get_actuator_range(&self) -> JointStateRange;

    /// Returns the [Sender] that can be used to send command values to the
    /// actuator implementation.
    fn get_command_sender(&self) -> Result<Sender<JointState>, Error>;

    /// Returns the [Receiver] that is used to receive the current [ActuatorState]
    /// and the currently available minimum and maximum rate of change.
    fn get_current_state_receiver(
        &self,
    ) -> Result<Receiver<(JointState, ActuatorAvailableRatesOfChange)>, Error>;

    /// Stores the notification function for updating the software actuator
    /// and the [ChangeID] that informs the software actuator which hardware
    /// actuator has been updated.
    fn on_change(id: ChangeID, notifier: Sender<ChangeID>);
}
