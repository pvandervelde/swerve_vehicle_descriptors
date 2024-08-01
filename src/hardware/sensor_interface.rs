//! Defines the interface for sensors

use crossbeam_channel::{Receiver, Sender};

use crate::{change_notification_processing::ChangeID, number_space::NumberSpaceType, Error};

use super::joint_state::{JointState, JointStateRange};

/// Defines the interface for hardware that senses the state of a robot joint element.
pub trait HardwareSensor {
    /// Returns the [Receiver] that is used to receive the current [JointState]
    /// and the currently available minimum and maximum rate of change.
    fn current_state_receiver(&self) -> Result<Receiver<JointState>, Error>;

    /// Returns the [NumberSpaceType] that is used to describe the motion of the joint.
    fn joint_motion_type(&self) -> NumberSpaceType;

    /// Returns the minimum and maximum states for the actuator.
    fn joint_range(&self) -> JointStateRange;

    /// Stores the notification function for updating the software actuator
    /// and the [ChangeID] that informs the software actuator which hardware
    /// actuator has been updated.
    fn on_change(&mut self, id: ChangeID, notifier: Sender<ChangeID>);
}
