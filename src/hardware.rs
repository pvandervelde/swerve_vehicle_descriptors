//! The `hardware` module contains the structures and traits used to interact with physical hardware,
//! like sensors and actuators. The [sensor_interface::HardwareSensor] trait provides the functions
//! necessary to get information from a physical, or simulated, sensor. The
//! [actuator_interface::HardwareActuator] trait provides functions necessary to get information
//! to and from a physical, or simulated, actuator.
//!

pub mod actuator_interface;
pub mod joint_state;
pub mod sensor_interface;
