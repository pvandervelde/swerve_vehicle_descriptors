#[cfg(test)]
#[path = "joint_state_tests.rs"]
mod joint_state_tests;

/// Stores the current position and motion state for a given joint.
///
/// A 'joint' is defined to only have 1 degree-of-freedom, so the stored state
/// refers to this degree of freedom, i.e. if the joint has a revolute degree-of-freedom
/// then the state refers to a rotational position, velocity, acceleration and jerk.
/// On the other hand if the joint has a prismatic degree-of-freedom then the state
/// refers to a linear position, velocity, acceleration and jerk.
///
/// All values are assumed to be in the range of the [minimum, maximum] value
/// for the joint. These minimum and maximum values are specified by the
/// [JointStateRange].
#[derive(Clone, Debug, PartialEq)]
pub struct JointState {
    /// The position of the joint.
    position: f64,

    /// The velocity fo the joint.
    velocity: Option<f64>,

    /// The acceleration of the joint.
    acceleration: Option<f64>,

    /// The jerk of the joint.
    jerk: Option<f64>,
}

impl JointState {
    /// Returns the current acceleration of the joint.
    pub fn get_acceleration(&self) -> &Option<f64> {
        &self.acceleration
    }

    /// Returns the current jerk of the joint.
    pub fn get_jerk(&self) -> &Option<f64> {
        &self.jerk
    }

    /// Returns the current position of the joint
    pub fn get_position(&self) -> f64 {
        self.position
    }

    /// Returns the current velocity of the joint.
    pub fn get_velocity(&self) -> &Option<f64> {
        &self.velocity
    }

    /// Creates a new [JointState] instance
    ///
    /// ## Parameters
    ///
    /// * 'position' - The current position of the joint
    /// * 'velocity' - The current velocity of the joint
    /// * 'acceleration' - The current acceleration of the joint
    /// * 'jerk' - The current jerk of the joint
    pub fn new(
        position: f64,
        velocity: Option<f64>,
        acceleration: Option<f64>,
        jerk: Option<f64>,
    ) -> Self {
        Self {
            position,
            velocity,
            acceleration,
            jerk,
        }
    }
}

/// Stores the maximum and minimum values for the [JointState] of an
/// Sensor or Actuator.
#[derive(Debug)]
pub struct JointStateRange {
    /// The minimum values of the actuator state.
    minimum: JointState,

    /// The maximum values of the actuator state.
    maximum: JointState,
}

impl JointStateRange {
    /// Gets the maximum acceleration for the joint.
    pub fn get_maximum_acceleration(&self) -> &Option<f64> {
        &self.maximum.get_acceleration()
    }

    /// Gets the maximum jerk for the joint.
    pub fn get_maximum_jerk(&self) -> &Option<f64> {
        &self.maximum.get_jerk()
    }

    /// Gets the maximum position for the joint.
    pub fn get_maximum_position(&self) -> f64 {
        self.maximum.get_position()
    }

    /// Gets the maximum velocity for the joint.
    pub fn get_maximum_velocity(&self) -> &Option<f64> {
        &self.maximum.get_velocity()
    }

    /// Gets the minimum acceleration for the joint.
    pub fn get_minimum_acceleration(&self) -> &Option<f64> {
        &self.minimum.get_acceleration()
    }

    /// Gets the minimum jerk for the joint.
    pub fn get_minimum_jerk(&self) -> &Option<f64> {
        &self.minimum.get_jerk()
    }

    /// Gets the minimum position for the joint.
    pub fn get_minimum_position(&self) -> f64 {
        self.minimum.get_position()
    }

    /// Gets the minimum velocity for the joint.
    pub fn get_minimum_velocity(&self) -> &Option<f64> {
        &self.minimum.get_velocity()
    }

    /// Creates a new [ActuatorState] with the given minimum and maximum
    ///
    /// ## Parameters
    ///
    /// * 'minimum' - The minimum values for the actuator state.
    /// * 'maximum' - The maximum values for the actuator state.
    pub fn new(minimum: JointState, maximum: JointState) -> Self {
        Self { minimum, maximum }
    }
}
