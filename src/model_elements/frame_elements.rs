//! Defines the different frame elements that are used to create a robot model

extern crate nalgebra as na;

use std::{
    fmt::Display,
    sync::{
        atomic::{AtomicUsize, Ordering},
        Arc, Mutex,
    },
};

use crossbeam_channel::Sender;
use na::{Matrix3, Matrix6, Vector3};

use crate::{
    change_notification_processing::HardwareChangeProcessor,
    hardware::{
        actuator_interface::{ActuatorAvailableRatesOfChange, HardwareActuator},
        joint_state::JointState,
        sensor_interface::HardwareSensor,
    },
    Error,
};

use crate::number_space::{to_number_space, RealNumberValueSpace};

#[cfg(test)]
#[path = "frame_elements_tests.rs"]
mod frame_elements_tests;

/// Defines the degree-of-freedom for a frame element relative to the parent.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum FrameDofType {
    /// The frame element is static relative to the parent.
    Static,
    /// The frame element has a rotational degree-of-freedom relative to the
    /// the parent frame. The element rotates around the X-axis of the element
    /// connection point with the parent frame.
    RevoluteX,
    /// The frame element has a rotational degree-of-freedom relative to the
    /// the parent frame. The element rotates around the Y-axis of the element
    /// connection point with the parent frame.
    RevoluteY,
    /// The frame element has a rotational degree-of-freedom relative to the
    /// the parent frame. The element rotates around the Z-axis of the element
    /// connection point with the parent frame.
    RevoluteZ,
    /// The frame element has a linear translation degree-of-freedom relative to
    /// the parent frame. The element translates along the X-axis of the element
    /// connection point with the parent frame.
    PrismaticX,
    /// The frame element has a linear translation degree-of-freedom relative to
    /// the parent frame. The element translates along the Y-axis of the element
    /// connection point with the parent frame.
    PrismaticY,
    /// The frame element has a linear translation degree-of-freedom relative to
    /// the parent frame. The element translates along the Y-axis of the element
    /// connection point with the parent frame.
    PrismaticZ,
}

/// The FrameID counter value for the 'NONE' ID.
static NONE_FRAME_ID: usize = 0;

/// Atomic counter for FrameID instances
/// The counter starts at 1 because 0 is reserved for the 'NONE' ID.
static FRAME_ID_COUNTER: AtomicUsize = AtomicUsize::new(1);

/// Defines a unique ID for ReferenceFrame types
///
/// - Can be cloned safely
/// - Can be created safely across many threads
#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct FrameID {
    /// The internal value that forms the actual ID. This is set in a
    /// thread-safe maner
    // Based on this StackOverflow answer: https://stackoverflow.com/a/32936288/539846
    id: usize,
}

impl FrameID {
    /// Returns a value indicating if the given ID is the [FrameID::none()] ID.
    pub fn is_none(&self) -> bool {
        self.id == NONE_FRAME_ID
    }

    /// Create a new ID in a thread safe manner.
    pub fn new() -> Self {
        Self {
            id: FRAME_ID_COUNTER.fetch_add(1, Ordering::SeqCst),
        }
    }

    /// Returns the FrameID that doesn't belong to any FrameElement. Can be used to initialize
    /// IDs that are unknown.
    pub fn none() -> Self {
        Self { id: NONE_FRAME_ID }
    }
}

impl Default for FrameID {
    fn default() -> Self {
        Self::new()
    }
}

impl Display for FrameID {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "FrameID [{}]", self.id)
    }
}

impl AsRef<FrameID> for FrameID {
    fn as_ref(&self) -> &FrameID {
        self
    }
}

/// Defines a single reference frame for use in a robotic model.
///
/// The frame has a cartesian right-handed coordinate system with the origin
/// defined at the joint location to the parent frame, or in the geometric middle
/// if there is no parent frame.
pub struct ReferenceFrame {
    /// The human readable name for the element.
    name: String,

    /// The unique ID for the element.
    id: FrameID,

    /// Defines the degree of freedom for the element. Is one of
    /// - Static
    /// - Rotational / Revolute around one of the axes
    /// - Translational / Prismatic along one of the axes
    ///
    /// An element can only have 1 degree of freedom. For cases where multiple degrees of freedom
    /// are required it is necessary to define multiple elements and child elements.
    degree_of_freedom_kind: FrameDofType,

    /// The homogeneous transform from the current frame to the parent frame at displacement = 0
    /// Homogeneous transform is 4x4 matrix: 3x4 matrix = [R|t] on top, bottom row = [0 0 0 1]
    // frame_transform_to_parent: Matrix4<f64>,
    is_actuated: bool,
}

impl ReferenceFrame {
    /// Indicates what type of degree-of-freedom the current element has, if any.  Is one of
    /// - Static
    /// - Rotational / Revolute around one of the axes
    /// - Translational / Prismatic along one of the axes
    ///
    /// An element can only have 1 degree of freedom. For cases where multiple degrees of freedom
    /// are required it is necessary to define multiple elements and child elements.
    pub fn degree_of_freedom_kind(&self) -> FrameDofType {
        self.degree_of_freedom_kind
    }

    /// Returns a reference to the FrameID of the element.
    pub fn id(&self) -> &FrameID {
        self.id.as_ref()
    }

    /// Returns a value indicating whether the element is actuated or not.
    pub fn is_actuated(&self) -> bool {
        self.is_actuated
    }

    /// Returns the name of the element.
    pub fn name(&self) -> &str {
        self.name.as_ref()
    }

    /// Creates a new ReferenceFrame.
    pub fn new(name: String, degree_of_freedom_kind: FrameDofType, is_actuated: bool) -> Self {
        Self {
            name,
            id: FrameID::new(),
            degree_of_freedom_kind,
            is_actuated,
        }
    }
}

/// Defines a part of the chassis that has its own [ReferenceFrame]
pub struct ChassisElement {
    /// Defines the mass of the element in kg.
    mass_in_kg: f64,
    /// Stores the location of the center of mass of the element, relative to the
    /// elements coordinate frame.
    center_of_mass: Vector3<f64>,
    /// Stores the moments of inertia for the element, relative to the elements
    /// coordinate frame.
    moment_of_inertia: Matrix3<f64>,

    /// The ID of the [ReferenceFrame] that is associated with the current chassis
    /// element.
    reference_frame: FrameID,

    /// The spatial inertia for the chassis element.
    spatial_inertia: Matrix6<f64>,

    /// The human readable name for the element.
    name: String,
}

impl ChassisElement {
    /// Returns the location of the center of mass of the element, relative to the
    /// elements coordinate frame.
    pub fn center_of_mass(&self) -> &Vector3<f64> {
        &self.center_of_mass
    }

    /// Returns the mass of the element in kg.
    pub fn mass_in_kg(&self) -> f64 {
        self.mass_in_kg
    }

    /// Returns the moments of inertia for the element, relative to the elements
    /// coordinate frame.
    pub fn moment_of_inertia(&self) -> &Matrix3<f64> {
        &self.moment_of_inertia
    }

    /// Returns the name of the element.
    pub fn name(&self) -> &str {
        self.name.as_ref()
    }

    /// Creates a new ChassisElement.
    ///
    /// ## Parameters
    ///
    /// * 'name' - The name of the element
    /// * 'mass' - The mass in kg of the element
    /// * 'center_of_mass' - The location of the center of mass for the element relative to the
    ///   elements own reference frame
    /// * 'moment_of_inertia' - The moment of inertia for the element, relative to the elements
    ///   own reference frame.
    /// * 'spatial_inertia' - The spatial inertia for the element, relative to the elements own
    ///   reference frame
    /// * 'reference_frame' - The [ReferenceFrame] for the element.
    pub fn new(
        name: String,
        mass: f64,
        center_of_mass: Vector3<f64>,
        moment_of_inertia: Matrix3<f64>,
        spatial_inertia: Matrix6<f64>,
        reference_frame: FrameID,
    ) -> Self {
        Self {
            name,
            mass_in_kg: mass,
            center_of_mass,
            moment_of_inertia,
            reference_frame,
            spatial_inertia,
        }
    }

    /// Returns the ID of the reference frame associated with this element.
    pub fn reference_frame(&self) -> &FrameID {
        &self.reference_frame
    }

    /// Returns information about the spatial inertia for this element.
    pub fn spatial_inertia(&self) -> &Matrix6<f64> {
        &self.spatial_inertia
    }
}

/// Defines a sensor that tracks the state of a joint.
pub struct JointSensor {
    // Might need a reference frame upon which the actuator acts, i.e. the velocity is determined
    // as the relative velocity between two reference frames, one attached to the non-moving part
    // of the actuator and one attached to the moving part of the actuator. Both in the same
    // orientation when in the 0 setting and in the same orientation
    // (and ideally overlapping)
    /// The current state of the actuator. Updated by a closure function which is invoked
    /// by the [HardwareChangeProcessor]
    current_state: Arc<Mutex<JointState>>,

    /// The number space for the actuator. Used to determine how the actuator behaves at
    /// the extremes of the number range, i.e. for linear it will stop, but for revolute
    /// it will continue on the other side of the number range.
    number_space: Box<dyn RealNumberValueSpace>,
}

impl JointSensor {
    /// Returns the number space for the sensor
    pub fn numberspace(&self) -> &dyn RealNumberValueSpace {
        self.number_space.as_ref()
    }

    /// Returns the sensor value at the current time.
    #[cfg_attr(test, mutants::skip)] // Cannot easily check mutations as this is a threaded lock situation
    pub fn value(&self) -> Result<JointState, Error> {
        let mut retries = 0;
        while retries < 3 {
            match self.current_state.lock() {
                Ok(r) => {
                    return Ok(JointState::new(
                        r.position(),
                        *r.velocity(),
                        *r.acceleration(),
                        *r.jerk(),
                    ));
                }
                Err(_) => {
                    // Failed to lock. Wait and try again.
                    retries += 1;
                }
            };
        }

        Err(Error::FailedToReadActuatorJointState)
    }

    /// Creates a new [JointSensor] instance
    ///
    /// ## Parameters
    ///
    /// * 'sensor' - The hardware interface that points to the actual sensor.
    /// * 'change_processor' - The change processor that will process updates from the hardware sensor
    pub fn new(
        sensor: &mut impl HardwareSensor,
        change_processor: &HardwareChangeProcessor,
    ) -> Result<Self, Error> {
        // Initially set the current state and the rates of change to be zero. These values will be overwritten
        // as soon as we get our first set of data from the actual actuator.
        let current_state = Arc::new(Mutex::new(JointState::new(
            0.0,
            Some(0.0),
            Some(0.0),
            Some(0.0),
        )));
        let current_state_clone = current_state.clone();

        let number_space = to_number_space(sensor.joint_motion_type());
        let result = Self {
            current_state,
            number_space,
        };

        let state_reciever = sensor.current_state_receiver()?;
        let on_notify_of_change = Box::new(move || {
            let result = state_reciever.recv();
            if result.is_err() {
                // Something isn't right. Nothing we can do. Just continue with the code
                return;
            }

            let s = result.unwrap();

            let mut retries = 0;
            while retries < 3 {
                match current_state_clone.lock() {
                    Ok(r) => {
                        let mut mutable_state = r;
                        *mutable_state = s;
                        break;
                    }
                    Err(_) => {
                        // Failed to lock. Wait and try again.
                        retries += 1;
                    }
                };
            }

            // Updated, yay
        });

        let (sender, id) = match change_processor.add(on_notify_of_change) {
            Ok(r) => r,
            Err(e) => return Err(e),
        };
        sensor.on_change(id, sender);

        Ok(result)
    }
}

/// Stores the current state and achievable rates of change for an actuator at a given point in time.
struct CurrentActuatorState {
    /// The current state of the reference frame attached to the moving part of the actuator
    state: JointState,

    /// The maximum and minimum rates of change available for the actuator at the current 'state',
    /// i.e. the maximum and minimum values of velocity, acceleration and jerk that the actuator
    /// could attain at the current state.
    rates_of_change: ActuatorAvailableRatesOfChange,
}

impl CurrentActuatorState {
    /// Creates a new [CurrentActuatorState] instance with the provided data
    ///
    /// ## Parameters
    ///
    /// * 'state' - The current state of the joint that the actuator controls
    /// * 'rates_of_change' - The maximum and minimum rates of change available to the actuator
    ///   for the current 'state', i.e. the maximum and minimum values of velocity, acceleration
    ///   and jerk that the actuator could attain at the current state.
    fn new(state: JointState, rates_of_change: ActuatorAvailableRatesOfChange) -> Self {
        Self {
            state,
            rates_of_change,
        }
    }
}

/// Defines an actuator that is attached to a [ReferenceFrame] or a [ChassisElement].
///
/// ## Notes
///
/// * It is assumed that once reference frames and/or chassis elements are created they
///   are never removed and will live for the application life time. This is reflected
///   in the fact that you cannot remove an actuator.
pub struct Actuator {
    // Might need a reference frame upon which the actuator acts, i.e. the velocity is determined
    // as the relative velocity between two reference frames, one attached to the non-moving part
    // of the actuator and one attached to the moving part of the actuator. Both in the same
    // orientation when in the 0 setting and in the same orientation
    // (and ideally overlapping)
    /// The current state of the actuator. Updated by a closure function which is invoked
    /// by the [HardwareChangeProcessor]
    current_state: Arc<Mutex<CurrentActuatorState>>,

    /// The number space for the actuator. Used to determine how the actuator behaves at
    /// the extremes of the number range, i.e. for linear it will stop, but for revolute
    /// it will continue on the other side of the number range.
    number_space: Box<dyn RealNumberValueSpace>,

    // TODO: The command sender should be sending a joint state to achieve and the
    //       approach to achieve it, i.e. the velocity, acceleration and jerk as well
    //       as the profile to achieve this.
    /// The channel sender that is used to send a state change command to the actuator
    command_sender: Sender<JointState>,
}

impl Actuator {
    /// Returns the number space for the actuator
    pub fn numberspace(&self) -> &dyn RealNumberValueSpace {
        self.number_space.as_ref()
    }

    /// Gets the current joint state for the actuator
    #[cfg_attr(test, mutants::skip)] // Cannot easily check mutations as this is a threaded lock situation
    pub fn value(&self) -> Result<JointState, Error> {
        let mut retries = 0;
        while retries < 3 {
            match self.current_state.lock() {
                Ok(r) => {
                    return Ok(JointState::new(
                        r.state.position(),
                        *r.state.velocity(),
                        *r.state.acceleration(),
                        *r.state.jerk(),
                    ));
                }
                Err(_) => {
                    // Failed to lock. Wait and try again.
                    retries += 1;
                }
            };
        }

        Err(Error::FailedToReadActuatorJointState)
    }

    /// Creates a new [Actuator] instance with the given get and set functions
    ///
    /// ## Parameters
    ///
    /// * 'actuator' - The hardware interface that points to the actual actuator.
    /// * 'change_processor' - The change processor that will process updates from the hardware actuator
    pub fn new(
        actuator: &mut impl HardwareActuator,
        change_processor: &HardwareChangeProcessor,
    ) -> Result<Self, Error> {
        // Initially set the current state and the rates of change to be zero. These values will be overwritten
        // as soon as we get our first set of data from the actual actuator.
        let current_state = Arc::new(Mutex::new(CurrentActuatorState::new(
            JointState::new(0.0, Some(0.0), Some(0.0), Some(0.0)),
            ActuatorAvailableRatesOfChange::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        )));
        let current_state_clone = current_state.clone();

        let number_space = to_number_space(actuator.actuator_motion_type());
        let command_sender = actuator.command_sender()?;
        let result = Self {
            current_state,
            number_space,
            command_sender,
        };

        let state_reciever = actuator.current_state_receiver()?;
        let on_notify_of_change = Box::new(move || {
            let result = state_reciever.recv();
            if result.is_err() {
                // Something isn't right. Nothing we can do. Just continue with the code
                return;
            }

            let (s, c) = result.unwrap();

            let mut retries = 0;
            while retries < 3 {
                match current_state_clone.lock() {
                    Ok(r) => {
                        let mut mutable_state = r;
                        mutable_state.state = s;
                        mutable_state.rates_of_change = c;
                        break;
                    }
                    Err(_) => {
                        // Failed to lock. Wait and try again.
                        retries += 1;
                    }
                };
            }

            // Updated, yay
        });

        let (sender, id) = match change_processor.add(on_notify_of_change) {
            Ok(r) => r,
            Err(e) => return Err(e),
        };
        actuator.on_change(id, sender);

        Ok(result)
    }

    /// Sets the desired actuator state.
    ///
    /// ## Parameters
    ///
    /// * 'new_state' - The desired state
    ///
    /// ## Returns
    ///
    /// A result indicating if the setting of the new desired state was
    /// successful or not.
    ///
    pub fn update_state(&self, new_state: JointState) -> Result<(), Error> {
        // Until https://github.com/rust-lang/rust/issues/99301 is fixed we can't send an error type
        // with generics (i.e. SendError<JointState>) into a thiserror source / backtrace error translator
        self.command_sender
            .send(new_state)
            .map_err(|_source| Error::FailedToSetActuatorJointState {})
    }
}

/// Defines a single constraint on a joint or element
pub struct JointConstraint {
    // state change + notification
}

impl JointConstraint {
    /// Creates a new [JointConstraint] instance.
    pub fn new() -> Self {
        Self {}
    }
}

impl Default for JointConstraint {
    fn default() -> Self {
        Self::new()
    }
}
