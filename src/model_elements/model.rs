extern crate nalgebra as na;

use std::collections::{BTreeSet, HashMap};

use na::{Isometry3, Matrix3, Matrix4, Matrix6, Translation3, UnitQuaternion, Vector3};

use crate::Error;

use super::frame_elements::{
    Actuator, ChassisElement, FrameDofType, FrameID, JointConstraint, JointSensor, ReferenceFrame,
};

#[cfg(test)]
#[path = "model_tests.rs"]
mod model_tests;

/// A delegating iterator for the KinematicTree so that we can return an iterator or an
/// empty iterator.
pub struct OptionIterator<I> {
    opt_iterator: Option<I>,
}

impl<I, T> Iterator for OptionIterator<I>
where
    I: Iterator<Item = T>,
{
    type Item = T;
    fn next(&mut self) -> Option<T> {
        match &mut self.opt_iterator {
            Some(iterator) => iterator.next(),
            None => None,
        }
    }
}

impl<I> OptionIterator<I> {
    /// Create a new OptionIterator
    ///
    /// ## Examples
    ///
    /// Create an empty iterator
    ///
    /// ```rust
    /// let empty_iterator = OptionIterator::new(None);
    /// ```
    ///
    /// Create an iterator with items
    /// ```
    /// let collection = vec![1, 2, 3, 4, 5];
    /// let full_iterator = OptionIterator::new(Some(collection.iter()));
    /// ```
    pub fn new(opt_iterator: Option<I>) -> OptionIterator<I> {
        OptionIterator { opt_iterator }
    }
}

/// Defines a kinematic tree that defines the kinematic model of a wheeled mobile robot. The root
/// of the tree is the robot body with six degrees of freedom (3 translations, 3 rotations) with
/// respect to the navigation / world reference frame.
///
/// Additional frames for structure, steering, suspension etc. are attached via a one degree-of-freedom
/// revolute (rotational) or prismatic (translational) joint.
///
/// All branches of the kinematic tree end with the wheel frames, which, by convention, are attached
/// to their parent frame by revolute joints around the y-axis.
///
/// ## References
///
/// * [A vector algebra formulation of mobile robot velocity kinematics](https://scholar.google.co.nz/citations?view_op=view_citation&hl=en&user=H10kxZgAAAAJ&cstart=20&pagesize=80&sortby=pubdate&citation_for_view=H10kxZgAAAAJ:qjMakFHDy7sC)
///   Neal Seegmiller and Alonzo Kelly
///   Field and Service Robotics: Results of the 8th International Conference
///   2013/12/31
///
struct KinematicTree {
    /// List of frame elements starting at the root.
    elements: HashMap<FrameID, ReferenceFrame>,

    /// The mapping from the parent elements to their direct children.
    children_of: HashMap<FrameID, BTreeSet<FrameID>>,

    /// The mapping from the child element to their parent. The child FrameID is
    /// used as the key. The value is a combination of the parent FrameID and the
    /// Homogeneous transform from the child to the parent when the joint displacement is zero.
    parent_of: HashMap<FrameID, (FrameID, Isometry3<f64>)>,

    /// The list of indices for the wheel frames.
    wheel_elements: BTreeSet<FrameID>,
}

impl KinematicTree {
    /// Add a new frame element to the kinematic tree.
    ///
    /// The first element that is added is assumed to be the robot body which is attached to the
    /// world (which has the 'FrameID::none()' id number). All other elements should have a parent
    /// element that is known to the tree.
    ///
    /// Elements that have a revolute degree of freedom around the y-axis and have no children are
    /// assumed to be the wheel elements.
    ///
    /// * 'element' - The element that should be stored.
    /// * 'parent_id' - The ID of the parent element. It is assumed that this element already exists
    ///   in the kinematic tree, except for the first element that is added that is added using the
    ///   [FrameID::none()] ID to signify that the element being added is the body element.
    /// * 'position_relative_to_parent' - The position vector of the child in the parents reference frame.
    /// * 'orientation_relative_to_parent' - The orientation quaternion of the child in the parents
    ///   reference frame
    ///
    ///
    /// ## Errors
    ///
    /// * [Error::FrameElementAlreadyExists] - Returned when trying to add a frame element with an ID that
    ///   is already stored in the tree
    /// * [Error::MissingFrameElement] - Returned when trying to add a frame element with a parent link
    ///   for a parent element that is not stored in the tree.
    /// * [Error::InvalidFrameID] - Returns when trying to add more than 1 frame element with no parent.
    ///   It is assumed that there is only 1 frame element with no parent. This element is assumed
    ///   to be the body element which by definition is attached to the world frame.
    ///
    fn add_element(
        &mut self,
        element: ReferenceFrame,
        parent_id: FrameID,
        position_relative_to_parent: Translation3<f64>,
        orientation_relative_to_parent: UnitQuaternion<f64>,
    ) -> Result<&FrameID, Error> {
        let element_id = element.id();
        let element_ref = &element;
        if self.elements.contains_key(&element_id) {
            return Err(Error::FrameElementAlreadyExists {
                id: element_id.clone(),
            });
        }

        // Only the first element can not have a parent. All the other ones should have a parent
        // Otherwise we have multiple bodies
        // It is assumed that the first element is attached to the world by definition.
        let parent_id_ref = parent_id.as_ref();
        if parent_id != FrameID::none() {
            if !self.elements.contains_key(parent_id_ref) {
                return Err(Error::MissingFrameElement {
                    id: parent_id.clone(),
                });
            }

            let cloned_element_id = element_id.clone();
            if !self.parent_of.contains_key(&cloned_element_id) {
                let parent_id_to_store = parent_id.clone();
                let isometry = Isometry3::from_parts(
                    position_relative_to_parent,
                    orientation_relative_to_parent,
                );

                self.parent_of
                    .insert(cloned_element_id, (parent_id_to_store, isometry));

                // A parent node can never be a wheel
                self.wheel_elements.remove(parent_id_ref);
            }

            if !self.children_of.contains_key(&parent_id_ref) {
                self.children_of.insert(parent_id.clone(), BTreeSet::new());
            }

            let child_id = element_id.clone();
            let children = match self.children_of.get_mut(parent_id_ref) {
                Some(c) => c,
                None => {
                    return Err(Error::MissingFrameElement {
                        id: parent_id.clone(),
                    })
                }
            };

            if !children.contains(&child_id) {
                children.insert(child_id);
            }
        } else {
            // There only should be one element with no parent ID. And by definition that should be
            // the first element that is added.
            if self.elements.len() > 0 {
                return Err(Error::InvalidFrameID {
                    id: parent_id.clone(),
                });
            }
        }

        // We assume the element is a wheel if:
        // - It is a leaf node, i.e. it doesn't have any children
        // - it has a revolute motion around the Y-axis
        let has_children = self.children_of.contains_key(&element_id);
        if !has_children && element_ref.degree_of_freedom_kind() == FrameDofType::RevoluteY {
            self.wheel_elements.insert(element_id.clone());
        }

        let key = element_id.clone();
        self.elements.insert(key, element);

        let result: &ReferenceFrame;
        match self.elements.get(&key) {
            Some(v) => result = v,
            None => return Err(Error::MissingFrameElement { id: key }),
        }

        // Finally return the index at which the element is stored.
        Ok(result.id())
    }

    /// Returns the body element if it exists
    ///
    /// ## Errors
    ///
    /// * [Error::MissingFrameElement] - Returned when there is no body element stored
    ///   in the tree
    fn get_body_element(&self) -> Result<&ReferenceFrame, Error> {
        if self.elements.is_empty() {
            return Err(Error::MissingFrameElement {
                id: FrameID::none(),
            });
        }

        for elt in self.elements.values() {
            if !self.parent_of.contains_key(elt.id()) {
                return Ok(elt);
            }
        }

        return Err(Error::MissingFrameElement {
            id: FrameID::none(),
        });
    }

    /// Returns an iterator that can be used to iterate over the children of the specified reference frame
    ///
    /// ## Parameters
    ///
    /// * 'id' - The ID of the reference frame from which the direct child frames should be returned
    ///
    /// ## Errors
    ///
    /// * [Error::InvalidFrameID] - Returned when there is no reference frame with ID 'id'
    fn get_children(&self, id: &FrameID) -> Result<impl Iterator<Item = &ReferenceFrame>, Error> {
        if !self.elements.contains_key(id) {
            return Err(Error::InvalidFrameID { id: id.clone() });
        }

        if !self.children_of.contains_key(id) {
            return Ok(OptionIterator::new(None));
        }

        let children = &self.children_of[id];
        Ok(OptionIterator::new(Some(
            children.iter().map(|id| self.get_element_unchecked(id)),
        )))
    }

    /// Returns the reference frame with the given ID
    ///
    /// ## Parameters
    ///
    /// * 'id' - The ID of the reference frame that should be returned
    ///
    /// ## Errors
    ///
    /// * [Error::InvalidFrameID] - Returned when there is no reference frame with ID 'id'
    fn get_element(&self, id: &FrameID) -> Result<&ReferenceFrame, Error> {
        if !self.elements.contains_key(id) {
            return Err(Error::InvalidFrameID { id: id.clone() });
        }

        Ok(self.get_element_unchecked(id))
    }

    /// Returns the reference frame for the given ID without checking that this
    /// reference frame actually exists.
    ///
    /// ## Parameters
    ///
    /// * 'id' - The ID of the reference frame that should be returned
    ///
    /// This function will panic if there is no [ReferenceFrame] with the given
    /// ID.
    fn get_element_unchecked(&self, id: &FrameID) -> &ReferenceFrame {
        &(self.elements[id])
    }

    /// Returns an iterator that iterates over all the reference frames in the tree.
    ///
    /// The order of iteration is not guaranteed.
    fn get_elements(&self) -> impl Iterator<Item = &ReferenceFrame> {
        self.elements.values()
    }

    /// Returns the homogeneous transform that turns coordinates in the child reference frame into
    /// coordinates in the parent reference frame.
    ///
    /// ## Parameters
    ///
    /// * 'id' - The ID of the reference frame that should be returned
    ///
    /// ## Errors
    ///
    /// * [Error::InvalidFrameID] - Returned when there is no reference frame with ID 'id'
    pub fn get_homogeneous_transform_to_parent(
        &self,
        id: &FrameID,
    ) -> Result<&Isometry3<f64>, Error> {
        if !self.elements.contains_key(&id) {
            return Err(Error::InvalidFrameID { id: id.clone() });
        }

        let (_, transform) = self.parent_of.get(id).unwrap();
        Ok(transform)
    }

    /// Returns the parent reference frame for the given reference frame
    ///
    /// ## Parameters
    ///
    /// * 'child_id' - The ID of the child reference frame
    ///
    /// ## Errors
    ///
    /// * [Error::InvalidFrameID] - Returned when there is no reference frame with ID 'id'
    /// * [Error::MissingFrameElement] - Returned when the reference frame has no parent.
    fn get_parent(&self, child_id: &FrameID) -> Result<&ReferenceFrame, Error> {
        if !self.elements.contains_key(child_id) {
            return Err(Error::InvalidFrameID {
                id: child_id.clone(),
            });
        }

        if !self.parent_of.contains_key(child_id) {
            return Err(Error::MissingFrameElement {
                id: child_id.clone(),
            });
        }

        let parent_id_ref = self.parent_of[child_id].0.as_ref();
        Ok(self.get_element_unchecked(parent_id_ref))
    }

    /// Returns an iterator that returns all the wheel reference frames in the tree
    ///
    /// ## Errors
    ///
    /// * [Error::MissingFrameElement] - Returned when the tree is empty
    fn get_wheels(&self) -> Result<impl Iterator<Item = &ReferenceFrame>, Error> {
        if self.elements.is_empty() {
            return Err(Error::MissingFrameElement {
                id: FrameID::none(),
            });
        }

        Ok(self
            .wheel_elements
            .iter()
            .map(|id| self.get_element_unchecked(id)))
    }

    /// Returns a value indicating if the given reference frame has children.
    ///
    /// ## Parameters
    ///
    /// * 'id' - The ID of the reference frame
    ///
    /// ## Errors
    ///
    /// * [Error::InvalidFrameID] - Returned when there is no reference frame with ID 'id'
    fn has_children(&self, id: &FrameID) -> Result<bool, Error> {
        if !self.elements.contains_key(id) {
            return Err(Error::InvalidFrameID { id: id.clone() });
        }

        Ok(self.parent_of.contains_key(id))
    }

    /// Returns a value indicating whether the kinematic tree contains a [ReferenceFrame]
    /// with the given ID.
    ///
    /// ## Parameters
    ///
    /// * 'id' - The ID of the reference frame
    fn has_element(&self, id: &FrameID) -> bool {
        self.elements.contains_key(id)
    }

    /// Returns a value indicating whether the [ReferenceFrame] with the given ID has
    /// a parent frame.
    ///
    /// ## Parameters
    ///
    /// * 'id' - The ID of the reference frame
    ///
    /// ## Errors
    ///
    /// * [Error::InvalidFrameID] - Returned when there is no reference frame with ID 'id'
    fn has_parent(&self, id: &FrameID) -> Result<bool, Error> {
        if !self.elements.contains_key(id) {
            return Err(Error::InvalidFrameID { id: id.clone() });
        }

        Ok(self.parent_of.contains_key(id))
    }

    /// Returns a value indicating whether the [ReferenceFrame] with the given ID is the
    /// body frame
    ///
    /// ## Parameters
    ///
    /// * 'id' - The ID of the reference frame
    ///
    /// ## Errors
    ///
    /// * [Error::InvalidFrameID] - Returned when there is no reference frame with ID 'id'
    fn is_body(&self, id: &FrameID) -> Result<bool, Error> {
        if !self.elements.contains_key(id) {
            return Err(Error::InvalidFrameID { id: id.clone() });
        }

        Ok(!self.parent_of.contains_key(id))
    }

    /// Returns a value indicating whether there are any [ReferenceFrame] instances in
    /// the [KinematicTree]
    fn is_empty(&self) -> bool {
        self.elements.is_empty()
    }

    /// Returns a value indicating whether the [ReferenceFrame] with the given ID is
    /// a wheel
    ///
    /// ## Parameters
    ///
    /// * 'id' - The ID of the reference frame
    ///
    /// ## Errors
    ///
    /// * [Error::InvalidFrameID] - Returned when there is no reference frame with ID 'id'
    fn is_wheel(&self, id: &FrameID) -> Result<bool, Error> {
        if !self.elements.contains_key(id) {
            return Err(Error::InvalidFrameID { id: id.clone() });
        }

        Ok(self.wheel_elements.contains(id))
    }

    /// Creates a new [KinematicTree]
    fn new() -> Self {
        Self {
            elements: HashMap::new(),
            parent_of: HashMap::new(),
            children_of: HashMap::new(),
            wheel_elements: BTreeSet::new(),
        }
    }

    /// Returns the number of wheel reference frames
    pub fn number_of_wheels(&self) -> usize {
        self.wheel_elements.len()
    }
}

/// A kinematic model for a swerve robot.
///
/// It is assumed that the robot will have N wheels, where N > 2. Each wheel has
/// a single steering frame in the wheel-to-body chain of [ReferenceFrame] elements.
/// Each steering frame should only link to exactly one wheel and each wheel should have
/// exactly one steering frame.
pub struct SwerveRobotModel {
    /// The [ChassisElement] instances that make up the model.
    chassis_elements: HashMap<FrameID, ChassisElement>,

    /// The collection of [ReferenceFrame] for all the [ChassisElement] in the model.
    reference_frames: KinematicTree,

    /// The collection of [FrameID] pointing to the steering frames and their
    /// associated wheels.
    steering_frame_to_wheel: HashMap<FrameID, FrameID>,

    /// The collection of [FrameID] pointing to the wheels and their associated
    /// steering frames.
    wheel_to_steering_frame: HashMap<FrameID, FrameID>,

    /// The collection of [Actuator] instances
    actuators: HashMap<FrameID, Actuator>,

    /// The collection of [Sensor] instances
    sensors: HashMap<FrameID, JointSensor>,

    /// The collection of [JointConstraint] instances
    joint_constraints: HashMap<FrameID, JointConstraint>,
}

impl SwerveRobotModel {
    /// Adds the chassis element that represents a non-actuated joint for the robot.
    ///
    /// It is assumed that the body is the first element to be added.
    ///
    /// ## Parameters
    ///
    /// * 'name' - The name of the new chassis element
    /// * 'degree_of_freedom' - The degree of freedom for the element
    /// * 'parent_id' - The ID of the parent reference frame
    /// * 'position_relative_to_parent' - The position of the element relative to the parent
    ///   reference frame
    /// * 'orientation_relative_to_parent' - The orientation of the element relative to the parent
    ///   reference frame
    /// * 'mass' - The mass, in kg, of the chassis element
    /// * 'center_of_mass' - The location of the center of mass for the element relative to the
    ///   elements own reference frame
    /// * 'moment_of_inertia' - The moment of inertia for the element, relative to the elements
    ///   own reference frame.
    /// * 'spatial_inertia' - The spatial inertia for the element, relative to the elements own
    ///   reference frame
    /// * actuator - A reference to the actuator and its controller for the joint
    ///
    /// ## Errors
    ///
    /// * [Error::MissingFrameElement] - Returned when the parent [ReferenceFrame] is not part of the model.
    /// * [Error::InvalidFrameID] - Returned the parent [ReferenceFrame] is connected to a wheel.
    pub fn add_actuated_chassis_element(
        &mut self,
        name: String,
        degree_of_freedom: FrameDofType,
        parent_id: FrameID,
        position_relative_to_parent: Translation3<f64>,
        orientation_relative_to_parent: UnitQuaternion<f64>,
        mass: f64,
        center_of_mass: Vector3<f64>,
        moment_of_inertia: Matrix3<f64>,
        spatial_inertia: Matrix6<f64>,
        actuator: Actuator,
    ) -> Result<FrameID, Error> {
        if !parent_id.is_none() && !self.reference_frames.has_element(&parent_id) {
            return Err(Error::MissingFrameElement {
                id: parent_id.clone(),
            });
        }

        if self.reference_frames.is_wheel(&parent_id)? {
            return Err(Error::InvalidFrameID {
                id: parent_id.clone(),
            });
        }

        let reference_frame = ReferenceFrame::new(name.clone(), degree_of_freedom, true);

        self.actuators
            .insert(reference_frame.id().clone(), actuator);

        self.add_element_unchecked(
            reference_frame,
            parent_id,
            position_relative_to_parent,
            orientation_relative_to_parent,
            name,
            mass,
            center_of_mass,
            moment_of_inertia,
            spatial_inertia,
        )
    }

    /// Adds the chassis element that represents the body of the robot.
    ///
    /// It is assumed that the body is the first element to be added.
    ///
    /// ## Parameters
    ///
    /// * 'name' - The name of the new chassis element
    /// * 'position_relative_to_world' - The position of the element relative to the world
    ///   reference frame
    /// * 'orientation_relative_to_world' - The orientation of the element relative to the world
    ///   reference frame
    /// * 'mass' - The mass, in kg, of the chassis element
    /// * 'center_of_mass' - The location of the center of mass for the element relative to the
    ///   elements own reference frame
    /// * 'moment_of_inertia' - The moment of inertia for the element, relative to the elements
    ///   own reference frame.
    /// * 'spatial_inertia' - The spatial inertia for the element, relative to the elements own
    ///   reference frame
    /// * 'parent_id' - The [FrameID] of the parent [ReferenceFrame]
    ///
    /// ## Errors
    ///
    /// * [Error::InvalidFrameID] - Returned when there is already a chassis element in the collection
    ///   of elements.
    /// * [Error::MissingFrameElement] - Returned when the parent [ReferenceFrame] is not part of the model.
    /// * [Error::InvalidFrameID] - Returned the parent [ReferenceFrame] is connected to a wheel.
    pub fn add_body(
        &mut self,
        name: String,
        position_relative_to_world: Translation3<f64>,
        orientation_relative_to_world: UnitQuaternion<f64>,
        mass: f64,
        center_of_mass: Vector3<f64>,
        moment_of_inertia: Matrix3<f64>,
        spatial_inertia: Matrix6<f64>,
    ) -> Result<FrameID, Error> {
        if !self.reference_frames.is_empty() {
            let body_id = match self.reference_frames.get_body_element() {
                Ok(f) => f.id().clone(),
                Err(_) => FrameID::none(),
            };

            return Err(Error::InvalidFrameID { id: body_id });
        }

        let reference_frame = ReferenceFrame::new(name.clone(), FrameDofType::Static, false);

        self.add_element_unchecked(
            reference_frame,
            FrameID::none(),
            position_relative_to_world,
            orientation_relative_to_world,
            name,
            mass,
            center_of_mass,
            moment_of_inertia,
            spatial_inertia,
        )
    }

    /// Adds a new [ChassisElement] to the model.
    ///
    /// ## Parameters
    ///
    /// * 'reference_frame' - The [ReferenceFrame] for the new chassis element
    /// * 'name' - The name of the new chassis element
    /// * 'mass' - The mass, in kg, of the chassis element
    /// * 'center_of_mass' - The location of the center of mass for the element relative to the
    ///   elements own reference frame
    /// * 'moment_of_inertia' - The moment of inertia for the element, relative to the elements
    ///   own reference frame.
    /// * 'spatial_inertia' - The spatial inertia for the element, relative to the elements own
    ///   reference frame
    /// * 'parent_id' - The [FrameID] of the parent [ReferenceFrame]
    /// * 'position_relative_to_parent' - The position of the element relative to the parents
    ///   reference frame
    /// * 'orientation_relative_to_parent' - The orientation of the element relative to the parents
    ///   reference frame
    ///
    /// ## Errors
    ///
    /// This method assumes everything has been checked. If something is wrong it will panic.
    fn add_element_unchecked(
        &mut self,
        reference_frame: ReferenceFrame,
        parent_id: FrameID,
        position_relative_to_parent: Translation3<f64>,
        orientation_relative_to_parent: UnitQuaternion<f64>,
        name: String,
        mass: f64,
        center_of_mass: Vector3<f64>,
        moment_of_inertia: Matrix3<f64>,
        spatial_inertia: Matrix6<f64>,
    ) -> Result<FrameID, Error> {
        let id = self.reference_frames.add_element(
            reference_frame,
            parent_id,
            position_relative_to_parent,
            orientation_relative_to_parent,
        )?;

        let element = ChassisElement::new(
            name,
            mass,
            center_of_mass,
            moment_of_inertia,
            spatial_inertia,
            id.clone(),
        );
        self.chassis_elements.insert(id.clone(), element);

        Ok(id.clone())
    }

    /// Adds the chassis element that represents a static joint for the robot.
    ///
    /// It is assumed that the body is the first element to be added.
    ///
    /// ## Parameters
    ///
    /// * 'name' - The name of the new chassis element
    /// * 'degree_of_freedom' - The degree of freedom for the element
    /// * 'parent_id' - The ID of the parent reference frame
    /// * 'position_relative_to_parent' - The position of the element relative to the parent
    ///   reference frame
    /// * 'orientation_relative_to_parent' - The orientation of the element relative to the parent
    ///   reference frame
    /// * 'mass' - The mass, in kg, of the chassis element
    /// * 'center_of_mass' - The location of the center of mass for the element relative to the
    ///   elements own reference frame
    /// * 'moment_of_inertia' - The moment of inertia for the element, relative to the elements
    ///   own reference frame.
    /// * 'spatial_inertia' - The spatial inertia for the element, relative to the elements own
    ///   reference frame
    /// * 'parent_id' - The [FrameID] of the parent [ReferenceFrame]
    ///
    /// ## Errors
    ///
    /// * [Error::MissingFrameElement] - Returned when the parent [ReferenceFrame] is not part of the model.
    /// * [Error::InvalidFrameID] - Returned the parent [ReferenceFrame] is connected to a wheel.
    pub fn add_static_chassis_element(
        &mut self,
        name: String,
        degree_of_freedom: FrameDofType,
        parent_id: FrameID,
        position_relative_to_parent: Translation3<f64>,
        orientation_relative_to_parent: UnitQuaternion<f64>,
        mass: f64,
        center_of_mass: Vector3<f64>,
        moment_of_inertia: Matrix3<f64>,
        spatial_inertia: Matrix6<f64>,
    ) -> Result<FrameID, Error> {
        if !parent_id.is_none() && !self.reference_frames.has_element(&parent_id) {
            return Err(Error::MissingFrameElement {
                id: parent_id.clone(),
            });
        }

        if self.reference_frames.is_wheel(&parent_id)? {
            return Err(Error::InvalidFrameID {
                id: parent_id.clone(),
            });
        }

        let reference_frame = ReferenceFrame::new(name.clone(), degree_of_freedom, false);

        self.add_element_unchecked(
            reference_frame,
            parent_id,
            position_relative_to_parent,
            orientation_relative_to_parent,
            name,
            mass,
            center_of_mass,
            moment_of_inertia,
            spatial_inertia,
        )
    }

    /// Adds a steering element to the robot.
    ///
    /// ## Parameters
    ///
    /// * 'name' - The name of the new chassis element
    /// * 'parent_id' - The ID of the parent reference frame
    /// * 'position_relative_to_parent' - The position of the element relative to the parent
    ///   reference frame
    /// * 'orientation_relative_to_parent' - The orientation of the element relative to the parent
    ///   reference frame
    /// * 'mass' - The mass, in kg, of the chassis element
    /// * 'center_of_mass' - The location of the center of mass for the element relative to the
    ///   elements own reference frame
    /// * 'moment_of_inertia' - The moment of inertia for the element, relative to the elements
    ///   own reference frame.
    /// * 'spatial_inertia' - The spatial inertia for the element, relative to the elements own
    ///   reference frame
    /// * 'parent_id' - The [FrameID] of the parent [ReferenceFrame]
    /// * actuator - A reference to the actuator and its controller for the joint
    ///
    /// ## Errors
    ///
    /// * [Error::MissingFrameElement] - Returned when the parent [ReferenceFrame] is not part of the model.
    /// * [Error::InvalidFrameID] - Returned the parent [ReferenceFrame] is connected to a wheel.
    /// * [Error::MultipleSteeringFramesInChain] - Returned when there is already a steering frame
    ///   in the chain of parent frames
    pub fn add_steering_element(
        &mut self,
        name: String,
        parent_id: FrameID,
        position_relative_to_parent: Translation3<f64>,
        orientation_relative_to_parent: UnitQuaternion<f64>,
        mass: f64,
        center_of_mass: Vector3<f64>,
        moment_of_inertia: Matrix3<f64>,
        spatial_inertia: Matrix6<f64>,
        actuator: Actuator,
    ) -> Result<FrameID, Error> {
        if !parent_id.is_none() && !self.reference_frames.has_element(&parent_id) {
            return Err(Error::MissingFrameElement {
                id: parent_id.clone(),
            });
        }

        if self.reference_frames.is_wheel(&parent_id)? {
            return Err(Error::InvalidFrameID {
                id: parent_id.clone(),
            });
        }

        // There should only be one steering element in the chain
        let mut element_in_chain = &parent_id;
        while !self.is_body(element_in_chain) {
            if self.steering_frame_to_wheel.contains_key(&element_in_chain) {
                return Err(Error::MultipleSteeringFramesInChain { id: parent_id });
            }

            element_in_chain = self.get_parent(element_in_chain)?;
        }

        let reference_frame = ReferenceFrame::new(name.clone(), FrameDofType::RevoluteZ, true);

        self.actuators
            .insert(reference_frame.id().clone(), actuator);

        self.steering_frame_to_wheel
            .insert(reference_frame.id().clone(), FrameID::none());

        self.add_element_unchecked(
            reference_frame,
            parent_id,
            position_relative_to_parent,
            orientation_relative_to_parent,
            name,
            mass,
            center_of_mass,
            moment_of_inertia,
            spatial_inertia,
        )
    }

    /// Adds a passive suspension element to the robot.
    ///
    /// A suspension element is an element that can passively absorb bumps and shocks. Active
    /// suspension elements are combinations of a passive suspension element and an actuated
    /// frame element.
    ///
    /// ## Parameters
    ///
    /// * 'name' - The name of the new chassis element
    /// * 'degree_of_freedom' - The degree of freedom for the element
    /// * 'parent_id' - The ID of the parent reference frame
    /// * 'position_relative_to_parent' - The position of the element relative to the parent
    ///   reference frame
    /// * 'orientation_relative_to_parent' - The orientation of the element relative to the parent
    ///   reference frame
    /// * 'mass' - The mass, in kg, of the chassis element
    /// * 'center_of_mass' - The location of the center of mass for the element relative to the
    ///   elements own reference frame
    /// * 'moment_of_inertia' - The moment of inertia for the element, relative to the elements
    ///   own reference frame.
    /// * 'spatial_inertia' - The spatial inertia for the element, relative to the elements own
    ///   reference frame
    /// * 'parent_id' - The [FrameID] of the parent [ReferenceFrame]
    /// * joint_constraint - A reference to the joint constraint for the joint
    ///
    /// ## Errors
    ///
    /// * [Error::MissingFrameElement] - Returned when the parent [ReferenceFrame] is not part of the model.
    /// * [Error::InvalidFrameID] - Returned the parent [ReferenceFrame] is connected to a wheel.
    pub fn add_suspension_element(
        &mut self,
        name: String,
        degree_of_freedom: FrameDofType,
        parent_id: FrameID,
        position_relative_to_parent: Translation3<f64>,
        orientation_relative_to_parent: UnitQuaternion<f64>,
        mass: f64,
        center_of_mass: Vector3<f64>,
        moment_of_inertia: Matrix3<f64>,
        spatial_inertia: Matrix6<f64>,
        joint_constraint: JointConstraint,
    ) -> Result<FrameID, Error> {
        if !parent_id.is_none() && !self.reference_frames.has_element(&parent_id) {
            return Err(Error::MissingFrameElement {
                id: parent_id.clone(),
            });
        }

        if self.reference_frames.is_wheel(&parent_id)? {
            return Err(Error::InvalidFrameID {
                id: parent_id.clone(),
            });
        }

        let reference_frame = ReferenceFrame::new(name.clone(), degree_of_freedom, false);

        self.joint_constraints
            .insert(reference_frame.id().clone(), joint_constraint);

        self.add_element_unchecked(
            reference_frame,
            parent_id,
            position_relative_to_parent,
            orientation_relative_to_parent,
            name,
            mass,
            center_of_mass,
            moment_of_inertia,
            spatial_inertia,
        )
    }

    /// Adds a new wheel element to the robot
    ///
    /// ## Parameters
    ///
    /// * 'name' - The name of the new wheel element
    /// * 'parent_id' - The ID of the parent reference frame
    /// * 'position_relative_to_parent' - The position of the element relative to the parent
    ///   reference frame
    /// * 'orientation_relative_to_parent' - The orientation of the element relative to the parent
    ///   reference frame
    /// * 'mass' - The mass, in kg, of the chassis element
    /// * 'center_of_mass' - The location of the center of mass for the element relative to the
    ///   elements own reference frame
    /// * 'moment_of_inertia' - The moment of inertia for the element, relative to the elements
    ///   own reference frame.
    /// * 'spatial_inertia' - The spatial inertia for the element, relative to the elements own
    ///   reference frame
    /// * 'parent_id' - The [FrameID] of the parent [ReferenceFrame]
    /// * actuator - A reference to the actuator and its controller for the joint
    ///
    /// ## Errors
    ///
    /// * [Error::MissingFrameElement] - Returned when the parent [ReferenceFrame] is not part of the model.
    /// * [Error::NoSteeringFramesInChain] - Returned when the parent [ReferenceFrame] is not part of the model.
    /// * [Error::InvalidFrameID] - Returned the parent [ReferenceFrame] is connected to a wheel.
    pub fn add_wheel(
        &mut self,
        name: String,
        parent_id: FrameID,
        position_relative_to_parent: Translation3<f64>,
        orientation_relative_to_parent: UnitQuaternion<f64>,
        mass: f64,
        center_of_mass: Vector3<f64>,
        moment_of_inertia: Matrix3<f64>,
        spatial_inertia: Matrix6<f64>,
        actuator: Actuator,
    ) -> Result<FrameID, Error> {
        if !parent_id.is_none() && !self.reference_frames.has_element(&parent_id) {
            return Err(Error::MissingFrameElement {
                id: parent_id.clone(),
            });
        }

        if self.reference_frames.is_wheel(&parent_id)? {
            return Err(Error::InvalidFrameID {
                id: parent_id.clone(),
            });
        }

        // There should exactly one steering element in the chain
        let mut element_in_chain = &parent_id;
        let mut steering_frame_id = FrameID::none();
        while !self.is_body(element_in_chain) {
            if self.steering_frame_to_wheel.contains_key(&element_in_chain) {
                steering_frame_id = element_in_chain.clone();
                break;
            }

            element_in_chain = self.get_parent(element_in_chain)?;
        }

        if steering_frame_id.is_none() {
            return Err(Error::NoSteeringFramesInChain { id: parent_id });
        }

        let reference_frame = ReferenceFrame::new(name.clone(), FrameDofType::RevoluteY, true);

        self.actuators
            .insert(reference_frame.id().clone(), actuator);

        self.steering_frame_to_wheel
            .insert(steering_frame_id.clone(), reference_frame.id().clone());

        self.wheel_to_steering_frame
            .insert(reference_frame.id().clone(), steering_frame_id.clone());

        self.add_element_unchecked(
            reference_frame,
            parent_id,
            position_relative_to_parent,
            orientation_relative_to_parent,
            name,
            mass,
            center_of_mass,
            moment_of_inertia,
            spatial_inertia,
        )
    }

    /// Returns the [Actuator] for the given joint
    ///
    /// ## Parameters
    ///
    /// * 'frame_id' - The [FrameID] of the element that should be returned.
    ///
    /// ## Errors
    ///
    /// * [Error::MissingFrameElement] - Returned when the [ReferenceFrame] is not an actuated joint.
    pub fn get_actuator(&mut self, frame_id: &FrameID) -> Result<&mut Actuator, Error> {
        match self.actuators.get_mut(frame_id) {
            Some(a) => return Ok(a),
            None => {
                return Err(Error::MissingFrameElement {
                    id: frame_id.clone(),
                })
            }
        }
    }

    /// Returns the [FrameID] of the body element.
    ///
    /// ## Errors
    ///
    /// * [Error::MissingFrameElement] - Returned when there are no elements in the model.
    pub fn get_body(&self) -> Result<&FrameID, Error> {
        if self.reference_frames.is_empty() {
            return Err(Error::MissingFrameElement {
                id: FrameID::none(),
            });
        }

        let frame = self.reference_frames.get_body_element()?;
        Ok(frame.id())
    }

    /// Returns the [ChassisElement] for a given joint
    ///
    /// ## Parameters
    ///
    /// * 'frame_id' - The [FrameID] of the element that should be returned.
    ///
    /// ## Errors
    ///
    /// * [Error::MissingFrameElement] - Returned when the [ReferenceFrame] is not part of the model.
    pub fn get_chassis_element(&self, frame_id: &FrameID) -> Result<&ChassisElement, Error> {
        match self.chassis_elements.get(frame_id) {
            Some(c) => return Ok(c),
            None => {
                return Err(Error::MissingFrameElement {
                    id: frame_id.clone(),
                })
            }
        }
    }

    /// Returns the collection containing all the [FrameID] of the child elements of the
    /// element with the given ID.
    ///
    /// ## Parameters
    ///
    /// * 'frame_id' - The [FrameID] of the element from which the child elements should be returned.
    ///
    /// ## Errors
    ///
    /// * [Error::MissingFrameElement] - Returned when the [ReferenceFrame] is not part of the model.
    pub fn get_children(&self, frame_id: &FrameID) -> Result<Vec<&FrameID>, Error> {
        if !self.reference_frames.has_element(frame_id) {
            return Err(Error::MissingFrameElement {
                id: frame_id.clone(),
            });
        }

        let child_ids: Vec<&FrameID> = self
            .reference_frames
            .get_children(frame_id)?
            .map(|e| e.id())
            .collect();
        Ok(child_ids)
    }

    /// Returns the [FrameDofType] for the given frame
    ///
    /// ## Parameters
    ///
    /// * 'frame_id' - The [FrameID] of the element from which the [FrameDofType] should be returned.
    ///
    /// ## Errors
    ///
    /// * [Error::MissingFrameElement] - Returned when the [ReferenceFrame] is not part of the model.
    pub fn get_frame_degree_of_freedom(&self, frame_id: &FrameID) -> Result<FrameDofType, Error> {
        if !self.reference_frames.has_element(frame_id) {
            return Err(Error::MissingFrameElement {
                id: frame_id.clone(),
            });
        }

        let frame = self.reference_frames.get_element(frame_id)?;
        Ok(frame.degree_of_freedom_kind())
    }

    /// Returns the homogeneous transform matrix from the given reference frame to the
    /// destination frame, taking into account the current position and orientation of the
    /// frame relative to the destination frame.
    ///
    /// ## Parameters
    ///
    /// * 'from' - The source element for which the transform is requested
    /// * 'to' - The target element
    ///
    /// ## Errors
    ///
    pub fn get_homogeneous_transform_between_frames(
        &self,
        from: &FrameID,
        to: &FrameID,
    ) -> Result<Matrix4<f64>, Error> {
        if !self.reference_frames.has_element(from) {
            return Err(Error::MissingFrameElement {
                id: from.clone(),
            });
        }

        if !self.reference_frames.has_element(to) {
            return Err(Error::MissingFrameElement {
                id: to.clone(),
            });
        }

        if from == to {
            return Ok(Matrix4::<f64>::identity())
        }

        // If 'to' is an ancestor then we can just calculate the stack
        if self.is_ancestor(from, to) {
            return self.get_homogeneous_transform_to_ancestor(from, to);
        }

        // 'to' is a sibbling. Calculate both stacks and invert the sibbling stack
        let from_transform_to_body = self.get_homogeneous_transform_to_body(from)?;
        let mut to_transform_to_body = self.get_homogeneous_transform_to_body(to)?;

        // Invert the to transform
        let invert_result = to_transform_to_body.try_inverse_mut();
        if !invert_result {
            // This really shouldn't happen because homogeneous transforms should be invertible. So now we're in trouble ....
            return Err(Error::FailedToComputeTransform { from: self.get_body()?.clone(), to: to.clone() });
        }

        Ok(from_transform_to_body * to_transform_to_body)
    }

    /// Returns the homogeneous transform matrix from the given reference frame to the
    /// a parent element further up the chain, taking into account the current position and
    /// orientation of the frame relative to the parent frame.
    ///
    /// It is assumed that the parent frame is in the chain from the 'from' element to the
    /// body.
    ///
    /// ## Parameters
    ///
    /// * 'from' - The source element for which the transform is requested
    /// * 'to' - The target parent element.
    ///
    /// ## Errors
    ///
    /// * [Error::MissingFrameElement] - Returned when the [ReferenceFrame] is not part of the model
    pub fn get_homogeneous_transform_to_ancestor(&self, from: &FrameID, to: &FrameID) -> Result<Matrix4<f64>, Error> {
        if !self.reference_frames.has_element(from) {
            return Err(Error::MissingFrameElement {
                id: from.clone(),
            });
        }

        if !self.reference_frames.has_element(to) {
            return Err(Error::MissingFrameElement {
                id: to.clone(),
            });
        }

        if from == to {
            return Ok(Matrix4::<f64>::identity())
        }

        let mut transform = Matrix4::<f64>::identity();
        let mut parent_element = self.reference_frames.get_parent(from)?;
        let mut child_element = self.reference_frames.get_element(from)?;
        while child_element.id() != to {
            let dof = child_element.degree_of_freedom_kind();

            let transform_result = self
                .reference_frames
                .get_homogeneous_transform_to_parent(child_element.id())?;

            let motion_transform = match self.actuators.get(parent_element.id()) {
                None => Matrix4::<f64>::identity(),
                Some(a) => self.transform_for_motion(a, dof),
            };

            transform = motion_transform * (transform_result.to_homogeneous()) * transform;

            child_element = parent_element;
            parent_element = self.reference_frames.get_parent(child_element.id())?;
        }

        Ok(transform)
    }

    /// Returns the homogeneous transform matrix from the given reference frame to the
    /// parent frame, taking into account the current position and orientation of the
    /// frame relative to the parent frame.
    ///
    /// ## Parameters
    ///
    /// * 'starting_element' - The source element for which the transform is requested
    ///
    /// ## Errors
    ///
    pub fn get_homogeneous_transform_to_parent(
        &self,
        starting_element: &FrameID,
    ) -> Result<Matrix4<f64>, Error> {
        if !self.reference_frames.has_element(starting_element) {
            return Err(Error::MissingFrameElement {
                id: starting_element.clone(),
            });
        }

        let is_body = self.reference_frames.is_body(starting_element)?;
        if is_body {
            return Ok(Matrix4::<f64>::identity())
        }

        let reference_frame = self.reference_frames.get_element(starting_element)?;
        let dof = reference_frame.degree_of_freedom_kind();

        let transform_result = self
            .reference_frames
            .get_homogeneous_transform_to_parent(starting_element)?;

        let parent_frame = self.reference_frames.get_parent(starting_element)?;

        let motion_transform = match self.actuators.get(parent_frame.id()) {
            None => Matrix4::<f64>::identity(),
            Some(a) => self.transform_for_motion(a, dof),
        };

        Ok(motion_transform * (transform_result.to_homogeneous()))
    }

    /// Returns the homogeneous transform matrix from the given reference frame to the
    /// body frame, taking into account the current position and orientation of the
    /// frame relative to the body frame.
    ///
    /// ## Parameters
    ///
    /// * 'starting_element' - The source element for which the transform is requested
    ///
    /// ## Errors
    ///
    /// * [Error::MissingFrameElement] - Returned when the [ReferenceFrame] is not part of the model
    pub fn get_homogeneous_transform_to_body(
        &self,
        starting_element: &FrameID,
    ) -> Result<Matrix4<f64>, Error> {
        let body_frame = self.get_body()?;
        self.get_homogeneous_transform_to_ancestor(starting_element, body_frame)
    }

    /// Returns the [FrameID] of the parent of the given element.
    ///
    /// ## Parameters
    ///
    /// * 'frame_id' - The [FrameID] of the element from which the parent [FrameID] should be returned.
    ///
    /// ## Errors
    ///
    /// * [Error::MissingFrameElement] - Returned when the [ReferenceFrame] is not part of the model
    pub fn get_parent(&self, frame_id: &FrameID) -> Result<&FrameID, Error> {
        if !self.reference_frames.has_element(frame_id) {
            return Err(Error::MissingFrameElement {
                id: frame_id.clone(),
            });
        }

        let parent = self.reference_frames.get_parent(frame_id)?;
        Ok(parent.id())
    }

    /// Returns the [ReferenceFrame] for a given joint
    ///
    /// ## Parameters
    ///
    /// * 'frame_id' - The [FrameID] of the [ReferenceFrame] that should be returned.
    ///
    /// ## Errors
    ///
    /// /// * [Error::MissingFrameElement] - Returned when the [ReferenceFrame] is not part of the model
    pub fn get_reference_frame(&self, frame_id: &FrameID) -> Result<&ReferenceFrame, Error> {
        if !self.reference_frames.has_element(frame_id) {
            return Err(Error::MissingFrameElement {
                id: frame_id.clone(),
            });
        }

        self.reference_frames.get_element(frame_id)
    }

    /// Returns the [FrameID] of the steering frame that is linked to the given wheel frame
    ///
    /// ## Parameters
    ///
    /// * 'wheel_frame' - The [FrameID] of the wheel for which the steering frame should be located.
    ///
    ///  ## Errors
    ///
    /// * [Error::MissingFrameElement] - Returned when the [ReferenceFrame] is not part of the model.
    /// * [Error::NoSteeringFramesInChain] - Returned when there is no steering frame attached to the wheel.
    pub fn get_steering_frame_for_wheel(&self, wheel_frame: &FrameID) -> Result<&FrameID, Error> {
        if !self.reference_frames.has_element(wheel_frame) {
            return Err(Error::MissingFrameElement {
                id: wheel_frame.clone(),
            });
        }

        let id_ref: &FrameID;
        match self.wheel_to_steering_frame.get(wheel_frame) {
            Some(i) => id_ref = i,
            None => {
                return Err(Error::NoSteeringFramesInChain {
                    id: wheel_frame.clone(),
                })
            }
        };

        Ok(id_ref)
    }

    /// Returns a list of [FrameID] of all the wheels
    pub fn get_wheels(&self) -> Result<Vec<&FrameID>, Error> {
        let list = self
            .reference_frames
            .get_wheels()?
            .map(|f| f.id())
            .collect();
        Ok(list)
    }

    /// Indicates whether there are any actuated joints between the steering frames and the body frame
    /// or the wheel frame and the steering frame.
    pub fn has_active_suspension(&self) -> bool {
        let number_of_actuators = self.actuators.len();
        let number_of_wheels = self.reference_frames.number_of_wheels();

        // Both the wheels and the steering frames are actuated, so if there are
        // more actuators then there are wheels and steering frames then we have
        // active suspension
        number_of_actuators > 2 * number_of_wheels
    }

    /// Indicates whether the given joint has a sensor
    ///
    /// ## Parameters
    ///
    /// * 'frame_id' - The [FrameID] of the joint.
    pub fn has_sensor(&self, frame_id: &FrameID) -> bool {
        self.sensors.contains_key(frame_id)
    }

    /// Returns a value indicating if the joint with the given [FrameID] is an actuated joint
    ///
    /// ## Parameters
    ///
    /// * 'frame_id' - The [FrameID] of the joint.
    pub fn is_actuated(&self, frame_id: &FrameID) -> bool {
        self.actuators.contains_key(frame_id)
    }

    /// Returns a value indicating if the given 'to' frame is an ancestor of the 'from' frame.
    ///
    /// ## Parameters
    ///
    /// * 'from' - The starting frame
    /// * 'to' - The potential ancestor frame
    pub fn is_ancestor(&self, from: &FrameID, to: &FrameID) -> bool {
        if !self.reference_frames.has_element(from) {
            return false;
        }

        if !self.reference_frames.has_element(to) {
            return false;
        }

        if from == to {
            return true;
        }

        let mut frame_id = from;
        while !self.is_body(frame_id) {
            let parent = match self.get_parent(frame_id) {
                Ok(f) => f,
                Err(_) => return false,
            };

            if parent == to {
                return true
            }

            frame_id = parent;
        }

        return false;
    }

    /// Returns a value indicating if the given [FrameID] points to the body frame.
    ///
    /// Note that providing a [FrameID] to a non-existing frame returns 'false'
    ///
    /// ## Parameters
    ///
    /// * 'frame_id' - The [FrameID] of the joint.
    pub fn is_body(&self, frame_id: &FrameID) -> bool {
        match self.reference_frames.is_body(frame_id) {
            Ok(b) => b,
            Err(_) => false,
        }
    }

    pub fn is_valid(&self) -> (bool, Vec<String>) {
        // Validate that the model is valid. Return what isn't valid
        //
        // Validations:
        // - There is at least 3 wheels
        // - If there is a steering joint, then there must be one steering joint for each wheel
        // - Each steering joint has a z-rotation. The steering xy-plane can be different than the body xy-plane
        // - Each wheel has rotates in the xz-plane
        // - There are an identical number of joints between each steering joint and the body
        // - The joints between each steering joint and the body are the same type in the same order
        // - There are an identical number of joints between each wheel and the steering joint
        // -
        todo!()

        foobar()
    }

    /// Returns a value indicating if the given [FrameID] points to the world frame
    pub fn is_world(&self, frame_id: &FrameID) -> bool {
        frame_id.is_none()
    }

    /// Returns a new [SwerveRobotModel] instance.
    pub fn new() -> Self {
        Self {
            reference_frames: KinematicTree::new(),
            chassis_elements: HashMap::new(),
            steering_frame_to_wheel: HashMap::new(),
            wheel_to_steering_frame: HashMap::new(),
            actuators: HashMap::new(),
            sensors: HashMap::new(),
            joint_constraints: HashMap::new(),
        }
    }

    /// Returns the number of elements with a joint constraint.
    pub fn number_of_joint_constraints(&self) -> usize {
        self.joint_constraints.len()
    }

    /// Returns the number of wheels the robot has.
    pub fn number_of_wheels(&self) -> usize {
        self.reference_frames.number_of_wheels()
    }

    fn transform_for_motion(&self, actuator: &Actuator, dof: FrameDofType) -> Matrix4<f64> {
        match dof {
            FrameDofType::Static => Matrix4::<f64>::identity(),
            FrameDofType::RevoluteX => self.transform_for_revolute_x_motion(actuator),
            FrameDofType::RevoluteY => self.transform_for_revolute_y_motion(actuator),
            FrameDofType::RevoluteZ => self.transform_for_revolute_z_motion(actuator),
            FrameDofType::PrismaticX => self.transform_for_prismatic_x_motion(actuator),
            FrameDofType::PrismaticY => self.transform_for_prismatic_y_motion(actuator),
            FrameDofType::PrismaticZ => self.transform_for_prismatic_z_motion(actuator),
        }
    }

    fn transform_for_prismatic_x_motion(&self, actuator: &Actuator) -> Matrix4<f64> {
        let mut result = Matrix4::<f64>::identity();

        let distance_moved = match actuator.get_value() {
            Ok(v) => todo!(),
            Err(_) => 0.0,
        };
        result[(0, 3)] = distance_moved;

        result
    }

    fn transform_for_prismatic_y_motion(&self, actuator: &Actuator) -> Matrix4<f64> {
        let mut result = Matrix4::<f64>::identity();

        let distance_moved = match actuator.get_value() {
            Ok(v) => todo!(),
            Err(_) => 0.0,
        };
        result[(1, 3)] = distance_moved;

        result
    }

    fn transform_for_prismatic_z_motion(&self, actuator: &Actuator) -> Matrix4<f64> {
        let mut result = Matrix4::<f64>::identity();

        let distance_moved = match actuator.get_value() {
            Ok(v) => todo!(),
            Err(_) => 0.0,
        };
        result[(2, 3)] = distance_moved;

        result
    }

    fn transform_for_revolute_x_motion(&self, actuator: &Actuator) -> Matrix4<f64> {
        let mut result = Matrix4::<f64>::identity();

        let distance_rotated = match actuator.get_value() {
            Ok(v) => todo!(),
            Err(_) => 0.0,
        };

        // Rotation matrix for rotation around the x-axis is:
        //
        // [1    0           0      ]
        // [0    cos(θ)   -sin(θ)   ]
        // [0    sin(θ)    cos(θ)   ]

        result[(1, 1)] = f64::cos(distance_rotated);
        result[(1, 2)] = -f64::sin(distance_rotated);
        result[(2, 1)] = f64::sin(distance_rotated);
        result[(2, 2)] = f64::cos(distance_rotated);

        result
    }

    fn transform_for_revolute_y_motion(&self, actuator: &Actuator) -> Matrix4<f64> {
        let mut result = Matrix4::<f64>::identity();

        let distance_rotated = match actuator.get_value() {
            Ok(v) => todo!(),
            Err(_) => 0.0,
        };

        // Rotation matrix for rotation around the y-axis is:
        //
        // [ cos(θ)    0    sin(θ) ]
        // [   0       1      0    ]
        // [-sin(θ)    0    cos(θ) ]

        result[(0, 0)] = f64::cos(distance_rotated);
        result[(0, 2)] = f64::sin(distance_rotated);
        result[(2, 0)] = -f64::sin(distance_rotated);
        result[(2, 2)] = f64::cos(distance_rotated);

        result
    }

    fn transform_for_revolute_z_motion(&self, actuator: &Actuator) -> Matrix4<f64> {
        let mut result = Matrix4::<f64>::identity();

        let distance_rotated = match actuator.get_value() {
            Ok(v) => todo!(),
            Err(_) => 0.0,
        };

        // Rotation matrix for rotation around the z-axis is:
        //
        // [ cos(θ)   -sin(θ)   0 ]
        // [ sin(θ)    cos(θ)   0 ]
        // [   0         0      1 ]
        result[(0, 0)] = f64::cos(distance_rotated);
        result[(0, 1)] = -f64::sin(distance_rotated);
        result[(1, 0)] = f64::sin(distance_rotated);
        result[(1, 1)] = f64::cos(distance_rotated);

        result
    }
}
