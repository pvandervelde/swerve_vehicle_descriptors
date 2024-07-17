use std::{f64::consts::PI, time::Duration};

use crossbeam_channel::{Receiver, Sender};
use float_cmp::{ApproxEq, F64Margin};
use nalgebra::{
    Matrix3, Matrix4, Matrix6, RowVector4, Translation3, UnitQuaternion, Vector3, Vector4,
};

use crate::{
    change_notification_processing::{ChangeID, HardwareChangeProcessor},
    hardware::{
        actuator_interface::{ActuatorAvailableRatesOfChange, HardwareActuator},
        joint_state::JointState,
    },
    model_elements::frame_elements::{
        Actuator, FrameDofType, FrameID, JointConstraint, ReferenceFrame,
    },
    number_space::NumberSpaceType,
    Error,
};

use super::{KinematicTree, MotionModel};

fn create_generic_non_actuated_element(name: String) -> ReferenceFrame {
    let degree_of_freedom_kind = FrameDofType::PrismaticX;
    let is_actuated = false;

    ReferenceFrame::new(name, degree_of_freedom_kind, is_actuated)
}

fn create_wheel_element(name: String) -> ReferenceFrame {
    let degree_of_freedom_kind = FrameDofType::RevoluteY;
    let is_actuated = true;

    ReferenceFrame::new(name, degree_of_freedom_kind, is_actuated)
}

// KinematicTree

#[test]
fn when_adding_an_single_element_with_no_parent_to_a_kinematic_tree_it_should_be_a_body() {
    let mut tree = KinematicTree::new();

    let name = "a".to_string();
    let element = create_generic_non_actuated_element(name.clone());
    let element_id = element.id().clone();

    // Get the mutable tree to add something
    {
        match tree.add_element(
            element,
            FrameID::none(),
            Translation3::<f64>::identity(),
            UnitQuaternion::identity(),
        ) {
            Err(e) => {
                assert!(
                    false,
                    "Got an error adding an element to the tree. Should not have. Error was: {}",
                    e
                );
            }
            Ok(id) => {
                assert_eq!(id.as_ref(), &element_id);
            }
        };
    }

    let element_ref = tree.get_element(&element_id).unwrap();
    assert_eq!(element_ref.name(), name);

    let body_ref = tree.get_body_element().unwrap();

    assert_eq!(body_ref.name(), name);
}

#[test]
fn when_adding_an_multiple_elements_to_a_kinematic_tree_it_should_only_have_one_body() {
    let mut tree = KinematicTree::new();

    let first_name = "a".to_string();
    let first_element = create_generic_non_actuated_element(first_name);
    let first_id = first_element.id().clone();

    let second_name = "b".to_string();
    let second_element = create_generic_non_actuated_element(second_name);
    let second_id = second_element.id().clone();

    // Get the mutable tree to add something
    {
        match tree.add_element(
            first_element,
            FrameID::none(),
            Translation3::<f64>::identity(),
            UnitQuaternion::identity(),
        ) {
            Err(e) => {
                assert!(
                    false,
                    "Got an error adding an element to the tree. Should not have. Error was: {}",
                    e
                );
            }
            Ok(id) => {
                assert_eq!(id.as_ref(), &first_id);
            }
        };

        match tree.add_element(
            second_element,
            first_id.clone(),
            Translation3::<f64>::identity(),
            UnitQuaternion::identity(),
        ) {
            Err(e) => {
                assert!(
                    false,
                    "Got an error adding an element to the tree. Should not have. Error was: {}",
                    e
                );
            }
            Ok(id) => {
                assert_eq!(id.as_ref(), &second_id);
            }
        };
    }

    let imtree = &tree;
    let coll = imtree.get_elements().collect::<Vec<&ReferenceFrame>>();
    assert_eq!(2, coll.len());

    assert!(coll.iter().any(|e| *e.id() == first_id));
    assert!(coll.iter().any(|e| *e.id() == second_id));
}

#[test]
fn when_adding_multiple_elements_without_parents_to_a_kinematic_tree_it_should_error() {
    let mut tree = KinematicTree::new();

    let first_name = "a".to_string();
    let first_element = create_generic_non_actuated_element(first_name);
    let first_id = first_element.id().clone();

    let second_name = "b".to_string();
    let second_element = create_generic_non_actuated_element(second_name);

    // Get the mutable tree to add something
    {
        match tree.add_element(
            first_element,
            FrameID::none(),
            Translation3::<f64>::identity(),
            UnitQuaternion::identity(),
        ) {
            Err(e) => {
                assert!(
                    false,
                    "Got an error adding an element to the tree. Should not have. Error was: {}",
                    e
                );
            }
            Ok(id) => {
                assert_eq!(id.as_ref(), &first_id);
            }
        };

        assert!(tree
            .add_element(
                second_element,
                FrameID::none(),
                Translation3::<f64>::identity(),
                UnitQuaternion::identity()
            )
            .is_err())
    }
}

#[test]
fn when_adding_an_element_to_a_kinematic_tree_it_should_only_be_a_wheel_in_a_specific_case() {
    let mut tree = KinematicTree::new();

    let first_name = "a".to_string();
    let first_element = create_generic_non_actuated_element(first_name);
    let first_id = first_element.id().clone();

    let second_name = "b".to_string();
    let second_element = create_generic_non_actuated_element(second_name);
    let second_id = second_element.id().clone();

    let third_name = "c".to_string();
    let third_element = create_wheel_element(third_name);
    let third_id = third_element.id().clone();

    // Get the mutable tree to add something
    {
        match tree.add_element(
            first_element,
            FrameID::none(),
            Translation3::<f64>::identity(),
            UnitQuaternion::identity(),
        ) {
            Err(e) => {
                assert!(
                    false,
                    "Got an error adding an element to the tree. Should not have. Error was: {}",
                    e
                );
            }
            Ok(id) => {
                assert_eq!(id.as_ref(), &first_id);
            }
        };

        match tree.add_element(
            second_element,
            first_id.clone(),
            Translation3::<f64>::identity(),
            UnitQuaternion::identity(),
        ) {
            Err(e) => {
                assert!(
                    false,
                    "Got an error adding an element to the tree. Should not have. Error was: {}",
                    e
                );
            }
            Ok(id) => {
                assert_eq!(id.as_ref(), &second_id);
            }
        };

        match tree.add_element(
            third_element,
            first_id.clone(),
            Translation3::<f64>::identity(),
            UnitQuaternion::identity(),
        ) {
            Err(e) => {
                assert!(
                    false,
                    "Got an error adding an element to the tree. Should not have. Error was: {}",
                    e
                );
            }
            Ok(id) => {
                assert_eq!(id.as_ref(), &third_id);
            }
        };
    }

    let imtree = &tree;
    let coll = imtree.get_elements().collect::<Vec<&ReferenceFrame>>();
    assert_eq!(3, coll.len());

    assert!(!imtree.is_wheel(&first_id).unwrap());
    assert!(!imtree.is_wheel(&second_id).unwrap());
    assert!(imtree.is_wheel(&third_id).unwrap());

    let wheels: Vec<&ReferenceFrame> = imtree.get_wheels().unwrap().collect();

    assert_eq!(1, wheels.len());
    assert_eq!(&third_id, wheels[0].id());
}

#[test]
fn when_adding_a_child_to_an_element_in_a_kinematic_tree_it_should_not_be_a_wheel_anymore() {
    let mut tree = KinematicTree::new();

    let first_name = "a".to_string();
    let first_element = create_generic_non_actuated_element(first_name);
    let first_id = first_element.id().clone();

    let second_name = "b".to_string();
    let second_element = create_wheel_element(second_name);
    let second_id = second_element.id().clone();

    let third_name = "c".to_string();
    let third_element = create_wheel_element(third_name);
    let third_id = third_element.id().clone();

    // Get the mutable tree to add something
    {
        match tree.add_element(
            first_element,
            FrameID::none(),
            Translation3::<f64>::identity(),
            UnitQuaternion::identity(),
        ) {
            Err(e) => {
                assert!(
                    false,
                    "Got an error adding an element to the tree. Should not have. Error was: {}",
                    e
                );
            }
            Ok(id) => {
                assert_eq!(id.as_ref(), &first_id);
            }
        };

        match tree.add_element(
            second_element,
            first_id.clone(),
            Translation3::<f64>::identity(),
            UnitQuaternion::identity(),
        ) {
            Err(e) => {
                assert!(
                    false,
                    "Got an error adding an element to the tree. Should not have. Error was: {}",
                    e
                );
            }
            Ok(id) => {
                assert_eq!(id.as_ref(), &second_id);
            }
        };

        assert!(!tree.is_wheel(&first_id).unwrap());
        assert!(tree.is_wheel(&second_id).unwrap());

        let wheels = tree.get_wheels().unwrap();
        for elt in wheels {
            let id_ref = elt.id();
            if id_ref != &second_id {
                assert!(false, "Found an ID for an invalid wheel. ID:")
            }
        }

        match tree.add_element(
            third_element,
            second_id.clone(),
            Translation3::<f64>::identity(),
            UnitQuaternion::identity(),
        ) {
            Err(e) => {
                assert!(
                    false,
                    "Got an error adding an element to the tree. Should not have. Error was: {}",
                    e
                );
            }
            Ok(id) => {
                assert_eq!(id.as_ref(), &third_id);
            }
        };

        assert!(!tree.is_wheel(&first_id).unwrap());
        assert!(!tree.is_wheel(&second_id).unwrap());
        assert!(tree.is_wheel(&third_id).unwrap());

        let wheels = tree.get_wheels().unwrap();
        for elt in wheels {
            let id_ref = elt.id();
            if id_ref != &third_id {
                assert!(false, "Found an ID for an invalid wheel. ID")
            }
        }
    }
}

#[test]
fn when_adding_an_element_with_an_unknown_parent_to_a_kinematic_tree_it_should_error() {
    let mut tree = KinematicTree::new();

    let first_name = "a".to_string();
    let first_element = create_generic_non_actuated_element(first_name);
    let first_id = first_element.id().clone();

    let second_name = "b".to_string();
    let second_element = create_wheel_element(second_name);
    let second_id = second_element.id().clone();

    let third_name = "c".to_string();
    let third_element = create_wheel_element(third_name);

    // Get the mutable tree to add something
    {
        match tree.add_element(
            first_element,
            FrameID::none(),
            Translation3::<f64>::identity(),
            UnitQuaternion::identity(),
        ) {
            Err(e) => {
                assert!(
                    false,
                    "Got an error adding an element to the tree. Should not have. Error was: {}",
                    e
                );
            }
            Ok(id) => {
                assert_eq!(id.as_ref(), &first_id);
            }
        };

        match tree.add_element(
            third_element,
            second_id.clone(),
            Translation3::<f64>::identity(),
            UnitQuaternion::identity(),
        ) {
            Err(e) => assert_eq!(
                e,
                Error::MissingFrameElement {
                    id: second_id.clone(),
                }
            ),
            Ok(_) => assert!(
                false,
                "was able to add an element with a non-existant parent."
            ),
        };
    }
}

#[test]
fn when_getting_the_body_with_no_frame_elements_it_should_error() {
    let tree = KinematicTree::new();
    match tree.get_body_element() {
        Ok(_) => assert!(
            false,
            "Retrieved a body element when no elements were present in the tree."
        ),
        Err(e) => assert_eq!(
            e,
            Error::MissingFrameElement {
                id: FrameID::none()
            }
        ),
    };
}

#[test]
fn when_getting_the_children_it_should_return_all_the_directly_connected_elements() {
    let mut tree = KinematicTree::new();

    let first_name = "a".to_string();
    let first_element = create_generic_non_actuated_element(first_name);
    let first_id = first_element.id().clone();

    let second_name = "b".to_string();
    let second_element = create_generic_non_actuated_element(second_name);
    let second_id = second_element.id().clone();

    let third_name = "c".to_string();
    let third_element = create_wheel_element(third_name);
    let third_id = third_element.id().clone();

    // Get the mutable tree to add something
    {
        match tree.add_element(
            first_element,
            FrameID::none(),
            Translation3::<f64>::identity(),
            UnitQuaternion::identity(),
        ) {
            Err(e) => {
                assert!(
                    false,
                    "Got an error adding an element to the tree. Should not have. Error was: {}",
                    e
                );
            }
            Ok(id) => {
                assert_eq!(id.as_ref(), &first_id);
            }
        };

        match tree.add_element(
            second_element,
            first_id.clone(),
            Translation3::<f64>::identity(),
            UnitQuaternion::identity(),
        ) {
            Err(e) => {
                assert!(
                    false,
                    "Got an error adding an element to the tree. Should not have. Error was: {}",
                    e
                );
            }
            Ok(id) => {
                assert_eq!(id.as_ref(), &second_id);
            }
        };

        match tree.add_element(
            third_element,
            first_id.clone(),
            Translation3::<f64>::identity(),
            UnitQuaternion::identity(),
        ) {
            Err(e) => {
                assert!(
                    false,
                    "Got an error adding an element to the tree. Should not have. Error was: {}",
                    e
                );
            }
            Ok(id) => {
                assert_eq!(id.as_ref(), &third_id);
            }
        };
    }

    match tree.get_children(&first_id) {
        Err(e) => assert!(
            false,
            "Got an error retrieving the children, but should not have. Error: {}.",
            e,
        ),
        Ok(c) => {
            for elt in c {
                let id_ref = elt.id();
                if id_ref != &second_id && id_ref != &third_id {
                    assert!(false, "Found an ID for an invalid child. ID")
                }
            }
        }
    };
}

#[test]
fn when_getting_the_children_with_invalid_parent_it_should_error() {
    let mut tree = KinematicTree::new();

    let first_name = "a".to_string();
    let first_element = create_generic_non_actuated_element(first_name);
    let first_id = first_element.id().clone();

    let second_name = "b".to_string();
    let second_element = create_generic_non_actuated_element(second_name);
    let second_id = second_element.id().clone();

    let third_name = "c".to_string();
    let third_element = create_wheel_element(third_name);
    let third_id = third_element.id().clone();

    // Get the mutable tree to add something
    {
        match tree.add_element(
            first_element,
            FrameID::none(),
            Translation3::<f64>::identity(),
            UnitQuaternion::identity(),
        ) {
            Err(e) => {
                assert!(
                    false,
                    "Got an error adding an element to the tree. Should not have. Error was: {}",
                    e
                );
            }
            Ok(id) => {
                assert_eq!(id.as_ref(), &first_id);
            }
        };

        match tree.add_element(
            second_element,
            first_id.clone(),
            Translation3::<f64>::identity(),
            UnitQuaternion::identity(),
        ) {
            Err(e) => {
                assert!(
                    false,
                    "Got an error adding an element to the tree. Should not have. Error was: {}",
                    e
                );
            }
            Ok(id) => {
                assert_eq!(id.as_ref(), &second_id);
            }
        };

        match tree.add_element(
            third_element,
            first_id.clone(),
            Translation3::<f64>::identity(),
            UnitQuaternion::identity(),
        ) {
            Err(e) => {
                assert!(
                    false,
                    "Got an error adding an element to the tree. Should not have. Error was: {}",
                    e
                );
            }
            Ok(id) => {
                assert_eq!(id.as_ref(), &third_id);
            }
        };
    }

    match tree.get_children(&second_id) {
        Err(_) => assert!(false),
        Ok(mut i) => {
            assert!(!i.any(|_e| true));
            //assert!(false, "Found children for an element that is not a parent.")
        }
    };
}

#[test]
fn when_getting_the_children_with_no_parent_it_should_error() {
    let mut tree = KinematicTree::new();

    let first_name = "a".to_string();
    let first_element = create_generic_non_actuated_element(first_name);
    let first_id = first_element.id().clone();

    let second_name = "b".to_string();
    let second_element = create_generic_non_actuated_element(second_name);
    let second_id = second_element.id().clone();

    let third_name = "c".to_string();
    let third_element = create_wheel_element(third_name);
    let third_id = third_element.id().clone();

    // Get the mutable tree to add something
    {
        match tree.add_element(
            first_element,
            FrameID::none(),
            Translation3::<f64>::identity(),
            UnitQuaternion::identity(),
        ) {
            Err(e) => {
                assert!(
                    false,
                    "Got an error adding an element to the tree. Should not have. Error was: {}",
                    e
                );
            }
            Ok(id) => {
                assert_eq!(id.as_ref(), &first_id);
            }
        };

        match tree.add_element(
            second_element,
            first_id.clone(),
            Translation3::<f64>::identity(),
            UnitQuaternion::identity(),
        ) {
            Err(e) => {
                assert!(
                    false,
                    "Got an error adding an element to the tree. Should not have. Error was: {}",
                    e
                );
            }
            Ok(id) => {
                assert_eq!(id.as_ref(), &second_id);
            }
        };

        match tree.add_element(
            third_element,
            first_id.clone(),
            Translation3::<f64>::identity(),
            UnitQuaternion::identity(),
        ) {
            Err(e) => {
                assert!(
                    false,
                    "Got an error adding an element to the tree. Should not have. Error was: {}",
                    e
                );
            }
            Ok(id) => {
                assert_eq!(id.as_ref(), &third_id);
            }
        };
    }

    match tree.get_children(&FrameID::none()) {
        Err(e) => assert!(
            e == Error::InvalidFrameID {
                id: FrameID::none()
            }
        ),
        Ok(_) => assert!(false, "Found children for an element that is not a parent."),
    };
}

#[test]
fn when_getting_the_parent_it_should_return_the_correct_element() {
    let mut tree = KinematicTree::new();

    let first_name = "a".to_string();
    let first_element = create_generic_non_actuated_element(first_name);
    let first_id = first_element.id().clone();

    let second_name = "b".to_string();
    let second_element = create_generic_non_actuated_element(second_name);
    let second_id = second_element.id().clone();

    let third_name = "c".to_string();
    let third_element = create_wheel_element(third_name);
    let third_id = third_element.id().clone();

    // Get the mutable tree to add something
    {
        match tree.add_element(
            first_element,
            FrameID::none(),
            Translation3::<f64>::identity(),
            UnitQuaternion::identity(),
        ) {
            Err(e) => {
                assert!(
                    false,
                    "Got an error adding an element to the tree. Should not have. Error was: {}",
                    e
                );
            }
            Ok(id) => {
                assert_eq!(id.as_ref(), &first_id);
            }
        };

        match tree.add_element(
            second_element,
            first_id.clone(),
            Translation3::<f64>::identity(),
            UnitQuaternion::identity(),
        ) {
            Err(e) => {
                assert!(
                    false,
                    "Got an error adding an element to the tree. Should not have. Error was: {}",
                    e
                );
            }
            Ok(id) => {
                assert_eq!(id.as_ref(), &second_id);
            }
        };

        match tree.add_element(
            third_element,
            second_id.clone(),
            Translation3::<f64>::identity(),
            UnitQuaternion::identity(),
        ) {
            Err(e) => {
                assert!(
                    false,
                    "Got an error adding an element to the tree. Should not have. Error was: {}",
                    e
                );
            }
            Ok(id) => {
                assert_eq!(id.as_ref(), &third_id);
            }
        };
    }

    let imtree = &tree;
    match imtree.get_parent(&second_id) {
        Err(e) => assert!(
            false,
            "Got an error retrieving the children, but should not have. Error was: {}",
            e
        ),
        Ok(c) => {
            assert_eq!(c.id(), &first_id)
        }
    };

    match imtree.get_parent(&third_id) {
        Err(e) => assert!(
            false,
            "Got an error retrieving the children, but should not have. Error was: {}",
            e
        ),
        Ok(c) => {
            assert_eq!(c.id(), &second_id)
        }
    };
}

#[test]
fn when_getting_the_parent_with_invalid_frame_elements_it_should_error() {
    let mut tree = KinematicTree::new();

    let first_name = "a".to_string();
    let first_element = create_generic_non_actuated_element(first_name);
    let first_id = first_element.id().clone();

    let second_name = "b".to_string();
    let second_element = create_generic_non_actuated_element(second_name);
    let second_id = second_element.id().clone();

    let third_name = "c".to_string();
    let third_element = create_wheel_element(third_name);
    let third_id = third_element.id().clone();

    // Get the mutable tree to add something
    {
        match tree.add_element(
            first_element,
            FrameID::none(),
            Translation3::<f64>::identity(),
            UnitQuaternion::identity(),
        ) {
            Err(e) => {
                assert!(
                    false,
                    "Got an error adding an element to the tree. Should not have. Error was: {}",
                    e
                );
            }
            Ok(id) => {
                assert_eq!(id.as_ref(), &first_id);
            }
        };

        match tree.add_element(
            second_element,
            first_id.clone(),
            Translation3::<f64>::identity(),
            UnitQuaternion::identity(),
        ) {
            Err(e) => {
                assert!(
                    false,
                    "Got an error adding an element to the tree. Should not have. Error was: {}",
                    e
                );
            }
            Ok(id) => {
                assert_eq!(id.as_ref(), &second_id);
            }
        };

        match tree.add_element(
            third_element,
            second_id.clone(),
            Translation3::<f64>::identity(),
            UnitQuaternion::identity(),
        ) {
            Err(e) => {
                assert!(
                    false,
                    "Got an error adding an element to the tree. Should not have. Error was: {}",
                    e
                );
            }
            Ok(id) => {
                assert_eq!(id.as_ref(), &third_id);
            }
        };
    }

    let imtree = &tree;
    let unknown_id = FrameID::new();
    match imtree.get_parent(&unknown_id) {
        Err(e) => assert_eq!(e, Error::InvalidFrameID { id: unknown_id }),
        Ok(_) => assert!(
            false,
            "Found a parent for an element that doesn't exist in the collection."
        ),
    };
}

#[test]
fn when_getting_the_parent_with_no_frame_elements_it_should_error() {
    let tree = KinematicTree::new();
    let child_id = FrameID::new();
    match tree.get_parent(&child_id) {
        Ok(_) => assert!(
            false,
            "Expected the test to produce an error, but it didn't."
        ),
        Err(e) => assert_eq!(e, Error::InvalidFrameID { id: child_id }),
    };
}

#[test]
fn when_getting_the_wheels_with_no_frame_elements_it_should_error() {
    let tree = KinematicTree::new();
    match tree.get_wheels() {
        Ok(_) => assert!(
            false,
            "Expected the test to produce an error, but it didn't."
        ),
        Err(e) => assert_eq!(
            e,
            Error::MissingFrameElement {
                id: FrameID::none()
            }
        ),
    };
}

// MotionModel

// The following functions assume that they are creating a robot with the following layout:
//
// body - reference frame is assumed to be in the middle of all the parts
//   suspension-1 (left front)
//     steering-1
//       wheel-1
//   suspension-2 (left rear)
//     steering-1
//       wheel-1
//   suspension-3 (right rear)
//     steering-1
//       wheel-1
//   suspension-4 (right front)
//     steering-1
//       wheel-1
//
// The relative positions and orientations are as follows
//
// - suspension left front
//   - position relative to parent: (1.0, 0.5, 0.0)
//   - orientation relative to parent: 30 degree rotation around the z-axis
// - steering left front
//   - position relative to parent: (0.25, 0.0, -0.1)
//   - orientation relative to parent: -30 degree rotation around the z-axis
// - wheel left front
//   - position relative to parent: (0.0, 0.0, -0.1)
//   - orientation relative to parent: 0 degree

#[derive(Clone, Copy, Debug, PartialEq)]
enum DriveModulePosition {
    LeftFront,
    LeftRear,
    RightRear,
    RightFront,
}

fn position_multipliers(relative_position: DriveModulePosition) -> (i32, i32, i32) {
    match relative_position {
        DriveModulePosition::LeftFront => (1, 1, 1),
        DriveModulePosition::LeftRear => (-1, 1, 1),
        DriveModulePosition::RightRear => (-1, -1, 1),
        DriveModulePosition::RightFront => (1, -1, 1),
    }
}

fn frame_angles_in_degrees_for(relative_position: DriveModulePosition) -> (f64, f64) {
    match relative_position {
        DriveModulePosition::LeftFront => (30.0, -30.0),
        DriveModulePosition::LeftRear => (150.0, -150.0),
        DriveModulePosition::RightRear => (210.0, -210.0),
        DriveModulePosition::RightFront => (330.0, -330.0),
    }
}

struct MockHardwareActuator {
    receiver: Receiver<(JointState, ActuatorAvailableRatesOfChange)>,
    sender: Sender<(JointState, ActuatorAvailableRatesOfChange)>,
    command_sender: Sender<JointState>,
    update_sender: Option<Sender<ChangeID>>,
    id: Option<ChangeID>,
}

impl HardwareActuator for MockHardwareActuator {
    fn get_actuator_motion_type(&self) -> NumberSpaceType {
        NumberSpaceType::LinearUnlimited
    }

    fn get_current_state_receiver(
        &self,
    ) -> Result<Receiver<(JointState, ActuatorAvailableRatesOfChange)>, Error> {
        Ok(self.receiver.clone())
    }

    fn get_command_sender(&self) -> Result<Sender<JointState>, Error> {
        Ok(self.command_sender.clone())
    }

    fn on_change(&mut self, id: ChangeID, sender: Sender<ChangeID>) {
        self.id = Some(id);
        self.update_sender = Some(sender);
    }

    fn get_actuator_range(&self) -> crate::hardware::joint_state::JointStateRange {
        todo!()
    }
}

fn add_actuated_joint_to_model<'a>(
    model: &mut MotionModel,
    parent_id: &FrameID,
    position: DriveModulePosition,
    dof: FrameDofType,
    actuator: Actuator,
) -> Result<FrameID, Error> {
    let (mul_x, mul_y, mul_z) = position_multipliers(position);
    let (angle, _) = frame_angles_in_degrees_for(position);
    let deg_to_rad = PI / 180.0;

    let name = "actuated".to_string();
    let position_relative_to_parent =
        Translation3::<f64>::new(1.0 * mul_x as f64, 0.5 * mul_y as f64, 0.0 * mul_z as f64);
    let orientation_relative_to_parent =
        UnitQuaternion::<f64>::from_euler_angles(0.0, 0.0, angle * deg_to_rad);
    let mass = 1.0;
    let center_of_mass = Vector3::<f64>::identity();
    let moment_of_inertia = Matrix3::<f64>::identity();
    let spatial_inertia = Matrix6::<f64>::identity();

    model.add_actuated_chassis_element(
        name.clone(),
        dof,
        parent_id.clone(),
        position_relative_to_parent,
        orientation_relative_to_parent,
        mass,
        center_of_mass,
        moment_of_inertia,
        spatial_inertia,
        actuator,
    )
}

fn add_body_to_model(model: &mut MotionModel) -> Result<FrameID, Error> {
    let name = "body".to_string();
    let position_relative_to_world = Translation3::<f64>::identity();
    let orientation_relative_to_world = UnitQuaternion::<f64>::identity();
    let mass = 1.0;
    let center_of_mass = Vector3::<f64>::identity();
    let moment_of_inertia = Matrix3::<f64>::identity();
    let spatial_inertia = Matrix6::<f64>::identity();

    model.add_body(
        name.clone(),
        position_relative_to_world,
        orientation_relative_to_world,
        mass,
        center_of_mass,
        moment_of_inertia,
        spatial_inertia,
    )
}

fn add_steering_to_model<'a>(
    model: &mut MotionModel,
    parent_id: &FrameID,
    position: DriveModulePosition,
    actuator: Actuator,
) -> Result<FrameID, Error> {
    let (mul_x, mul_y, mul_z) = position_multipliers(position);
    let (_, angle) = frame_angles_in_degrees_for(position);
    let deg_to_rad = PI / 180.0;

    let name = "steering".to_string();
    let position_relative_to_parent =
        Translation3::<f64>::new(0.25 * mul_x as f64, 0.0 * mul_y as f64, -0.1 * mul_z as f64);
    let orientation_relative_to_parent =
        UnitQuaternion::<f64>::from_euler_angles(0.0, 0.0, angle * deg_to_rad);
    let mass = 1.0;
    let center_of_mass = Vector3::<f64>::identity();
    let moment_of_inertia = Matrix3::<f64>::identity();
    let spatial_inertia = Matrix6::<f64>::identity();

    model.add_steering_element(
        name.clone(),
        parent_id.clone(),
        position_relative_to_parent,
        orientation_relative_to_parent,
        mass,
        center_of_mass,
        moment_of_inertia,
        spatial_inertia,
        actuator,
    )
}

fn add_suspension_to_model(
    model: &mut MotionModel,
    parent_id: &FrameID,
    position: DriveModulePosition,
) -> Result<FrameID, Error> {
    let (mul_x, mul_y, mul_z) = position_multipliers(position);
    let (angle, _) = frame_angles_in_degrees_for(position);
    let deg_to_rad = PI / 180.0;

    let name: String = "suspension".to_string();
    let position_relative_to_parent =
        Translation3::<f64>::new(1.0 * mul_x as f64, 0.5 * mul_y as f64, 0.0 * mul_z as f64);
    let orientation_relative_to_parent =
        UnitQuaternion::<f64>::from_euler_angles(0.0, 0.0, angle * deg_to_rad);
    let mass = 1.0;
    let center_of_mass = Vector3::<f64>::identity();
    let moment_of_inertia = Matrix3::<f64>::identity();
    let spatial_inertia = Matrix6::<f64>::identity();

    model.add_suspension_element(
        name.clone(),
        FrameDofType::PrismaticZ,
        parent_id.clone(),
        position_relative_to_parent,
        orientation_relative_to_parent,
        mass,
        center_of_mass,
        moment_of_inertia,
        spatial_inertia,
        JointConstraint::new(),
    )
}

fn add_wheel_to_model<'a>(
    model: &mut MotionModel,
    parent_id: &FrameID,
    actuator: Actuator,
) -> Result<FrameID, Error> {
    let name = "wheel".to_string();

    // Assume that the steering is the
    let position_relative_to_parent = Translation3::<f64>::new(0.0, 0.0, -0.1);

    // Assume that the parent is the steering and it has the same orientation
    let orientation_relative_to_parent = UnitQuaternion::<f64>::identity();
    let mass = 1.0;
    let center_of_mass = Vector3::<f64>::identity();
    let moment_of_inertia = Matrix3::<f64>::identity();
    let spatial_inertia = Matrix6::<f64>::identity();

    model.add_wheel(
        name.clone(),
        parent_id.clone(),
        position_relative_to_parent,
        orientation_relative_to_parent,
        mass,
        center_of_mass,
        moment_of_inertia,
        spatial_inertia,
        actuator,
    )
}

#[test]
fn when_adding_actuated_chassis_element_it_should_store_the_element() {
    let mut model = MotionModel::new();
    let body_id = add_body_to_model(&mut model).unwrap();

    let name = "a".to_string();
    let position_relative_to_parent = Translation3::<f64>::identity();
    let orientation_relative_to_parent = UnitQuaternion::<f64>::identity();
    let mass = 1.0;
    let center_of_mass = Vector3::<f64>::identity();
    let moment_of_inertia = Matrix3::<f64>::identity();
    let spatial_inertia = Matrix6::<f64>::identity();

    let (sender, receiver) = crossbeam_channel::unbounded();
    let (cmd_sender, _cmd_receiver) = crossbeam_channel::unbounded();
    let mut hardware_actuator = MockHardwareActuator {
        receiver,
        sender,
        command_sender: cmd_sender,
        update_sender: None,
        id: None,
    };
    let change_processor = Box::new(HardwareChangeProcessor::new(10));

    let actuator = Actuator::new(&mut hardware_actuator, &change_processor).unwrap();

    let result = model.add_actuated_chassis_element(
        name.clone(),
        FrameDofType::PrismaticX,
        body_id.clone(),
        position_relative_to_parent,
        orientation_relative_to_parent,
        mass,
        center_of_mass,
        moment_of_inertia,
        spatial_inertia,
        actuator,
    );

    assert!(result.is_ok());

    let frame_id = result.unwrap();
    assert!(!frame_id.is_none());

    let degree_of_freedom_result = model.get_frame_degree_of_freedom(&frame_id);
    assert!(degree_of_freedom_result.is_ok());

    let dof = degree_of_freedom_result.unwrap();
    assert_eq!(FrameDofType::PrismaticX, dof);

    let frame_result = model.get_reference_frame(&frame_id);
    assert!(frame_result.is_ok());

    let frame = frame_result.unwrap();
    assert_eq!(dof, frame.degree_of_freedom_kind());
    assert!(frame.is_actuated());

    let chassis_result = model.get_chassis_element(&frame_id);
    assert!(chassis_result.is_ok());

    let chassis = chassis_result.unwrap();
    assert_eq!(name, chassis.name());
}

#[test]
fn when_adding_actuated_chassis_element_with_invalid_parent_it_should_error() {
    let mut model = MotionModel::new();
    let _ = add_body_to_model(&mut model).unwrap();

    let name = "a".to_string();
    let position_relative_to_parent = Translation3::<f64>::identity();
    let orientation_relative_to_parent = UnitQuaternion::<f64>::identity();
    let mass = 1.0;
    let center_of_mass = Vector3::<f64>::identity();
    let moment_of_inertia = Matrix3::<f64>::identity();
    let spatial_inertia = Matrix6::<f64>::identity();

    let (sender, receiver) = crossbeam_channel::unbounded();
    let (cmd_sender, _cmd_receiver) = crossbeam_channel::unbounded();
    let mut hardware_actuator = MockHardwareActuator {
        receiver,
        sender,
        command_sender: cmd_sender,
        update_sender: None,
        id: None,
    };
    let change_processor = Box::new(HardwareChangeProcessor::new(10));

    let actuator = Actuator::new(&mut hardware_actuator, &change_processor).unwrap();

    let result = model.add_actuated_chassis_element(
        name.clone(),
        FrameDofType::PrismaticX,
        FrameID::new(),
        position_relative_to_parent,
        orientation_relative_to_parent,
        mass,
        center_of_mass,
        moment_of_inertia,
        spatial_inertia,
        actuator,
    );

    assert!(result.is_err());
}

#[test]
fn when_adding_actuated_chassis_element_with_parent_wheel_it_should_error() {
    let mut model = MotionModel::new();
    let body_id = add_body_to_model(&mut model).unwrap();

    let (steering_sender, steering_receiver) = crossbeam_channel::unbounded();
    let (steering_cmd_sender, _) = crossbeam_channel::unbounded();
    let mut steering_hardware_actuator = MockHardwareActuator {
        receiver: steering_receiver,
        sender: steering_sender,
        command_sender: steering_cmd_sender,
        update_sender: None,
        id: None,
    };
    let change_processor = Box::new(HardwareChangeProcessor::new(10));

    let steering_actuator =
        Actuator::new(&mut steering_hardware_actuator, &change_processor).unwrap();

    let steering_id = add_steering_to_model(
        &mut model,
        &body_id,
        DriveModulePosition::LeftFront,
        steering_actuator,
    )
    .unwrap();

    let (wheel_sender, wheel_receiver) = crossbeam_channel::unbounded();
    let (wheel_cmd_sender, _) = crossbeam_channel::unbounded();
    let mut wheel_hardware_actuator = MockHardwareActuator {
        receiver: wheel_receiver,
        sender: wheel_sender,
        command_sender: wheel_cmd_sender,
        update_sender: None,
        id: None,
    };
    let wheel_actuator = Actuator::new(&mut wheel_hardware_actuator, &change_processor).unwrap();

    let wheel_id = add_wheel_to_model(&mut model, &steering_id, wheel_actuator).unwrap();

    let name = "a".to_string();
    let position_relative_to_parent = Translation3::<f64>::identity();
    let orientation_relative_to_parent = UnitQuaternion::<f64>::identity();
    let mass = 1.0;
    let center_of_mass = Vector3::<f64>::identity();
    let moment_of_inertia = Matrix3::<f64>::identity();
    let spatial_inertia = Matrix6::<f64>::identity();

    let (sender, receiver) = crossbeam_channel::unbounded();
    let (cmd_sender, _cmd_receiver) = crossbeam_channel::unbounded();
    let mut hardware_actuator = MockHardwareActuator {
        receiver,
        sender,
        command_sender: cmd_sender,
        update_sender: None,
        id: None,
    };
    let change_processor = Box::new(HardwareChangeProcessor::new(10));

    let actuator = Actuator::new(&mut hardware_actuator, &change_processor).unwrap();

    let result = model.add_actuated_chassis_element(
        name.clone(),
        FrameDofType::PrismaticX,
        wheel_id.clone(),
        position_relative_to_parent,
        orientation_relative_to_parent,
        mass,
        center_of_mass,
        moment_of_inertia,
        spatial_inertia,
        actuator,
    );

    assert!(result.is_err());
}

#[test]
fn when_adding_body_it_should_store_the_element() {
    let name = "a".to_string();
    let position_relative_to_world = Translation3::<f64>::identity();
    let orientation_relative_to_world = UnitQuaternion::<f64>::identity();
    let mass = 1.0;
    let center_of_mass = Vector3::<f64>::identity();
    let moment_of_inertia = Matrix3::<f64>::identity();
    let spatial_inertia = Matrix6::<f64>::identity();

    let mut model = MotionModel::new();
    let result = model.add_body(
        name.clone(),
        position_relative_to_world,
        orientation_relative_to_world,
        mass,
        center_of_mass,
        moment_of_inertia,
        spatial_inertia,
    );

    assert!(result.is_ok());

    let body_id = result.unwrap();
    assert!(!body_id.is_none());

    let body_result = model.get_body();
    assert!(body_result.is_ok());

    let id = body_result.unwrap();
    assert_eq!(body_id, *id);

    let degree_of_freedom_result = model.get_frame_degree_of_freedom(id);
    assert!(degree_of_freedom_result.is_ok());

    let dof = degree_of_freedom_result.unwrap();
    assert_eq!(FrameDofType::Static, dof);

    let frame_result = model.get_reference_frame(id);
    assert!(frame_result.is_ok());

    let frame = frame_result.unwrap();
    assert_eq!(dof, frame.degree_of_freedom_kind());
    assert!(!frame.is_actuated());

    let chassis_result = model.get_chassis_element(id);
    assert!(chassis_result.is_ok());

    let chassis = chassis_result.unwrap();
    assert_eq!(name, chassis.name());
}

#[test]
fn when_adding_body_multiple_times_it_should_error() {
    let mut model = MotionModel::new();
    let first_result = add_body_to_model(&mut model);

    assert!(first_result.is_ok());

    let second_result = add_body_to_model(&mut model);
    assert!(second_result.is_err());
}

#[test]
fn when_adding_static_chassis_element_it_should_store_the_element() {
    let mut model = MotionModel::new();
    let body_id = add_body_to_model(&mut model).unwrap();

    let name = "a".to_string();
    let position_relative_to_parent = Translation3::<f64>::identity();
    let orientation_relative_to_parent = UnitQuaternion::<f64>::identity();
    let mass = 1.0;
    let center_of_mass = Vector3::<f64>::identity();
    let moment_of_inertia = Matrix3::<f64>::identity();
    let spatial_inertia = Matrix6::<f64>::identity();

    let result = model.add_static_chassis_element(
        name.clone(),
        body_id.clone(),
        position_relative_to_parent,
        orientation_relative_to_parent,
        mass,
        center_of_mass,
        moment_of_inertia,
        spatial_inertia,
    );

    assert!(result.is_ok());

    let frame_id = result.unwrap();
    assert!(!frame_id.is_none());

    let degree_of_freedom_result = model.get_frame_degree_of_freedom(&frame_id);
    assert!(degree_of_freedom_result.is_ok());

    let dof = degree_of_freedom_result.unwrap();
    assert_eq!(FrameDofType::Static, dof);

    let frame_result = model.get_reference_frame(&frame_id);
    assert!(frame_result.is_ok());

    let frame = frame_result.unwrap();
    assert_eq!(dof, frame.degree_of_freedom_kind());
    assert!(!frame.is_actuated());

    let chassis_result = model.get_chassis_element(&frame_id);
    assert!(chassis_result.is_ok());

    let chassis = chassis_result.unwrap();
    assert_eq!(name, chassis.name());
}

#[test]
fn when_adding_static_chassis_element_with_invalid_parent_it_should_error() {
    let mut model = MotionModel::new();
    let _ = add_body_to_model(&mut model).unwrap();

    let name = "a".to_string();
    let position_relative_to_parent = Translation3::<f64>::identity();
    let orientation_relative_to_parent = UnitQuaternion::<f64>::identity();
    let mass = 1.0;
    let center_of_mass = Vector3::<f64>::identity();
    let moment_of_inertia = Matrix3::<f64>::identity();
    let spatial_inertia = Matrix6::<f64>::identity();

    let result = model.add_static_chassis_element(
        name.clone(),
        FrameID::new(),
        position_relative_to_parent,
        orientation_relative_to_parent,
        mass,
        center_of_mass,
        moment_of_inertia,
        spatial_inertia,
    );

    assert!(result.is_err());
}

#[test]
fn when_adding_static_chassis_element_with_parent_wheel_it_should_error() {
    let mut model = MotionModel::new();
    let body_id = add_body_to_model(&mut model).unwrap();

    let (steering_sender, steering_receiver) = crossbeam_channel::unbounded();
    let (steering_cmd_sender, _) = crossbeam_channel::unbounded();
    let mut steering_hardware_actuator = MockHardwareActuator {
        receiver: steering_receiver,
        sender: steering_sender,
        command_sender: steering_cmd_sender,
        update_sender: None,
        id: None,
    };
    let change_processor = Box::new(HardwareChangeProcessor::new(10));

    let steering_actuator =
        Actuator::new(&mut steering_hardware_actuator, &change_processor).unwrap();

    let steering_id = add_steering_to_model(
        &mut model,
        &body_id,
        DriveModulePosition::LeftFront,
        steering_actuator,
    )
    .unwrap();

    let (wheel_sender, wheel_receiver) = crossbeam_channel::unbounded();
    let (wheel_cmd_sender, _) = crossbeam_channel::unbounded();
    let mut wheel_hardware_actuator = MockHardwareActuator {
        receiver: wheel_receiver,
        sender: wheel_sender,
        command_sender: wheel_cmd_sender,
        update_sender: None,
        id: None,
    };

    let wheel_actuator = Actuator::new(&mut wheel_hardware_actuator, &change_processor).unwrap();

    let wheel_id = add_wheel_to_model(&mut model, &steering_id, wheel_actuator).unwrap();

    let name = "a".to_string();
    let position_relative_to_parent = Translation3::<f64>::identity();
    let orientation_relative_to_parent = UnitQuaternion::<f64>::identity();
    let mass = 1.0;
    let center_of_mass = Vector3::<f64>::identity();
    let moment_of_inertia = Matrix3::<f64>::identity();
    let spatial_inertia = Matrix6::<f64>::identity();

    let result = model.add_static_chassis_element(
        name.clone(),
        wheel_id.clone(),
        position_relative_to_parent,
        orientation_relative_to_parent,
        mass,
        center_of_mass,
        moment_of_inertia,
        spatial_inertia,
    );

    assert!(result.is_err());
}

#[test]
fn when_adding_steering_element_it_should_store_the_element() {
    let mut model = MotionModel::new();
    let body_id = add_body_to_model(&mut model).unwrap();

    let name = "a".to_string();
    let position_relative_to_parent = Translation3::<f64>::identity();
    let orientation_relative_to_parent = UnitQuaternion::<f64>::identity();
    let mass = 1.0;
    let center_of_mass = Vector3::<f64>::identity();
    let moment_of_inertia = Matrix3::<f64>::identity();
    let spatial_inertia = Matrix6::<f64>::identity();

    let (sender, receiver) = crossbeam_channel::unbounded();
    let (cmd_sender, _cmd_receiver) = crossbeam_channel::unbounded();
    let mut hardware_actuator = MockHardwareActuator {
        receiver,
        sender,
        command_sender: cmd_sender,
        update_sender: None,
        id: None,
    };
    let change_processor = Box::new(HardwareChangeProcessor::new(10));

    let actuator = Actuator::new(&mut hardware_actuator, &change_processor).unwrap();

    let result = model.add_steering_element(
        name.clone(),
        body_id.clone(),
        position_relative_to_parent,
        orientation_relative_to_parent,
        mass,
        center_of_mass,
        moment_of_inertia,
        spatial_inertia,
        actuator,
    );

    assert!(result.is_ok());

    let frame_id = result.unwrap();
    assert!(!frame_id.is_none());

    let degree_of_freedom_result = model.get_frame_degree_of_freedom(&frame_id);
    assert!(degree_of_freedom_result.is_ok());

    let dof = degree_of_freedom_result.unwrap();
    assert_eq!(FrameDofType::RevoluteZ, dof);

    let frame_result = model.get_reference_frame(&frame_id);
    assert!(frame_result.is_ok());

    let frame = frame_result.unwrap();
    assert_eq!(dof, frame.degree_of_freedom_kind());
    assert!(frame.is_actuated());

    let chassis_result = model.get_chassis_element(&frame_id);
    assert!(chassis_result.is_ok());

    let chassis = chassis_result.unwrap();
    assert_eq!(name, chassis.name());
}

#[test]
fn when_adding_steering_element_with_invalid_parent_it_should_error() {
    let mut model = MotionModel::new();
    let _ = add_body_to_model(&mut model).unwrap();

    let name = "a".to_string();
    let position_relative_to_parent = Translation3::<f64>::identity();
    let orientation_relative_to_parent = UnitQuaternion::<f64>::identity();
    let mass = 1.0;
    let center_of_mass = Vector3::<f64>::identity();
    let moment_of_inertia = Matrix3::<f64>::identity();
    let spatial_inertia = Matrix6::<f64>::identity();

    let (sender, receiver) = crossbeam_channel::unbounded();
    let (cmd_sender, _cmd_receiver) = crossbeam_channel::unbounded();
    let mut hardware_actuator = MockHardwareActuator {
        receiver,
        sender,
        command_sender: cmd_sender,
        update_sender: None,
        id: None,
    };
    let change_processor = Box::new(HardwareChangeProcessor::new(10));

    let actuator = Actuator::new(&mut hardware_actuator, &change_processor).unwrap();

    let result = model.add_steering_element(
        name.clone(),
        FrameID::new(),
        position_relative_to_parent,
        orientation_relative_to_parent,
        mass,
        center_of_mass,
        moment_of_inertia,
        spatial_inertia,
        actuator,
    );

    assert!(result.is_err());
}

#[test]
fn when_adding_steering_element_with_multiple_steering_elements_in_chain_it_should_error() {
    let mut model = MotionModel::new();
    let body_id = add_body_to_model(&mut model).unwrap();

    let (steering_sender, steering_receiver) = crossbeam_channel::unbounded();
    let (steering_cmd_sender, _) = crossbeam_channel::unbounded();
    let mut steering_hardware_actuator = MockHardwareActuator {
        receiver: steering_receiver,
        sender: steering_sender,
        command_sender: steering_cmd_sender,
        update_sender: None,
        id: None,
    };
    let change_processor = Box::new(HardwareChangeProcessor::new(10));

    let steering_actuator =
        Actuator::new(&mut steering_hardware_actuator, &change_processor).unwrap();

    let steering_id = add_steering_to_model(
        &mut model,
        &body_id,
        DriveModulePosition::LeftFront,
        steering_actuator,
    )
    .unwrap();

    let name = "a".to_string();
    let position_relative_to_parent = Translation3::<f64>::identity();
    let orientation_relative_to_parent = UnitQuaternion::<f64>::identity();
    let mass = 1.0;
    let center_of_mass = Vector3::<f64>::identity();
    let moment_of_inertia = Matrix3::<f64>::identity();
    let spatial_inertia = Matrix6::<f64>::identity();

    let (sender2, receiver2) = crossbeam_channel::unbounded();
    let (cmd_sender2, _) = crossbeam_channel::unbounded();
    let mut hardware_actuator2 = MockHardwareActuator {
        receiver: receiver2,
        sender: sender2.clone(),
        command_sender: cmd_sender2,
        update_sender: None,
        id: None,
    };

    let actuator2 = Actuator::new(&mut hardware_actuator2, &change_processor).unwrap();

    let result = model.add_steering_element(
        name.clone(),
        steering_id.clone(),
        position_relative_to_parent,
        orientation_relative_to_parent,
        mass,
        center_of_mass,
        moment_of_inertia,
        spatial_inertia,
        actuator2,
    );

    assert!(result.is_err());
}

#[test]
fn when_adding_steering_element_with_parent_wheel_it_should_error() {
    let mut model = MotionModel::new();
    let body_id = add_body_to_model(&mut model).unwrap();

    let (steering_sender, steering_receiver) = crossbeam_channel::unbounded();
    let (steering_cmd_sender, _) = crossbeam_channel::unbounded();
    let mut steering_hardware_actuator = MockHardwareActuator {
        receiver: steering_receiver,
        sender: steering_sender,
        command_sender: steering_cmd_sender,
        update_sender: None,
        id: None,
    };
    let change_processor = Box::new(HardwareChangeProcessor::new(10));

    let steering_actuator =
        Actuator::new(&mut steering_hardware_actuator, &change_processor).unwrap();

    let steering_id: FrameID = add_steering_to_model(
        &mut model,
        &body_id,
        DriveModulePosition::LeftFront,
        steering_actuator,
    )
    .unwrap();

    let (wheel_sender, wheel_receiver) = crossbeam_channel::unbounded();
    let (wheel_cmd_sender, _) = crossbeam_channel::unbounded();
    let mut wheel_hardware_actuator = MockHardwareActuator {
        receiver: wheel_receiver,
        sender: wheel_sender,
        command_sender: wheel_cmd_sender,
        update_sender: None,
        id: None,
    };

    let wheel_actuator = Actuator::new(&mut wheel_hardware_actuator, &change_processor).unwrap();
    let wheel_id = add_wheel_to_model(&mut model, &steering_id, wheel_actuator).unwrap();

    let name = "a".to_string();
    let position_relative_to_parent = Translation3::<f64>::identity();
    let orientation_relative_to_parent = UnitQuaternion::<f64>::identity();
    let mass = 1.0;
    let center_of_mass = Vector3::<f64>::identity();
    let moment_of_inertia = Matrix3::<f64>::identity();
    let spatial_inertia = Matrix6::<f64>::identity();

    let (sender, receiver) = crossbeam_channel::unbounded();
    let (cmd_sender, _cmd_receiver) = crossbeam_channel::unbounded();
    let mut hardware_actuator = MockHardwareActuator {
        receiver,
        sender,
        command_sender: cmd_sender,
        update_sender: None,
        id: None,
    };
    let change_processor = Box::new(HardwareChangeProcessor::new(10));

    let actuator = Actuator::new(&mut hardware_actuator, &change_processor).unwrap();

    let result = model.add_steering_element(
        name.clone(),
        wheel_id.clone(),
        position_relative_to_parent,
        orientation_relative_to_parent,
        mass,
        center_of_mass,
        moment_of_inertia,
        spatial_inertia,
        actuator,
    );

    assert!(result.is_err());
}

#[test]
fn when_adding_suspension_element_it_should_store_the_element() {
    let mut model = MotionModel::new();
    let body_id = add_body_to_model(&mut model).unwrap();

    let name = "a".to_string();
    let position_relative_to_parent = Translation3::<f64>::identity();
    let orientation_relative_to_parent = UnitQuaternion::<f64>::identity();
    let mass = 1.0;
    let center_of_mass = Vector3::<f64>::identity();
    let moment_of_inertia = Matrix3::<f64>::identity();
    let spatial_inertia = Matrix6::<f64>::identity();

    let joint_constraint = JointConstraint::new();

    let result = model.add_suspension_element(
        name.clone(),
        FrameDofType::PrismaticX,
        body_id.clone(),
        position_relative_to_parent,
        orientation_relative_to_parent,
        mass,
        center_of_mass,
        moment_of_inertia,
        spatial_inertia,
        joint_constraint,
    );

    assert!(result.is_ok());

    let frame_id = result.unwrap();
    assert!(!frame_id.is_none());

    let degree_of_freedom_result = model.get_frame_degree_of_freedom(&frame_id);
    assert!(degree_of_freedom_result.is_ok());

    let dof = degree_of_freedom_result.unwrap();
    assert_eq!(FrameDofType::PrismaticX, dof);

    let frame_result = model.get_reference_frame(&frame_id);
    assert!(frame_result.is_ok());

    let frame = frame_result.unwrap();
    assert_eq!(dof, frame.degree_of_freedom_kind());
    assert!(!frame.is_actuated());

    let chassis_result = model.get_chassis_element(&frame_id);
    assert!(chassis_result.is_ok());

    let chassis = chassis_result.unwrap();
    assert_eq!(name, chassis.name());
}

#[test]
fn when_adding_suspension_element_with_invalid_parent_it_should_error() {
    let mut model = MotionModel::new();
    let _ = add_body_to_model(&mut model).unwrap();

    let name = "a".to_string();
    let position_relative_to_parent = Translation3::<f64>::identity();
    let orientation_relative_to_parent = UnitQuaternion::<f64>::identity();
    let mass = 1.0;
    let center_of_mass = Vector3::<f64>::identity();
    let moment_of_inertia = Matrix3::<f64>::identity();
    let spatial_inertia = Matrix6::<f64>::identity();

    let joint_constraint = JointConstraint::new();

    let result = model.add_suspension_element(
        name.clone(),
        FrameDofType::PrismaticX,
        FrameID::new(),
        position_relative_to_parent,
        orientation_relative_to_parent,
        mass,
        center_of_mass,
        moment_of_inertia,
        spatial_inertia,
        joint_constraint,
    );

    assert!(result.is_err());
}

#[test]
fn when_adding_suspension_element_with_wheel_parent_it_should_error() {
    let mut model = MotionModel::new();
    let body_id = add_body_to_model(&mut model).unwrap();

    let (steering_sender, steering_receiver) = crossbeam_channel::unbounded();
    let (steering_cmd_sender, _) = crossbeam_channel::unbounded();
    let mut steering_hardware_actuator = MockHardwareActuator {
        receiver: steering_receiver,
        sender: steering_sender,
        command_sender: steering_cmd_sender,
        update_sender: None,
        id: None,
    };
    let change_processor = Box::new(HardwareChangeProcessor::new(10));

    let steering_actuator =
        Actuator::new(&mut steering_hardware_actuator, &change_processor).unwrap();

    let steering_id = add_steering_to_model(
        &mut model,
        &body_id,
        DriveModulePosition::LeftFront,
        steering_actuator,
    )
    .unwrap();

    let (wheel_sender, wheel_receiver) = crossbeam_channel::unbounded();
    let (wheel_cmd_sender, _) = crossbeam_channel::unbounded();
    let mut wheel_hardware_actuator = MockHardwareActuator {
        receiver: wheel_receiver,
        sender: wheel_sender,
        command_sender: wheel_cmd_sender,
        update_sender: None,
        id: None,
    };

    let wheel_actuator = Actuator::new(&mut wheel_hardware_actuator, &change_processor).unwrap();
    let wheel_id = add_wheel_to_model(&mut model, &steering_id, wheel_actuator).unwrap();

    let name = "a".to_string();
    let position_relative_to_parent = Translation3::<f64>::identity();
    let orientation_relative_to_parent = UnitQuaternion::<f64>::identity();
    let mass = 1.0;
    let center_of_mass = Vector3::<f64>::identity();
    let moment_of_inertia = Matrix3::<f64>::identity();
    let spatial_inertia = Matrix6::<f64>::identity();

    let joint_constraint = JointConstraint::new();

    let result = model.add_suspension_element(
        name.clone(),
        FrameDofType::PrismaticX,
        wheel_id.clone(),
        position_relative_to_parent,
        orientation_relative_to_parent,
        mass,
        center_of_mass,
        moment_of_inertia,
        spatial_inertia,
        joint_constraint,
    );

    assert!(result.is_err());
}

#[test]
fn when_adding_wheel_element_it_should_store_the_element() {
    let mut model = MotionModel::new();
    let body_id = add_body_to_model(&mut model).unwrap();

    let (steering_sender, steering_receiver) = crossbeam_channel::unbounded();
    let (steering_cmd_sender, _) = crossbeam_channel::unbounded();
    let mut steering_hardware_actuator = MockHardwareActuator {
        receiver: steering_receiver,
        sender: steering_sender,
        command_sender: steering_cmd_sender,
        update_sender: None,
        id: None,
    };
    let change_processor = Box::new(HardwareChangeProcessor::new(10));

    let steering_actuator =
        Actuator::new(&mut steering_hardware_actuator, &change_processor).unwrap();

    let steering_id = add_steering_to_model(
        &mut model,
        &body_id,
        DriveModulePosition::LeftFront,
        steering_actuator,
    )
    .unwrap();

    let name = "a".to_string();
    let position_relative_to_parent = Translation3::<f64>::identity();
    let orientation_relative_to_parent = UnitQuaternion::<f64>::identity();
    let mass = 1.0;
    let center_of_mass = Vector3::<f64>::identity();
    let moment_of_inertia = Matrix3::<f64>::identity();
    let spatial_inertia = Matrix6::<f64>::identity();

    let (sender2, receiver2) = crossbeam_channel::unbounded();
    let (cmd_sender2, _) = crossbeam_channel::unbounded();
    let mut hardware_actuator2 = MockHardwareActuator {
        receiver: receiver2,
        sender: sender2.clone(),
        command_sender: cmd_sender2,
        update_sender: None,
        id: None,
    };

    let actuator2 = Actuator::new(&mut hardware_actuator2, &change_processor).unwrap();

    let result = model.add_wheel(
        name.clone(),
        steering_id.clone(),
        position_relative_to_parent,
        orientation_relative_to_parent,
        mass,
        center_of_mass,
        moment_of_inertia,
        spatial_inertia,
        actuator2,
    );

    assert!(result.is_ok());

    let frame_id = result.unwrap();
    assert!(!frame_id.is_none());

    let degree_of_freedom_result = model.get_frame_degree_of_freedom(&frame_id);
    assert!(degree_of_freedom_result.is_ok());

    let dof = degree_of_freedom_result.unwrap();
    assert_eq!(FrameDofType::RevoluteY, dof);

    let frame_result = model.get_reference_frame(&frame_id);
    assert!(frame_result.is_ok());

    let frame = frame_result.unwrap();
    assert_eq!(dof, frame.degree_of_freedom_kind());
    assert!(frame.is_actuated());

    let chassis_result = model.get_chassis_element(&frame_id);
    assert!(chassis_result.is_ok());

    let chassis = chassis_result.unwrap();
    assert_eq!(name, chassis.name());

    let wheels_results = model.get_wheels();
    assert!(wheels_results.is_ok());

    let wheels = wheels_results.unwrap();
    assert!(wheels.len() == 1);
    assert_eq!(frame_id, *wheels[0]);

    let steering_result = model.get_steering_frame_for_wheel(&frame_id);
    assert!(steering_result.is_ok());

    let steering_from_wheel = steering_result.unwrap();
    assert_eq!(steering_id, *steering_from_wheel);
}

#[test]
fn when_adding_wheel_element_with_invalid_parent_it_should_error() {
    let mut model = MotionModel::new();
    let body_id = add_body_to_model(&mut model).unwrap();

    let (steering_sender, steering_receiver) = crossbeam_channel::unbounded();
    let (steering_cmd_sender, _) = crossbeam_channel::unbounded();
    let mut steering_hardware_actuator = MockHardwareActuator {
        receiver: steering_receiver,
        sender: steering_sender,
        command_sender: steering_cmd_sender,
        update_sender: None,
        id: None,
    };
    let change_processor = Box::new(HardwareChangeProcessor::new(10));

    let steering_actuator =
        Actuator::new(&mut steering_hardware_actuator, &change_processor).unwrap();

    let _ = add_steering_to_model(
        &mut model,
        &body_id,
        DriveModulePosition::LeftFront,
        steering_actuator,
    )
    .unwrap();

    let name = "a".to_string();
    let position_relative_to_parent = Translation3::<f64>::identity();
    let orientation_relative_to_parent = UnitQuaternion::<f64>::identity();
    let mass = 1.0;
    let center_of_mass = Vector3::<f64>::identity();
    let moment_of_inertia = Matrix3::<f64>::identity();
    let spatial_inertia = Matrix6::<f64>::identity();

    let (sender2, receiver2) = crossbeam_channel::unbounded();
    let (cmd_sender2, _) = crossbeam_channel::unbounded();
    let mut hardware_actuator2 = MockHardwareActuator {
        receiver: receiver2,
        sender: sender2.clone(),
        command_sender: cmd_sender2,
        update_sender: None,
        id: None,
    };

    let actuator2 = Actuator::new(&mut hardware_actuator2, &change_processor).unwrap();

    let result = model.add_wheel(
        name.clone(),
        FrameID::new(),
        position_relative_to_parent,
        orientation_relative_to_parent,
        mass,
        center_of_mass,
        moment_of_inertia,
        spatial_inertia,
        actuator2,
    );

    assert!(result.is_err());
}

#[test]
fn when_adding_wheel_element_with_wheel_parent_it_should_error() {
    let mut model = MotionModel::new();
    let body_id = add_body_to_model(&mut model).unwrap();

    let (steering_sender, steering_receiver) = crossbeam_channel::unbounded();
    let (steering_cmd_sender, _) = crossbeam_channel::unbounded();
    let mut steering_hardware_actuator = MockHardwareActuator {
        receiver: steering_receiver,
        sender: steering_sender,
        command_sender: steering_cmd_sender,
        update_sender: None,
        id: None,
    };
    let change_processor = Box::new(HardwareChangeProcessor::new(10));

    let steering_actuator =
        Actuator::new(&mut steering_hardware_actuator, &change_processor).unwrap();

    let steering_id = add_steering_to_model(
        &mut model,
        &body_id,
        DriveModulePosition::LeftFront,
        steering_actuator,
    )
    .unwrap();

    let (wheel_sender, wheel_receiver) = crossbeam_channel::unbounded();
    let (wheel_cmd_sender, _) = crossbeam_channel::unbounded();
    let mut wheel_hardware_actuator = MockHardwareActuator {
        receiver: wheel_receiver,
        sender: wheel_sender,
        command_sender: wheel_cmd_sender,
        update_sender: None,
        id: None,
    };

    let wheel_actuator = Actuator::new(&mut wheel_hardware_actuator, &change_processor).unwrap();
    let wheel_id = add_wheel_to_model(&mut model, &steering_id, wheel_actuator).unwrap();

    let name = "a".to_string();
    let position_relative_to_parent = Translation3::<f64>::identity();
    let orientation_relative_to_parent = UnitQuaternion::<f64>::identity();
    let mass = 1.0;
    let center_of_mass = Vector3::<f64>::identity();
    let moment_of_inertia = Matrix3::<f64>::identity();
    let spatial_inertia = Matrix6::<f64>::identity();

    let (sender2, receiver2) = crossbeam_channel::unbounded();
    let (cmd_sender2, _) = crossbeam_channel::unbounded();
    let mut hardware_actuator2 = MockHardwareActuator {
        receiver: receiver2,
        sender: sender2.clone(),
        command_sender: cmd_sender2,
        update_sender: None,
        id: None,
    };

    let actuator2 = Actuator::new(&mut hardware_actuator2, &change_processor).unwrap();

    let result = model.add_wheel(
        name.clone(),
        wheel_id.clone(),
        position_relative_to_parent,
        orientation_relative_to_parent,
        mass,
        center_of_mass,
        moment_of_inertia,
        spatial_inertia,
        actuator2,
    );

    assert!(result.is_err());
}

#[test]
fn when_adding_wheel_element_without_steering_parent_it_should_error() {
    let mut model = MotionModel::new();
    let body_id = add_body_to_model(&mut model).unwrap();

    let name = "a".to_string();
    let position_relative_to_parent = Translation3::<f64>::identity();
    let orientation_relative_to_parent = UnitQuaternion::<f64>::identity();
    let mass = 1.0;
    let center_of_mass = Vector3::<f64>::identity();
    let moment_of_inertia = Matrix3::<f64>::identity();
    let spatial_inertia = Matrix6::<f64>::identity();

    let (sender, receiver) = crossbeam_channel::unbounded();
    let (cmd_sender, _cmd_receiver) = crossbeam_channel::unbounded();
    let mut hardware_actuator = MockHardwareActuator {
        receiver,
        sender,
        command_sender: cmd_sender,
        update_sender: None,
        id: None,
    };
    let change_processor = Box::new(HardwareChangeProcessor::new(10));

    let actuator = Actuator::new(&mut hardware_actuator, &change_processor).unwrap();

    let result = model.add_wheel(
        name.clone(),
        body_id.clone(),
        position_relative_to_parent,
        orientation_relative_to_parent,
        mass,
        center_of_mass,
        moment_of_inertia,
        spatial_inertia,
        actuator,
    );

    assert!(result.is_err());
}

#[test]
fn when_checking_is_valid_with_missing_wheel_it_should_fail() {
    let mut model = MotionModel::new();
    let body_id = add_body_to_model(&mut model).unwrap();

    // Leg 1
    let suspension_id_leg1 =
        add_suspension_to_model(&mut model, &body_id, DriveModulePosition::LeftFront).unwrap();

    let (sender1, receiver1) = crossbeam_channel::unbounded();
    let (cmd_sender1, _) = crossbeam_channel::unbounded();
    let mut hardware_actuator1 = MockHardwareActuator {
        receiver: receiver1,
        sender: sender1.clone(),
        command_sender: cmd_sender1,
        update_sender: None,
        id: None,
    };
    let change_processor = Box::new(HardwareChangeProcessor::new(10));

    let actuator1 = Actuator::new(&mut hardware_actuator1, &change_processor).unwrap();

    let steering_id_leg1 = add_steering_to_model(
        &mut model,
        &suspension_id_leg1,
        DriveModulePosition::LeftFront,
        actuator1,
    )
    .unwrap();

    let (wheel_sender, wheel_receiver) = crossbeam_channel::unbounded();
    let (wheel_cmd_sender, _) = crossbeam_channel::unbounded();
    let mut wheel_hardware_actuator = MockHardwareActuator {
        receiver: wheel_receiver,
        sender: wheel_sender,
        command_sender: wheel_cmd_sender,
        update_sender: None,
        id: None,
    };

    let wheel_actuator = Actuator::new(&mut wheel_hardware_actuator, &change_processor).unwrap();

    let _ = add_wheel_to_model(&mut model, &steering_id_leg1, wheel_actuator).unwrap();

    // Leg 2
    let suspension_id_leg2 =
        add_suspension_to_model(&mut model, &body_id, DriveModulePosition::RightFront).unwrap();

    let (sender2, receiver2) = crossbeam_channel::unbounded();
    let (cmd_sender2, _) = crossbeam_channel::unbounded();
    let mut hardware_actuator2 = MockHardwareActuator {
        receiver: receiver2,
        sender: sender2.clone(),
        command_sender: cmd_sender2,
        update_sender: None,
        id: None,
    };

    let actuator2 = Actuator::new(&mut hardware_actuator2, &change_processor).unwrap();

    let steering_id_leg2 = add_steering_to_model(
        &mut model,
        &suspension_id_leg2,
        DriveModulePosition::RightFront,
        actuator2,
    )
    .unwrap();

    let (wheel_sender_2, wheel_receiver_2) = crossbeam_channel::unbounded();
    let (wheel_cmd_sender_2, _) = crossbeam_channel::unbounded();
    let mut wheel_hardware_actuator_2 = MockHardwareActuator {
        receiver: wheel_receiver_2,
        sender: wheel_sender_2,
        command_sender: wheel_cmd_sender_2,
        update_sender: None,
        id: None,
    };

    let wheel_actuator_2 =
        Actuator::new(&mut wheel_hardware_actuator_2, &change_processor).unwrap();

    let _ = add_wheel_to_model(&mut model, &steering_id_leg2, wheel_actuator_2).unwrap();

    // Leg 3
    let suspension_id_leg3 =
        add_suspension_to_model(&mut model, &body_id, DriveModulePosition::RightRear).unwrap();

    let (sender3, receiver3) = crossbeam_channel::unbounded();
    let (cmd_sender3, _) = crossbeam_channel::unbounded();
    let mut hardware_actuator3 = MockHardwareActuator {
        receiver: receiver3,
        sender: sender3.clone(),
        command_sender: cmd_sender3,
        update_sender: None,
        id: None,
    };

    let actuator3 = Actuator::new(&mut hardware_actuator3, &change_processor).unwrap();

    let steering_id_leg3 = add_steering_to_model(
        &mut model,
        &suspension_id_leg3,
        DriveModulePosition::RightRear,
        actuator3,
    )
    .unwrap();

    let results = model.is_valid();
    assert!(!results.0);

    assert_eq!(1, results.1.len());
    assert_eq!(format!("Swerve model expects each steering joint to be connected to a wheel. Steering joint {} is not connected to a wheel.", steering_id_leg3), results.1[0]);
}

#[test]
fn when_checking_is_valid_with_valid_model_it_should_approve() {
    let mut model = MotionModel::new();
    let body_id = add_body_to_model(&mut model).unwrap();

    // Leg 1
    let suspension_id_leg1 =
        add_suspension_to_model(&mut model, &body_id, DriveModulePosition::LeftFront).unwrap();

    let (sender1, receiver1) = crossbeam_channel::unbounded();
    let (cmd_sender1, _) = crossbeam_channel::unbounded();
    let mut hardware_actuator1 = MockHardwareActuator {
        receiver: receiver1,
        sender: sender1.clone(),
        command_sender: cmd_sender1,
        update_sender: None,
        id: None,
    };
    let change_processor = Box::new(HardwareChangeProcessor::new(10));

    let actuator1 = Actuator::new(&mut hardware_actuator1, &change_processor).unwrap();

    let steering_id_leg1 = add_steering_to_model(
        &mut model,
        &suspension_id_leg1,
        DriveModulePosition::LeftFront,
        actuator1,
    )
    .unwrap();

    let (wheel_sender1, wheel_receiver1) = crossbeam_channel::unbounded();
    let (wheel_cmd_sender1, _) = crossbeam_channel::unbounded();
    let mut wheel_hardware_actuator1 = MockHardwareActuator {
        receiver: wheel_receiver1,
        sender: wheel_sender1,
        command_sender: wheel_cmd_sender1,
        update_sender: None,
        id: None,
    };

    let wheel_actuator1 = Actuator::new(&mut wheel_hardware_actuator1, &change_processor).unwrap();

    let _ = add_wheel_to_model(&mut model, &steering_id_leg1, wheel_actuator1).unwrap();

    // Leg 2
    let suspension_id_leg2 =
        add_suspension_to_model(&mut model, &body_id, DriveModulePosition::RightFront).unwrap();

    let (sender2, receiver2) = crossbeam_channel::unbounded();
    let (cmd_sender2, _) = crossbeam_channel::unbounded();
    let mut hardware_actuator2 = MockHardwareActuator {
        receiver: receiver2,
        sender: sender2.clone(),
        command_sender: cmd_sender2,
        update_sender: None,
        id: None,
    };

    let actuator2 = Actuator::new(&mut hardware_actuator2, &change_processor).unwrap();

    let steering_id_leg2 = add_steering_to_model(
        &mut model,
        &suspension_id_leg2,
        DriveModulePosition::RightFront,
        actuator2,
    )
    .unwrap();

    let (wheel_sender2, wheel_receiver2) = crossbeam_channel::unbounded();
    let (wheel_cmd_sender2, _) = crossbeam_channel::unbounded();
    let mut wheel_hardware_actuator2 = MockHardwareActuator {
        receiver: wheel_receiver2,
        sender: wheel_sender2,
        command_sender: wheel_cmd_sender2,
        update_sender: None,
        id: None,
    };

    let wheel_actuator2 = Actuator::new(&mut wheel_hardware_actuator2, &change_processor).unwrap();

    let _ = add_wheel_to_model(&mut model, &steering_id_leg2, wheel_actuator2).unwrap();

    let results = model.is_valid();
    assert!(results.0);
    assert_eq!(0, results.1.len());
}

#[test]
fn when_getting_body_without_elements_it_should_error() {
    let model = MotionModel::new();

    let result = model.get_body();
    assert!(result.is_err());
}

#[test]
fn when_getting_children_it_should_return_the_children() {
    let mut model = MotionModel::new();
    let body_id = add_body_to_model(&mut model).unwrap();

    // Leg 1
    let suspension_id_leg1 =
        add_suspension_to_model(&mut model, &body_id, DriveModulePosition::LeftFront).unwrap();

    let (sender1, receiver1) = crossbeam_channel::unbounded();
    let (cmd_sender1, _) = crossbeam_channel::unbounded();
    let mut hardware_actuator1 = MockHardwareActuator {
        receiver: receiver1,
        sender: sender1.clone(),
        command_sender: cmd_sender1,
        update_sender: None,
        id: None,
    };
    let change_processor = Box::new(HardwareChangeProcessor::new(10));

    let actuator1 = Actuator::new(&mut hardware_actuator1, &change_processor).unwrap();

    let steering_id_leg1 = add_steering_to_model(
        &mut model,
        &suspension_id_leg1,
        DriveModulePosition::LeftFront,
        actuator1,
    )
    .unwrap();

    let (wheel_sender1, wheel_receiver1) = crossbeam_channel::unbounded();
    let (wheel_cmd_sender1, _) = crossbeam_channel::unbounded();
    let mut wheel_hardware_actuator1 = MockHardwareActuator {
        receiver: wheel_receiver1,
        sender: wheel_sender1,
        command_sender: wheel_cmd_sender1,
        update_sender: None,
        id: None,
    };

    let wheel_actuator1 = Actuator::new(&mut wheel_hardware_actuator1, &change_processor).unwrap();

    let _ = add_wheel_to_model(&mut model, &steering_id_leg1, wheel_actuator1).unwrap();

    // Leg 2
    let suspension_id_leg2 =
        add_suspension_to_model(&mut model, &body_id, DriveModulePosition::RightFront).unwrap();

    let (sender2, receiver2) = crossbeam_channel::unbounded();
    let (cmd_sender2, _) = crossbeam_channel::unbounded();
    let mut hardware_actuator2 = MockHardwareActuator {
        receiver: receiver2,
        sender: sender2.clone(),
        command_sender: cmd_sender2,
        update_sender: None,
        id: None,
    };

    let actuator2 = Actuator::new(&mut hardware_actuator2, &change_processor).unwrap();

    let steering_id_leg2 = add_steering_to_model(
        &mut model,
        &suspension_id_leg2,
        DriveModulePosition::RightFront,
        actuator2,
    )
    .unwrap();

    let (wheel_sender2, wheel_receiver2) = crossbeam_channel::unbounded();
    let (wheel_cmd_sender2, _) = crossbeam_channel::unbounded();
    let mut wheel_hardware_actuator2 = MockHardwareActuator {
        receiver: wheel_receiver2,
        sender: wheel_sender2,
        command_sender: wheel_cmd_sender2,
        update_sender: None,
        id: None,
    };

    let wheel_actuator2 = Actuator::new(&mut wheel_hardware_actuator2, &change_processor).unwrap();

    let _ = add_wheel_to_model(&mut model, &steering_id_leg2, wheel_actuator2).unwrap();

    let result = model.get_children(&body_id);
    assert!(result.is_ok());

    let children = result.unwrap();
    assert_eq!(2, children.len());
    assert!(children.contains(&&suspension_id_leg1));
    assert!(children.contains(&&suspension_id_leg2));
}

#[test]
fn when_getting_children_with_invalid_parent_it_should_error() {
    let mut model = MotionModel::new();
    let body_id = add_body_to_model(&mut model).unwrap();

    let _ = add_suspension_to_model(&mut model, &body_id, DriveModulePosition::LeftFront).unwrap();

    let invalid_id = FrameID::new();
    let result = model.get_children(&invalid_id);

    assert!(result.is_err());
}

#[test]
fn when_getting_frame_degree_of_freedom_with_invalid_frame_it_should_error() {
    let mut model = MotionModel::new();
    let _ = add_body_to_model(&mut model).unwrap();

    let invalid_id = FrameID::new();
    let result = model.get_frame_degree_of_freedom(&invalid_id);

    assert!(result.is_err());
}

#[test]
fn when_getting_homogeneous_transform_to_body_with_one_element_and_motion_it_should_return_the_transform(
) {
    let mut model = MotionModel::new();
    let body_id = add_body_to_model(&mut model).unwrap();

    let (sender, receiver) = crossbeam_channel::unbounded();
    let (cmd_sender, _) = crossbeam_channel::unbounded();
    let mut hardware_actuator = MockHardwareActuator {
        receiver: receiver,
        sender: sender.clone(),
        command_sender: cmd_sender,
        update_sender: None,
        id: None,
    };
    let change_processor = Box::new(HardwareChangeProcessor::new(1000));

    let actuator = Actuator::new(&mut hardware_actuator, &change_processor).unwrap();
    let id = add_actuated_joint_to_model(
        &mut model,
        &body_id,
        DriveModulePosition::LeftFront,
        FrameDofType::RevoluteX,
        actuator,
    )
    .unwrap();

    // | cos(30) -sin(30) 0.0 1.0 |
    // | sin(30) cos(30)  0.0 0.5 |
    // | 0.0     0.0      1.0 0.0 |
    // | 0.0     0.0      0.0 1.0 |
    let (angle_deg, _) = frame_angles_in_degrees_for(DriveModulePosition::LeftFront);
    let angle_rad = angle_deg * (PI / 180.0);

    #[rustfmt::skip]
    let original = Matrix4::new(
        angle_rad.cos(), -angle_rad.sin(), 0.0, 1.0,
        angle_rad.sin(),  angle_rad.cos(), 0.0, 0.5,
        0.0,              0.0,             1.0, 0.0,
        0.0,              0.0,             0.0, 1.0,
    );

    // Push the actuator out
    let angle_x_deg = 30.0;
    let angle_x_rad = angle_x_deg * (PI / 180.0);
    let msg = (
        JointState::new(angle_x_rad, None, None, None),
        ActuatorAvailableRatesOfChange::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    );
    let _ = sender.send(msg).unwrap();
    let _ = hardware_actuator
        .update_sender
        .as_ref()
        .unwrap()
        .send(hardware_actuator.id.unwrap())
        .unwrap();

    // | 1.0 0.0      0.0      0.0 |
    // | 0.0 cos(30)  -sin(30) 0.5 |
    // | 0.0 sin(30)  cos(30)  0.0 |
    // | 0.0 0.0      0.0      1.0 |
    #[rustfmt::skip]
    let rotation_x = Matrix4::new(
        1.0, 0.0,               0.0,                0.0,
        0.0, angle_x_rad.cos(), -angle_x_rad.sin(), 0.0,
        0.0, angle_x_rad.sin(),  angle_x_rad.cos(), 0.0,
        0.0, 0.0,               0.0,                1.0,
    );

    let expected = rotation_x * original;

    // Allow some time to ensure the task is not processed
    std::thread::sleep(Duration::from_millis(20));

    let actuator_to_body = model.get_homogeneous_transform_to_body(&id);
    assert!(actuator_to_body.is_ok());

    let actuator_to_body_matrix = actuator_to_body.unwrap();
    let mut expected_it = expected.iter();
    let mut calculated_it = actuator_to_body_matrix.iter();
    loop {
        match (expected_it.next(), calculated_it.next()) {
            (Some(a), Some(b)) => {
                assert!(
                    (*a).approx_eq(
                        *b,
                        F64Margin {
                            ulps: 2,
                            epsilon: 1e-6
                        }
                    ),
                    "Expected {:.5} and {:.5} to be equal within 2 ulps or 1e-6",
                    *a,
                    *b
                );
            }
            (None, None) => break,
            _ => assert!(false),
        }
    }

    // Pull the actuator in
    let angle_x_deg = -30.0;
    let angle_x_rad = angle_x_deg * (PI / 180.0);
    let msg = (
        JointState::new(angle_x_rad, None, None, None),
        ActuatorAvailableRatesOfChange::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    );
    let _ = sender.send(msg).unwrap();
    let _ = hardware_actuator
        .update_sender
        .as_ref()
        .unwrap()
        .send(hardware_actuator.id.unwrap())
        .unwrap();

    // | 1.0 0.0      0.0      0.0 |
    // | 0.0 cos(-30)  -sin(-30) 0.5 |
    // | 0.0 sin(-30)  cos(-30)  0.0 |
    // | 0.0 0.0      0.0      1.0 |
    #[rustfmt::skip]
    let rotation_x = Matrix4::new(
        1.0, 0.0,               0.0,                0.0,
        0.0, angle_x_rad.cos(), -angle_x_rad.sin(), 0.0,
        0.0, angle_x_rad.sin(),  angle_x_rad.cos(), 0.0,
        0.0, 0.0,               0.0,                1.0,
    );

    let expected = rotation_x * original;

    // Allow some time to ensure the task is not processed
    std::thread::sleep(Duration::from_millis(20));

    let actuator_to_body = model.get_homogeneous_transform_to_body(&id);
    assert!(actuator_to_body.is_ok());

    let actuator_to_body_matrix = actuator_to_body.unwrap();

    let mut expected_it = expected.iter();
    let mut calculated_it = actuator_to_body_matrix.iter();
    loop {
        match (expected_it.next(), calculated_it.next()) {
            (Some(a), Some(b)) => {
                assert!(
                    (*a).approx_eq(
                        *b,
                        F64Margin {
                            ulps: 2,
                            epsilon: 1e-6
                        }
                    ),
                    "Expected {:.5} and {:.5} to be equal within 2 ulps or 1e-6",
                    *a,
                    *b
                );
            }
            (None, None) => break,
            _ => assert!(false),
        }
    }
}

#[test]
fn when_getting_homogeneous_transform_to_body_with_one_element_and_no_motion_it_should_return_the_transform(
) {
    let mut model = MotionModel::new();
    let body_id = add_body_to_model(&mut model).unwrap();

    // Leg 1
    let suspension_id_leg1 =
        add_suspension_to_model(&mut model, &body_id, DriveModulePosition::LeftFront).unwrap();

    // | cos(30) -sin(30) 0.0 1.0 |
    // | sin(30) cos(30)  0.0 0.5 |
    // | 0.0     0.0      1.0 0.0 |
    // | 0.0     0.0      0.0 1.0 |
    let (angle_suspension_deg, _) = frame_angles_in_degrees_for(DriveModulePosition::LeftFront);
    let angle_suspension_rad = angle_suspension_deg * (PI / 180.0);

    #[rustfmt::skip]
    let homogenous_suspension_to_body = Matrix4::new(
        angle_suspension_rad.cos(), -angle_suspension_rad.sin(), 0.0, 1.0,
        angle_suspension_rad.sin(), angle_suspension_rad.cos(),  0.0, 0.5,
        0.0,                        0.0,                         1.0, 0.0,
        0.0,                        0.0,                         0.0, 1.0,
    );

    let expected = homogenous_suspension_to_body;

    let suspension_to_body = model.get_homogeneous_transform_to_body(&suspension_id_leg1);
    assert!(suspension_to_body.is_ok());

    let wheel_to_body_matrix = suspension_to_body.unwrap();
    assert_eq!(expected, wheel_to_body_matrix);
}

#[test]
fn when_getting_homogeneous_transform_to_body_with_multiple_elements_and_no_motion_should_return_the_transform(
) {
    let mut model = MotionModel::new();
    let body_id = add_body_to_model(&mut model).unwrap();

    // Actuator 1
    let (sender1, receiver1) = crossbeam_channel::unbounded();
    let (cmd_sender1, _) = crossbeam_channel::unbounded();
    let mut hardware_actuator1 = MockHardwareActuator {
        receiver: receiver1,
        sender: sender1.clone(),
        command_sender: cmd_sender1,
        update_sender: None,
        id: None,
    };
    let change_processor = Box::new(HardwareChangeProcessor::new(1000));

    let actuator_1 = Actuator::new(&mut hardware_actuator1, &change_processor).unwrap();
    let id_joint_1 = add_actuated_joint_to_model(
        &mut model,
        &body_id,
        DriveModulePosition::LeftFront,
        FrameDofType::PrismaticX,
        actuator_1,
    )
    .unwrap();

    // Actuator 2
    let (sender2, receiver2) = crossbeam_channel::unbounded();
    let (cmd_sender2, _) = crossbeam_channel::unbounded();
    let mut hardware_actuator2 = MockHardwareActuator {
        receiver: receiver2,
        sender: sender2.clone(),
        command_sender: cmd_sender2,
        update_sender: None,
        id: None,
    };

    let actuator_2 = Actuator::new(&mut hardware_actuator2, &change_processor).unwrap();
    let id_joint_2 = add_actuated_joint_to_model(
        &mut model,
        &id_joint_1,
        DriveModulePosition::LeftFront,
        FrameDofType::PrismaticY,
        actuator_2,
    )
    .unwrap();

    // | cos(30) -sin(30) 0.0 1.0 |
    // | sin(30) cos(30)  0.0 0.5 |
    // | 0.0     0.0      1.0 0.0 |
    // | 0.0     0.0      0.0 1.0 |
    let (angle_deg, _) = frame_angles_in_degrees_for(DriveModulePosition::LeftFront);
    let angle_rad = angle_deg * (PI / 180.0);

    #[rustfmt::skip]
    let joint_2_to_joint_1_matrix = Matrix4::new(
        angle_rad.cos(), -angle_rad.sin(), 0.0, 1.0,
        angle_rad.sin(), angle_rad.cos(),  0.0, 0.5,
        0.0,             0.0,              1.0, 0.0,
        0.0,             0.0,              0.0, 1.0,
    );

    #[rustfmt::skip]
    let joint_1_to_body_matrix = Matrix4::new(
        angle_rad.cos(), -angle_rad.sin(), 0.0, 1.0,
        angle_rad.sin(), angle_rad.cos(),  0.0, 0.5,
        0.0,             0.0,              1.0, 0.0,
        0.0,             0.0,              0.0, 1.0,
    );

    let expected = joint_1_to_body_matrix * joint_2_to_joint_1_matrix;

    let joint_2_to_body = model.get_homogeneous_transform_to_body(&id_joint_2);
    assert!(joint_2_to_body.is_ok());
    let joint_2_to_body_matrix = joint_2_to_body.unwrap();

    let mut expected_it = expected.iter();
    let mut calculated_it = joint_2_to_body_matrix.iter();
    loop {
        match (expected_it.next(), calculated_it.next()) {
            (Some(a), Some(b)) => {
                assert!(
                    (*a).approx_eq(
                        *b,
                        F64Margin {
                            ulps: 2,
                            epsilon: 1e-6
                        }
                    ),
                    "Expected {:.5} and {:.5} to be equal within 2 ulps or 1e-6",
                    *a,
                    *b
                );
            }
            (None, None) => break,
            _ => assert!(false),
        }
    }
}

#[test]
fn when_getting_homogeneous_transform_to_body_with_primatic_x_and_prismatic_y_motion_should_return_the_transform(
) {
    let mut model = MotionModel::new();
    let body_id = add_body_to_model(&mut model).unwrap();

    // Actuator 1
    let (sender1, receiver1) = crossbeam_channel::unbounded();
    let (cmd_sender1, _) = crossbeam_channel::unbounded();
    let mut hardware_actuator1 = MockHardwareActuator {
        receiver: receiver1,
        sender: sender1.clone(),
        command_sender: cmd_sender1,
        update_sender: None,
        id: None,
    };
    let change_processor = Box::new(HardwareChangeProcessor::new(1000));

    let actuator_1 = Actuator::new(&mut hardware_actuator1, &change_processor).unwrap();
    let id_joint_1 = add_actuated_joint_to_model(
        &mut model,
        &body_id,
        DriveModulePosition::LeftFront,
        FrameDofType::PrismaticX,
        actuator_1,
    )
    .unwrap();

    // Actuator 2
    let (sender2, receiver2) = crossbeam_channel::unbounded();
    let (cmd_sender2, _) = crossbeam_channel::unbounded();
    let mut hardware_actuator2 = MockHardwareActuator {
        receiver: receiver2,
        sender: sender2.clone(),
        command_sender: cmd_sender2,
        update_sender: None,
        id: None,
    };

    let actuator_2 = Actuator::new(&mut hardware_actuator2, &change_processor).unwrap();
    let id_joint_2 = add_actuated_joint_to_model(
        &mut model,
        &id_joint_1,
        DriveModulePosition::LeftFront,
        FrameDofType::PrismaticY,
        actuator_2,
    )
    .unwrap();

    // | cos(30) -sin(30) 0.0 1.0 |
    // | sin(30) cos(30)  0.0 0.5 |
    // | 0.0     0.0      1.0 0.0 |
    // | 0.0     0.0      0.0 1.0 |
    let (angle_deg, _) = frame_angles_in_degrees_for(DriveModulePosition::LeftFront);
    let angle_rad = angle_deg * (PI / 180.0);

    #[rustfmt::skip]
    let joint_2_to_joint_1_matrix = Matrix4::new(
        angle_rad.cos(), -angle_rad.sin(), 0.0, 1.0,
        angle_rad.sin(), angle_rad.cos(),  0.0, 0.5,
        0.0,             0.0,              1.0, 0.0,
        0.0,             0.0,              0.0, 1.0,
    );

    #[rustfmt::skip]
    let joint_1_to_body_matrix = Matrix4::new(
        angle_rad.cos(), -angle_rad.sin(), 0.0, 1.0,
        angle_rad.sin(), angle_rad.cos(),  0.0, 0.5,
        0.0,             0.0,              1.0, 0.0,
        0.0,             0.0,              0.0, 1.0,
    );

    // Push the actuators out
    let joint_1_x = 1.0;
    let msg = (
        JointState::new(joint_1_x, None, None, None),
        ActuatorAvailableRatesOfChange::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    );
    let _ = sender1.send(msg).unwrap();
    let _ = hardware_actuator1
        .update_sender
        .unwrap()
        .send(hardware_actuator1.id.unwrap())
        .unwrap();

    #[rustfmt::skip]
    let translation_x = Matrix4::new(
        1.0, 0.0, 0.0, joint_1_x,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0,
    );

    let joint_1_to_body_motion_matrix = translation_x * joint_1_to_body_matrix;

    let joint_2_y = -1.0;
    let msg = (
        JointState::new(joint_2_y, None, None, None),
        ActuatorAvailableRatesOfChange::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    );
    let _ = sender2.send(msg).unwrap();
    let _ = hardware_actuator2
        .update_sender
        .unwrap()
        .send(hardware_actuator2.id.unwrap())
        .unwrap();

    #[rustfmt::skip]
    let translation_y = Matrix4::new(
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, joint_2_y,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0,
    );

    let joint_2_to_joint_1_motion_matrix = translation_y * joint_2_to_joint_1_matrix;
    let expected = joint_1_to_body_motion_matrix * joint_2_to_joint_1_motion_matrix;

    // Allow some time to ensure the task is not processed
    std::thread::sleep(Duration::from_millis(20));

    let joint_2_to_body = model.get_homogeneous_transform_to_body(&id_joint_2);
    assert!(joint_2_to_body.is_ok());
    let joint_2_to_body_matrix = joint_2_to_body.unwrap();

    let mut expected_it = expected.iter();
    let mut calculated_it = joint_2_to_body_matrix.iter();
    loop {
        match (expected_it.next(), calculated_it.next()) {
            (Some(a), Some(b)) => {
                assert!(
                    (*a).approx_eq(
                        *b,
                        F64Margin {
                            ulps: 2,
                            epsilon: 1e-6
                        }
                    ),
                    "Expected {:.5} and {:.5} to be equal within 2 ulps or 1e-6",
                    *a,
                    *b
                );
            }
            (None, None) => break,
            _ => assert!(false),
        }
    }
}

#[test]
fn when_getting_homogeneous_transform_to_body_with_primatic_x_and_prismatic_z_motion_should_return_the_transform(
) {
    let mut model = MotionModel::new();
    let body_id = add_body_to_model(&mut model).unwrap();

    // Actuator 1
    let (sender1, receiver1) = crossbeam_channel::unbounded();
    let (cmd_sender1, _) = crossbeam_channel::unbounded();
    let mut hardware_actuator1 = MockHardwareActuator {
        receiver: receiver1,
        sender: sender1.clone(),
        command_sender: cmd_sender1,
        update_sender: None,
        id: None,
    };
    let change_processor = Box::new(HardwareChangeProcessor::new(1000));

    let actuator_1 = Actuator::new(&mut hardware_actuator1, &change_processor).unwrap();
    let id_joint_1 = add_actuated_joint_to_model(
        &mut model,
        &body_id,
        DriveModulePosition::LeftFront,
        FrameDofType::PrismaticX,
        actuator_1,
    )
    .unwrap();

    // Actuator 2
    let (sender2, receiver2) = crossbeam_channel::unbounded();
    let (cmd_sender2, _) = crossbeam_channel::unbounded();
    let mut hardware_actuator2 = MockHardwareActuator {
        receiver: receiver2,
        sender: sender2.clone(),
        command_sender: cmd_sender2,
        update_sender: None,
        id: None,
    };

    let actuator_2 = Actuator::new(&mut hardware_actuator2, &change_processor).unwrap();
    let id_joint_2 = add_actuated_joint_to_model(
        &mut model,
        &id_joint_1,
        DriveModulePosition::LeftFront,
        FrameDofType::PrismaticZ,
        actuator_2,
    )
    .unwrap();

    // | cos(30) -sin(30) 0.0 1.0 |
    // | sin(30) cos(30)  0.0 0.5 |
    // | 0.0     0.0      1.0 0.0 |
    // | 0.0     0.0      0.0 1.0 |
    let (angle_deg, _) = frame_angles_in_degrees_for(DriveModulePosition::LeftFront);
    let angle_rad = angle_deg * (PI / 180.0);

    #[rustfmt::skip]
    let joint_2_to_joint_1_matrix = Matrix4::new(
        angle_rad.cos(), -angle_rad.sin(), 0.0, 1.0,
        angle_rad.sin(), angle_rad.cos(),  0.0, 0.5,
        0.0,             0.0,              1.0, 0.0,
        0.0,             0.0,              0.0, 1.0,
    );

    #[rustfmt::skip]
    let joint_1_to_body_matrix = Matrix4::new(
        angle_rad.cos(), -angle_rad.sin(), 0.0, 1.0,
        angle_rad.sin(), angle_rad.cos(),  0.0, 0.5,
        0.0,             0.0,              1.0, 0.0,
        0.0,             0.0,              0.0, 1.0,
    );

    // Push the actuators out
    let joint_1_x = 1.0;
    let msg = (
        JointState::new(joint_1_x, None, None, None),
        ActuatorAvailableRatesOfChange::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    );
    let _ = sender1.send(msg).unwrap();
    let _ = hardware_actuator1
        .update_sender
        .unwrap()
        .send(hardware_actuator1.id.unwrap())
        .unwrap();

    #[rustfmt::skip]
    let translation_x = Matrix4::new(
        1.0, 0.0, 0.0, joint_1_x,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0,
    );

    let joint_1_to_body_motion_matrix = translation_x * joint_1_to_body_matrix;

    let joint_2_z = -1.0;
    let msg = (
        JointState::new(joint_2_z, None, None, None),
        ActuatorAvailableRatesOfChange::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    );
    let _ = sender2.send(msg).unwrap();
    let _ = hardware_actuator2
        .update_sender
        .unwrap()
        .send(hardware_actuator2.id.unwrap())
        .unwrap();

    // | cos(30)  0.0 sin(30) 0.0 |
    // | 0.0      1.0 0.0     0.0 |
    // | -sin(30) 0.0 cos(30) 0.0 |
    // | 0.0      0.0 0.0     1.0 |
    #[rustfmt::skip]
    let translation_z = Matrix4::new(
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, joint_2_z,
        0.0, 0.0, 0.0, 1.0,
    );

    let joint_2_to_joint_1_motion_matrix = translation_z * joint_2_to_joint_1_matrix;
    let expected = joint_1_to_body_motion_matrix * joint_2_to_joint_1_motion_matrix;

    // Allow some time to ensure the task is not processed
    std::thread::sleep(Duration::from_millis(20));

    let joint_2_to_body = model.get_homogeneous_transform_to_body(&id_joint_2);
    assert!(joint_2_to_body.is_ok());
    let joint_2_to_body_matrix = joint_2_to_body.unwrap();

    let mut expected_it = expected.iter();
    let mut calculated_it = joint_2_to_body_matrix.iter();
    loop {
        match (expected_it.next(), calculated_it.next()) {
            (Some(a), Some(b)) => {
                assert!(
                    (*a).approx_eq(
                        *b,
                        F64Margin {
                            ulps: 2,
                            epsilon: 1e-6
                        }
                    ),
                    "Expected {:.5} and {:.5} to be equal within 2 ulps or 1e-6",
                    *a,
                    *b
                );
            }
            (None, None) => break,
            _ => assert!(false),
        }
    }
}

#[test]
fn when_getting_homogeneous_transform_to_body_with_primatic_y_and_prismatic_z_motion_should_return_the_transform(
) {
    let mut model = MotionModel::new();
    let body_id = add_body_to_model(&mut model).unwrap();

    // Actuator 1
    let (sender1, receiver1) = crossbeam_channel::unbounded();
    let (cmd_sender1, _) = crossbeam_channel::unbounded();
    let mut hardware_actuator1 = MockHardwareActuator {
        receiver: receiver1,
        sender: sender1.clone(),
        command_sender: cmd_sender1,
        update_sender: None,
        id: None,
    };
    let change_processor = Box::new(HardwareChangeProcessor::new(1000));

    let actuator_1 = Actuator::new(&mut hardware_actuator1, &change_processor).unwrap();
    let id_joint_1 = add_actuated_joint_to_model(
        &mut model,
        &body_id,
        DriveModulePosition::LeftFront,
        FrameDofType::PrismaticY,
        actuator_1,
    )
    .unwrap();

    // Actuator 2
    let (sender2, receiver2) = crossbeam_channel::unbounded();
    let (cmd_sender2, _) = crossbeam_channel::unbounded();
    let mut hardware_actuator2 = MockHardwareActuator {
        receiver: receiver2,
        sender: sender2.clone(),
        command_sender: cmd_sender2,
        update_sender: None,
        id: None,
    };

    let actuator_2 = Actuator::new(&mut hardware_actuator2, &change_processor).unwrap();
    let id_joint_2 = add_actuated_joint_to_model(
        &mut model,
        &id_joint_1,
        DriveModulePosition::LeftFront,
        FrameDofType::PrismaticZ,
        actuator_2,
    )
    .unwrap();

    // | cos(30) -sin(30) 0.0 1.0 |
    // | sin(30) cos(30)  0.0 0.5 |
    // | 0.0     0.0      1.0 0.0 |
    // | 0.0     0.0      0.0 1.0 |
    let (angle_deg, _) = frame_angles_in_degrees_for(DriveModulePosition::LeftFront);
    let angle_rad = angle_deg * (PI / 180.0);

    #[rustfmt::skip]
    let joint_2_to_joint_1_matrix = Matrix4::new(
        angle_rad.cos(), -angle_rad.sin(), 0.0, 1.0,
        angle_rad.sin(), angle_rad.cos(),  0.0, 0.5,
        0.0,             0.0,              1.0, 0.0,
        0.0,             0.0,              0.0, 1.0,
    );

    #[rustfmt::skip]
    let joint_1_to_body_matrix = Matrix4::new(
        angle_rad.cos(), -angle_rad.sin(), 0.0, 1.0,
        angle_rad.sin(), angle_rad.cos(),  0.0, 0.5,
        0.0,             0.0,              1.0, 0.0,
        0.0,             0.0,              0.0, 1.0,
    );

    // Push the actuators out
    let joint_1_y = 1.0;
    let msg = (
        JointState::new(joint_1_y, None, None, None),
        ActuatorAvailableRatesOfChange::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    );
    let _ = sender1.send(msg).unwrap();
    let _ = hardware_actuator1
        .update_sender
        .unwrap()
        .send(hardware_actuator1.id.unwrap())
        .unwrap();

    #[rustfmt::skip]
    let translation_y = Matrix4::new(
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, joint_1_y,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0,
    );

    let joint_1_to_body_moved_matrix = translation_y * joint_1_to_body_matrix;

    let joint_2_z = -1.0;
    let msg = (
        JointState::new(joint_2_z, None, None, None),
        ActuatorAvailableRatesOfChange::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    );
    let _ = sender2.send(msg).unwrap();
    let _ = hardware_actuator2
        .update_sender
        .unwrap()
        .send(hardware_actuator2.id.unwrap())
        .unwrap();

    // | cos(30)  0.0 sin(30) 0.0 |
    // | 0.0      1.0 0.0     0.0 |
    // | -sin(30) 0.0 cos(30) 0.0 |
    // | 0.0      0.0 0.0     1.0 |
    #[rustfmt::skip]
    let translation_z = Matrix4::new(
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, joint_2_z,
        0.0, 0.0, 0.0, 1.0,
    );

    let joint_2_to_joint_1_moved_matrix = translation_z * joint_2_to_joint_1_matrix;
    let expected = joint_1_to_body_moved_matrix * joint_2_to_joint_1_moved_matrix;

    // Allow some time to ensure the task is not processed
    std::thread::sleep(Duration::from_millis(20));

    let joint_2_to_body = model.get_homogeneous_transform_to_body(&id_joint_2);
    assert!(joint_2_to_body.is_ok());
    let joint_2_to_body_matrix = joint_2_to_body.unwrap();

    let mut expected_it = expected.iter();
    let mut calculated_it = joint_2_to_body_matrix.iter();
    loop {
        match (expected_it.next(), calculated_it.next()) {
            (Some(a), Some(b)) => {
                assert!(
                    (*a).approx_eq(
                        *b,
                        F64Margin {
                            ulps: 2,
                            epsilon: 1e-6
                        }
                    ),
                    "Expected {:.5} and {:.5} to be equal within 2 ulps or 1e-6",
                    *a,
                    *b
                );
            }
            (None, None) => break,
            _ => assert!(false),
        }
    }
}

#[test]
fn when_getting_homogeneous_transform_to_frame_across_wheel_chains_and_motion_it_should_return_the_transform(
) {
    let mut model = MotionModel::new();
    let body_id = add_body_to_model(&mut model).unwrap();

    // Actuator 1
    let (sender1, receiver1) = crossbeam_channel::unbounded();
    let (cmd_sender1, _) = crossbeam_channel::unbounded();
    let mut hardware_actuator1 = MockHardwareActuator {
        receiver: receiver1,
        sender: sender1.clone(),
        command_sender: cmd_sender1,
        update_sender: None,
        id: None,
    };
    let change_processor = Box::new(HardwareChangeProcessor::new(1000));

    let actuator_1 = Actuator::new(&mut hardware_actuator1, &change_processor).unwrap();
    let id_joint_1 = add_actuated_joint_to_model(
        &mut model,
        &body_id,
        DriveModulePosition::LeftFront,
        FrameDofType::RevoluteX,
        actuator_1,
    )
    .unwrap();

    // Actuator 2
    let (sender2, receiver2) = crossbeam_channel::unbounded();
    let (cmd_sender2, _) = crossbeam_channel::unbounded();
    let mut hardware_actuator2 = MockHardwareActuator {
        receiver: receiver2,
        sender: sender2.clone(),
        command_sender: cmd_sender2,
        update_sender: None,
        id: None,
    };

    let actuator_2 = Actuator::new(&mut hardware_actuator2, &change_processor).unwrap();
    let id_joint_2 = add_actuated_joint_to_model(
        &mut model,
        &body_id,
        DriveModulePosition::RightFront,
        FrameDofType::RevoluteZ,
        actuator_2,
    )
    .unwrap();

    // Joint 1
    // | cos(30) -sin(30) 0.0 1.0 |
    // | sin(30) cos(30)  0.0 0.5 |
    // | 0.0     0.0      1.0 0.0 |
    // | 0.0     0.0      0.0 1.0 |
    let (angle_deg, _) = frame_angles_in_degrees_for(DriveModulePosition::LeftFront);
    let angle_rad = angle_deg * (PI / 180.0);

    #[rustfmt::skip]
    let joint_1_to_body_static = Matrix4::new(
        angle_rad.cos(), -angle_rad.sin(), 0.0, 1.0,
        angle_rad.sin(), angle_rad.cos(),  0.0, 0.5,
        0.0,             0.0,              1.0, 0.0,
        0.0,             0.0,              0.0, 1.0,
    );

    let (angle_deg, _) = frame_angles_in_degrees_for(DriveModulePosition::RightFront);
    let angle_rad = angle_deg * (PI / 180.0);
    #[rustfmt::skip]
    let joint_2_to_body_static = Matrix4::new(
        angle_rad.cos(), -angle_rad.sin(), 0.0, 1.0,
        angle_rad.sin(), angle_rad.cos(),  0.0, -0.5,
        0.0,             0.0,              1.0, 0.0,
        0.0,             0.0,              0.0, 1.0,
    );

    // Push the actuators out
    let angle_joint_1_x_deg = 30.0;
    let angle_joint_1_x_rad = angle_joint_1_x_deg * (PI / 180.0);
    let msg = (
        JointState::new(angle_joint_1_x_rad, None, None, None),
        ActuatorAvailableRatesOfChange::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    );
    let _ = sender1.send(msg).unwrap();
    let _ = hardware_actuator1
        .update_sender
        .unwrap()
        .send(hardware_actuator1.id.unwrap())
        .unwrap();

    // | 1.0 0.0      0.0      0.0 |
    // | 0.0 cos(30)  -sin(30) 0.5 |
    // | 0.0 sin(30)  cos(30)  0.0 |
    // | 0.0 0.0      0.0      1.0 |
    #[rustfmt::skip]
    let rotation_joint_1_x = Matrix4::new(
        1.0, 0.0,                       0.0,                        0.0,
        0.0, angle_joint_1_x_rad.cos(), -angle_joint_1_x_rad.sin(), 0.0,
        0.0, angle_joint_1_x_rad.sin(), angle_joint_1_x_rad.cos(),  0.0,
        0.0, 0.0,                       0.0,                        1.0,
    );

    let expected_joint_1_to_body_matrix = rotation_joint_1_x * joint_1_to_body_static;
    let expected_joint_1_to_body_inverse = expected_joint_1_to_body_matrix.try_inverse().unwrap();

    let angle_joint_2_z_deg = 30.0;
    let angle_joint_2_z_rad = angle_joint_2_z_deg * (PI / 180.0);
    let msg = (
        JointState::new(angle_joint_2_z_rad, None, None, None),
        ActuatorAvailableRatesOfChange::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    );
    let _ = sender2.send(msg).unwrap();
    let _ = hardware_actuator2
        .update_sender
        .unwrap()
        .send(hardware_actuator2.id.unwrap())
        .unwrap();

    // | cos(30)  0.0 sin(30) 0.0 |
    // | 0.0      1.0 0.0     0.0 |
    // | -sin(30) 0.0 cos(30) 0.0 |
    // | 0.0      0.0 0.0     1.0 |
    #[rustfmt::skip]
    let rotation_z = Matrix4::new(
        angle_joint_2_z_rad.cos(), -angle_joint_2_z_rad.sin(), 0.0, 0.0,
        angle_joint_2_z_rad.sin(), angle_joint_2_z_rad.cos(),  0.0, 0.0,
        0.0,                      0.0,                         1.0, 0.0,
        0.0,                      0.0,                         0.0, 1.0,
    );

    let expected_joint_2_to_joint_1_matrix = rotation_z * joint_2_to_body_static;
    let expected = expected_joint_1_to_body_inverse * expected_joint_2_to_joint_1_matrix;

    // Allow some time to ensure the task is not processed
    std::thread::sleep(Duration::from_millis(20));

    let joint_2_to_joint_1 =
        model.get_homogeneous_transform_between_frames(&id_joint_2, &id_joint_1);
    assert!(joint_2_to_joint_1.is_ok());
    let joint_2_to_joint_1_matrix = joint_2_to_joint_1.unwrap();

    let mut expected_it = expected.iter();
    let mut calculated_it = joint_2_to_joint_1_matrix.iter();
    loop {
        match (expected_it.next(), calculated_it.next()) {
            (Some(a), Some(b)) => {
                assert!(
                    (*a).approx_eq(
                        *b,
                        F64Margin {
                            ulps: 2,
                            epsilon: 1e-6
                        }
                    ),
                    "Expected {:.5} and {:.5} to be equal within 2 ulps or 1e-6",
                    *a,
                    *b
                );
            }
            (None, None) => break,
            _ => assert!(false),
        }
    }
}

#[test]
fn when_getting_homogeneous_transform_to_frame_across_wheel_chains_and_no_motion_it_should_return_the_transform(
) {
    let mut model = MotionModel::new();
    let body_id = add_body_to_model(&mut model).unwrap();

    // Actuator 1
    let (sender1, receiver1) = crossbeam_channel::unbounded();
    let (cmd_sender1, _) = crossbeam_channel::unbounded();
    let mut hardware_actuator1 = MockHardwareActuator {
        receiver: receiver1,
        sender: sender1.clone(),
        command_sender: cmd_sender1,
        update_sender: None,
        id: None,
    };
    let change_processor = Box::new(HardwareChangeProcessor::new(1000));

    let actuator_1 = Actuator::new(&mut hardware_actuator1, &change_processor).unwrap();
    let id_joint_1 = add_actuated_joint_to_model(
        &mut model,
        &body_id,
        DriveModulePosition::LeftFront,
        FrameDofType::RevoluteX,
        actuator_1,
    )
    .unwrap();

    // Actuator 2
    let (sender2, receiver2) = crossbeam_channel::unbounded();
    let (cmd_sender2, _) = crossbeam_channel::unbounded();
    let mut hardware_actuator2 = MockHardwareActuator {
        receiver: receiver2,
        sender: sender2.clone(),
        command_sender: cmd_sender2,
        update_sender: None,
        id: None,
    };

    let actuator_2 = Actuator::new(&mut hardware_actuator2, &change_processor).unwrap();
    let id_joint_2 = add_actuated_joint_to_model(
        &mut model,
        &body_id,
        DriveModulePosition::RightFront,
        FrameDofType::RevoluteZ,
        actuator_2,
    )
    .unwrap();

    let joint_2_to_joint_1 =
        model.get_homogeneous_transform_between_frames(&id_joint_2, &id_joint_1);
    assert!(joint_2_to_joint_1.is_ok());

    let joint_2_to_joint_1_matrix = joint_2_to_joint_1.unwrap();

    // | cos(30) -sin(30) 0.0 1.0 |
    // | sin(30) cos(30)  0.0 0.5 |
    // | 0.0     0.0      1.0 0.0 |
    // | 0.0     0.0      0.0 1.0 |
    let (angle_deg, _) = frame_angles_in_degrees_for(DriveModulePosition::LeftFront);
    let angle_rad = angle_deg * (PI / 180.0);

    #[rustfmt::skip]
    let expected_joint_1_to_body_matrix = Matrix4::new(
        angle_rad.cos(), -angle_rad.sin(), 0.0, 1.0,
        angle_rad.sin(), angle_rad.cos(),  0.0, 0.5,
        0.0,             0.0,              1.0, 0.0,
        0.0,             0.0,              0.0, 1.0,
    );

    let expected_joint_1_to_body_inverse = expected_joint_1_to_body_matrix.try_inverse().unwrap();

    let (angle_deg, _) = frame_angles_in_degrees_for(DriveModulePosition::RightFront);
    let angle_rad = angle_deg * (PI / 180.0);

    #[rustfmt::skip]
    let expected_joint_2_to_body_matrix = Matrix4::new(
        angle_rad.cos(), -angle_rad.sin(), 0.0, 1.0,
        angle_rad.sin(), angle_rad.cos(),  0.0, -0.5,
        0.0,             0.0,              1.0, 0.0,
        0.0,             0.0,              0.0, 1.0,
    );

    let expected = expected_joint_1_to_body_inverse * expected_joint_2_to_body_matrix;
    let mut expected_it = expected.iter();
    let mut calculated_it = joint_2_to_joint_1_matrix.iter();
    loop {
        match (expected_it.next(), calculated_it.next()) {
            (Some(a), Some(b)) => {
                assert!(
                    (*a).approx_eq(
                        *b,
                        F64Margin {
                            ulps: 2,
                            epsilon: 1e-6
                        }
                    ),
                    "Expected {:.5} and {:.5} to be equal within 2 ulps or 1e-6",
                    *a,
                    *b
                );
            }
            (None, None) => break,
            _ => assert!(false),
        }
    }
}

#[test]
fn when_getting_homogeneous_transform_to_parent_with_no_motion_it_should_return_the_transform() {
    // child -> parent
    // no motion
    let mut model = MotionModel::new();
    let body_id = add_body_to_model(&mut model).unwrap();

    // Leg
    let suspension_id_leg1 =
        add_suspension_to_model(&mut model, &body_id, DriveModulePosition::LeftFront).unwrap();

    let (sender, receiver) = crossbeam_channel::unbounded();
    let (cmd_sender, _cmd_receiver) = crossbeam_channel::unbounded();
    let mut hardware_actuator = MockHardwareActuator {
        receiver,
        sender,
        command_sender: cmd_sender,
        update_sender: None,
        id: None,
    };
    let change_processor = Box::new(HardwareChangeProcessor::new(10));

    let actuator = Actuator::new(&mut hardware_actuator, &change_processor).unwrap();

    let steering_id_leg1 = add_steering_to_model(
        &mut model,
        &suspension_id_leg1,
        DriveModulePosition::LeftFront,
        actuator,
    )
    .unwrap();

    let (wheel_sender, wheel_receiver) = crossbeam_channel::unbounded();
    let (wheel_cmd_sender, _) = crossbeam_channel::unbounded();
    let mut wheel_hardware_actuator = MockHardwareActuator {
        receiver: wheel_receiver,
        sender: wheel_sender,
        command_sender: wheel_cmd_sender,
        update_sender: None,
        id: None,
    };

    let wheel_actuator = Actuator::new(&mut wheel_hardware_actuator, &change_processor).unwrap();

    let wheel_id_leg1 = add_wheel_to_model(&mut model, &steering_id_leg1, wheel_actuator).unwrap();

    // wheel to steering
    let wheel_to_steering = model.get_homogeneous_transform_to_parent(&wheel_id_leg1);
    assert!(wheel_to_steering.is_ok());

    let wheel_to_steering_matrix = wheel_to_steering.unwrap();

    // | 1.0 0.0 0.0 0.0 |
    // | 0.0 1.0 0.0 0.0 |
    // | 0.0 0.0 1.0 -0.1 |
    // | 0.0 0.0 0.0 1.0 |
    let expected = Matrix4::<f64>::from_rows(&[
        RowVector4::new(1.0, 0.0, 0.0, 0.0),
        RowVector4::new(0.0, 1.0, 0.0, 0.0),
        RowVector4::new(0.0, 0.0, 1.0, -0.1),
        RowVector4::new(0.0, 0.0, 0.0, 1.0),
    ]);
    assert_eq!(expected, wheel_to_steering_matrix);

    // steering to suspension
    let steering_to_suspension = model.get_homogeneous_transform_to_parent(&steering_id_leg1);
    assert!(steering_to_suspension.is_ok());

    let steering_to_suspension_matrix = steering_to_suspension.unwrap();

    // | cos(-30) -sin(-30) 0.0 0.0 |
    // | sin(-30) cos(-30)  0.0 0.0 |
    // | 0.0      0.0       1.0 -0.1 |
    // | 0.0      0.0       0.0 1.0 |
    let (_, angle_deg) = frame_angles_in_degrees_for(DriveModulePosition::LeftFront);
    let angle_rad = angle_deg * (PI / 180.0);

    #[rustfmt::skip]
    let expected = Matrix4::new(
        angle_rad.cos(), -angle_rad.sin(), 0.0, 0.25,
        angle_rad.sin(),  angle_rad.cos(), 0.0, 0.0,
        0.0,              0.0,             1.0, -0.1,
        0.0,              0.0,             0.0, 1.0,
    );
    assert_eq!(expected, steering_to_suspension_matrix);

    // suspension to body
    let suspension_to_body = model.get_homogeneous_transform_to_parent(&suspension_id_leg1);
    assert!(suspension_to_body.is_ok());

    let suspension_to_body_matrix = suspension_to_body.unwrap();

    // | cos(30) -sin(30) 0.0 0.0 |
    // | sin(30) cos(30)  0.0 0.0 |
    // | 0.0     0.0      1.0 -0.1 |
    // | 0.0     0.0      0.0 1.0 |
    let (angle_deg, _) = frame_angles_in_degrees_for(DriveModulePosition::LeftFront);
    let angle_rad = angle_deg * (PI / 180.0);

    #[rustfmt::skip]
    let expected = Matrix4::new(
        angle_rad.cos(), -angle_rad.sin(), 0.0, 1.0,
        angle_rad.sin(),  angle_rad.cos(), 0.0, 0.5,
        0.0,              0.0,             1.0, 0.0,
        0.0,              0.0,             0.0, 1.0,
    );
    assert_eq!(expected, suspension_to_body_matrix);
}

#[test]
fn when_getting_homogeneous_transform_to_parent_with_primatic_x_motion_should_return_the_transform()
{
    let mut model = MotionModel::new();
    let body_id = add_body_to_model(&mut model).unwrap();

    let (sender, receiver) = crossbeam_channel::unbounded();
    let (cmd_sender, _) = crossbeam_channel::unbounded();
    let mut hardware_actuator = MockHardwareActuator {
        receiver: receiver,
        sender: sender.clone(),
        command_sender: cmd_sender,
        update_sender: None,
        id: None,
    };
    let change_processor = Box::new(HardwareChangeProcessor::new(1000));

    let actuator = Actuator::new(&mut hardware_actuator, &change_processor).unwrap();
    let id = add_actuated_joint_to_model(
        &mut model,
        &body_id,
        DriveModulePosition::LeftFront,
        FrameDofType::PrismaticX,
        actuator,
    )
    .unwrap();

    // wheel to steering
    let actuator_to_body = model.get_homogeneous_transform_to_parent(&id);
    assert!(actuator_to_body.is_ok());

    let actuator_to_body_matrix = actuator_to_body.unwrap();

    // | cos(30) -sin(30) 0.0 1.0 |
    // | sin(30) cos(30)  0.0 0.5 |
    // | 0.0     0.0      1.0 0.0 |
    // | 0.0     0.0      0.0 1.0 |
    let (angle_deg, _) = frame_angles_in_degrees_for(DriveModulePosition::LeftFront);
    let angle_rad = angle_deg * (PI / 180.0);

    #[rustfmt::skip]
    let expected_without_motion = Matrix4::new(
        angle_rad.cos(), -angle_rad.sin(), 0.0, 1.0,
        angle_rad.sin(),  angle_rad.cos(), 0.0, 0.5,
        0.0,              0.0,             1.0, 0.0,
        0.0,              0.0,             0.0, 1.0,
    );
    assert_eq!(expected_without_motion, actuator_to_body_matrix);

    // Push the actuator out
    let msg = (
        JointState::new(1.0, None, None, None),
        ActuatorAvailableRatesOfChange::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    );
    let _ = sender.send(msg).unwrap();
    let _ = hardware_actuator
        .update_sender
        .as_ref()
        .unwrap()
        .send(hardware_actuator.id.unwrap())
        .unwrap();

    // Allow some time to ensure the task is not processed
    std::thread::sleep(Duration::from_millis(20));

    let actuator_to_body = model.get_homogeneous_transform_to_parent(&id);
    assert!(actuator_to_body.is_ok());

    let actuator_to_body_matrix = actuator_to_body.unwrap();

    // | cos(30) -sin(30) 0.0 2.0 |
    // | sin(30) cos(30)  0.0 0.5 |
    // | 0.0     0.0      1.0 0.0 |
    // | 0.0     0.0      0.0 1.0 |
    let (angle_deg, _) = frame_angles_in_degrees_for(DriveModulePosition::LeftFront);
    let angle_rad = angle_deg * (PI / 180.0);

    #[rustfmt::skip]
    let expected = Matrix4::new(
        angle_rad.cos(), -angle_rad.sin(), 0.0, 2.0,
        angle_rad.sin(),  angle_rad.cos(), 0.0, 0.5,
        0.0,              0.0,             1.0, 0.0,
        0.0,              0.0,             0.0, 1.0,
    );
    assert_eq!(expected, actuator_to_body_matrix);

    // Pull the actuator in
    let msg = (
        JointState::new(-1.0, None, None, None),
        ActuatorAvailableRatesOfChange::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    );
    let _ = sender.send(msg).unwrap();
    let _ = hardware_actuator
        .update_sender
        .as_ref()
        .unwrap()
        .send(hardware_actuator.id.unwrap())
        .unwrap();

    // Allow some time to ensure the task is not processed
    std::thread::sleep(Duration::from_millis(20));

    let actuator_to_body = model.get_homogeneous_transform_to_parent(&id);
    assert!(actuator_to_body.is_ok());

    let actuator_to_body_matrix = actuator_to_body.unwrap();

    // | cos(30) -sin(30) 0.0 0.0 |
    // | sin(30) cos(30)  0.0 0.5 |
    // | 0.0     0.0      1.0 0.0 |
    // | 0.0     0.0      0.0 1.0 |
    let (angle_deg, _) = frame_angles_in_degrees_for(DriveModulePosition::LeftFront);
    let angle_rad = angle_deg * (PI / 180.0);

    #[rustfmt::skip]
    let expected = Matrix4::new(
        angle_rad.cos(), -angle_rad.sin(), 0.0, 0.0,
        angle_rad.sin(),  angle_rad.cos(), 0.0, 0.5,
        0.0,              0.0,             1.0, 0.0,
        0.0,              0.0,             0.0, 1.0,
    );
    assert_eq!(expected, actuator_to_body_matrix);
}

#[test]
fn when_getting_homogeneous_transform_to_parent_with_primatic_y_motion_should_return_the_transform()
{
    let mut model = MotionModel::new();
    let body_id = add_body_to_model(&mut model).unwrap();

    let (sender, receiver) = crossbeam_channel::unbounded();
    let (cmd_sender, _) = crossbeam_channel::unbounded();
    let mut hardware_actuator = MockHardwareActuator {
        receiver: receiver,
        sender: sender.clone(),
        command_sender: cmd_sender,
        update_sender: None,
        id: None,
    };
    let change_processor = Box::new(HardwareChangeProcessor::new(1000));

    let actuator = Actuator::new(&mut hardware_actuator, &change_processor).unwrap();
    let id = add_actuated_joint_to_model(
        &mut model,
        &body_id,
        DriveModulePosition::LeftFront,
        FrameDofType::PrismaticY,
        actuator,
    )
    .unwrap();

    // wheel to steering
    let actuator_to_body = model.get_homogeneous_transform_to_parent(&id);
    assert!(actuator_to_body.is_ok());

    let actuator_to_body_matrix = actuator_to_body.unwrap();

    // | cos(30) -sin(30) 0.0 1.0 |
    // | sin(30) cos(30)  0.0 0.5 |
    // | 0.0     0.0      1.0 0.0 |
    // | 0.0     0.0      0.0 1.0 |
    let (angle_deg, _) = frame_angles_in_degrees_for(DriveModulePosition::LeftFront);
    let angle_rad = angle_deg * (PI / 180.0);

    #[rustfmt::skip]
    let expected = Matrix4::new(
        angle_rad.cos(), -angle_rad.sin(), 0.0, 1.0,
        angle_rad.sin(),  angle_rad.cos(), 0.0, 0.5,
        0.0,              0.0,             1.0, 0.0,
        0.0,              0.0,             0.0, 1.0,
    );
    assert_eq!(expected, actuator_to_body_matrix);

    // Push the actuator out
    let msg = (
        JointState::new(1.0, None, None, None),
        ActuatorAvailableRatesOfChange::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    );
    let _ = sender.send(msg).unwrap();
    let _ = hardware_actuator
        .update_sender
        .as_ref()
        .unwrap()
        .send(hardware_actuator.id.unwrap())
        .unwrap();

    // Allow some time to ensure the task is not processed
    std::thread::sleep(Duration::from_millis(20));

    let actuator_to_body = model.get_homogeneous_transform_to_parent(&id);
    assert!(actuator_to_body.is_ok());

    let actuator_to_body_matrix = actuator_to_body.unwrap();

    // | cos(30) -sin(30) 0.0 1.0 |
    // | sin(30) cos(30)  0.0 1.5 |
    // | 0.0     0.0      1.0 0.0 |
    // | 0.0     0.0      0.0 1.0 |
    let (angle_deg, _) = frame_angles_in_degrees_for(DriveModulePosition::LeftFront);
    let angle_rad = angle_deg * (PI / 180.0);

    #[rustfmt::skip]
    let expected = Matrix4::new(
        angle_rad.cos(), -angle_rad.sin(), 0.0, 1.0,
        angle_rad.sin(),  angle_rad.cos(), 0.0, 1.5,
        0.0,              0.0,             1.0, 0.0,
        0.0,              0.0,             0.0, 1.0,
    );
    assert_eq!(expected, actuator_to_body_matrix);

    // Pull the actuator in
    let msg = (
        JointState::new(-1.0, None, None, None),
        ActuatorAvailableRatesOfChange::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    );
    let _ = sender.send(msg).unwrap();
    let _ = hardware_actuator
        .update_sender
        .as_ref()
        .unwrap()
        .send(hardware_actuator.id.unwrap())
        .unwrap();

    // Allow some time to ensure the task is not processed
    std::thread::sleep(Duration::from_millis(20));

    let actuator_to_body = model.get_homogeneous_transform_to_parent(&id);
    assert!(actuator_to_body.is_ok());

    let actuator_to_body_matrix = actuator_to_body.unwrap();

    // | cos(30) -sin(30) 0.0 1.0 |
    // | sin(30) cos(30)  0.0 -0.5 |
    // | 0.0     0.0      1.0 0.0 |
    // | 0.0     0.0      0.0 1.0 |
    let (angle_deg, _) = frame_angles_in_degrees_for(DriveModulePosition::LeftFront);
    let angle_rad = angle_deg * (PI / 180.0);

    #[rustfmt::skip]
    let expected = Matrix4::new(
        angle_rad.cos(), -angle_rad.sin(), 0.0, 1.0,
        angle_rad.sin(),  angle_rad.cos(), 0.0, -0.5,
        0.0,              0.0,             1.0, 0.0,
        0.0,              0.0,             0.0, 1.0,
    );
    assert_eq!(expected, actuator_to_body_matrix);
}

#[test]
fn when_getting_homogeneous_transform_to_parent_with_primatic_z_motion_should_return_the_transform()
{
    let mut model = MotionModel::new();
    let body_id = add_body_to_model(&mut model).unwrap();

    let (sender, receiver) = crossbeam_channel::unbounded();
    let (cmd_sender, _) = crossbeam_channel::unbounded();
    let mut hardware_actuator = MockHardwareActuator {
        receiver: receiver,
        sender: sender.clone(),
        command_sender: cmd_sender,
        update_sender: None,
        id: None,
    };
    let change_processor = Box::new(HardwareChangeProcessor::new(1000));

    let actuator = Actuator::new(&mut hardware_actuator, &change_processor).unwrap();
    let id = add_actuated_joint_to_model(
        &mut model,
        &body_id,
        DriveModulePosition::LeftFront,
        FrameDofType::PrismaticZ,
        actuator,
    )
    .unwrap();

    // wheel to steering
    let actuator_to_body = model.get_homogeneous_transform_to_parent(&id);
    assert!(actuator_to_body.is_ok());

    let actuator_to_body_matrix = actuator_to_body.unwrap();

    // | cos(30) -sin(30) 0.0 1.0 |
    // | sin(30) cos(30)  0.0 0.5 |
    // | 0.0     0.0      1.0 0.0 |
    // | 0.0     0.0      0.0 1.0 |
    let (angle_deg, _) = frame_angles_in_degrees_for(DriveModulePosition::LeftFront);
    let angle_rad = angle_deg * (PI / 180.0);

    #[rustfmt::skip]
    let expected = Matrix4::new(
        angle_rad.cos(), -angle_rad.sin(), 0.0, 1.0,
        angle_rad.sin(),  angle_rad.cos(), 0.0, 0.5,
        0.0,              0.0,             1.0, 0.0,
        0.0,              0.0,             0.0, 1.0,
    );
    assert_eq!(expected, actuator_to_body_matrix);

    // Push the actuator out
    let msg = (
        JointState::new(1.0, None, None, None),
        ActuatorAvailableRatesOfChange::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    );
    let _ = sender.send(msg).unwrap();
    let _ = hardware_actuator
        .update_sender
        .as_ref()
        .unwrap()
        .send(hardware_actuator.id.unwrap())
        .unwrap();

    // Allow some time to ensure the task is not processed
    std::thread::sleep(Duration::from_millis(20));

    let actuator_to_body = model.get_homogeneous_transform_to_parent(&id);
    assert!(actuator_to_body.is_ok());

    let actuator_to_body_matrix = actuator_to_body.unwrap();

    // | cos(30) -sin(30) 0.0 1.0 |
    // | sin(30) cos(30)  0.0 0.5 |
    // | 0.0     0.0      1.0 1.0 |
    // | 0.0     0.0      0.0 1.0 |
    let (angle_deg, _) = frame_angles_in_degrees_for(DriveModulePosition::LeftFront);
    let angle_rad = angle_deg * (PI / 180.0);

    #[rustfmt::skip]
    let expected = Matrix4::new(
        angle_rad.cos(), -angle_rad.sin(), 0.0, 1.0,
        angle_rad.sin(),  angle_rad.cos(), 0.0, 0.5,
        0.0,              0.0,             1.0, 1.0,
        0.0,              0.0,             0.0, 1.0,
    );
    assert_eq!(expected, actuator_to_body_matrix);

    // Pull the actuator in
    let msg = (
        JointState::new(-1.0, None, None, None),
        ActuatorAvailableRatesOfChange::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    );
    let _ = sender.send(msg).unwrap();
    let _ = hardware_actuator
        .update_sender
        .as_ref()
        .unwrap()
        .send(hardware_actuator.id.unwrap())
        .unwrap();

    // Allow some time to ensure the task is not processed
    std::thread::sleep(Duration::from_millis(20));

    let actuator_to_body = model.get_homogeneous_transform_to_parent(&id);
    assert!(actuator_to_body.is_ok());

    let actuator_to_body_matrix = actuator_to_body.unwrap();

    // | cos(30) -sin(30) 0.0 1.0 |
    // | sin(30) cos(30)  0.0 0.5 |
    // | 0.0     0.0      1.0 -1.0 |
    // | 0.0     0.0      0.0 1.0 |
    let (angle_deg, _) = frame_angles_in_degrees_for(DriveModulePosition::LeftFront);
    let angle_rad = angle_deg * (PI / 180.0);

    #[rustfmt::skip]
    let expected = Matrix4::new(
        angle_rad.cos(), -angle_rad.sin(), 0.0, 1.0,
        angle_rad.sin(),  angle_rad.cos(), 0.0, 0.5,
        0.0,              0.0,             1.0, -1.0,
        0.0,              0.0,             0.0, 1.0,
    );
    assert_eq!(expected, actuator_to_body_matrix);
}

#[test]
fn when_getting_homogeneous_transform_to_parent_with_revolute_x_motion_should_return_the_transform()
{
    let mut model = MotionModel::new();
    let body_id = add_body_to_model(&mut model).unwrap();

    let (sender, receiver) = crossbeam_channel::unbounded();
    let (cmd_sender, _) = crossbeam_channel::unbounded();
    let mut hardware_actuator = MockHardwareActuator {
        receiver: receiver,
        sender: sender.clone(),
        command_sender: cmd_sender,
        update_sender: None,
        id: None,
    };
    let change_processor = Box::new(HardwareChangeProcessor::new(1000));

    let actuator = Actuator::new(&mut hardware_actuator, &change_processor).unwrap();
    let id = add_actuated_joint_to_model(
        &mut model,
        &body_id,
        DriveModulePosition::LeftFront,
        FrameDofType::RevoluteX,
        actuator,
    )
    .unwrap();

    // wheel to steering
    let actuator_to_body = model.get_homogeneous_transform_to_parent(&id);
    assert!(actuator_to_body.is_ok());

    let actuator_to_body_matrix = actuator_to_body.unwrap();

    // | cos(30) -sin(30) 0.0 1.0 |
    // | sin(30) cos(30)  0.0 0.5 |
    // | 0.0     0.0      1.0 0.0 |
    // | 0.0     0.0      0.0 1.0 |
    let (angle_deg, _) = frame_angles_in_degrees_for(DriveModulePosition::LeftFront);
    let angle_rad = angle_deg * (PI / 180.0);

    #[rustfmt::skip]
    let expected_no_movement = Matrix4::new(
        angle_rad.cos(), -angle_rad.sin(), 0.0, 1.0,
        angle_rad.sin(),  angle_rad.cos(), 0.0, 0.5,
        0.0,              0.0,             1.0, 0.0,
        0.0,              0.0,             0.0, 1.0,
    );

    assert_eq!(expected_no_movement, actuator_to_body_matrix);

    // Push the actuator out
    let angle_x_deg = 30.0;
    let angle_x_rad = angle_x_deg * (PI / 180.0);
    let msg = (
        JointState::new(angle_x_rad, None, None, None),
        ActuatorAvailableRatesOfChange::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    );
    let _ = sender.send(msg).unwrap();
    let _ = hardware_actuator
        .update_sender
        .as_ref()
        .unwrap()
        .send(hardware_actuator.id.unwrap())
        .unwrap();

    // Allow some time to ensure the task is not processed
    std::thread::sleep(Duration::from_millis(20));

    let actuator_to_body = model.get_homogeneous_transform_to_parent(&id);
    assert!(actuator_to_body.is_ok());

    let actuator_to_body_matrix = actuator_to_body.unwrap();

    // | 1.0 0.0      0.0      |
    // | 0.0 cos(30)  -sin(30) |
    // | 0.0 sin(30)  cos(30)  |
    #[rustfmt::skip]
    let rotation_x = Matrix4::new(
        1.0, 0.0,               0.0,                0.0,
        0.0, angle_x_rad.cos(), -angle_x_rad.sin(), 0.0,
        0.0, angle_x_rad.sin(), angle_x_rad.cos(),  0.0,
        0.0, 0.0,               0.0,                1.0,
    );

    let expected = rotation_x * expected_no_movement;
    let mut expected_it = expected.iter();
    let mut actuator_to_body_it = actuator_to_body_matrix.iter();
    loop {
        match (expected_it.next(), actuator_to_body_it.next()) {
            (Some(a), Some(b)) => {
                assert!(
                    (*a).approx_eq(
                        *b,
                        F64Margin {
                            ulps: 2,
                            epsilon: 1e-6
                        }
                    ),
                    "Expected {:.5} and {:.5} to be equal within 2 ulps or 1e-6",
                    *a,
                    *b
                );
            }
            (None, None) => break,
            _ => assert!(false),
        }
    }

    // Pull the actuator in
    let angle_x_deg = -30.0;
    let angle_x_rad = angle_x_deg * (PI / 180.0);
    let msg = (
        JointState::new(angle_x_rad, None, None, None),
        ActuatorAvailableRatesOfChange::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    );
    let _ = sender.send(msg).unwrap();
    let _ = hardware_actuator
        .update_sender
        .as_ref()
        .unwrap()
        .send(hardware_actuator.id.unwrap())
        .unwrap();

    // Allow some time to ensure the task is not processed
    std::thread::sleep(Duration::from_millis(20));

    let actuator_to_body = model.get_homogeneous_transform_to_parent(&id);
    assert!(actuator_to_body.is_ok());

    let actuator_to_body_matrix = actuator_to_body.unwrap();

    // | 1.0 0.0        0.0      0.0 |
    // | 0.0 cos(-30)  -sin(-30) 0.5 |
    // | 0.0 sin(-30)   cos(-30) 0.0 |
    // | 0.0 0.0        0.0      1.0 |
    #[rustfmt::skip]
    let rotation_x = Matrix4::new(
        1.0, 0.0,               0.0,                0.0,
        0.0, angle_x_rad.cos(), -angle_x_rad.sin(), 0.0,
        0.0, angle_x_rad.sin(),  angle_x_rad.cos(), 0.0,
        0.0, 0.0,               0.0,                1.0,
    );

    let expected = rotation_x * expected_no_movement;
    let mut expected_it = expected.iter();
    let mut actuator_to_body_it = actuator_to_body_matrix.iter();
    loop {
        match (expected_it.next(), actuator_to_body_it.next()) {
            (Some(a), Some(b)) => {
                assert!(
                    (*a).approx_eq(
                        *b,
                        F64Margin {
                            ulps: 2,
                            epsilon: 1e-6
                        }
                    ),
                    "Expected {:.5} and {:.5} to be equal within 2 ulps or 1e-6",
                    *a,
                    *b
                );
            }
            (None, None) => break,
            _ => assert!(false),
        }
    }
}

#[test]
fn when_getting_homogeneous_transform_to_parent_with_revolute_y_motion_should_return_the_transform()
{
    let mut model = MotionModel::new();
    let body_id = add_body_to_model(&mut model).unwrap();

    let (sender, receiver) = crossbeam_channel::unbounded();
    let (cmd_sender, _) = crossbeam_channel::unbounded();
    let mut hardware_actuator = MockHardwareActuator {
        receiver: receiver,
        sender: sender.clone(),
        command_sender: cmd_sender,
        update_sender: None,
        id: None,
    };
    let change_processor = Box::new(HardwareChangeProcessor::new(1000));

    let actuator = Actuator::new(&mut hardware_actuator, &change_processor).unwrap();
    let id = add_actuated_joint_to_model(
        &mut model,
        &body_id,
        DriveModulePosition::LeftFront,
        FrameDofType::RevoluteY,
        actuator,
    )
    .unwrap();

    // wheel to steering
    let actuator_to_body = model.get_homogeneous_transform_to_parent(&id);
    assert!(actuator_to_body.is_ok());

    let actuator_to_body_matrix = actuator_to_body.unwrap();

    // | cos(30) -sin(30) 0.0 1.0 |
    // | sin(30) cos(30)  0.0 0.5 |
    // | 0.0     0.0      1.0 0.0 |
    // | 0.0     0.0      0.0 1.0 |
    let (angle_deg, _) = frame_angles_in_degrees_for(DriveModulePosition::LeftFront);
    let angle_rad = angle_deg * (PI / 180.0);

    #[rustfmt::skip]
    let expected_without_motion = Matrix4::new(
        angle_rad.cos(), -angle_rad.sin(), 0.0, 1.0,
        angle_rad.sin(),  angle_rad.cos(), 0.0, 0.5,
        0.0,              0.0,             1.0, 0.0,
        0.0,              0.0,             0.0, 1.0,
    );
    assert_eq!(expected_without_motion, actuator_to_body_matrix);

    // Push the actuator out
    let angle_y_deg = 30.0;
    let angle_y_rad = angle_y_deg * (PI / 180.0);
    let msg = (
        JointState::new(angle_y_rad, None, None, None),
        ActuatorAvailableRatesOfChange::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    );
    let _ = sender.send(msg).unwrap();
    let _ = hardware_actuator
        .update_sender
        .as_ref()
        .unwrap()
        .send(hardware_actuator.id.unwrap())
        .unwrap();

    // Allow some time to ensure the task is not processed
    std::thread::sleep(Duration::from_millis(20));

    let actuator_to_body = model.get_homogeneous_transform_to_parent(&id);
    assert!(actuator_to_body.is_ok());

    let actuator_to_body_matrix = actuator_to_body.unwrap();

    // | cos(30)  0.0 sin(30) 0.0 |
    // | 0.0      1.0 0.0     0.0 |
    // | -sin(30) 0.0 cos(30) 0.0 |
    // | 0.0      0.0 0.0     1.0 |
    #[rustfmt::skip]
    let rotation_y = Matrix4::new(
        angle_y_rad.cos(),  0.0, angle_y_rad.sin(), 0.0,
        0.0,                1.0, 0.0,               0.0,
        -angle_y_rad.sin(), 0.0, angle_y_rad.cos(), 0.0,
        0.0,                0.0, 0.0,               1.0,
    );

    let expected = rotation_y * expected_without_motion;
    let mut expected_it = expected.iter();
    let mut actuator_to_body_it = actuator_to_body_matrix.iter();
    loop {
        match (expected_it.next(), actuator_to_body_it.next()) {
            (Some(a), Some(b)) => {
                assert!(
                    (*a).approx_eq(
                        *b,
                        F64Margin {
                            ulps: 2,
                            epsilon: 1e-6
                        }
                    ),
                    "Expected {:.5} and {:.5} to be equal within 2 ulps or 1e-6",
                    *a,
                    *b
                );
            }
            (None, None) => break,
            _ => assert!(false),
        }
    }

    // Pull the actuator in
    let angle_y_deg = -30.0;
    let angle_y_rad = angle_y_deg * (PI / 180.0);
    let msg = (
        JointState::new(angle_y_rad, None, None, None),
        ActuatorAvailableRatesOfChange::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    );
    let _ = sender.send(msg).unwrap();
    let _ = hardware_actuator
        .update_sender
        .as_ref()
        .unwrap()
        .send(hardware_actuator.id.unwrap())
        .unwrap();

    // Allow some time to ensure the task is not processed
    std::thread::sleep(Duration::from_millis(20));

    let actuator_to_body = model.get_homogeneous_transform_to_parent(&id);
    assert!(actuator_to_body.is_ok());

    let actuator_to_body_matrix = actuator_to_body.unwrap();

    // | cos(-30)  0.0 sin(-30) 0.0 |
    // | 0.0       1.0 0.0      0.0 |
    // | -sin(-30) 0.0 cos(-30) 0.0 |
    // | 0.0       0.0 0.0      1.0 |
    #[rustfmt::skip]
    let rotation_y = Matrix4::new(
        angle_y_rad.cos(),  0.0, angle_y_rad.sin(), 0.0,
        0.0,                1.0, 0.0,               0.0,
        -angle_y_rad.sin(), 0.0, angle_y_rad.cos(), 0.0,
        0.0,                0.0, 0.0,               1.0,
    );

    let expected = rotation_y * expected_without_motion;
    let mut expected_it = expected.iter();
    let mut actuator_to_body_it = actuator_to_body_matrix.iter();
    loop {
        match (expected_it.next(), actuator_to_body_it.next()) {
            (Some(a), Some(b)) => {
                assert!(
                    (*a).approx_eq(
                        *b,
                        F64Margin {
                            ulps: 2,
                            epsilon: 1e-6
                        }
                    ),
                    "Expected {:.5} and {:.5} to be equal within 2 ulps or 1e-6",
                    *a,
                    *b
                );
            }
            (None, None) => break,
            _ => assert!(false),
        }
    }
}

#[test]
fn when_getting_homogeneous_transform_to_parent_with_revolute_z_motion_should_return_the_transform()
{
    let mut model = MotionModel::new();
    let body_id = add_body_to_model(&mut model).unwrap();

    let (sender, receiver) = crossbeam_channel::unbounded();
    let (cmd_sender, _) = crossbeam_channel::unbounded();
    let mut hardware_actuator = MockHardwareActuator {
        receiver: receiver,
        sender: sender.clone(),
        command_sender: cmd_sender,
        update_sender: None,
        id: None,
    };
    let change_processor = Box::new(HardwareChangeProcessor::new(1000));

    let actuator = Actuator::new(&mut hardware_actuator, &change_processor).unwrap();
    let id = add_actuated_joint_to_model(
        &mut model,
        &body_id,
        DriveModulePosition::LeftFront,
        FrameDofType::RevoluteZ,
        actuator,
    )
    .unwrap();

    let actuator_to_body = model.get_homogeneous_transform_to_parent(&id);
    assert!(actuator_to_body.is_ok());

    let actuator_to_body_matrix = actuator_to_body.unwrap();

    // | cos(30) -sin(30) 0.0 1.0 |
    // | sin(30) cos(30)  0.0 0.5 |
    // | 0.0     0.0      1.0 0.0 |
    // | 0.0     0.0      0.0 1.0 |
    let (angle_deg, _) = frame_angles_in_degrees_for(DriveModulePosition::LeftFront);
    let angle_rad = angle_deg * (PI / 180.0);

    #[rustfmt::skip]
    let expected_without_motion = Matrix4::new(
        angle_rad.cos(), -angle_rad.sin(), 0.0, 1.0,
        angle_rad.sin(),  angle_rad.cos(), 0.0, 0.5,
        0.0,              0.0,             1.0, 0.0,
        0.0,              0.0,             0.0, 1.0,
    );
    assert_eq!(expected_without_motion, actuator_to_body_matrix);

    // Push the actuator out
    let angle_z_deg = 30.0;
    let angle_z_rad = angle_z_deg * (PI / 180.0);
    let msg = (
        JointState::new(angle_z_rad, None, None, None),
        ActuatorAvailableRatesOfChange::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    );
    let _ = sender.send(msg).unwrap();
    let _ = hardware_actuator
        .update_sender
        .as_ref()
        .unwrap()
        .send(hardware_actuator.id.unwrap())
        .unwrap();

    // Allow some time to ensure the task is not processed
    std::thread::sleep(Duration::from_millis(20));

    let actuator_to_body = model.get_homogeneous_transform_to_parent(&id);
    assert!(actuator_to_body.is_ok());

    let actuator_to_body_matrix = actuator_to_body.unwrap();

    #[rustfmt::skip]
    let rotation_z = Matrix4::new(
        angle_z_rad.cos(), -angle_z_rad.sin(), 0.0, 0.0,
        angle_z_rad.sin(),  angle_z_rad.cos(), 0.0, 0.0,
        0.0,                0.0,               1.0, 0.0,
        0.0,                0.0,               0.0, 1.0,
    );

    let expected = rotation_z * expected_without_motion;
    let mut expected_it = expected.iter();
    let mut actuator_to_body_it = actuator_to_body_matrix.iter();
    loop {
        match (expected_it.next(), actuator_to_body_it.next()) {
            (Some(a), Some(b)) => {
                assert!(
                    (*a).approx_eq(
                        *b,
                        F64Margin {
                            ulps: 2,
                            epsilon: 1e-6
                        }
                    ),
                    "Expected {:.5} and {:.5} to be equal within 2 ulps or 1e-6",
                    *a,
                    *b
                );
            }
            (None, None) => break,
            _ => assert!(false),
        }
    }

    // Pull the actuator in
    let angle_z_deg = -30.0;
    let angle_z_rad = angle_z_deg * (PI / 180.0);
    let msg = (
        JointState::new(angle_z_rad, None, None, None),
        ActuatorAvailableRatesOfChange::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    );
    let _ = sender.send(msg).unwrap();
    let _ = hardware_actuator
        .update_sender
        .as_ref()
        .unwrap()
        .send(hardware_actuator.id.unwrap())
        .unwrap();

    // Allow some time to ensure the task is not processed
    std::thread::sleep(Duration::from_millis(20));

    let actuator_to_body = model.get_homogeneous_transform_to_parent(&id);
    assert!(actuator_to_body.is_ok());

    let actuator_to_body_matrix = actuator_to_body.unwrap();

    #[rustfmt::skip]
    let rotation_z = Matrix4::new(
        angle_z_rad.cos(), -angle_z_rad.sin(), 0.0, 0.0,
        angle_z_rad.sin(),  angle_z_rad.cos(), 0.0, 0.0,
        0.0,                0.0,               1.0, 0.0,
        0.0,                0.0,               0.0, 1.0,
    );

    let expected = rotation_z * expected_without_motion;
    let mut expected_it = expected.iter();
    let mut actuator_to_body_it = actuator_to_body_matrix.iter();
    loop {
        match (expected_it.next(), actuator_to_body_it.next()) {
            (Some(a), Some(b)) => {
                assert!(
                    (*a).approx_eq(
                        *b,
                        F64Margin {
                            ulps: 2,
                            epsilon: 1e-6
                        }
                    ),
                    "Expected {:.5} and {:.5} to be equal within 2 ulps or 1e-6",
                    *a,
                    *b,
                );
            }
            (None, None) => break,
            _ => assert!(false),
        }
    }
}

#[test]
fn when_getting_parent_with_invalid_frame_it_should_error() {
    let mut model = MotionModel::new();
    let _ = add_body_to_model(&mut model).unwrap();

    let invalid_id = FrameID::new();
    let result = model.get_parent(&invalid_id);

    assert!(result.is_err());
}

#[test]
fn when_getting_steering_frame_for_wheel_with_invalid_frame_it_should_error() {
    let mut model = MotionModel::new();
    let _ = add_body_to_model(&mut model).unwrap();

    let invalid_id = FrameID::new();
    let result = model.get_steering_frame_for_wheel(&invalid_id);

    assert!(result.is_err());
}
