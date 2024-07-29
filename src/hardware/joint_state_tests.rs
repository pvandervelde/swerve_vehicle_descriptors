use super::*;

#[test]
fn test_new_joint_state() {
    let position = 10.0;
    let velocity = Some(5.0);
    let acceleration = Some(2.0);
    let jerk = Some(1.0);

    let joint_state = JointState::new(position, velocity, acceleration, jerk);

    assert_eq!(joint_state.get_position(), position);
    assert_eq!(*joint_state.get_velocity(), velocity);
    assert_eq!(*joint_state.get_acceleration(), acceleration);
    assert_eq!(*joint_state.get_jerk(), jerk);
}

#[test]
fn test_joint_state_range_new() {
    let min_state = JointState::new(-100.0, Some(-50.0), Some(-20.0), Some(-10.0));
    let max_state = JointState::new(100.0, Some(50.0), Some(20.0), Some(10.0));

    let range = JointStateRange::new(min_state, max_state);

    assert_eq!(range.get_minimum_position(), -100.0);
    assert_eq!(range.get_maximum_position(), 100.0);
    assert_eq!(*range.get_minimum_velocity(), Some(-50.0));
    assert_eq!(*range.get_maximum_velocity(), Some(50.0));
    assert_eq!(*range.get_minimum_acceleration(), Some(-20.0));
    assert_eq!(*range.get_maximum_acceleration(), Some(20.0));
    assert_eq!(*range.get_minimum_jerk(), Some(-10.0));
    assert_eq!(*range.get_maximum_jerk(), Some(10.0));
}

#[test]
fn test_joint_state_defaults() {
    let joint_state = JointState::new(0.0, None, None, None);

    assert_eq!(joint_state.get_position(), 0.0);
    assert!(joint_state.get_velocity().is_none());
    assert!(joint_state.get_acceleration().is_none());
    assert!(joint_state.get_jerk().is_none());
}
