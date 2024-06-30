use super::*;
use std::f64::consts::PI;

#[test]
fn test_linear_unbounded_space_distance_between_values() {
    let space = LinearUnboundedSpace::new();
    assert_eq!(space.distance_between_values(1.0, 4.0), vec![3.0]);
    assert_eq!(space.distance_between_values(-2.0, 2.0), vec![4.0]);
    assert_eq!(space.distance_between_values(0.0, 0.0), vec![0.0]);
}

#[test]
fn test_linear_unbounded_space_normalize_value() {
    let space = LinearUnboundedSpace::new();
    assert_eq!(space.normalize_value(5.0), 5.0);
    assert_eq!(space.normalize_value(-3.0), -3.0);
    assert_eq!(space.normalize_value(0.0), 0.0);
}

#[test]
fn test_linear_unbounded_space_smallest_distance_between_values() {
    let space = LinearUnboundedSpace::new();
    assert_eq!(space.smallest_distance_between_values(1.0, 4.0), 3.0);
    assert_eq!(space.smallest_distance_between_values(-2.0, 2.0), 4.0);
    assert_eq!(space.smallest_distance_between_values(0.0, 0.0), 0.0);
}

#[test]
fn test_periodic_bounded_circular_space_distance_between_values() {
    let space = PeriodicBoundedCircularSpace::new_with_two_pi_range(0.0);
    let dist = space.distance_between_values(0.0, 2.0 * PI);
    assert!(dist.contains(&0.0) && dist.contains(&(-2.0 * PI)));
    let dist = space.distance_between_values(PI / 4.0, -PI / 4.0);
    assert!(dist.contains(&(-PI / 2.0)) && dist.contains(&(3.0 * PI / 2.0)));
}

#[test]
fn test_periodic_bounded_circular_space_normalize_value() {
    let space = PeriodicBoundedCircularSpace::new_with_two_pi_range(-PI);
    assert_eq!(space.normalize_value(3.0 * PI), -PI);
    assert_eq!(space.normalize_value(-3.0 * PI), PI);
    assert_eq!(space.normalize_value(PI), PI);
    assert_eq!(space.normalize_value(-PI), -PI);
}

#[test]
fn test_periodic_bounded_circular_space_smallest_distance_between_values() {
    let space = PeriodicBoundedCircularSpace::new_with_two_pi_range(-PI);
    assert_eq!(space.smallest_distance_between_values(0.0, 2.0 * PI), 0.0);
    assert_eq!(
        space.smallest_distance_between_values(PI / 4.0, -PI / 4.0),
        -PI / 2.0
    );
}

#[test]
fn test_periodic_bounded_circular_space_distance_between_values_zero_to_two_pi() {
    let space = PeriodicBoundedCircularSpace::new_with_two_pi_range(0.0);
    let dist = space.distance_between_values(0.0, 4.0 * PI);
    assert!(dist.contains(&0.0) && dist.contains(&(-4.0 * PI)));
    let dist = space.distance_between_values(PI / 4.0, 7.0 * PI / 4.0);
    assert!(dist.contains(&-PI) && dist.contains(&(3.0 * PI)));
}

#[test]
fn test_periodic_bounded_circular_space_normalize_value_zero_to_two_pi() {
    let space = PeriodicBoundedCircularSpace::new_with_two_pi_range(0.0);
    assert_eq!(space.normalize_value(3.0 * PI), PI);
    assert_eq!(space.normalize_value(-PI), PI);
    assert_eq!(space.normalize_value(2.0 * PI), 0.0);
    assert_eq!(space.normalize_value(-2.0 * PI), 0.0);
}

#[test]
fn test_periodic_bounded_circular_space_smallest_distance_between_values_zero_to_two_pi() {
    let space = PeriodicBoundedCircularSpace::new_with_two_pi_range(0.0);
    assert_eq!(space.smallest_distance_between_values(0.0, 4.0 * PI), 0.0);
    assert_eq!(
        space.smallest_distance_between_values(PI / 4.0, 7.0 * PI / 4.0),
        -PI
    );
}
