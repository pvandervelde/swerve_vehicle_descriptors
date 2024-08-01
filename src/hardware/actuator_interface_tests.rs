use super::*;

#[test]
fn test_new_instance() {
    let rates_of_change = ActuatorAvailableRatesOfChange::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);

    assert_eq!(rates_of_change.minimum_velocity(), 1.0);
    assert_eq!(rates_of_change.maximum_velocity(), 2.0);
    assert_eq!(rates_of_change.minimum_acceleration(), 3.0);
    assert_eq!(rates_of_change.maximum_acceleration(), 4.0);
    assert_eq!(rates_of_change.minimum_jerk(), 5.0);
    assert_eq!(rates_of_change.maximum_jerk(), 6.0);
}

#[test]
fn test_getters() {
    let rates_of_change = ActuatorAvailableRatesOfChange::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);

    assert_eq!(rates_of_change.minimum_velocity(), 1.0);
    assert_eq!(rates_of_change.maximum_velocity(), 2.0);
    assert_eq!(rates_of_change.minimum_acceleration(), 3.0);
    assert_eq!(rates_of_change.maximum_acceleration(), 4.0);
    assert_eq!(rates_of_change.minimum_jerk(), 5.0);
    assert_eq!(rates_of_change.maximum_jerk(), 6.0);
}
