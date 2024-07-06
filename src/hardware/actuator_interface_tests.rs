use super::*;

#[test]
fn test_new_instance() {
    let rates_of_change = ActuatorAvailableRatesOfChange::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);

    assert_eq!(rates_of_change.get_minimum_velocity(), 1.0);
    assert_eq!(rates_of_change.get_maximum_velocity(), 2.0);
    assert_eq!(rates_of_change.get_minimum_acceleration(), 3.0);
    assert_eq!(rates_of_change.get_maximum_acceleration(), 4.0);
    assert_eq!(rates_of_change.get_minimum_jerk(), 5.0);
    assert_eq!(rates_of_change.get_maximum_jerk(), 6.0);
}

#[test]
fn test_getters() {
    let rates_of_change = ActuatorAvailableRatesOfChange::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);

    assert_eq!(rates_of_change.get_minimum_velocity(), 1.0);
    assert_eq!(rates_of_change.get_maximum_velocity(), 2.0);
    assert_eq!(rates_of_change.get_minimum_acceleration(), 3.0);
    assert_eq!(rates_of_change.get_maximum_acceleration(), 4.0);
    assert_eq!(rates_of_change.get_minimum_jerk(), 5.0);
    assert_eq!(rates_of_change.get_maximum_jerk(), 6.0);
}

#[test]
fn test_setters() {
    let mut rates_of_change = ActuatorAvailableRatesOfChange::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);

    rates_of_change.set_minimum_velocity(10.0);
    assert_eq!(rates_of_change.get_minimum_velocity(), 10.0);

    rates_of_change.set_maximum_velocity(20.0);
    assert_eq!(rates_of_change.get_maximum_velocity(), 20.0);

    rates_of_change.set_minimum_acceleration(30.0);
    assert_eq!(rates_of_change.get_minimum_acceleration(), 30.0);

    rates_of_change.set_maximum_acceleration(40.0);
    assert_eq!(rates_of_change.get_maximum_acceleration(), 40.0);

    rates_of_change.set_minimum_jerk(50.0);
    assert_eq!(rates_of_change.get_minimum_jerk(), 50.0);

    rates_of_change.set_maximum_jerk(60.0);
    assert_eq!(rates_of_change.get_maximum_jerk(), 60.0);
}
