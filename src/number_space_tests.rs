use float_cmp::{ApproxEq, F64Margin};

use super::*;
use std::f64::consts::PI;

// Linear unbounded space

#[test]
fn test_linear_unbounded_space_distance_between_values() {
    let space = LinearUnboundedSpace::new();

    assert_eq!(space.distance_between_values(0.0, 0.0), vec![0.0]);
    assert_eq!(space.distance_between_values(0.0, 1.0), vec![1.0]);
    assert_eq!(space.distance_between_values(1.0, 0.0), vec![-1.0]);
    assert_eq!(space.distance_between_values(-1.0, 1.0), vec![2.0]);
    assert_eq!(space.distance_between_values(1.0, -1.0), vec![-2.0]);

    assert_eq!(
        space.distance_between_values(0.0, f64::INFINITY),
        vec![f64::INFINITY]
    );
    assert_eq!(
        space.distance_between_values(f64::NEG_INFINITY, 0.0),
        vec![f64::INFINITY]
    );

    assert_eq!(
        space.distance_between_values(f64::INFINITY, 0.0),
        vec![f64::NEG_INFINITY]
    );
    assert_eq!(
        space.distance_between_values(0.0, f64::NEG_INFINITY),
        vec![f64::NEG_INFINITY]
    );

    assert_eq!(
        space.distance_between_values(f64::NEG_INFINITY, f64::INFINITY),
        vec![f64::INFINITY]
    );
}

#[test]
fn test_linear_unbounded_space_normalize_value() {
    let space = LinearUnboundedSpace::new();
    assert!(space.normalize_value(0.0).approx_eq(
        0.0,
        F64Margin {
            ulps: 2,
            epsilon: 1e-6
        }
    ));

    assert!(space.normalize_value(1.0).approx_eq(
        1.0,
        F64Margin {
            ulps: 2,
            epsilon: 1e-6
        }
    ));

    assert!(space.normalize_value(-1.0).approx_eq(
        -1.0,
        F64Margin {
            ulps: 2,
            epsilon: 1e-6
        }
    ));

    assert!(space.normalize_value(100.0).approx_eq(
        100.0,
        F64Margin {
            ulps: 2,
            epsilon: 1e-6
        }
    ));
    assert!(space.normalize_value(-100.0).approx_eq(
        -100.0,
        F64Margin {
            ulps: 2,
            epsilon: 1e-6
        }
    ));

    assert_eq!(space.normalize_value(f64::INFINITY), f64::INFINITY);
    assert_eq!(space.normalize_value(f64::NEG_INFINITY), f64::NEG_INFINITY);
}

#[test]
fn test_linear_unbounded_space_smallest_distance_between_values() {
    let space = LinearUnboundedSpace::new();

    assert!(space.smallest_distance_between_values(0.0, 0.0).approx_eq(
        0.0,
        F64Margin {
            ulps: 2,
            epsilon: 1e-6
        }
    ));

    assert!(space.smallest_distance_between_values(0.0, 1.0).approx_eq(
        1.0,
        F64Margin {
            ulps: 2,
            epsilon: 1e-6
        }
    ));

    assert!(space.smallest_distance_between_values(1.0, 0.0).approx_eq(
        -1.0,
        F64Margin {
            ulps: 2,
            epsilon: 1e-6
        }
    ));

    assert!(space.smallest_distance_between_values(-1.0, 1.0).approx_eq(
        2.0,
        F64Margin {
            ulps: 2,
            epsilon: 1e-6
        }
    ));

    assert!(space.smallest_distance_between_values(1.0, -1.0).approx_eq(
        -2.0,
        F64Margin {
            ulps: 2,
            epsilon: 1e-6
        }
    ));

    assert!(f64::is_infinite(
        space.smallest_distance_between_values(0.0, f64::INFINITY)
    ));
    assert!(f64::is_infinite(
        space.smallest_distance_between_values(f64::NEG_INFINITY, 0.0)
    ));

    assert!(f64::is_infinite(
        space.smallest_distance_between_values(f64::INFINITY, 0.0)
    ));
    assert!(f64::is_infinite(
        space.smallest_distance_between_values(0.0, f64::INFINITY)
    ));

    assert!(f64::is_infinite(space.smallest_distance_between_values(
        f64::NEG_INFINITY,
        f64::INFINITY
    )));
}

// Periodic circular space

#[test]
fn test_periodic_bounded_circular_space_distance_between_values_minus_pi_to_pi() {
    let space = PeriodicBoundedCircularSpace::new_with_two_pi_range(-PI);

    assert_eq!(
        space.distance_between_values(0.0, 0.0),
        vec![0.0, -2.0 * PI]
    );
    assert_eq!(space.distance_between_values(0.0, PI), vec![PI, -PI]);
    assert_eq!(space.distance_between_values(PI, 0.0), vec![PI, -PI]);
    assert_eq!(space.distance_between_values(-PI, PI), vec![0.0, -2.0 * PI]);
    assert_eq!(space.distance_between_values(PI, -PI), vec![0.0, -2.0 * PI]);

    assert_eq!(
        space.distance_between_values(-0.5 * PI, PI),
        vec![1.5 * PI, -0.5 * PI]
    );

    assert_eq!(
        space.distance_between_values(0.0, 2.0 * PI),
        vec![0.0, -2.0 * PI]
    );
    assert_eq!(
        space.distance_between_values(0.0, 4.0 * PI),
        vec![0.0, -2.0 * PI]
    );

    assert_eq!(
        space.distance_between_values(PI, 3.0 * PI),
        vec![0.0, -2.0 * PI]
    );

    let diff = space.distance_between_values(11.0 * PI, 20.0 * PI);
    assert!(diff[0].approx_eq(
        PI,
        F64Margin {
            ulps: 2,
            epsilon: 1e-6
        }
    ));
    assert!(diff[1].approx_eq(
        -PI,
        F64Margin {
            ulps: 2,
            epsilon: 1e-6
        }
    ));

    assert_eq!(
        space.distance_between_values(0.25 * PI, 0.5 * PI),
        vec![0.25 * PI, -1.75 * PI]
    );

    assert_eq!(
        space.distance_between_values(1.0 * PI, 1.5 * PI),
        vec![0.5 * PI, -1.5 * PI]
    );
    assert_eq!(
        space.distance_between_values(0.75 * PI, 1.25 * PI),
        vec![0.5 * PI, -1.5 * PI]
    );

    assert_eq!(
        space.distance_between_values(1.5 * PI, 1.0 * PI),
        vec![1.5 * PI, -0.5 * PI]
    );
    assert_eq!(
        space.distance_between_values(1.25 * PI, 0.75 * PI),
        vec![1.5 * PI, -0.5 * PI]
    );
}

#[test]
fn test_periodic_bounded_circular_space_normalize_value_minus_pi_to_pi() {
    let space = PeriodicBoundedCircularSpace::new_with_two_pi_range(-PI);

    assert!(space.normalize_value(0.0).approx_eq(
        0.0,
        F64Margin {
            ulps: 2,
            epsilon: 1e-6
        }
    ));
    assert!(space.normalize_value(PI).approx_eq(
        PI,
        F64Margin {
            ulps: 2,
            epsilon: 1e-6
        }
    ));
    assert!(space.normalize_value(-PI).approx_eq(
        -PI,
        F64Margin {
            ulps: 2,
            epsilon: 1e-6
        }
    ));
    assert!(space.normalize_value(3.0 * PI).approx_eq(
        PI,
        F64Margin {
            ulps: 2,
            epsilon: 1e-6
        }
    ));
    assert!(space.normalize_value(-3.0 * PI).approx_eq(
        -PI,
        F64Margin {
            ulps: 2,
            epsilon: 1e-6
        }
    ));

    assert!(space.normalize_value(0.25 * PI).approx_eq(
        0.25 * PI,
        F64Margin {
            ulps: 2,
            epsilon: 1e-6
        }
    ));
    assert!(space.normalize_value(-0.25 * PI).approx_eq(
        -0.25 * PI,
        F64Margin {
            ulps: 2,
            epsilon: 1e-6
        }
    ));

    assert!(space.normalize_value(0.75 * PI).approx_eq(
        0.75 * PI,
        F64Margin {
            ulps: 2,
            epsilon: 1e-6
        }
    ));
    assert!(space.normalize_value(-0.75 * PI).approx_eq(
        -0.75 * PI,
        F64Margin {
            ulps: 2,
            epsilon: 1e-6
        }
    ));

    assert!(space.normalize_value(-1.5 * PI).approx_eq(
        0.5 * PI,
        F64Margin {
            ulps: 2,
            epsilon: 1e-6
        }
    ));
    assert!(space.normalize_value(1.5 * PI).approx_eq(
        -0.5 * PI,
        F64Margin {
            ulps: 2,
            epsilon: 1e-6
        }
    ));
}

#[test]
fn test_periodic_bounded_circular_space_smallest_distance_between_values_minus_pi_to_pi() {
    let space = PeriodicBoundedCircularSpace::new_with_two_pi_range(-PI);

    assert!(space.smallest_distance_between_values(0.0, 0.0).approx_eq(
        0.0,
        F64Margin {
            ulps: 2,
            epsilon: 1e-6
        }
    ));
    assert!(space.smallest_distance_between_values(0.0, PI).approx_eq(
        PI,
        F64Margin {
            ulps: 2,
            epsilon: 1e-6
        }
    ));
    assert!(space.smallest_distance_between_values(PI, 0.0).approx_eq(
        -PI,
        F64Margin {
            ulps: 2,
            epsilon: 1e-6
        }
    ));
    assert!(space.smallest_distance_between_values(-PI, PI).approx_eq(
        0.0,
        F64Margin {
            ulps: 2,
            epsilon: 1e-6
        }
    ));
    assert!(space.smallest_distance_between_values(PI, -PI).approx_eq(
        0.0,
        F64Margin {
            ulps: 2,
            epsilon: 1e-6
        }
    ));

    assert!(space
        .smallest_distance_between_values(0.0, 2.0 * PI)
        .approx_eq(
            0.0,
            F64Margin {
                ulps: 2,
                epsilon: 1e-6
            }
        ));
    assert!(space
        .smallest_distance_between_values(0.0, 4.0 * PI)
        .approx_eq(
            0.0,
            F64Margin {
                ulps: 2,
                epsilon: 1e-6
            }
        ));

    assert!(space
        .smallest_distance_between_values(PI, 3.0 * PI)
        .approx_eq(
            0.0,
            F64Margin {
                ulps: 2,
                epsilon: 1e-6
            }
        ));
    let diff = space.smallest_distance_between_values(11.0 * PI, 20.0 * PI);
    assert!(diff.approx_eq(
        -PI,
        F64Margin {
            ulps: 2,
            epsilon: 1e-6
        }
    ));

    assert!(space
        .smallest_distance_between_values(0.25 * PI, 0.5 * PI)
        .approx_eq(
            0.25 * PI,
            F64Margin {
                ulps: 2,
                epsilon: 1e-6
            }
        ));

    assert!(space
        .smallest_distance_between_values(1.0 * PI, 1.5 * PI)
        .approx_eq(
            0.5 * PI,
            F64Margin {
                ulps: 2,
                epsilon: 1e-6
            }
        ));
    assert!(space
        .smallest_distance_between_values(0.75 * PI, 1.25 * PI)
        .approx_eq(
            0.5 * PI,
            F64Margin {
                ulps: 2,
                epsilon: 1e-6
            }
        ));

    assert!(space
        .smallest_distance_between_values(1.5 * PI, 1.0 * PI)
        .approx_eq(
            -0.5 * PI,
            F64Margin {
                ulps: 2,
                epsilon: 1e-6
            }
        ));
    assert!(space
        .smallest_distance_between_values(1.25 * PI, 0.75 * PI)
        .approx_eq(
            -0.5 * PI,
            F64Margin {
                ulps: 2,
                epsilon: 1e-6
            }
        ));
}

#[test]
fn test_periodic_bounded_circular_space_distance_between_values_zero_to_two_pi() {
    let space = PeriodicBoundedCircularSpace::new_with_two_pi_range(0.0);

    assert_eq!(
        space.distance_between_values(0.0, 0.0),
        vec![0.0, -2.0 * PI]
    );
    assert_eq!(space.distance_between_values(0.0, PI), vec![PI, -PI]);
    assert_eq!(space.distance_between_values(PI, 0.0), vec![PI, -PI]);
    assert_eq!(space.distance_between_values(-PI, PI), vec![0.0, -2.0 * PI]);
    assert_eq!(space.distance_between_values(PI, -PI), vec![0.0, -2.0 * PI]);

    assert_eq!(
        space.distance_between_values(0.0, 2.0 * PI),
        vec![0.0, -2.0 * PI]
    );
    assert_eq!(
        space.distance_between_values(0.0, 4.0 * PI),
        vec![0.0, -2.0 * PI]
    );

    assert_eq!(
        space.distance_between_values(PI, 3.0 * PI),
        vec![0.0, -2.0 * PI]
    );
    let diff = space.distance_between_values(11.0 * PI, 20.0 * PI);
    assert!(diff[0].approx_eq(
        PI,
        F64Margin {
            ulps: 2,
            epsilon: 1e-6
        }
    ));
    assert!(diff[1].approx_eq(
        -PI,
        F64Margin {
            ulps: 2,
            epsilon: 1e-6
        }
    ));

    assert_eq!(
        space.distance_between_values(0.25 * PI, 0.5 * PI),
        vec![0.25 * PI, -1.75 * PI]
    );

    assert_eq!(
        space.distance_between_values(1.0 * PI, 1.5 * PI),
        vec![0.5 * PI, -1.5 * PI]
    );
    assert_eq!(
        space.distance_between_values(0.75 * PI, 1.25 * PI),
        vec![0.5 * PI, -1.5 * PI]
    );

    assert_eq!(
        space.distance_between_values(1.5 * PI, 1.0 * PI),
        vec![1.5 * PI, -0.5 * PI]
    );
    assert_eq!(
        space.distance_between_values(1.25 * PI, 0.75 * PI),
        vec![1.5 * PI, -0.5 * PI]
    );
}

#[test]
fn test_periodic_bounded_circular_space_normalize_value_zero_to_two_pi() {
    let space = PeriodicBoundedCircularSpace::new_with_two_pi_range(0.0);

    assert!(space.normalize_value(0.0).approx_eq(
        0.0,
        F64Margin {
            ulps: 2,
            epsilon: 1e-6
        }
    ));
    assert!(space.normalize_value(PI).approx_eq(
        PI,
        F64Margin {
            ulps: 2,
            epsilon: 1e-6
        }
    ));
    assert!(space.normalize_value(-PI).approx_eq(
        PI,
        F64Margin {
            ulps: 2,
            epsilon: 1e-6
        }
    ));
    assert!(space.normalize_value(3.0 * PI).approx_eq(
        PI,
        F64Margin {
            ulps: 2,
            epsilon: 1e-6
        }
    ));
    assert!(space.normalize_value(-3.0 * PI).approx_eq(
        PI,
        F64Margin {
            ulps: 2,
            epsilon: 1e-6
        }
    ));

    assert!(space.normalize_value(0.25 * PI).approx_eq(
        0.25 * PI,
        F64Margin {
            ulps: 2,
            epsilon: 1e-6
        }
    ));
    assert!(space.normalize_value(-0.25 * PI).approx_eq(
        1.75 * PI,
        F64Margin {
            ulps: 2,
            epsilon: 1e-6
        }
    ));

    assert!(space.normalize_value(0.75 * PI).approx_eq(
        0.75 * PI,
        F64Margin {
            ulps: 2,
            epsilon: 1e-6
        }
    ));
    assert!(space.normalize_value(-0.75 * PI).approx_eq(
        1.25 * PI,
        F64Margin {
            ulps: 2,
            epsilon: 1e-6
        }
    ));

    assert!(space.normalize_value(-1.5 * PI).approx_eq(
        0.5 * PI,
        F64Margin {
            ulps: 2,
            epsilon: 1e-6
        }
    ));
    assert!(space.normalize_value(1.5 * PI).approx_eq(
        1.5 * PI,
        F64Margin {
            ulps: 2,
            epsilon: 1e-6
        }
    ));
}

#[test]
fn test_periodic_bounded_circular_space_smallest_distance_between_values_zero_to_two_pi() {
    let space = PeriodicBoundedCircularSpace::new_with_two_pi_range(0.0);
    assert!(space.smallest_distance_between_values(0.0, 0.0).approx_eq(
        0.0,
        F64Margin {
            ulps: 2,
            epsilon: 1e-6
        }
    ));
    assert!(space.smallest_distance_between_values(0.0, PI).approx_eq(
        PI,
        F64Margin {
            ulps: 2,
            epsilon: 1e-6
        }
    ));
    assert!(space.smallest_distance_between_values(PI, 0.0).approx_eq(
        PI,
        F64Margin {
            ulps: 2,
            epsilon: 1e-6
        }
    ));
    assert!(space.smallest_distance_between_values(-PI, PI).approx_eq(
        0.0,
        F64Margin {
            ulps: 2,
            epsilon: 1e-6
        }
    ));
    assert!(space.smallest_distance_between_values(PI, -PI).approx_eq(
        0.0,
        F64Margin {
            ulps: 2,
            epsilon: 1e-6
        }
    ));

    assert!(space
        .smallest_distance_between_values(0.0, 2.0 * PI)
        .approx_eq(
            0.0,
            F64Margin {
                ulps: 2,
                epsilon: 1e-6
            }
        ));
    assert!(space
        .smallest_distance_between_values(0.0, 4.0 * PI)
        .approx_eq(
            0.0,
            F64Margin {
                ulps: 2,
                epsilon: 1e-6
            }
        ));

    assert!(space
        .smallest_distance_between_values(PI, 3.0 * PI)
        .approx_eq(
            0.0,
            F64Margin {
                ulps: 2,
                epsilon: 1e-6
            }
        ));
    assert!(space
        .smallest_distance_between_values(11.0 * PI, 20.0 * PI)
        .approx_eq(
            -PI,
            F64Margin {
                ulps: 2,
                epsilon: 1e-6
            }
        ));

    assert!(space
        .smallest_distance_between_values(0.25 * PI, 0.5 * PI)
        .approx_eq(
            0.25 * PI,
            F64Margin {
                ulps: 2,
                epsilon: 1e-6
            }
        ));

    assert!(space
        .smallest_distance_between_values(1.0 * PI, 1.5 * PI)
        .approx_eq(
            0.5 * PI,
            F64Margin {
                ulps: 2,
                epsilon: 1e-6
            }
        ));
    assert!(space
        .smallest_distance_between_values(0.75 * PI, 1.25 * PI)
        .approx_eq(
            0.5 * PI,
            F64Margin {
                ulps: 2,
                epsilon: 1e-6
            }
        ));

    assert!(space
        .smallest_distance_between_values(1.5 * PI, 1.0 * PI)
        .approx_eq(
            -0.5 * PI,
            F64Margin {
                ulps: 2,
                epsilon: 1e-6
            }
        ));
    assert!(space
        .smallest_distance_between_values(1.25 * PI, 0.75 * PI)
        .approx_eq(
            -0.5 * PI,
            F64Margin {
                ulps: 2,
                epsilon: 1e-6
            }
        ));
}
