use std::f64::consts::PI;

#[cfg(test)]
#[path = "number_space_tests.rs"]
mod number_space_tests;

/// Defines the different kinds of number spaces available.
pub enum NumberSpaceType {
    /// Indicates that a number space is a linear number space where numbers sequentially
    /// increase from -infinity to +infinity.
    LinearUnlimited,

    /// Indicates that a number space is an angular number space where numbers sequentially
    /// increase from the start angle to the start angle + 2 PI.
    AngularLimited { start_angle_in_radians: f64 },
}

/// Defines an abstraction over number spaces
pub(crate) trait RealNumberValueSpace {
    /// Returns all possible distances between two values in the space.
    ///
    /// For unbounded value spaces there will only be one distance, but for bounded value spaces
    /// there may be multiple distances depending on if the boundaries are periodic or not.
    /// For example, the distance between 0 and 2pi in a circular space running from -pi to +pi is 2pi,
    /// but the distance between 0 and 4pi is 2pi as well. Similarly the distance between 1/4 pi and
    /// -1/4 pi is 1/2 pi, and 3/2 pi depending on the direction of travel.
    ///
    /// * 'start' - The starting value.
    /// * 'end' - The ending value
    fn distance_between_values(&self, start: f64, end: f64) -> Vec<f64>;

    /// Returns the value in the space that is closest to the target value
    ///
    /// Normalizing the value is useful in periodic or limited number spaces.
    ///
    /// * 'value' - The value that should be normalized.
    fn normalize_value(&self, value: f64) -> f64;

    /// Returns the smallest distance between two values in the number space.
    ///
    /// The smallest distance for unlimited number spaces is equal to the distance.
    /// between the numbers. However for a periodic number space the distance across
    /// a boundary may be shorter.
    fn smallest_distance_between_values(&self, start: f64, end: f64) -> f64;
}

/// Defines a linear unbounded number space with no boundaries
///
/// The linear unbounded number space is what we normally think of as a set
/// of numbers, ranging from -infinity to +infinity.
pub(crate) struct LinearUnboundedSpace {}

impl LinearUnboundedSpace {
    pub fn new() -> LinearUnboundedSpace {
        LinearUnboundedSpace {}
    }
}

impl RealNumberValueSpace for LinearUnboundedSpace {
    fn distance_between_values(&self, start: f64, end: f64) -> Vec<f64> {
        vec![end - start]
    }

    fn normalize_value(&self, value: f64) -> f64 {
        value
    }

    fn smallest_distance_between_values(&self, start: f64, end: f64) -> f64 {
        end - start
    }
}

/// Defines a periodic number space that wraps around at the period.
///
/// The periodic number space is used for calculations of numbers in circular
/// cases.
pub(crate) struct PeriodicBoundedCircularSpace {
    range_start_in_radians: f64,
    range_end_in_radians: f64,
    range_size: f64,
}

impl PeriodicBoundedCircularSpace {
    pub fn new_with_two_pi_range(start_angle_in_radians: f64) -> PeriodicBoundedCircularSpace {
        PeriodicBoundedCircularSpace {
            range_start_in_radians: start_angle_in_radians,
            range_end_in_radians: start_angle_in_radians + 2.0 * PI,
            range_size: 2.0 * PI,
        }
    }
}

impl RealNumberValueSpace for PeriodicBoundedCircularSpace {
    fn distance_between_values(&self, start: f64, end: f64) -> Vec<f64> {
        let normalized_start = self.normalize_value(start);
        let normalized_end = self.normalize_value(end);

        let mut diff = normalized_end - normalized_start;

        // Bring the range back to the limits of the range
        diff = if diff >= self.range_end_in_radians {
            diff - self.range_size
        } else {
            if diff < self.range_start_in_radians {
                diff + self.range_size
            } else {
                diff
            }
        };

        if diff >= 0.0 {
            vec![diff, diff - self.range_size]
        } else {
            vec![diff + self.range_size, diff]
        }
    }

    fn normalize_value(&self, value: f64) -> f64 {
        // reduce the angle to be no larger than the range
        let normalized_value = value % self.range_size;

        if self.range_start_in_radians != 0.0 {
            normalized_value - 0.5 * self.range_size
        } else {
            normalized_value
        }
    }

    fn smallest_distance_between_values(&self, start: f64, end: f64) -> f64 {
        let normalized_start = self.normalize_value(start);
        let normalized_end = self.normalize_value(end);

        let mut diff = normalized_end - normalized_start;

        // Bring the range back to the limits of the range
        diff = if diff >= self.range_end_in_radians {
            diff - self.range_size
        } else {
            if diff < self.range_start_in_radians {
                diff + self.range_size
            } else {
                diff
            }
        };

        if diff >= 0.5 * self.range_size {
            diff - self.range_size
        } else {
            diff
        }
    }
}

/// Returns a [RealNumberValueSpace] instance for the given number space type.
pub(crate) fn to_number_space(number_space_type: NumberSpaceType) -> Box<dyn RealNumberValueSpace> {
    match number_space_type {
        NumberSpaceType::LinearUnlimited => Box::new(LinearUnboundedSpace::new()),
        NumberSpaceType::AngularLimited {
            start_angle_in_radians,
        } => Box::new(PeriodicBoundedCircularSpace::new_with_two_pi_range(
            start_angle_in_radians,
        )),
    }
}
