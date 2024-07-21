//! Defines different a way to describe a space of numbers and how these spaces behave at the
//! boundaries.
//!
//! For instance a linear unbounded space has boundaries at +infinity and -infinity. This
//! type of space does not wrap around, i.e. at they only way to get from the lower boundary to the
//! upper boundary is to pass through all the numbers between the boundaries.
//! On the contrairy to this a periodic number space has lower and upper boundaries at specific
//! non-infinity numbers and wraps around, i.e. in order to go from the lower boundary to the upper
//! boundary you can pass through all the numbers between the lower and upper boundary, or you can
//! go backwards from lower boundary and end up directly at the upper boundary. An example of this
//! kind of space is a space that describes the position on a circle.
//!
//! Currently implemented are a linear unbounded space and an angular bounded space. The
//! [to_number_space()] function is used to create either of these number spaces. For the angular
//! space you can specify the starting angle in radians by creating the [NumberSpaceType::AngularLimited]
//! value with the given starting angle. The [to_number_space()] function assumes that the
//! angular number space is 2 * [Pi](core::f64::consts::PI) in size.
//!

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
    AngularLimited {
        /// The starting angle in radians
        start_angle_in_radians: f64,
    },
}

/// Defines an abstraction over number spaces
pub trait RealNumberValueSpace {
    /// Returns all possible distances between two values in the space.
    ///
    /// For unbounded value spaces there will only be one distance, but for bounded value spaces
    /// there may be multiple distances depending on if the boundaries are periodic or not.
    ///
    /// ## Parameters
    ///
    /// * `start` - The starting value
    /// * `end` - The ending value
    ///
    /// ## Example
    ///
    /// ```
    /// use core::f64::consts::PI;
    /// use swerve_vehicle_descriptors::number_space::{ NumberSpaceType, to_number_space };
    ///
    /// // Create a linear space
    /// let space = to_number_space(NumberSpaceType::LinearUnlimited);
    /// let linear_distances = space.distance_between_values(1.0, 2.0);
    /// assert!(linear_distances.len() == 1);
    /// assert_eq!(1.0, linear_distances[0]);
    ///
    /// // Create a periodic space that starts at 0.0 and runs to 2 * PI
    /// let space = to_number_space(NumberSpaceType::AngularLimited { start_angle_in_radians: 0.0 });
    /// let angular_distances = space.distance_between_values(0.0, PI);
    /// assert!(angular_distances.len() == 2);
    /// assert_eq!(PI, angular_distances[0]);
    /// assert_eq!(-PI, angular_distances[1]);
    /// ```
    fn distance_between_values(&self, start: f64, end: f64) -> Vec<f64>;

    /// Returns the value in the space that is closest to the target value
    ///
    /// Normalizing the value is useful in periodic or limited number spaces.
    ///
    /// ## Parameters
    ///
    /// * `value` - The value that should be normalized.
    ///
    /// ## Example
    ///
    /// ```
    /// use core::f64::consts::PI;
    /// use swerve_vehicle_descriptors::number_space::{ NumberSpaceType, to_number_space };
    ///
    /// // Create a linear space
    /// let space = to_number_space(NumberSpaceType::LinearUnlimited);
    /// let value = space.normalize_value(1.0);
    /// assert_eq!(1.0, value);
    ///
    /// // Create a periodic space that starts at 0.0 and runs to 2 * PI
    /// let space = to_number_space(NumberSpaceType::AngularLimited { start_angle_in_radians: 0.0 });
    /// let value = space.normalize_value(5.0 * PI);
    /// assert_eq!(PI, value);
    /// ```
    fn normalize_value(&self, value: f64) -> f64;

    /// Returns the smallest distance between two values in the number space.
    ///
    /// The smallest distance for unlimited number spaces is equal to the distance.
    /// between the numbers. However for a periodic number space the distance across
    /// a boundary may be shorter.
    ///
    /// ## Parameters
    ///
    /// * `start` - The starting value.
    /// * `end` - The ending value
    ///
    /// ## Example
    ///
    /// ```
    /// use core::f64::consts::PI;
    /// use swerve_vehicle_descriptors::number_space::{ NumberSpaceType, to_number_space };
    ///
    /// // Create a linear space
    /// let space = to_number_space(NumberSpaceType::LinearUnlimited);
    /// let value = space.smallest_distance_between_values(1.0, 2.0);
    /// assert_eq!(1.0, value);
    ///
    /// // Create a periodic space that starts at 0.0 and runs to 2 * PI
    /// let space = to_number_space(NumberSpaceType::AngularLimited { start_angle_in_radians: 0.0 });
    /// let value = space.smallest_distance_between_values(0.0, 1.5 * PI);
    /// assert_eq!(-0.5 * PI, value);
    /// ```
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
        // reduce the angle to be [-range, range]
        let mut normalized_value = value % self.range_size;

        // reduce the angle to the positive range
        if normalized_value < self.range_start_in_radians {
            normalized_value = (normalized_value + self.range_size) % self.range_size;
        }

        if (self.range_start_in_radians != 0.0) && (normalized_value > self.range_end_in_radians) {
            normalized_value - self.range_size
        } else {
            normalized_value
        }
    }

    fn smallest_distance_between_values(&self, start: f64, end: f64) -> f64 {
        let normalized_start = self.normalize_value(start);
        let normalized_end = self.normalize_value(end);

        let mut diff = normalized_end - normalized_start;

        // Bring the range back to the limits of the range
        diff = if diff > self.range_end_in_radians {
            diff - self.range_size
        } else {
            if diff < self.range_start_in_radians {
                diff + self.range_size
            } else {
                diff
            }
        };

        let abs_diff = diff.abs();
        if abs_diff > 0.5 * self.range_size {
            if diff > 0.0 {
                diff - self.range_size
            } else {
                diff + self.range_size
            }
        } else {
            diff
        }
    }
}

/// Returns a [RealNumberValueSpace] instance for the given number space type.
///
/// ```
/// use core::f64::consts::PI;
/// use swerve_vehicle_descriptors::number_space::{ NumberSpaceType, to_number_space };
///
/// // Create a linear space
/// let space = to_number_space(NumberSpaceType::LinearUnlimited);
/// let linear_distances = space.distance_between_values(1.0, 2.0);
/// assert!(linear_distances.len() == 1);
/// assert_eq!(1.0, linear_distances[0]);
///
/// // Create a periodic space that starts at 0.0 and runs to 2 * PI
/// let space = to_number_space(NumberSpaceType::AngularLimited { start_angle_in_radians: 0.0 });
/// let angular_distances = space.distance_between_values(0.0, PI);
/// assert!(angular_distances.len() == 2);
/// assert_eq!(PI, angular_distances[0]);
/// assert_eq!(-PI, angular_distances[1]);
/// ```
pub fn to_number_space(number_space_type: NumberSpaceType) -> Box<dyn RealNumberValueSpace> {
    match number_space_type {
        NumberSpaceType::LinearUnlimited => Box::new(LinearUnboundedSpace::new()),
        NumberSpaceType::AngularLimited {
            start_angle_in_radians,
        } => Box::new(PeriodicBoundedCircularSpace::new_with_two_pi_range(
            start_angle_in_radians,
        )),
    }
}
