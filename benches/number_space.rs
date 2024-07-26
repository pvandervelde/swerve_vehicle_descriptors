use std::f64::consts::PI;

use criterion::{black_box, criterion_group, criterion_main, Criterion};
use swerve_vehicle_descriptors::number_space::{to_number_space, NumberSpaceType};

criterion_group! {
    name = benches;
    config = Criterion::default();
    targets =
        linear_unbounded_distance_between_values,
        linear_unbounded_normalize_value,
        linear_unbounded_smallest_distance_between_values,
        periodic_zero_to_two_pi_distance_between_values,
        periodic_zero_to_two_pi_normalize_value,
        periodic_zero_to_two_pi_smallest_distance_between_values,
        periodic_minus_pi_to_pi_distance_between_values,
        periodic_minus_pi_to_pi_normalize_value,
        periodic_minus_pi_to_pi_smallest_distance_between_values,
}

criterion_main!(benches);

pub fn linear_unbounded_distance_between_values(c: &mut Criterion) {
    let number_space = to_number_space(NumberSpaceType::LinearUnlimited);

    c.bench_function("LinearUnlimited::distance_between_values", |b| {
        b.iter(|| number_space.distance_between_values(black_box(10.0), black_box(20.0)))
    });
}

pub fn linear_unbounded_normalize_value(c: &mut Criterion) {
    let number_space = to_number_space(NumberSpaceType::LinearUnlimited);

    c.bench_function("LinearUnlimited::normalize_value", |b| {
        b.iter(|| number_space.normalize_value(black_box(10.0)))
    });
}

pub fn linear_unbounded_smallest_distance_between_values(c: &mut Criterion) {
    let number_space = to_number_space(NumberSpaceType::LinearUnlimited);

    c.bench_function("LinearUnlimited::smallest_distance_between_values", |b| {
        b.iter(|| number_space.smallest_distance_between_values(black_box(10.0), black_box(20.0)))
    });
}

pub fn periodic_zero_to_two_pi_distance_between_values(c: &mut Criterion) {
    let number_space = to_number_space(NumberSpaceType::AngularLimited {
        start_angle_in_radians: 0.0,
    });

    c.bench_function("AngularLimited::<0.0, 2PI>::distance_between_values", |b| {
        b.iter(|| number_space.distance_between_values(black_box(0.25 * PI), black_box(1.25 * PI)))
    });
}

pub fn periodic_zero_to_two_pi_normalize_value(c: &mut Criterion) {
    let number_space = to_number_space(NumberSpaceType::AngularLimited {
        start_angle_in_radians: 0.0,
    });

    c.bench_function("AngularLimited::<0.0, 2PI>::normalize_value", |b| {
        b.iter(|| number_space.normalize_value(black_box(7.75 * PI)))
    });
}

pub fn periodic_zero_to_two_pi_smallest_distance_between_values(c: &mut Criterion) {
    let number_space = to_number_space(NumberSpaceType::AngularLimited {
        start_angle_in_radians: 0.0,
    });

    c.bench_function(
        "AngularLimited::<0.0, 2PI>::smallest_distance_between_values",
        |b| {
            b.iter(|| {
                number_space
                    .smallest_distance_between_values(black_box(0.25 * PI), black_box(1.25 * PI))
            })
        },
    );
}

pub fn periodic_minus_pi_to_pi_distance_between_values(c: &mut Criterion) {
    let number_space = to_number_space(NumberSpaceType::AngularLimited {
        start_angle_in_radians: 0.0,
    });

    c.bench_function("AngularLimited::<-PI, 1PI>::distance_between_values", |b| {
        b.iter(|| number_space.distance_between_values(black_box(0.75 * PI), black_box(1.25 * PI)))
    });
}

pub fn periodic_minus_pi_to_pi_normalize_value(c: &mut Criterion) {
    let number_space = to_number_space(NumberSpaceType::AngularLimited {
        start_angle_in_radians: 0.0,
    });

    c.bench_function("AngularLimited::<-PI, 1PI>::normalize_value", |b| {
        b.iter(|| number_space.normalize_value(black_box(7.75 * PI)))
    });
}

pub fn periodic_minus_pi_to_pi_smallest_distance_between_values(c: &mut Criterion) {
    let number_space = to_number_space(NumberSpaceType::AngularLimited {
        start_angle_in_radians: 0.0,
    });

    c.bench_function(
        "AngularLimited::<-PI, 1PI>::smallest_distance_between_values",
        |b| {
            b.iter(|| {
                number_space
                    .smallest_distance_between_values(black_box(0.75 * PI), black_box(1.25 * PI))
            })
        },
    );
}
