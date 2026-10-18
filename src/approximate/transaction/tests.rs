use super::*;
use crate::approximate::Shape;
#[test]
fn stamp_epoch_wrap_still_captures_the_first_before_image() {
    let body = Body::fixture(7, Shape::Sphere(1.0), Vector::ZERO, 1.0);
    let mut journal = Journal {
        epoch: u64::MAX,
        stamps: vec![1],
        ..Journal::default()
    };
    journal.begin(1);
    journal.body(0, &body);
    journal.body(0, &body);
    assert_eq!(journal.stats(false).journaled_bodies, 1);
}
