//! Constant-size floating-point arithmetic for the legacy quantized state boundary.
//!
//! Time composition and multiply/divide calculations use f64, never a multi-limb fallback.
//! Outputs are rounded only where the existing integer API requires it. This is not an exact-ratio
//! emulation: equivalent calculation paths may differ within floating-point precision.

use std::{error::Error, fmt};

use crate::numeric::Scalar;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum ArithmeticError {
    ZeroDenominator,
    NonFinite,
    OutOfRange,
    Underflow,
}

impl fmt::Display for ArithmeticError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str(match self {
            Self::ZeroDenominator => "numerical denominator must be non-zero",
            Self::NonFinite => "numerical calculation produced a non-finite value",
            Self::OutOfRange => "numerical result is outside the destination range",
            Self::Underflow => "positive time scale underflowed to zero",
        })
    }
}

impl Error for ArithmeticError {}

/// A finite non-negative f64 scale. Private construction excludes NaN, infinities and negative zero.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Ratio(Scalar);

// All inhabitants are finite, so PartialEq is reflexive. Do not introduce unchecked construction.
impl Eq for Ratio {}

impl Ratio {
    pub(crate) fn new(numerator: u128, denominator: u128) -> Result<Self, ArithmeticError> {
        if denominator == 0 {
            return Err(ArithmeticError::ZeroDenominator);
        }
        Self::from_scalar(numerator as Scalar / denominator as Scalar)
    }

    pub(crate) fn from_scalar(value: Scalar) -> Result<Self, ArithmeticError> {
        if !value.is_finite() {
            return Err(ArithmeticError::NonFinite);
        }
        if value < 0.0 {
            return Err(ArithmeticError::OutOfRange);
        }
        Ok(Self(if value == 0.0 { 0.0 } else { value }))
    }

    #[must_use]
    pub(crate) fn is_zero(self) -> bool {
        self.0 == 0.0
    }

    #[cfg(test)]
    pub(crate) fn value(self) -> Scalar {
        self.0
    }

    pub(crate) fn scaled_u32(
        self,
        numerator: u32,
        denominator: u32,
    ) -> Result<Self, ArithmeticError> {
        if denominator == 0 {
            return Err(ArithmeticError::ZeroDenominator);
        }
        if numerator == 0 || self.is_zero() {
            return Ok(Self(0.0));
        }
        if numerator == denominator {
            return Ok(self);
        }
        let scaled = self.0 * (Scalar::from(numerator) / Scalar::from(denominator));
        if scaled == 0.0 {
            return Err(ArithmeticError::Underflow);
        }
        Self::from_scalar(scaled)
    }

    /// Rounds at the integer compatibility boundary, with halfway cases away from zero.
    pub(crate) fn mul_round_u128(self, value: u128) -> Result<u128, ArithmeticError> {
        if self.0 == 1.0 {
            return Ok(value);
        }
        checked_u128((value as Scalar * self.0).round())
    }

    pub(crate) fn mul_round_i128(self, value: i128) -> Result<i128, ArithmeticError> {
        if self.0 == 1.0 {
            return Ok(value);
        }
        checked_i128((value as Scalar * self.0).round())
    }

    /// Outward-rounded bound for broad-phase/safety calculations, not a position correction.
    /// One ULP of outward expansion prevents a rounded-down product from shrinking a sweep.
    pub(crate) fn mul_ceil_u128(self, value: u128) -> Result<u128, ArithmeticError> {
        if self.is_zero() || value == 0 {
            return Ok(0);
        }
        if self.0 == 1.0 {
            return Ok(value);
        }
        let upper_value = (value as Scalar).next_up();
        checked_u128((upper_value * self.0).next_up().ceil())
    }
}

// Float-to-integer casts saturate in Rust. Check the *exclusive* positive endpoint first instead
// of silently converting infinities or rounded-up integer maxima into valid-looking state.
fn checked_u128(value: Scalar) -> Result<u128, ArithmeticError> {
    if !value.is_finite() {
        return Err(ArithmeticError::NonFinite);
    }
    if !(0.0..340_282_366_920_938_463_463_374_607_431_768_211_456.0).contains(&value) {
        return Err(ArithmeticError::OutOfRange);
    }
    Ok(value as u128)
}

fn checked_i128(value: Scalar) -> Result<i128, ArithmeticError> {
    if !value.is_finite() {
        return Err(ArithmeticError::NonFinite);
    }
    const LIMIT: Scalar = 170_141_183_460_469_231_731_687_303_715_884_105_728.0;
    if !(-LIMIT..LIMIT).contains(&value) {
        return Err(ArithmeticError::OutOfRange);
    }
    Ok(value as i128)
}

pub(crate) fn mul_div_round_u128(
    left: u128,
    right: u128,
    denominator: u128,
) -> Result<u128, ArithmeticError> {
    Ratio::new(right, denominator)?.mul_round_u128(left)
}

pub(crate) fn mul_div_round_i128(
    left: i128,
    right: i128,
    denominator: i128,
) -> Result<i128, ArithmeticError> {
    if denominator <= 0 {
        return Err(if denominator == 0 {
            ArithmeticError::ZeroDenominator
        } else {
            ArithmeticError::OutOfRange
        });
    }
    let scale = right as Scalar / denominator as Scalar;
    checked_i128((left as Scalar * scale).round())
}

#[cfg(test)]
mod tests {
    use super::{ArithmeticError, Ratio, checked_i128, checked_u128, mul_div_round_i128};

    #[test]
    fn time_storage_is_one_float_not_two_multilimb_integers() {
        assert_eq!(size_of::<Ratio>(), size_of::<f64>());
    }

    #[test]
    fn invalid_numbers_and_denominators_are_rejected() {
        for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert_eq!(Ratio::from_scalar(value), Err(ArithmeticError::NonFinite));
        }
        assert_eq!(Ratio::from_scalar(-1.0), Err(ArithmeticError::OutOfRange));
        assert_eq!(Ratio::new(0, 0), Err(ArithmeticError::ZeroDenominator));
        let ratio = Ratio::new(0, 1).unwrap();
        assert_eq!(
            ratio.scaled_u32(0, 0),
            Err(ArithmeticError::ZeroDenominator)
        );
        assert_eq!(Ratio::from_scalar(-0.0).unwrap().value().to_bits(), 0);
    }

    #[test]
    fn composition_is_bounded_and_repeatable_without_growing_denominators() {
        let mut first = Ratio::new(1, 60).unwrap();
        let mut second = first;
        for _ in 0..64 {
            first = first.scaled_u32(511, 512).unwrap();
            second = second.scaled_u32(511, 512).unwrap();
        }
        assert_eq!(first, second);
        let expected = (1.0 / 60.0) * (511.0_f64 / 512.0).powi(64);
        assert!((first.value() - expected).abs() <= expected * 64.0 * f64::EPSILON);
        assert_eq!(first.scaled_u32(0, 1).unwrap().value(), 0.0);
        assert_eq!(first.scaled_u32(512, 512).unwrap(), first);
    }

    #[test]
    fn scalar_range_failures_do_not_silently_end_a_step() {
        assert_eq!(
            Ratio::from_scalar(f64::from_bits(1))
                .unwrap()
                .scaled_u32(1, 2),
            Err(ArithmeticError::Underflow)
        );
        assert_eq!(
            Ratio::from_scalar(f64::MAX).unwrap().scaled_u32(2, 1),
            Err(ArithmeticError::NonFinite)
        );
    }

    #[test]
    fn legacy_integer_rounding_remains_symmetric() {
        let half = Ratio::new(1, 2).unwrap();
        for value in -101_i128..=101 {
            assert_eq!(
                half.mul_round_i128(value).unwrap(),
                (value as f64 * 0.5).round() as i128
            );
        }
        assert_eq!(mul_div_round_i128(-3, -1, 2), Ok(2));
        assert_eq!(mul_div_round_i128(3, -1, 2), Ok(-2));
    }

    #[test]
    fn output_range_checks_reject_saturating_casts() {
        let unsigned_limit = 2.0_f64.powi(128);
        let signed_limit = 2.0_f64.powi(127);
        assert_eq!(
            checked_u128(unsigned_limit),
            Err(ArithmeticError::OutOfRange)
        );
        assert_eq!(checked_i128(signed_limit), Err(ArithmeticError::OutOfRange));
        assert_eq!(checked_i128(-signed_limit), Ok(i128::MIN));
        assert_eq!(checked_u128(f64::NAN), Err(ArithmeticError::NonFinite));
        assert_eq!(checked_i128(f64::INFINITY), Err(ArithmeticError::NonFinite));
        assert!(checked_u128(unsigned_limit.next_down()).is_ok());
        assert!(checked_i128(signed_limit.next_down()).is_ok());
    }

    #[test]
    fn sweep_bounds_round_outward_and_preserve_zero() {
        for denominator in [3_u128, 60, 512, u32::MAX.into()] {
            let ratio = Ratio::new(1, denominator).unwrap();
            for value in [0_u128, 1, 60, 3600, u64::MAX.into()] {
                let ceiling = ratio.mul_ceil_u128(value).unwrap();
                assert!(ceiling >= value.div_ceil(denominator));
                assert!(ceiling >= ratio.mul_round_u128(value).unwrap());
            }
        }
        assert_eq!(Ratio::new(1, 60).unwrap().mul_ceil_u128(0), Ok(0));
    }
    #[test]
    #[ignore = "advisory numerical-backend microbenchmark; run explicitly in release mode"]
    fn floating_scale_microbenchmark() {
        use std::{hint::black_box, time::Instant};
        let ratio = Ratio::new(511, 512).unwrap();
        let start = Instant::now();
        let mut checksum = 0_i128;
        for value in 0..1_000_000_i128 {
            checksum += black_box(ratio).mul_round_i128(black_box(value)).unwrap();
        }
        println!(
            "float64 scaled arithmetic: operations=1000000 ratio_bytes={} elapsed_ns={} checksum={checksum}",
            size_of::<Ratio>(),
            start.elapsed().as_nanos()
        );
    }
}
