use std::{cmp::Ordering, error::Error, fmt};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum WideRatioError {
    ZeroDenominator,
    QuotientOverflow,
}

impl fmt::Display for WideRatioError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::ZeroDenominator => write!(formatter, "wide ratio denominator must be non-zero"),
            Self::QuotientOverflow => write!(formatter, "wide ratio quotient overflowed u128"),
        }
    }
}

impl Error for WideRatioError {}

/// Multiplies two `u128` values in 256-bit space, divides by a `u128`, and rounds upward.
pub(crate) fn mul_div_ceil_u128(
    left: u128,
    right: u128,
    denominator: u128,
) -> Result<u128, WideRatioError> {
    let (mut quotient, remainder) = div_wide_u128(mul_wide_u128(left, right), denominator)?;
    if remainder != 0 {
        quotient = quotient
            .checked_add(1)
            .ok_or(WideRatioError::QuotientOverflow)?;
    }
    Ok(quotient)
}

/// Multiplies two `u128` values in 256-bit space, divides by a `u128`, and rounds half up.
pub(crate) fn mul_div_round_u128(
    left: u128,
    right: u128,
    denominator: u128,
) -> Result<u128, WideRatioError> {
    let (mut quotient, remainder) = div_wide_u128(mul_wide_u128(left, right), denominator)?;
    let threshold = denominator / 2 + denominator % 2;
    if remainder >= threshold && remainder != 0 {
        quotient = quotient
            .checked_add(1)
            .ok_or(WideRatioError::QuotientOverflow)?;
    }
    Ok(quotient)
}

/// Signed counterpart to [`mul_div_round_u128`]. Rounding is symmetric around zero.
pub(crate) fn mul_div_round_i128(
    left: i128,
    right: i128,
    denominator: i128,
) -> Result<i128, WideRatioError> {
    if denominator <= 0 {
        return Err(if denominator == 0 {
            WideRatioError::ZeroDenominator
        } else {
            WideRatioError::QuotientOverflow
        });
    }
    signed_quotient(
        left,
        right,
        mul_div_round_u128(
            left.unsigned_abs(),
            right.unsigned_abs(),
            denominator as u128,
        )?,
    )
}

/// Multiplies two signed `i128` numerators and divides by the exact product of two positive `i128`
/// denominator factors in 256-bit space.
///
/// This avoids materializing a denominator product in `i128`. It is used by exact rational integration
/// when the reduced physical timestep is representable, but multiplying that denominator by a fixed unit
/// scale would exceed 128 bits. Rounding remains symmetric around zero and half away from zero.
pub(crate) fn mul_div_round_i128_wide_denominator(
    left: i128,
    right: i128,
    denominator_left: i128,
    denominator_right: i128,
) -> Result<i128, WideRatioError> {
    if denominator_left <= 0 || denominator_right <= 0 {
        return Err(if denominator_left == 0 || denominator_right == 0 {
            WideRatioError::ZeroDenominator
        } else {
            WideRatioError::QuotientOverflow
        });
    }

    let numerator = mul_wide_u128(left.unsigned_abs(), right.unsigned_abs());
    let denominator = mul_wide_u128(denominator_left as u128, denominator_right as u128);
    let (mut quotient, remainder) = div_wide_u256(numerator, denominator)?;
    let threshold = half_ceil_wide(denominator);
    if !is_zero_wide(remainder) && cmp_wide(remainder, threshold) != Ordering::Less {
        quotient = quotient
            .checked_add(1)
            .ok_or(WideRatioError::QuotientOverflow)?;
    }
    signed_quotient(left, right, quotient)
}

fn signed_quotient(left: i128, right: i128, magnitude: u128) -> Result<i128, WideRatioError> {
    let negative = (left < 0) ^ (right < 0);
    if negative {
        if magnitude == (1_u128 << 127) {
            return Ok(i128::MIN);
        }
        let value = i128::try_from(magnitude).map_err(|_| WideRatioError::QuotientOverflow)?;
        value.checked_neg().ok_or(WideRatioError::QuotientOverflow)
    } else {
        i128::try_from(magnitude).map_err(|_| WideRatioError::QuotientOverflow)
    }
}

fn mul_wide_u128(left: u128, right: u128) -> [u64; 4] {
    let left = [left as u64, (left >> 64) as u64];
    let right = [right as u64, (right >> 64) as u64];
    let mut result = [0_u64; 4];

    for (left_index, left_limb) in left.into_iter().enumerate() {
        let mut carry = 0_u128;
        for (right_index, right_limb) in right.into_iter().enumerate() {
            let index = left_index + right_index;
            let value =
                u128::from(result[index]) + u128::from(left_limb) * u128::from(right_limb) + carry;
            result[index] = value as u64;
            carry = value >> 64;
        }
        let mut index = left_index + 2;
        while carry != 0 {
            debug_assert!(index < result.len());
            let value = u128::from(result[index]) + carry;
            result[index] = value as u64;
            carry = value >> 64;
            index += 1;
        }
    }

    result
}

fn div_wide_u128(numerator: [u64; 4], denominator: u128) -> Result<(u128, u128), WideRatioError> {
    if denominator == 0 {
        return Err(WideRatioError::ZeroDenominator);
    }
    let mut quotient = 0_u128;
    let mut remainder = 0_u128;

    for bit_index in (0_u32..256).rev() {
        let limb = usize::try_from(bit_index / 64).expect("wide bit index fits usize");
        let bit_in_limb = bit_index % 64;
        let incoming = (numerator[limb] >> bit_in_limb) & 1;
        let carry = remainder >> 127 != 0;
        let shifted = (remainder << 1) | u128::from(incoming);
        if carry || shifted >= denominator {
            remainder = shifted.wrapping_sub(denominator);
            if bit_index >= 128 {
                return Err(WideRatioError::QuotientOverflow);
            }
            quotient |= 1_u128 << bit_index;
        } else {
            remainder = shifted;
        }
    }

    Ok((quotient, remainder))
}

fn div_wide_u256(
    numerator: [u64; 4],
    denominator: [u64; 4],
) -> Result<(u128, [u64; 4]), WideRatioError> {
    if is_zero_wide(denominator) {
        return Err(WideRatioError::ZeroDenominator);
    }
    let mut quotient = 0_u128;
    let mut remainder = [0_u64; 4];

    for bit_index in (0_u32..256).rev() {
        let limb = usize::try_from(bit_index / 64).expect("wide bit index fits usize");
        let incoming = (numerator[limb] >> (bit_index % 64)) & 1;
        shift_left_one(&mut remainder, incoming);
        if cmp_wide(remainder, denominator) != Ordering::Less {
            remainder = sub_wide(remainder, denominator);
            if bit_index >= 128 {
                return Err(WideRatioError::QuotientOverflow);
            }
            quotient |= 1_u128 << bit_index;
        }
    }

    Ok((quotient, remainder))
}

fn shift_left_one(value: &mut [u64; 4], incoming: u64) {
    let mut carry = incoming;
    for limb in value.iter_mut() {
        let next_carry = *limb >> 63;
        *limb = (*limb << 1) | carry;
        carry = next_carry;
    }
    debug_assert_eq!(carry, 0, "wide division remainder exceeded 256 bits");
}

fn sub_wide(left: [u64; 4], right: [u64; 4]) -> [u64; 4] {
    debug_assert!(cmp_wide(left, right) != Ordering::Less);
    let mut result = [0_u64; 4];
    let mut borrow = false;
    for index in 0..4 {
        let (first, first_borrow) = left[index].overflowing_sub(right[index]);
        let (value, second_borrow) = first.overflowing_sub(u64::from(borrow));
        result[index] = value;
        borrow = first_borrow || second_borrow;
    }
    debug_assert!(!borrow);
    result
}

fn cmp_wide(left: [u64; 4], right: [u64; 4]) -> Ordering {
    for index in (0..4).rev() {
        match left[index].cmp(&right[index]) {
            Ordering::Equal => {}
            ordering => return ordering,
        }
    }
    Ordering::Equal
}

fn half_ceil_wide(value: [u64; 4]) -> [u64; 4] {
    let round_up = value[0] & 1 != 0;
    let mut half = [0_u64; 4];
    let mut carry = 0_u64;
    for index in (0..4).rev() {
        half[index] = (value[index] >> 1) | (carry << 63);
        carry = value[index] & 1;
    }
    if round_up {
        add_one_wide(&mut half);
    }
    half
}

fn add_one_wide(value: &mut [u64; 4]) {
    for limb in value.iter_mut() {
        let (next, carry) = limb.overflowing_add(1);
        *limb = next;
        if !carry {
            return;
        }
    }
}

fn is_zero_wide(value: [u64; 4]) -> bool {
    value == [0; 4]
}

#[cfg(test)]
mod tests {
    use super::{
        mul_div_ceil_u128, mul_div_round_i128, mul_div_round_i128_wide_denominator,
        mul_div_round_u128,
    };

    #[test]
    fn multiplication_can_exceed_u128_before_division() {
        let left = u128::MAX / 3;
        let right = 9_u128;
        assert_eq!(mul_div_round_u128(left, right, 9), Ok(left));
        assert_eq!(mul_div_ceil_u128(left, right, 9), Ok(left));
    }

    #[test]
    fn wide_ceil_rounds_fraction_up() {
        assert_eq!(mul_div_ceil_u128(2, 2, 3), Ok(2));
    }

    #[test]
    fn signed_rounding_is_symmetric() {
        assert_eq!(mul_div_round_i128(2, 2, 5), Ok(1));
        assert_eq!(mul_div_round_i128(-2, 2, 5), Ok(-1));
    }

    #[test]
    fn half_values_round_away_from_zero() {
        assert_eq!(mul_div_round_i128(1, 1, 2), Ok(1));
        assert_eq!(mul_div_round_i128(-1, 1, 2), Ok(-1));
    }

    #[test]
    fn denominator_product_can_exceed_i128_without_losing_exact_quotient() {
        let huge = i128::MAX / 2 + 1;
        assert_eq!(mul_div_round_i128_wide_denominator(6, huge, 2, huge), Ok(3));
        assert_eq!(
            mul_div_round_i128_wide_denominator(-6, huge, 2, huge),
            Ok(-3)
        );
    }
}
