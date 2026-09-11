use std::{error::Error, fmt};

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
    let negative = (left < 0) ^ (right < 0);
    let magnitude = mul_div_round_u128(
        left.unsigned_abs(),
        right.unsigned_abs(),
        denominator as u128,
    )?;
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
            let value = u128::from(result[index])
                + u128::from(left_limb) * u128::from(right_limb)
                + carry;
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

fn div_wide_u128(
    numerator: [u64; 4],
    denominator: u128,
) -> Result<(u128, u128), WideRatioError> {
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

#[cfg(test)]
mod tests {
    use super::{mul_div_round_i128, mul_div_round_u128};

    #[test]
    fn multiplication_can_exceed_u128_before_division() {
        let left = u128::MAX / 3;
        let right = 9_u128;
        assert_eq!(mul_div_round_u128(left, right, 9), Ok(left));
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
}
