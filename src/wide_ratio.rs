use std::{cmp::Ordering, error::Error, fmt};

const EXACT_LIMBS: usize = 40;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum WideRatioError {
    ZeroDenominator,
    QuotientOverflow,
    CapacityOverflow,
}

impl fmt::Display for WideRatioError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::ZeroDenominator => write!(formatter, "wide ratio denominator must be non-zero"),
            Self::QuotientOverflow => write!(formatter, "wide ratio quotient overflowed u128"),
            Self::CapacityOverflow => {
                write!(formatter, "wide ratio exceeded deterministic limb capacity")
            }
        }
    }
}

impl Error for WideRatioError {}

/// Exact positive rational storage for engine-internal timestep composition.
///
/// Forty 64-bit limbs provide 2,560 bits per side. The repeated-event contract admits at most 64
/// sampled fractions, each with a `u32` numerator/denominator, on top of an `i128` base timestep. The
/// capacity therefore covers the full repeated-event bound plus the bounded tail slice, sampled-search,
/// fixed-unit, and `u128` evaluation factors without allocating or depending on platform bigint code.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct ExactRatio {
    numerator: WideNatural,
    denominator: WideNatural,
}

impl ExactRatio {
    pub(crate) fn new(numerator: u128, denominator: u128) -> Result<Self, WideRatioError> {
        if denominator == 0 {
            return Err(WideRatioError::ZeroDenominator);
        }
        if numerator == 0 {
            return Ok(Self {
                numerator: WideNatural::ZERO,
                denominator: WideNatural::ONE,
            });
        }
        let divisor = greatest_common_divisor_u128(numerator, denominator);
        Ok(Self {
            numerator: WideNatural::from_u128(numerator / divisor),
            denominator: WideNatural::from_u128(denominator / divisor),
        })
    }

    #[must_use]
    pub(crate) fn is_zero(self) -> bool {
        self.numerator.is_zero()
    }

    /// Returns the canonical pair when both sides fit the signed 128-bit compatibility surface.
    #[must_use]
    pub(crate) fn as_i128_pair(self) -> Option<(i128, i128)> {
        let numerator = self.numerator.to_u128()?;
        let denominator = self.denominator.to_u128()?;
        Some((
            i128::try_from(numerator).ok()?,
            i128::try_from(denominator).ok()?,
        ))
    }

    /// Multiplies this ratio by a bounded sampled fraction while preserving canonical exactness.
    pub(crate) fn scaled_u32(
        self,
        numerator: u32,
        denominator: u32,
    ) -> Result<Self, WideRatioError> {
        if denominator == 0 {
            return Err(WideRatioError::ZeroDenominator);
        }
        if numerator == 0 || self.numerator.is_zero() {
            return Self::new(0, 1);
        }

        let fraction_divisor = greatest_common_divisor_u128(
            u128::from(numerator),
            u128::from(denominator),
        );
        let mut fraction_numerator =
            numerator / u32::try_from(fraction_divisor).expect("u32 gcd fits u32");
        let mut fraction_denominator =
            denominator / u32::try_from(fraction_divisor).expect("u32 gcd fits u32");
        let mut scaled = self;

        let numerator_cross = scaled.denominator.gcd_small(fraction_numerator);
        scaled.denominator.div_small_exact(numerator_cross)?;
        fraction_numerator /= numerator_cross;

        let denominator_cross = scaled.numerator.gcd_small(fraction_denominator);
        scaled.numerator.div_small_exact(denominator_cross)?;
        fraction_denominator /= denominator_cross;

        scaled.numerator.mul_small(fraction_numerator)?;
        scaled.denominator.mul_small(fraction_denominator)?;
        Ok(scaled)
    }

    /// Rounds `value * self` to nearest, with half values away from zero.
    pub(crate) fn mul_round_i128(self, value: i128) -> Result<i128, WideRatioError> {
        if value == 0 || self.numerator.is_zero() {
            return Ok(0);
        }
        let numerator = self.numerator.multiplied_u128(value.unsigned_abs())?;
        let (mut quotient, remainder) = div_natural_to_u128(numerator, self.denominator)?;
        if !remainder.is_zero()
            && remainder
                .multiplied_u128(2)?
                .cmp_natural(self.denominator)
                != Ordering::Less
        {
            quotient = quotient
                .checked_add(1)
                .ok_or(WideRatioError::QuotientOverflow)?;
        }
        signed_magnitude(value < 0, quotient)
    }

    /// Rounds `value * self` upward.
    pub(crate) fn mul_ceil_u128(self, value: u128) -> Result<u128, WideRatioError> {
        if value == 0 || self.numerator.is_zero() {
            return Ok(0);
        }
        let numerator = self.numerator.multiplied_u128(value)?;
        let (mut quotient, remainder) = div_natural_to_u128(numerator, self.denominator)?;
        if !remainder.is_zero() {
            quotient = quotient
                .checked_add(1)
                .ok_or(WideRatioError::QuotientOverflow)?;
        }
        Ok(quotient)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct WideNatural {
    limbs: [u64; EXACT_LIMBS],
}

impl WideNatural {
    const ZERO: Self = Self {
        limbs: [0; EXACT_LIMBS],
    };
    const ONE: Self = {
        let mut limbs = [0; EXACT_LIMBS];
        limbs[0] = 1;
        Self { limbs }
    };

    fn from_u128(value: u128) -> Self {
        let mut result = Self::ZERO;
        result.limbs[0] = value as u64;
        result.limbs[1] = (value >> 64) as u64;
        result
    }

    fn to_u128(self) -> Option<u128> {
        if self.limbs[2..].iter().any(|limb| *limb != 0) {
            return None;
        }
        Some(u128::from(self.limbs[0]) | (u128::from(self.limbs[1]) << 64))
    }

    fn is_zero(self) -> bool {
        self == Self::ZERO
    }

    fn gcd_small(self, value: u32) -> u32 {
        if value == 0 {
            return 1;
        }
        let mut left = value;
        let mut right = self.remainder_small(value);
        while right != 0 {
            let remainder = left % right;
            left = right;
            right = remainder;
        }
        left.max(1)
    }

    fn remainder_small(self, divisor: u32) -> u32 {
        debug_assert!(divisor != 0);
        let mut remainder = 0_u128;
        for limb in self.limbs.iter().rev() {
            remainder = ((remainder << 64) | u128::from(*limb)) % u128::from(divisor);
        }
        u32::try_from(remainder).expect("remainder is smaller than u32 divisor")
    }

    fn div_small_exact(&mut self, divisor: u32) -> Result<(), WideRatioError> {
        if divisor == 0 {
            return Err(WideRatioError::ZeroDenominator);
        }
        let mut remainder = 0_u128;
        for limb in self.limbs.iter_mut().rev() {
            let current = (remainder << 64) | u128::from(*limb);
            *limb = u64::try_from(current / u128::from(divisor))
                .expect("single-limb quotient fits u64");
            remainder = current % u128::from(divisor);
        }
        debug_assert_eq!(remainder, 0, "cross-cancellation must divide exactly");
        Ok(())
    }

    fn mul_small(&mut self, factor: u32) -> Result<(), WideRatioError> {
        if factor == 0 {
            *self = Self::ZERO;
            return Ok(());
        }
        let mut carry = 0_u128;
        for limb in &mut self.limbs {
            let product = u128::from(*limb) * u128::from(factor) + carry;
            *limb = product as u64;
            carry = product >> 64;
        }
        if carry != 0 {
            return Err(WideRatioError::CapacityOverflow);
        }
        Ok(())
    }

    fn multiplied_u128(self, factor: u128) -> Result<Self, WideRatioError> {
        if factor == 0 || self.is_zero() {
            return Ok(Self::ZERO);
        }
        let factors = [factor as u64, (factor >> 64) as u64];
        let mut result = Self::ZERO;

        for (left_index, left_limb) in self.limbs.into_iter().enumerate() {
            if left_limb == 0 {
                continue;
            }
            let mut carry = 0_u128;
            for (right_index, right_limb) in factors.into_iter().enumerate() {
                let index = left_index + right_index;
                if index >= EXACT_LIMBS {
                    if right_limb != 0 || carry != 0 {
                        return Err(WideRatioError::CapacityOverflow);
                    }
                    continue;
                }
                let value = u128::from(result.limbs[index])
                    + u128::from(left_limb) * u128::from(right_limb)
                    + carry;
                result.limbs[index] = value as u64;
                carry = value >> 64;
            }
            let mut index = left_index + factors.len();
            while carry != 0 {
                if index >= EXACT_LIMBS {
                    return Err(WideRatioError::CapacityOverflow);
                }
                let value = u128::from(result.limbs[index]) + carry;
                result.limbs[index] = value as u64;
                carry = value >> 64;
                index += 1;
            }
        }
        Ok(result)
    }

    fn bit_length(self) -> usize {
        for index in (0..EXACT_LIMBS).rev() {
            let limb = self.limbs[index];
            if limb != 0 {
                return index * 64 + (64 - limb.leading_zeros() as usize);
            }
        }
        0
    }

    fn bit(self, index: usize) -> u64 {
        let limb = index / 64;
        let offset = index % 64;
        (self.limbs[limb] >> offset) & 1
    }

    fn shift_left_one_with_bit(&mut self, incoming: u64) -> Result<(), WideRatioError> {
        let mut carry = incoming;
        for limb in &mut self.limbs {
            let next_carry = *limb >> 63;
            *limb = (*limb << 1) | carry;
            carry = next_carry;
        }
        if carry != 0 {
            return Err(WideRatioError::CapacityOverflow);
        }
        Ok(())
    }

    fn cmp_natural(self, right: Self) -> Ordering {
        for index in (0..EXACT_LIMBS).rev() {
            match self.limbs[index].cmp(&right.limbs[index]) {
                Ordering::Equal => {}
                ordering => return ordering,
            }
        }
        Ordering::Equal
    }

    fn sub_assign(&mut self, right: Self) {
        debug_assert!(self.cmp_natural(right) != Ordering::Less);
        let mut borrow = false;
        for index in 0..EXACT_LIMBS {
            let (first, first_borrow) = self.limbs[index].overflowing_sub(right.limbs[index]);
            let (value, second_borrow) = first.overflowing_sub(u64::from(borrow));
            self.limbs[index] = value;
            borrow = first_borrow || second_borrow;
        }
        debug_assert!(!borrow);
    }
}

fn div_natural_to_u128(
    numerator: WideNatural,
    denominator: WideNatural,
) -> Result<(u128, WideNatural), WideRatioError> {
    if denominator.is_zero() {
        return Err(WideRatioError::ZeroDenominator);
    }
    let mut quotient = 0_u128;
    let mut remainder = WideNatural::ZERO;
    for bit_index in (0..numerator.bit_length()).rev() {
        remainder.shift_left_one_with_bit(numerator.bit(bit_index))?;
        if remainder.cmp_natural(denominator) != Ordering::Less {
            remainder.sub_assign(denominator);
            if bit_index >= 128 {
                return Err(WideRatioError::QuotientOverflow);
            }
            quotient |= 1_u128 << bit_index;
        }
    }
    Ok((quotient, remainder))
}

fn signed_magnitude(negative: bool, magnitude: u128) -> Result<i128, WideRatioError> {
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

fn greatest_common_divisor_u128(mut left: u128, mut right: u128) -> u128 {
    while right != 0 {
        let remainder = left % right;
        left = right;
        right = remainder;
    }
    left
}

#[cfg(test)]
mod tests {
    use super::ExactRatio;

    #[test]
    fn exact_ratio_rounding_is_symmetric() {
        let ratio = ExactRatio::new(1, 2).expect("valid ratio");
        assert_eq!(ratio.mul_round_i128(1), Ok(1));
        assert_eq!(ratio.mul_round_i128(-1), Ok(-1));
        assert_eq!(ratio.mul_ceil_u128(1), Ok(1));
    }

    #[test]
    fn exact_ratio_can_grow_beyond_i128_and_cancel_back_exactly() {
        let mut ratio = ExactRatio::new(1, 1).expect("valid ratio");
        for _ in 0..20 {
            ratio = ratio.scaled_u32(511, 512).expect("bounded exact growth");
        }
        assert!(ratio.as_i128_pair().is_none());
        assert_eq!(ratio.mul_round_i128(60), Ok(58));

        for _ in 0..20 {
            ratio = ratio.scaled_u32(512, 511).expect("exact inverse scaling");
        }
        assert_eq!(ratio.as_i128_pair(), Some((1, 1)));
    }
}
