//! Contact history of pairs with no awake side: sleeping bodies on each other or on fixed
//! geometry. It waits here so the step's per-substep contact-cache walks (`World::cache`) never
//! see it (#524); a wake moves a body's pairs back, a step start parks the pairs that went quiet.

use super::{CachedPoint, Order};
use std::collections::{BTreeMap, BTreeSet};

type Pair = (Order, Order);

#[derive(Clone, Debug, Default)]
pub(super) struct ParkedContacts {
    pairs: BTreeMap<Pair, Vec<CachedPoint>>,
    /// Every key reversed, `(b, a)`: the pairs of a body as the higher side.
    reversed: BTreeSet<Pair>,
}

impl ParkedContacts {
    pub fn is_empty(&self) -> bool {
        self.pairs.is_empty()
    }
    pub fn len(&self) -> usize {
        self.pairs.len()
    }
    pub fn iter(&self) -> impl Iterator<Item = (&Pair, &Vec<CachedPoint>)> {
        self.pairs.iter()
    }
    pub fn keys(&self) -> impl Iterator<Item = &Pair> {
        self.pairs.keys()
    }
    pub fn insert(&mut self, key: Pair, points: Vec<CachedPoint>) {
        self.reversed.insert((key.1, key.0));
        self.pairs.insert(key, points);
    }
    pub fn retain(&mut self, mut keep: impl FnMut(&Pair, &mut Vec<CachedPoint>) -> bool) {
        let reversed = &mut self.reversed;
        self.pairs.retain(|key, points| {
            let kept = keep(key, points);
            if !kept {
                reversed.remove(&(key.1, key.0));
            }
            kept
        });
    }
    /// Removes every pair of `order`: O(its pairs · log parked).
    pub fn drain_body(&mut self, order: Order, mut each: impl FnMut(Pair, Vec<CachedPoint>)) {
        let (low, high) = (Order(0), Order(u128::MAX));
        let lower: Vec<Pair> = self
            .pairs
            .range((order, low)..=(order, high))
            .map(|(key, _)| *key)
            .collect();
        let higher: Vec<Pair> = self
            .reversed
            .range((order, low)..=(order, high))
            .map(|&(b, a)| (a, b))
            .collect();
        for key in lower.into_iter().chain(higher) {
            if let Some(points) = self.pairs.remove(&key) {
                self.reversed.remove(&(key.1, key.0));
                each(key, points);
            }
        }
    }
}
