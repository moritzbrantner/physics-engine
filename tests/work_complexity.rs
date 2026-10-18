#![allow(unsafe_code)] // The counting global allocator below; the library forbids unsafe code.
//! Complexity contracts (`.performance/contract.json` schemaVersion 2 `operations`): each test
//! builds one world family, scales one dimension at doubling points while holding the others,
//! checks results against simple references, and records deterministic work counters. With
//! `PHYSICS_ENGINE_WORK_EVIDENCE_DIR` set (`scripts/collect-work-evidence.sh`), samples go to
//! `<operation>.json` for `coding-tooling performance work`; otherwise the test asserts the
//! declared bounds itself.
use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;
use std::sync::atomic::{AtomicU64, Ordering};

use physics_engine::{
    BodyId,
    approximate::{
        Body, BodyStorage, CharacterMove, CharacterMoveResult, Config, HeightfieldDesc,
        QueryFilter, QueryHit, QueryPose, RayCast, RayHit, Shape, ShapeCast, Vector as V, World,
    },
    work::{self, Counter},
};

// ---- counting allocator (this test binary only) ----
struct Counting;
static ALLOCATIONS: AtomicU64 = AtomicU64::new(0);
std::thread_local! { static COUNTING: Cell<bool> = const { Cell::new(false) }; }
unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        if COUNTING.with(Cell::get) {
            ALLOCATIONS.fetch_add(1, Ordering::Relaxed);
        }
        unsafe { System.alloc(layout) }
    }
    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        unsafe { System.dealloc(ptr, layout) }
    }
    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, size: usize) -> *mut u8 {
        if COUNTING.with(Cell::get) {
            ALLOCATIONS.fetch_add(1, Ordering::Relaxed);
        }
        unsafe { System.realloc(ptr, layout, size) }
    }
}
#[global_allocator]
static ALLOCATOR: Counting = Counting;

/// Heap allocations (alloc + realloc) made by `f` on this thread.
fn allocations(f: impl FnOnce()) -> u64 {
    ALLOCATIONS.store(0, Ordering::Relaxed);
    COUNTING.with(|c| c.set(true));
    f();
    COUNTING.with(|c| c.set(false));
    ALLOCATIONS.load(Ordering::Relaxed)
}

// ---- evidence ----
type Sample = (Vec<(&'static str, u64)>, Vec<(&'static str, u64)>);

fn evidence_dir() -> Option<std::path::PathBuf> {
    std::env::var_os("PHYSICS_ENGINE_WORK_EVIDENCE_DIR").map(Into::into)
}

fn write(operation: &str, samples: &[Sample]) {
    let fields = |pairs: &[(&str, u64)]| {
        pairs
            .iter()
            .map(|(k, v)| format!("\"{k}\":{v}"))
            .collect::<Vec<_>>()
            .join(",")
    };
    let body = samples
        .iter()
        .map(|(d, m)| {
            format!(
                "{{\"dimensions\":{{{}}},\"metrics\":{{{}}}}}",
                fields(d),
                fields(m)
            )
        })
        .collect::<Vec<_>>()
        .join(",");
    let dir = evidence_dir().expect("evidence directory");
    std::fs::write(
        dir.join(format!("{operation}.json")),
        format!("{{\"id\":\"{operation}\",\"samples\":[{body}]}}\n"),
    )
    .expect("writable evidence directory");
}

/// Self-check when not collecting: the exact classes `constant` and `linear` (marginal work
/// never increases); the tolerant classes (`logarithmic`, `polynomial`, products) are judged by
/// `coding-tooling performance work` in CI. Groups agree on the other dimensions, as there.
fn judge(operation: &str, samples: &[Sample], bounds: &[(&str, &str, &str)]) {
    if std::env::var_os("WORK_PRINT").is_some() {
        eprintln!("{operation}: {samples:?}");
        return;
    }
    if evidence_dir().is_some() {
        write(operation, samples);
        return;
    }
    for &(metric, dimension, bound) in bounds {
        let value = |s: &Sample, name: &str| {
            s.0.iter()
                .chain(&s.1)
                .find(|(k, _)| *k == name)
                .map(|(_, v)| *v)
                .unwrap()
        };
        let mut groups: Vec<Vec<&Sample>> = Vec::new();
        for s in samples {
            let key: Vec<_> = s.0.iter().filter(|(k, _)| *k != dimension).collect();
            match groups.iter_mut().find(|g| {
                g[0].0
                    .iter()
                    .filter(|(k, _)| *k != dimension)
                    .collect::<Vec<_>>()
                    == key
            }) {
                Some(g) => g.push(s),
                None => groups.push(vec![s]),
            }
        }
        for mut g in groups.into_iter().filter(|g| g.len() > 1) {
            g.sort_by_key(|s| value(s, dimension));
            let x: Vec<i128> = g.iter().map(|s| value(s, dimension) as i128).collect();
            let v: Vec<i128> = g.iter().map(|s| value(s, metric) as i128).collect();
            match bound {
                "constant" => assert!(
                    v.iter().all(|&y| y <= v[0]),
                    "{operation}.{metric} grows with {dimension}: {v:?}"
                ),
                _ => {
                    let first = (v[1] - v[0]).max(0);
                    for i in 1..v.len() - 1 {
                        assert!(
                            (v[i + 1] - v[i]) * (x[1] - x[0]) <= first * (x[i + 1] - x[i]),
                            "{operation}.{metric} grows faster than linear in {dimension}: {v:?} at {x:?}"
                        );
                    }
                }
            }
        }
    }
}

// ---- world families ----

const SPACING: f32 = 2.0;
const BOX: f32 = 0.5;

/// `2^log2` unit boxes on a regular 2 m grid (constant density), keyed by grid index.
fn town(log2: u32) -> World {
    town_with(log2, BodyStorage::Ordered, 1)
}

/// `town` on `storage`, with keys `stride` apart (gaps for inserts in the middle of the order).
fn town_with(log2: u32, storage: BodyStorage, stride: u64) -> World {
    let mut w = World::new(Config {
        gravity: V::ZERO,
        body_storage: storage,
        ..Config::default()
    })
    .unwrap();
    let (cols, rows) = (1usize << log2.div_ceil(2), 1usize << (log2 / 2));
    for j in 0..rows {
        for i in 0..cols {
            let key = (j * cols + i) as u64 * stride;
            let p = V(i as f32 * SPACING, BOX, j as f32 * SPACING);
            w.add_body(Body::keyed(key, Shape::Box(V(BOX, BOX, BOX)), p, 0.0))
                .unwrap();
        }
    }
    w
}

/// A box centre on the grid (always inside the smallest town).
fn cell(i: usize, j: usize) -> V {
    V(i as f32 * SPACING, BOX, j as f32 * SPACING)
}

fn counted(f: impl FnOnce()) -> (u64, u64) {
    work::reset();
    f();
    (
        work::get(Counter::TreeNodes),
        work::get(Counter::ExactTests),
    )
}

const SIZES: [u32; 6] = [9, 10, 11, 12, 13, 14];
const HITS: [u64; 5] = [1, 2, 4, 8, 16];
const QUERIES: u64 = 64;

/// Rays along a grid row crossing exactly `hits` boxes, from a few starting columns.
fn row_ray(start: usize, hits: u64) -> RayCast {
    let o = cell(start, 3) - V(SPACING * 0.5, 0.0, 0.0);
    RayCast::new(o, V(SPACING * hits as f32, 0.0, 0.0))
}

#[test]
fn ray_casts_are_log_n_plus_k() {
    let mut first = Vec::new();
    let mut all = Vec::new();
    let mut hits_out: Vec<RayHit> = Vec::with_capacity(64);
    for (log2, hits) in SIZES
        .iter()
        .map(|&s| (s, 4))
        .chain(HITS.iter().filter(|&&k| k != 4).map(|&k| (12, k)))
    {
        let w = town(log2);
        let (mut n_first, mut e_first, mut n_all, mut e_all) = (0, 0, 0, 0);
        for q in 0..QUERIES as usize {
            let cast = row_ray(q % 8, hits);
            let (n, e) = counted(|| {
                let hit = w
                    .cast_ray_first(cast, QueryFilter::default())
                    .unwrap()
                    .unwrap();
                assert_eq!(
                    w.body(hit.body).unwrap().key,
                    Some(3 * ((1 << log2.div_ceil(2)) as u64) + (q % 8) as u64)
                );
            });
            n_first += n;
            e_first += e;
            let (n, e) = counted(|| {
                w.cast_ray(cast, QueryFilter::default(), &mut hits_out)
                    .unwrap();
            });
            assert_eq!(hits_out.len() as u64, hits);
            n_all += n;
            e_all += e;
        }
        if hits == 4 {
            first.push((
                vec![("bodies", 1u64 << log2)],
                vec![
                    ("tree-nodes", n_first / QUERIES),
                    ("exact-tests", e_first / QUERIES),
                ],
            ));
        }
        all.push((
            vec![("bodies", 1u64 << log2), ("hits", hits)],
            vec![
                ("tree-nodes", n_all / QUERIES),
                ("exact-tests", e_all / QUERIES),
            ],
        ));
    }
    judge(
        "ray-first",
        &first,
        &[("exact-tests", "bodies", "constant")],
    );
    judge(
        "ray-all",
        &all,
        &[
            ("exact-tests", "bodies", "constant"),
            ("exact-tests", "hits", "linear"),
        ],
    );
}

#[test]
fn shape_queries_are_log_n_plus_k() {
    let mut overlap = Vec::new();
    let mut sweep = Vec::new();
    let mut out: Vec<QueryHit> = Vec::with_capacity(256);
    // Overlap squares of side s boxes give s² hits; sweeps along a row give `hits` hits.
    for (log2, side) in SIZES
        .iter()
        .map(|&s| (s, 2u64))
        .chain([1u64, 4, 8, 16].iter().map(|&s| (12, s)))
    {
        let w = town(log2);
        let (mut n, mut e) = (0, 0);
        for q in 0..QUERIES as usize {
            let corner = cell(q % 8, 2);
            let half = SPACING * side as f32 * 0.5 - 0.6;
            let centre = corner
                + V(
                    SPACING * (side as f32 - 1.0) * 0.5,
                    0.0,
                    SPACING * (side as f32 - 1.0) * 0.5,
                );
            let (a, b) = counted(|| {
                w.overlap_shape(
                    QueryPose::new(Shape::Box(V(half, 0.4, half)), centre),
                    QueryFilter::default(),
                    &mut out,
                )
                .unwrap();
            });
            assert_eq!(out.len() as u64, side * side);
            n += a;
            e += b;
        }
        overlap.push((
            vec![("bodies", 1u64 << log2), ("hits", side * side)],
            vec![("tree-nodes", n / QUERIES), ("exact-tests", e / QUERIES)],
        ));
    }
    for (log2, hits) in SIZES
        .iter()
        .map(|&s| (s, 4))
        .chain(HITS.iter().filter(|&&k| k != 4).map(|&k| (12, k)))
    {
        let w = town(log2);
        let (mut n, mut e) = (0, 0);
        for q in 0..QUERIES as usize {
            let cast = row_ray(q % 8, hits);
            let pose = QueryPose::new(Shape::Sphere(0.2), cast.origin);
            let (a, b) = counted(|| {
                w.cast_shape(
                    ShapeCast::new(pose, cast.displacement),
                    QueryFilter::default(),
                    &mut out,
                )
                .unwrap();
            });
            assert_eq!(out.len() as u64, hits);
            n += a;
            e += b;
        }
        sweep.push((
            vec![("bodies", 1u64 << log2), ("hits", hits)],
            vec![("tree-nodes", n / QUERIES), ("exact-tests", e / QUERIES)],
        ));
    }
    let bounds = [
        ("exact-tests", "bodies", "constant"),
        ("exact-tests", "hits", "linear"),
    ];
    judge("overlap-shape", &overlap, &bounds);
    judge("cast-shape", &sweep, &bounds);
}

/// Per-operation work of `op` over `count` operations: (storage moves, tree inserts, order
/// comparisons) averaged, and the worst single operation's total work.
fn churn(
    w: &mut World,
    count: u64,
    mut op: impl FnMut(&mut World, u64),
) -> Vec<(&'static str, u64)> {
    work::reset();
    let mut worst = 0;
    for k in 0..count {
        let before = work::read();
        op(w, k);
        let after = work::read();
        worst = worst.max(after.iter().zip(before).map(|(a, b)| a - b).sum());
    }
    vec![
        (
            "storage-shifts-per-op",
            work::get(Counter::StorageShifts) / count,
        ),
        (
            "tree-inserts-per-op",
            work::get(Counter::TreeInserts) / count,
        ),
        (
            "order-comparisons-per-op",
            work::get(Counter::OrderComparisons) / count,
        ),
        ("worst-op-work", worst),
    ]
}

fn churn_box(key: u64) -> Body {
    Body::keyed(
        key,
        Shape::Box(V(BOX, BOX, BOX)),
        cell(1, 1) + V(0.0, 5.0, 0.0),
        0.0,
    )
}

/// A scrambled key: consecutive `k` land all over the key range.
fn scramble(k: u64, n: u64) -> u64 {
    (k.wrapping_mul(0x9e37_79b9_7f4a_7c15) >> 20) % n
}

#[test]
fn body_churn_is_amortized_over_a_long_sequence() {
    let mut ordered = Vec::new();
    let mut middle = Vec::new();
    let mut indexed = Vec::new();
    for &log2 in &SIZES {
        let n = 1u64 << log2;
        // Ordered: remove from the middle of the order, add after every body. 4n operations,
        // so the tombstone compactions (at half the records) are part of the average.
        let mut w = town_with(log2, BodyStorage::Ordered, 1);
        let mut live = (0..n).collect::<Vec<_>>();
        let metrics = churn(&mut w, 4 * n, |w, k| {
            let victim = live.swap_remove(scramble(k, live.len() as u64) as usize);
            w.remove_body(w.body_id(victim).unwrap()).unwrap();
            w.add_body(churn_box(n + k)).unwrap();
            live.push(n + k);
        });
        ordered.push((vec![("bodies", n)], metrics));
        // Ordered: add in the middle of the order (keys between existing ones).
        let mut w = town_with(log2, BodyStorage::Ordered, 2);
        let metrics = churn(&mut w, 64, |w, k| {
            w.add_body(churn_box(n + 2 * k + 1)).unwrap();
        });
        middle.push((vec![("bodies", n)], metrics));
        // Indexed: add and remove anywhere in the order.
        let mut w = town_with(log2, BodyStorage::Indexed, 2);
        let mut live = (0..n).map(|i| 2 * i).collect::<Vec<_>>();
        let metrics = churn(&mut w, 4 * n, |w, k| {
            let victim = live.swap_remove(scramble(k, live.len() as u64) as usize);
            w.remove_body(w.body_id(victim).unwrap()).unwrap();
            // Fresh keys in scrambled order: a bijection on each block of n (n is a power of
            // two), shifted per block so keys stay unique and land all over the order.
            let key = 2 * (k.wrapping_mul(2_654_435_761) % n) + 1 + 2 * n * (k / n);
            w.add_body(churn_box(key)).unwrap();
            live.push(key);
        });
        indexed.push((vec![("bodies", n)], metrics));
    }
    judge(
        "body-churn",
        &ordered,
        &[
            ("storage-shifts-per-op", "bodies", "constant"),
            ("tree-inserts-per-op", "bodies", "constant"),
            ("worst-op-work", "bodies", "linear"),
        ],
    );
    judge(
        "body-insert-middle",
        &middle,
        &[
            ("storage-shifts-per-op", "bodies", "linear"),
            ("tree-inserts-per-op", "bodies", "constant"),
        ],
    );
    judge(
        "body-churn-indexed",
        &indexed,
        &[
            ("storage-shifts-per-op", "bodies", "constant"),
            ("tree-inserts-per-op", "bodies", "constant"),
        ],
    );
}

/// `characters` characters in 4 × 4 m blocks of the town's free lanes, `per_block` of them
/// scattered (deterministically) inside each block.
fn crowd(log2: u32, characters: u64, per_block: u64) -> (World, Vec<BodyId>) {
    let mut w = town(log2);
    let settings = CharacterSettingsHelper::plane(&w);
    let mut ids = Vec::new();
    let mut seed = 0x51u64;
    let mut next = || {
        seed = seed
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        (seed >> 40) as f32 / (1u64 << 24) as f32
    };
    for c in 0..characters {
        let block = c / per_block;
        let (bx, bz) = (block % 8, block / 8);
        // Lanes between box columns: x and z in (0.5, 1.5) mod 2 m.
        let p = V(
            1.0 + bx as f32 * 4.0 + (next() - 0.5) * 0.6 + if next() < 0.5 { 0.0 } else { 2.0 },
            BOX + 0.8 + 0.0025,
            1.0 + bz as f32 * 4.0 + (next() - 0.5) * 0.6 + if next() < 0.5 { 0.0 } else { 2.0 },
        );
        let mut body = Body::keyed(1_000_000 + c, Shape::capsule(0.2, 0.1), p, 1.0);
        body.external = true;
        let id = w.add_body(body).unwrap();
        w.add_character(id, settings).unwrap();
        ids.push(id);
    }
    (w, ids)
}

struct CharacterSettingsHelper;
impl CharacterSettingsHelper {
    fn plane(w: &World) -> physics_engine::approximate::CharacterSettings {
        physics_engine::approximate::CharacterSettings {
            plane: Some(BOX + 0.8 + 0.0025),
            ..w.character_settings()
        }
    }
}

#[test]
fn character_batches_scale_with_movers_and_density_not_with_the_world() {
    let mut samples = Vec::new();
    let mut out: Vec<CharacterMoveResult> = Vec::new();
    let points = SIZES
        .iter()
        .map(|&s| (s, 32u64, 1u64))
        .chain([8u64, 16, 64, 128].iter().map(|&m| (12, m, 1)))
        .chain([2u64, 4, 8].iter().map(|&d| (12, 32, d)));
    for (log2, m, density) in points {
        let (mut w, ids) = crowd(log2, m, density);
        // Long moves (1 m) so the swept bounds reach the neighbours a crowd has.
        let moves: Vec<_> = ids
            .iter()
            .map(|&body| CharacterMove {
                body,
                desired: V(0.7, 0.0, 0.7),
            })
            .collect();
        w.move_characters(&[], &mut out).unwrap();
        let (n, e) = counted(|| w.move_characters(&moves, &mut out).unwrap());
        samples.push((
            vec![
                ("bodies", 1u64 << log2),
                ("characters", m),
                ("density", density),
            ],
            vec![
                ("tree-nodes", n),
                ("exact-tests", e),
                ("tree-nodes-per-character", n / m),
            ],
        ));
    }
    judge(
        "move-characters",
        &samples,
        &[("exact-tests", "characters", "linear")],
    );
}

fn terrain(cells: usize) -> World {
    let mut w = World::new(Config {
        gravity: V::ZERO,
        ..Config::default()
    })
    .unwrap();
    let n = cells + 1;
    // Gentle hills plus one far spike, so the terrain's bounds reach above the query capsules.
    let mut heights: Vec<f32> = (0..n * n)
        .map(|k| 0.1 * ((k % n) as f32 * 0.3).sin() * ((k / n) as f32 * 0.2).cos())
        .collect();
    heights[n * n - 1] = 5.0;
    w.add_heightfield(
        HeightfieldDesc {
            origin: V::ZERO,
            cell_size: (1.0, 1.0),
            columns: n,
            rows: n,
            heights,
        },
        Some(1),
    )
    .unwrap();
    w
}

#[test]
fn terrain_queries_follow_cells_crossed_not_terrain_size() {
    let mut ray = Vec::new();
    let mut sweep = Vec::new();
    let mut out: Vec<QueryHit> = Vec::new();
    for (side, crossed) in [32usize, 64, 128, 256]
        .iter()
        .map(|&s| (s, 8u64))
        .chain([2u64, 4, 16, 32].iter().map(|&c| (128, c)))
    {
        let w = terrain(side);
        work::reset();
        // Diagonal rays above the surface crossing about `crossed` cells along X (and fewer in Z).
        for q in 0..16 {
            let o = V(2.5 + q as f32 * 0.1, 3.0, 2.25);
            let hit = w
                .cast_ray_first(
                    RayCast::new(o, V(crossed as f32, -0.01, crossed as f32 * 0.5)),
                    QueryFilter::default(),
                )
                .unwrap();
            assert!(hit.is_none());
        }
        ray.push((
            vec![
                ("terrainCells", (side * side) as u64),
                ("cellsCrossed", crossed),
            ],
            vec![("cells-visited", work::get(Counter::TerrainCells) / 16)],
        ));
        work::reset();
        for q in 0..16 {
            let p = V(2.5 + q as f32 * 0.1, 1.0, 2.5);
            let st = w
                .cast_shape(
                    ShapeCast::new(
                        QueryPose::new(Shape::capsule(0.3, 0.2), p),
                        V(crossed as f32, 0.0, 0.0),
                    ),
                    QueryFilter::default(),
                    &mut out,
                )
                .unwrap();
            let _ = st;
        }
        sweep.push((
            vec![
                ("terrainCells", (side * side) as u64),
                ("cellsCovered", crossed),
            ],
            vec![
                ("cells-visited", work::get(Counter::TerrainCells) / 16),
                ("triangle-tests", work::get(Counter::TerrainTriangles) / 16),
            ],
        ));
    }
    judge(
        "terrain-ray",
        &ray,
        &[
            ("cells-visited", "terrainCells", "constant"),
            ("cells-visited", "cellsCrossed", "linear"),
        ],
    );
    judge(
        "terrain-capsule-sweep",
        &sweep,
        &[
            ("cells-visited", "terrainCells", "constant"),
            ("cells-visited", "cellsCovered", "linear"),
            ("triangle-tests", "terrainCells", "constant"),
        ],
    );
}

#[test]
fn a_warm_tick_allocates_nothing_at_any_size() {
    let mut samples = Vec::new();
    for &log2 in &SIZES {
        let (mut w, ids) = crowd(log2, 32, 1);
        w.add_body({
            let mut b = Body::new(Shape::Sphere(0.3), cell(2, 2) + V(1.0, 3.0, 1.0), 1.0);
            b.velocity = V(0.0, -1.0, 0.0);
            b
        })
        .unwrap();
        let moves: Vec<_> = ids
            .iter()
            .map(|&body| CharacterMove {
                body,
                desired: V(0.02, 0.0, 0.01),
            })
            .collect();
        let mut out: Vec<CharacterMoveResult> = Vec::new();
        let mut rays: Vec<RayHit> = Vec::new();
        let mut shapes: Vec<QueryHit> = Vec::new();
        let mut tick = |w: &mut World| {
            w.move_characters(&moves, &mut out).unwrap();
            for q in 0..8 {
                let cast = row_ray(q, 4);
                w.cast_ray_first(cast, QueryFilter::default()).unwrap();
                w.cast_ray(cast, QueryFilter::default(), &mut rays).unwrap();
                w.overlap_shape(
                    QueryPose::new(Shape::Sphere(1.0), cell(q, 2)),
                    QueryFilter::default(),
                    &mut shapes,
                )
                .unwrap();
            }
            w.step(1.0 / 60.0).unwrap();
        };
        tick(&mut w);
        tick(&mut w);
        let allocs = allocations(|| tick(&mut w));
        samples.push((
            vec![("bodies", 1u64 << log2)],
            vec![("heap-allocations", allocs)],
        ));
    }
    judge(
        "warm-tick-allocations",
        &samples,
        &[("heap-allocations", "bodies", "constant")],
    );
    if evidence_dir().is_none() {
        assert!(samples.iter().all(|s| s.1[0].1 == 0), "{samples:?}");
    }
}
