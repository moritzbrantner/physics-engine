//! Actual authored collision from moenarch-games/mmorpg PR #640 at 9ccc3a57.
//! The source uses centimetres; tests convert to metres, matching its Rapier adapter.
use physics_engine::approximate::{TriangleMeshDesc, Vector as V};

pub fn chunks() -> Vec<(u64, TriangleMeshDesc)> {
    let json: serde_json::Value =
        serde_json::from_str(include_str!("../fixtures/redbrand-cave.json")).unwrap();
    let vector = |v: &serde_json::Value| {
        V(
            v[0].as_f64().unwrap() as f32 * 0.01,
            v[1].as_f64().unwrap() as f32 * 0.01,
            v[2].as_f64().unwrap() as f32 * 0.01,
        )
    };
    json.as_array()
        .unwrap()
        .iter()
        .map(|chunk| {
            (
                chunk["id"].as_u64().unwrap(),
                TriangleMeshDesc {
                    position: vector(&chunk["position"]),
                    vertices: chunk["vertices"]
                        .as_array()
                        .unwrap()
                        .iter()
                        .map(vector)
                        .collect(),
                    triangles: chunk["triangles"]
                        .as_array()
                        .unwrap()
                        .iter()
                        .map(|t| {
                            [
                                t[0].as_u64().unwrap() as u32,
                                t[1].as_u64().unwrap() as u32,
                                t[2].as_u64().unwrap() as u32,
                            ]
                        })
                        .collect(),
                    material: Default::default(),
                },
            )
        })
        .collect()
}
