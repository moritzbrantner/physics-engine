# Resting-body performance policy

The engine treats stable rest as a simulation boundary rather than a state that must be recomputed every frame.

- Awake bodies still use the existing rotating contact, response, and stabilization pipeline.
- Once the existing sleep policy admits a dynamic body, its public dynamic state is retained while the active solver keeps one persistent fixed proxy at the exact settled pose.
- Parked bodies therefore remain real collision geometry and can support characters or block movement, but they no longer receive gravity, dynamic response, persistent dynamic-tail processing, or repeated sleep bookkeeping.
- The proxy is created once at sleep and removed only on a disruptive wake; sleeping bodies are no longer toggled fixed and dynamic every frame, avoiding the corresponding broad-phase membership churn.
- An awake body's conservative collision-enabled sweep wakes a parked body only when it can disrupt it. The existing passive-support policy keeps ordinary character landings from waking and dropping settled crates.
- Fixed-geometry additions and removals wake parked bodies conservatively because support topology changed.
- If every dynamic is parked, the kernel returns a zero-work physics report without contact, tail, or broad-phase work. The ECS facade also skips its body-by-body writeback when the world was already quiescent.

This is intentionally an approximation policy. Stable visible behavior, bounded work, deterministic behavior within one build, and fast wake-up are more important than exact cross-version replay hashes or invisible per-frame sleep-bookkeeping equivalence with the strict reference implementation.
