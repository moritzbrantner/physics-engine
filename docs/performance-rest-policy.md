# Resting-body performance policy

The engine now treats stable rest as a simulation boundary rather than a state that must be recomputed every frame.

- Awake bodies still use the existing rotating contact, response, and stabilization pipeline.
- Once the existing sleep policy admits a dynamic body, the performance-oriented world parks it outside active collision, tail, current-contact, and fixed-boundary work.
- A parked body keeps its exact last pose and zero motion until an awake dynamic's conservative sweep can reach it.
- Fixed-geometry additions and body removals wake parked bodies conservatively because support topology changed.
- Fully parked scenes return a zero-work physics report without rebuilding collision trees or re-solving contacts.

This is intentionally an approximation policy. Stable visible behavior, bounded work, and fast wake-up are more important than preserving invisible per-frame solver activity or exact sleep-bookkeeping equivalence with the strict reference implementation.
