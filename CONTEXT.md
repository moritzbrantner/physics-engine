# Repository context

`physics-engine` provides the physics capabilities the owner's games use from Rapier, through its
own performance-oriented API, and is measured against Rapier natively and in WASM. See
`docs/technical-decisions.md` and `docs/consumer-capabilities.md`.

There is one world, `approximate::World`, with `f32` physical state and `f64` time. It is a
persistent-manifold, warm-started solver with bounded iterations and translation CCD. Returned
step errors restore touched physical state, pending inputs and warm-start/sleep history; see
`docs/floating-step-transactions.md`. Opt-in versioned physical checkpoints (Cargo feature
`checkpoints`) support debugging and logging; see `docs/floating-checkpoints.md`.

The repository is not a game engine. ECS storage, rendering, game loops and gameplay rules stay in
consumers. Domain-neutral geometry kernels come from `rust-kernels`.
