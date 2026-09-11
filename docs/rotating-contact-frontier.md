# Rotating contact frontier

The rotating contact frontier is the handoff between sampled contact discovery and future rotational response.

It reconstructs every rotating body from one common interval start at the earliest sampled contact fraction and then evaluates the conservative candidate set in that shared state. All contacts present at that fraction are retained together so response can be staged against one deterministic world snapshot rather than pair-by-pair discovery order.

This boundary is intentionally narrower than rotational CCD. The admitted time comes from the existing sampled rotating contact search; therefore contact or separation islands that exist wholly between coarse samples can still be missed. Refinement sharpens an observed bracket but does not make the search analytic.

The frontier owns no ECS IDs, renderer state, browser state, or game-loop policy. Those remain consumer concerns.
