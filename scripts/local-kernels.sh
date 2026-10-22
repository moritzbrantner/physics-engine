#!/usr/bin/env bash
# Points every workspace (root, oracle, demo-wasm) at a local rust-kernels checkout, for testing
# a kernel change before it is merged, and puts everything back afterwards.
#
#   scripts/local-kernels.sh on [path]   # default path: ../rust-kernels
#   scripts/local-kernels.sh off         # restores manifests and lockfiles exactly
#   scripts/local-kernels.sh status
#
# The originals are kept in target/local-kernels/ while the patch is on; never commit while it
# is on (`git status` shows the patched manifests).
set -euo pipefail
root="$(git rev-parse --show-toplevel)"
backup="$root/target/local-kernels"
workspaces=(. oracle demo-wasm)
crates=(geometry-kernels bvh-kernels spatial-kernels)

usage() {
  sed -n '2,10p' "$0" >&2
  exit 2
}

case "${1:-}" in
  on)
    kernels="$(cd "${2:-$root/../rust-kernels}" && pwd)"
    [[ -f "$kernels/crates/geometry-kernels/Cargo.toml" ]] || {
      echo "not a rust-kernels checkout: $kernels" >&2
      exit 1
    }
    [[ ! -d "$backup" ]] || {
      echo "already on (scripts/local-kernels.sh off first)" >&2
      exit 1
    }
    mkdir -p "$backup"
    for ws in "${workspaces[@]}"; do
      mkdir -p "$backup/$ws"
      cp "$root/$ws/Cargo.toml" "$root/$ws/Cargo.lock" "$backup/$ws/"
      {
        echo
        echo '[patch."https://github.com/moritzbrantner/rust-kernels"]'
        for crate in "${crates[@]}"; do
          echo "$crate = { path = \"$kernels/crates/$crate\" }"
        done
      } >>"$root/$ws/Cargo.toml"
      # Without --locked: the lockfile follows the local crates until `off`.
      packages=()
      for crate in "${crates[@]}"; do packages+=(-p "$crate"); done
      (cd "$root/$ws" && cargo update --quiet "${packages[@]}")
    done
    echo "rust-kernels -> $kernels in: ${workspaces[*]} (run without --locked)"
    ;;
  off)
    [[ -d "$backup" ]] || {
      echo "not on" >&2
      exit 1
    }
    for ws in "${workspaces[@]}"; do
      cp "$backup/$ws/Cargo.toml" "$backup/$ws/Cargo.lock" "$root/$ws/"
    done
    rm -rf "$backup"
    echo "restored: ${workspaces[*]}"
    ;;
  status)
    if [[ -d "$backup" ]]; then echo on; else echo off; fi
    ;;
  *) usage ;;
esac
