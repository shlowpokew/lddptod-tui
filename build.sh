#!/usr/bin/env bash
# Optional local build without installing Rust: compiles the static aarch64
# binary inside a Rust Alpine container (native on Apple Silicon), and can
# copy it to the Pi. CI does the same thing on every push to main.
#
#   bash build.sh                  # -> ./rover-tui-aarch64-linux
#   bash build.sh pi@rover.local   # build, then scp to the Pi
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"

engine=$(command -v docker || command -v podman || true)
[[ -n $engine ]] || { echo 'Needs docker or podman.' >&2; exit 1; }

"$engine" run --rm --platform linux/arm64 \
  -v "$PWD":/src -w /src \
  -v rover-tui-cargo:/usr/local/cargo/registry \
  -e CARGO_TARGET_DIR=/src/target/alpine \
  rust:1-alpine sh -c 'apk add --no-cache musl-dev >/dev/null && cargo build --release'
cp target/alpine/release/rover-tui rover-tui-aarch64-linux
ls -la rover-tui-aarch64-linux

if [[ -n ${1:-} ]]; then
  ssh "$1" 'mkdir -p ~/.local/share/rover-tui'
  scp rover-tui-aarch64-linux "$1":.local/share/rover-tui/rover-tui
  scp rover-tui.sh "$1":
  echo "On the Pi: bash rover-tui.sh --offline"
fi
