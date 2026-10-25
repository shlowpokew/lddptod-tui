#!/usr/bin/env bash
# Run rover-tui on the Pi. Fetches the latest prebuilt binary from GitHub
# releases when it changed (one small checksum download otherwise), then runs
# it. No Rust on the Pi.
#
#   bash rover-tui.sh                 # update if needed, run
#   bash rover-tui.sh --offline       # skip the update check
#   bash rover-tui.sh --lidar         # any other args go to rover-tui (--help)
#
# Install as a command:  install -m 0755 rover-tui.sh ~/.local/bin/tui
set -euo pipefail

repo=${TUI_REPO:-shlowpokew/lddptod-tui}
asset=rover-tui-aarch64-linux
url=https://github.com/$repo/releases/download/latest/$asset
dir=${TUI_DIR:-$HOME/.local/share/rover-tui}
bin=$dir/rover-tui

case "$(uname -s)/$(uname -m)" in
  Linux/aarch64) ;;
  *) echo "rover-tui.sh: prebuilt binary is for 64-bit ARM Linux (the Pi), not $(uname -sm)." >&2; exit 1 ;;
esac

offline=false
args=()
for a in "$@"; do
  if [[ $a == --offline ]]; then offline=true; else args+=("$a"); fi
done

mkdir -p "$dir"
if ! $offline; then
  if remote=$(curl -fsSL --connect-timeout 5 --max-time 15 "$url.sha256" 2>/dev/null) && [[ -n $remote ]]; then
    local_sum=$( [[ -x $bin ]] && sha256sum "$bin" | cut -d' ' -f1 || true )
    if [[ ${remote%% *} != "$local_sum" ]]; then
      echo "Downloading rover-tui from $repo ..."
      if curl -fL --connect-timeout 5 --progress-bar -o "$bin.part" "$url" &&
         [[ $(sha256sum "$bin.part" | cut -d' ' -f1) == "${remote%% *}" ]]; then
        chmod +x "$bin.part" && mv "$bin.part" "$bin"
      else
        rm -f "$bin.part"
        echo "Download failed or checksum mismatch; keeping the binary I have." >&2
      fi
    fi
  else
    echo "Can't reach GitHub; using the binary I have." >&2
  fi
fi

if [[ ! -x $bin ]]; then
  echo "No rover-tui binary yet and no network. Copy one to $bin" >&2
  echo "(e.g. from your laptop: scp rover-tui-aarch64-linux pi@<host>:$bin)" >&2
  exit 1
fi
exec "$bin" ${args[@]+"${args[@]}"}
