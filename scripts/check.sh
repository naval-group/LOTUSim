#!/usr/bin/env bash
# Headless smoke test: vessels reach the physics interface, a gpu_lidar builds
# its render context, and gz logs no error. gz exits 0 whether or not a model
# resolved, so the log is the only signal.
#
#     mise run check

set -uo pipefail

: "${LOTUSIM_PATH:?run this through mise: mise run check}"
cd "$LOTUSIM_PATH"

if [ ! -f install/setup.bash ]; then
  echo "No workspace build found — run 'mise run build' first." >&2
  exit 1
fi

glwrap=()
if wrapper=$(bash scripts/gl-wrapper.sh); then
  if [ -n "$wrapper" ]; then glwrap=("$wrapper"); fi
else
  echo "No GPU driver bridge — the sensor case below will fail." >&2
  echo "  nix profile add github:nix-community/nixGL#nixGLIntel" >&2
fi

FIXTURE="$(mktemp -d)"
trap 'rm -rf "$FIXTURE"' EXIT

# A world of our own: the sensors system builds a render context only when a
# rendering sensor actually appears, and no shipped world pairs the plugin with
# a model that carries one.
cat > "${FIXTURE}/sensors.world" <<'EOF'
<?xml version="1.0"?>
<sdf version="1.7">
  <world name="sensor_check">
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics" />
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors" />
    <include><uri>model://dtmb_hull</uri></include>
  </world>
</sdf>
EOF

pass=0
fail=0

check_world() { # check_world <label> <world> <expected-in-log> <seconds>
    local label=$1 world=$2 want=$3 budget=$4 log
    log=$(mktemp)
    # A gz server that built a render context never returns from --iterations:
    # its rendering thread does not join. Bound every run and read the log.
    { timeout -s KILL "$budget" "${glwrap[@]}" gz sim -s -r --iterations 100 -v 3 "$world" >"$log" 2>&1 || true; } 2>/dev/null

    if grep -aq '\[Err\]' "$log"; then
        echo "  FAIL  ${label}: gz logged an error"
        grep -a '\[Err\]' "$log" | head -3 >&2
        fail=$((fail + 1))
    elif ! grep -aq "$want" "$log"; then
        # Without this, a world that silently loaded nothing would also pass.
        echo "  FAIL  ${label}: '${want}' never appeared"
        fail=$((fail + 1))
    else
        echo "  PASS  ${label}"
        pass=$((pass + 1))
    fi
    rm -f "$log"
}

check_world "vessels reach the physics interface" assets/worlds/xdyn_multithread_test.world frigate 120
check_world "a gpu_lidar builds its render context" "${FIXTURE}/sensors.world" dtmb 45

echo
echo "===== ${pass} passed / ${fail} failed ====="
[ "$fail" -eq 0 ]
