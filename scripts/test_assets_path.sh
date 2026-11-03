#!/bin/bash
# Checks that --assets-path adds assets roots to the core one rather than
# replacing it, accepts a colon-separated list, is repeatable, and that a world
# is resolved from whichever root holds it.
#
# Run inside the devShell, after `mise run build`:
#
#     mise run test:assets
#
# Creates its own throwaway assets roots; touches nothing in the workspace.

FIXTURE="$(mktemp -d)"
trap 'rm -rf "$FIXTURE"' EXIT

A="${FIXTURE}/rootA"
B="${FIXTURE}/rootB"
CORE="${LOTUSIM_PATH}/assets"
mkdir -p "${A}/models/alpha" "${B}/worlds"

cat > "${A}/models/alpha/model.config" <<'EOF'
<?xml version="1.0"?>
<model>
  <name>alpha</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
</model>
EOF

cat > "${A}/models/alpha/model.sdf" <<'EOF'
<?xml version="1.0"?>
<sdf version="1.6">
  <model name="alpha">
    <static>true</static>
    <link name="link">
      <visual name="v">
        <geometry><box><size>1 1 1</size></box></geometry>
      </visual>
    </link>
  </model>
</sdf>
EOF

# The world lives in rootB but includes a model from rootA: it only loads when both roots are searched.
cat > "${B}/worlds/compose_test.world" <<'EOF'
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="compose_test">
    <include><uri>model://alpha</uri></include>
  </world>
</sdf>
EOF

pass=0
fail=0
check() { # check <name> <expected-regex> <output>
    if grep -Eq "$2" <<<"$3"; then
        echo "  PASS  $1"
        pass=$((pass + 1))
    else
        echo "  FAIL  $1"
        echo "        expected /$2/"
        echo "        got:     $(grep -aE 'GZ_SIM_RESOURCE_PATH|World:|not found in' <<<"$3" | head -2)"
        fail=$((fail + 1))
    fi
}

sim() { mise run sim "$@" 2>&1; }

# Kills a process and all its descendants. The sim task runs gz as a child
# in a process group of its own, so neither killing mise nor its group would
# reach gz.
kill_tree() {
    local child
    for child in $(cat /proc/"$1"/task/*/children 2>/dev/null); do
        kill_tree "$child"
    done
    kill -KILL "$1" 2>/dev/null
}

# A world that loads keeps gz running: stop the sim task after 30 s.
sim_for_30s() {
    local log="${FIXTURE}/sim.log" pid i
    mise run sim "$@" >"$log" 2>&1 &
    pid=$!
    for ((i = 0; i < 30; i++)); do
        kill -0 "$pid" 2>/dev/null || break
        sleep 1
    done
    kill_tree "$pid"
    wait "$pid" 2>/dev/null
    cat "$log"
}

echo "1. no option: the core assets root, unchanged"
out=$(sim --debug __no_such__.world)
check "GZ_SIM_RESOURCE_PATH is the core root and its models dir" \
    "GZ_SIM_RESOURCE_PATH: ${CORE}:${CORE}/models\$" "$out"

echo "2. one option: added after the core root, which stays implicit"
out=$(sim --assets-path "$A" __no_such__.world)
check "GZ_SIM_RESOURCE_PATH is core then rootA" \
    "GZ_SIM_RESOURCE_PATH: ${CORE}:${CORE}/models:${A}:${A}/models\$" "$out"

echo "3. one option, colon-separated"
out=$(sim --assets-path "${A}:${B}" __no_such__.world)
check "GZ_SIM_RESOURCE_PATH is core then rootA then rootB" \
    "GZ_SIM_RESOURCE_PATH: ${CORE}:${CORE}/models:${A}:${A}/models:${B}:${B}/models\$" "$out"

echo "4. repeated option: same result as the colon-separated form"
out=$(sim --assets-path "$A" --assets-path "$B" __no_such__.world)
check "GZ_SIM_RESOURCE_PATH is core then rootA then rootB" \
    "GZ_SIM_RESOURCE_PATH: ${CORE}:${CORE}/models:${A}:${A}/models:${B}:${B}/models\$" "$out"
check "a missing world names every searched root" \
    "World '__no_such__.world' not found in: ${CORE} ${A} ${B}" "$out"

echo "5. the world is resolved from the root that holds it"
out=$(sim_for_30s --assets-path "${A}:${B}" compose_test.world)
check "world found under rootB" \
    "World: ${B}/worlds/compose_test.world" "$out"

echo "6. gz A/B: rootA's model resolves from rootB's world"
# --debug (-v4) is required: at -v1 gz reports nothing and the A/B would prove nothing.
both=$(sim_for_30s --debug --assets-path "${A}:${B}" compose_test.world)
only=$(sim_for_30s --debug --assets-path "$B" compose_test.world)

if grep -aqi "unable to find uri\[model://alpha\]" <<<"$both"; then
    echo "  FAIL  with both roots, gz cannot resolve model://alpha"
    fail=$((fail + 1))
else
    echo "  PASS  with both roots, gz resolves model://alpha"
    pass=$((pass + 1))
fi

if grep -aqi "unable to find uri\[model://alpha\]" <<<"$only"; then
    echo "  PASS  without rootA, gz fails: composition is what makes the difference"
    pass=$((pass + 1))
else
    echo "  FAIL  without rootA, gz should have failed; test 6 proves nothing"
    fail=$((fail + 1))
fi

echo
echo "===== ${pass} passed / ${fail} failed ====="
[ "$fail" -eq 0 ]
