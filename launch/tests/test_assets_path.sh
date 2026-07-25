#!/bin/bash
# Checks that --assets-path adds assets roots to the core one rather than
# replacing it, accepts a colon-separated list, is repeatable, and that a world
# is resolved from whichever root holds it.
#
# Requires a built LOTUSim environment (LOTUSIM_WS/LOTUSIM_PATH set, gz available),
# i.e. run it inside the LOTUSim image or after `lotusim install`:
#
#     bash launch/tests/test_assets_path.sh
#
# Creates its own throwaway assets roots; touches nothing in the workspace.

LOTUSIM="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/lotusim"
FIXTURE="$(mktemp -d)"
trap 'rm -rf "$FIXTURE"' EXIT

A="${FIXTURE}/rootA"
B="${FIXTURE}/rootB"
CORE="${LOTUSIM_PATH}/assets/models"
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

# The world lives in rootB but includes a model from rootA: it only loads when
# both roots are searched.
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
        echo "        got:     $(grep -a GZ_SIM_RESOURCE_PATH <<<"$3" | head -1)"
        fail=$((fail + 1))
    fi
}

echo "1. no option: the core assets root, unchanged"
out=$("$LOTUSIM" run __no_such__.world 2>&1)
check "GZ_SIM_RESOURCE_PATH is the core models dir" \
    "GZ_SIM_RESOURCE_PATH.*: ${CORE}\$" "$out"

echo "2. one option: added after the core root, which stays implicit"
out=$("$LOTUSIM" --assets-path "$A" run __no_such__.world 2>&1)
check "GZ_SIM_RESOURCE_PATH is core then rootA" \
    "GZ_SIM_RESOURCE_PATH.*: ${CORE}:${A}/models\$" "$out"

echo "3. one option, colon-separated"
out=$("$LOTUSIM" --assets-path "${A}:${B}" run __no_such__.world 2>&1)
check "GZ_SIM_RESOURCE_PATH is core then rootA then rootB" \
    "GZ_SIM_RESOURCE_PATH.*: ${CORE}:${A}/models:${B}/models\$" "$out"

echo "4. repeated option: same result as the colon-separated form"
out=$("$LOTUSIM" --assets-path "$A" --assets-path "$B" run __no_such__.world 2>&1)
check "GZ_SIM_RESOURCE_PATH is core then rootA then rootB" \
    "GZ_SIM_RESOURCE_PATH.*: ${CORE}:${A}/models:${B}/models\$" "$out"
check "a missing world names every searched root" \
    "World '__no_such__.world' not found in: ${LOTUSIM_PATH}/assets ${A} ${B}" "$out"

echo "5. the world is resolved from the root that holds it"
out=$(timeout -s KILL 20 "$LOTUSIM" --assets-path "${A}:${B}" \
    run compose_test.world 2>&1)
check "world found under rootB" \
    "Running the simulation world.*: ${B}/worlds/compose_test.world" "$out"

echo "6. gz A/B: rootA's model resolves from rootB's world"
# --debug (-v4) is required: at the default -v0 gz reports nothing and the A/B
# would prove nothing.
both=$(timeout -s KILL 20 "$LOTUSIM" --debug --assets-path "${A}:${B}" \
    run compose_test.world 2>&1)
only=$(timeout -s KILL 20 "$LOTUSIM" --debug --assets-path "$B" \
    run compose_test.world 2>&1)

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

echo "7. completion offers worlds from the extra roots too"
# shellcheck source=/dev/null
source "$(dirname "$LOTUSIM")/bash_completion.sh"
COMP_WORDS=(lotusim --assets-path "$B" run "")
COMP_CWORD=4
COMPREPLY=()
lotusim_script_completion
if printf '%s\n' "${COMPREPLY[@]}" | grep -qx "compose_test.world"; then
    echo "  PASS  rootB's world is completed"
    pass=$((pass + 1))
else
    echo "  FAIL  rootB's world missing from completion: ${COMPREPLY[*]}"
    fail=$((fail + 1))
fi

echo
echo "===== ${pass} passed / ${fail} failed ====="
[ "$fail" -eq 0 ]
