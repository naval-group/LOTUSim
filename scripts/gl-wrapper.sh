#!/usr/bin/env bash
# Prints the command that lets a nix-built binary reach the host's GPU driver,
# or nothing when none is needed. Exits 1 when a bridge is needed and none is
# installed; the caller decides whether that is fatal.
#
#     glwrap=$(scripts/gl-wrapper.sh) || echo "no GPU driver bridge" >&2
#
# Gazebo needs a GL context for the GUI and for every rendering sensor — camera,
# depth camera, thermal, gpu_lidar — which it implements by rendering the scene
# from the sensor's viewpoint and reading the buffer back. `gz sim -s` drops the
# window, not the renderer, so headless runs need this too.

set -euo pipefail

# NixOS exposes the host driver here, so a nix-built binary already finds it.
if [ -d /run/opengl-driver ]; then
  exit 0
fi

# Absolute fallbacks for contexts that start with a bare PATH, like nix develop -i.
resolve() {
  local name path dir match search_dirs
  for name in "$@"; do
    for path in "$name" "$HOME/.nix-profile/bin/$name" "/nix/var/nix/profiles/default/bin/$name"; do
      if command -v "$path" >/dev/null 2>&1; then
        echo "$path"
        return 0
      fi
    done

    # nixGLNvidia has no unversioned alias — nixGL suffixes it with the
    # detected driver version, e.g. nixGLNvidia-570.195.03.
    IFS=':' read -ra search_dirs <<< "$PATH"
    search_dirs+=("$HOME/.nix-profile/bin" "/nix/var/nix/profiles/default/bin")
    for dir in "${search_dirs[@]}"; do
      for match in "$dir/$name"-*; do
        if [ -x "$match" ]; then
          echo "$match"
          return 0
        fi
      done
    done
  done
  return 1
}

if [ -n "${LOTUSIM_GL_WRAPPER:-}" ]; then
  resolve "$LOTUSIM_GL_WRAPPER" || {
    echo "LOTUSIM_GL_WRAPPER=$LOTUSIM_GL_WRAPPER not found." >&2
    exit 1
  }
  exit 0
fi

# nixGLNvidia first: prefer the NVIDIA GPU when both wrappers are installed.
# Set LOTUSIM_GL_WRAPPER to force a specific one instead.
resolve nixGLNvidia nixGLIntel nixGL