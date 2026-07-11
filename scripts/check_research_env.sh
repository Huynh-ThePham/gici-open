#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

ok() {
  printf '[ OK ] %s\n' "$1"
}

warn() {
  printf '[WARN] %s\n' "$1"
}

fail() {
  printf '[FAIL] %s\n' "$1"
  FAILED=1
}

need_cmd() {
  local cmd="$1"
  if command -v "$cmd" >/dev/null 2>&1; then
    ok "$cmd: $(command -v "$cmd")"
  else
    fail "$cmd is missing"
  fi
}

need_pkg_config() {
  local pkg="$1"
  local label="${2:-$pkg}"
  local recommended_min="${3:-}"
  if pkg-config --exists "$pkg"; then
    local version
    version="$(pkg-config --modversion "$pkg")"
    if [ -n "$recommended_min" ] && ! version_ge "$version" "$recommended_min"; then
      warn "$label: ${version}; README recommends >= ${recommended_min}"
    else
      ok "$label: $version"
    fi
  else
    fail "$label is missing from pkg-config"
  fi
}

need_cmake_package() {
  local pkg="$1"
  local recommended_min="${2:-}"
  local tmp_dir
  local output
  local version
  tmp_dir="$(mktemp -d)"
  cat > "${tmp_dir}/CMakeLists.txt" <<EOF
cmake_minimum_required(VERSION 3.10)
project(check_${pkg} LANGUAGES CXX)
find_package(${pkg} REQUIRED)
if(DEFINED ${pkg}_VERSION)
  message(STATUS "${pkg}_VERSION=\${${pkg}_VERSION}")
endif()
EOF
  if output="$(cmake -S "$tmp_dir" -B "${tmp_dir}/build" 2>&1)"; then
    version="$(printf '%s\n' "$output" | sed -n "s/^-- ${pkg}_VERSION=//p" | tail -n 1)"
    if [ -n "$version" ]; then
      if [ -n "$recommended_min" ] && ! version_ge "$version" "$recommended_min"; then
        warn "CMake package ${pkg}: ${version}; README recommends >= ${recommended_min}"
      else
        ok "CMake package ${pkg}: ${version}"
      fi
    else
      ok "CMake package ${pkg}"
    fi
  else
    fail "CMake package ${pkg} is missing"
  fi
  rm -rf "$tmp_dir"
}

version_ge() {
  local current="$1"
  local minimum="$2"
  [ "$(printf '%s\n%s\n' "$minimum" "$current" | sort -V | head -n 1)" = "$minimum" ]
}

FAILED=0

printf 'GICI research environment check\n'
printf 'Root: %s\n\n' "$ROOT_DIR"

need_cmd git
need_cmd cmake
need_cmd g++
need_cmd make
need_cmd pkg-config
need_cmd python3

need_pkg_config eigen3 Eigen3
need_pkg_config opencv4 OpenCV
need_pkg_config yaml-cpp yaml-cpp
need_pkg_config libglog glog 0.6.0
need_cmake_package Ceres 2.1.0

if [ -f /opt/ros/noetic/setup.bash ]; then
  ok 'ROS Noetic: /opt/ros/noetic/setup.bash'
else
  warn 'ROS Noetic not found; non-ROS build is still supported'
fi

printf '\n'
if [ "$FAILED" -eq 0 ]; then
  ok 'Required non-ROS dependencies are available'
else
  fail 'Install missing packages listed in research/apt-packages.ubuntu22.04.txt'
  exit 1
fi
