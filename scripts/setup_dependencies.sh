#!/usr/bin/env bash
set -euo pipefail

PACKAGE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
THIRD_PARTY="${PACKAGE_ROOT}/third_party"
GCOPTER_DIR="${THIRD_PARTY}/GCOPTER"

mkdir -p "${THIRD_PARTY}"
if [[ ! -d "${GCOPTER_DIR}/.git" ]]; then
  git clone --depth 1 https://github.com/ZJU-FAST-Lab/GCOPTER.git "${GCOPTER_DIR}"
else
  echo "GCOPTER already exists: ${GCOPTER_DIR}"
fi

HEADER="${GCOPTER_DIR}/gcopter/include/gcopter/minco.hpp"
if [[ ! -f "${HEADER}" ]]; then
  echo "ERROR: MINCO header not found at ${HEADER}" >&2
  exit 1
fi

echo "Dependencies are ready. Build this package in a ROS1 catkin workspace."
