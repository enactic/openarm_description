#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE_DIR="$(cd "${ROOT_DIR}/../.." && pwd)"
WORKSPACE_SETUP="${WORKSPACE_DIR}/install/setup.bash"
PRESET_DIR="${ROOT_DIR}/assets/robot/openarm_v2.0/config/robot_presets"
XACRO_FILE="${ROOT_DIR}/assets/robot/openarm_v2.0/urdf/openarm_v20.urdf.xacro"
BUILD_DIR="${ROOT_DIR}/urdf"

COLLAPSE_ARG="true"
ROS2_CONTROL_ARG="true"
PRESET_NAME=""
GRASP_FRAME_ONLY="false"
GENERATE_ALL="false"

generate_one() {
  local preset_name="$1"
  local emit_grasp="$2"
  local suffix=""
  local output_file=""

  if [[ "${COLLAPSE_ARG}" == "false" ]]; then
    suffix="${suffix}_no_collapse"
  fi
  if [[ "${emit_grasp}" == "true" ]]; then
    suffix="${suffix}_grasp"
  fi

  output_file="${BUILD_DIR}/openarm_${preset_name}${suffix}.urdf"
  xacro "${XACRO_FILE}" \
    robot_preset:="${preset_name}" \
    collapse_internal_empty_links:="${COLLAPSE_ARG}" \
    emit_grasp_frame:="${emit_grasp}" \
    ros2_control:="${ROS2_CONTROL_ARG}" \
    > "${output_file}"
  echo "Generated ${output_file}"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --preset)
      PRESET_NAME="${2:-}"
      if [[ -z "${PRESET_NAME}" ]]; then
        echo "Missing value for --preset" >&2
        exit 1
      fi
      shift 2
      ;;
    --grasp-frame)
      GRASP_FRAME_ONLY="true"
      shift
      ;;
    --all)
      GENERATE_ALL="true"
      shift
      ;;
    --no-ros2-control)
      ROS2_CONTROL_ARG="false"
      shift
      ;;
    --keep-empty-links)
      COLLAPSE_ARG="false"
      shift
      ;;
    -h|--help)
      cat <<'EOF'
Usage:
  bash openarm_description/scripts/generate_v20_urdfs.sh
  bash openarm_description/scripts/generate_v20_urdfs.sh --preset <preset_name> [--grasp-frame] [--keep-empty-links] [--no-ros2-control]
  bash openarm_description/scripts/generate_v20_urdfs.sh [--all] [--grasp-frame] [--keep-empty-links] [--no-ros2-control]

Options:
  --preset <name>     Generate only the selected robot preset.
  --all               Generate every real preset instead of the default dual-arm preset only.
  --grasp-frame       Generate only the grasp-frame variant.
  --keep-empty-links  Keep the arm tip link and its fixed EE mount joint.
  --no-ros2-control   Omit control interfaces from exported URDFs.

Behavior:
  With no arguments, generate only default_bimanual in both standard and grasp variants.
  With --all, generate all real presets in both standard and grasp variants.
  With --preset only, generate the standard collapsed variant for that preset.
EOF
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      echo "Use --help for usage." >&2
      exit 1
      ;;
  esac
done

if [[ -f "${WORKSPACE_SETUP}" ]]; then
  # xacro includes still rely on package discovery through the ROS 2 workspace.
  # Source the local install space automatically when it exists.
  restore_nounset=0
  if [[ $- == *u* ]]; then
    restore_nounset=1
    set +u
  fi
  export COLCON_TRACE="${COLCON_TRACE-}"
  # shellcheck disable=SC1090
  source "${WORKSPACE_SETUP}"
  if [[ "${restore_nounset}" -eq 1 ]]; then
    set -u
  fi
fi

mkdir -p "${BUILD_DIR}"

if [[ -n "${PRESET_NAME}" ]]; then
  if [[ ! -f "${PRESET_DIR}/${PRESET_NAME}.yaml" ]]; then
    echo "Unknown preset: ${PRESET_NAME}" >&2
    exit 1
  fi

  generate_one "${PRESET_NAME}" "${GRASP_FRAME_ONLY}"
  exit 0
fi

if [[ "${GRASP_FRAME_ONLY}" == "true" ]]; then
  EMIT_GRASP_VALUES=("true")
else
  EMIT_GRASP_VALUES=("false" "true")
fi

if [[ "${GENERATE_ALL}" == "true" ]]; then
  for preset_file in "${PRESET_DIR}"/*.yaml; do
    preset_name="$(basename "${preset_file}" .yaml)"
    if [[ "${preset_name}" == example_* ]]; then
      continue
    fi
    for emit_grasp in "${EMIT_GRASP_VALUES[@]}"; do
      generate_one "${preset_name}" "${emit_grasp}"
    done
  done
else
  for emit_grasp in "${EMIT_GRASP_VALUES[@]}"; do
    generate_one "default_bimanual" "${emit_grasp}"
  done
fi
