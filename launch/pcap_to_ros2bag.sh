#!/usr/bin/env bash
set -Eeuo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PKG_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly HESAI_WS="$(cd "${PKG_DIR}/../.." && pwd)"

readonly DEFAULT_BASE_CONFIG="${PKG_DIR}/config/pcap_to_ros2bag_config.yaml"
readonly DEFAULT_SETUP_FILE="${HESAI_WS}/install/setup.bash"
readonly DEFAULT_FIRETIMES_PATH="${PKG_DIR}/src/driver/HesaiLidar_SDK_2.0/correction/firetime_correction/PandarXT_Firetime Correction File.csv"

usage() {
  cat <<EOF
Usage:
  $(basename "$0") [--default-frame-frequency hz] <input_seq> <gps_date> <lio_ws> [output_bag_path]

Arguments:
  input_seq        Sequence folder containing lidar.pcap, lidar_calibration.csv, and imu.csv.
  gps_date         GPS date for IMU timestamps, formatted as YYYYMMDD.
  lio_ws           FAST-LIO workspace containing src/fast_lio/python/tersus/csv_imu_to_ros2bag.py.
  output_bag_path  Optional lidar bag output path. Default: <input_seq>/pandarxt32.

Options:
  --default-frame-frequency hz
                   Override driver.default_frame_frequency in the runtime config.

Example:
  $(basename "$0") /path/to/park5 20260521 /home/admin/Documents/lidar/fastlio_ws
  $(basename "$0") --default-frame-frequency 10.0 /path/to/park5 20260521 /home/admin/Documents/lidar/fastlio_ws
EOF
}

die() {
  echo "Error: $*" >&2
  exit 1
}

require_file() {
  local path="$1"
  local label="$2"

  [[ -f "${path}" ]] || die "${label} not found: ${path}"
}

require_dir() {
  local path="$1"
  local label="$2"

  [[ -d "${path}" ]] || die "${label} not found: ${path}"
}

main() {
  local default_frame_frequency=""
  local args=()

  while [[ $# -gt 0 ]]; do
    case "$1" in
      -h|--help)
        usage
        return 0
        ;;
      --default-frame-frequency)
        [[ $# -ge 2 ]] || die "--default-frame-frequency requires a value"
        default_frame_frequency="$2"
        shift 2
        ;;
      --default-frame-frequency=*)
        default_frame_frequency="${1#*=}"
        shift
        ;;
      --)
        shift
        args+=("$@")
        break
        ;;
      -*)
        die "unknown option: $1"
        ;;
      *)
        args+=("$1")
        shift
        ;;
    esac
  done

  [[ ${#args[@]} -ge 3 && ${#args[@]} -le 4 ]] || {
    usage >&2
    return 2
  }

  local input_seq="${args[0]}"
  local gps_date="${args[1]}"
  local lio_ws="${args[2]}"
  local output_bag_path="${args[3]:-${input_seq}/pandarxt32}"

  local pcap_path="${input_seq}/lidar.pcap"
  local correction_file_path="${input_seq}/lidar_calibration.csv"
  local imu_csv_path="${input_seq}/imu.csv"
  local imu_script="${lio_ws}/src/fast_lio/python/tersus/csv_imu_to_ros2bag.py"

  echo "[Params] input_seq=${input_seq}"
  echo "[Params] gps_date=${gps_date}"
  echo "[Params] lio_ws=${lio_ws}"
  echo "[Params] output_bag_path=${output_bag_path}"
  echo "[Params] default_frame_frequency=${default_frame_frequency:-<base_config>}"
  echo "[Params] pcap_path=${pcap_path}"
  echo "[Params] correction_file_path=${correction_file_path}"
  echo "[Params] imu_csv_path=${imu_csv_path}"
  echo "[Params] imu_script=${imu_script}"

  require_dir "${input_seq}" "input sequence folder"
  require_dir "${lio_ws}" "LIO workspace"
  require_file "${DEFAULT_SETUP_FILE}" "ROS setup file"
  require_file "${DEFAULT_BASE_CONFIG}" "Hesai base config"
  require_file "${DEFAULT_FIRETIMES_PATH}" "Hesai firetimes file"
  require_file "${pcap_path}" "PCAP file"
  require_file "${correction_file_path}" "LiDAR calibration file"
  require_file "${imu_csv_path}" "IMU CSV file"
  require_file "${imu_script}" "IMU conversion script"

  echo "[LiDAR] Writing bag to: ${output_bag_path}"
  # shellcheck source=/dev/null
  set +u
  source "${DEFAULT_SETUP_FILE}"
  set -u

  local launch_args=(
    "base_config_path:=${DEFAULT_BASE_CONFIG}"
    "pcap_path:=${pcap_path}"
    "correction_file_path:=${correction_file_path}"
    "output_bag_path:=${output_bag_path}"
  )
  if [[ -n "${default_frame_frequency}" ]]; then
    launch_args+=("default_frame_frequency:=${default_frame_frequency}")
  fi

  ros2 launch hesai_ros_driver pcap_to_ros2bag.py "${launch_args[@]}"
  # "firetimes_path:=${DEFAULT_FIRETIMES_PATH}" \

  echo "[IMU] Converting CSV to ROS 2 bag"
  python3 "${imu_script}" "${imu_csv_path}" --overwrite --gps-date "${gps_date}"
}

main "$@"
