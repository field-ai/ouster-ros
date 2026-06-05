#!/usr/bin/env bash
#
# pcap_to_mcap.sh — convert an Ouster tcpdump PCAP capture session into a
# pointcloud mcap rosbag using ouster_ros' `pcap_to_mcap` tool.
#
# A capture session (produced by core_logging's tcpdump_recorder) looks like:
#
#   <...>/ouster/
#   ├── ouster_metadata.json
#   └── <robot>_ouster_<YYYY-MM-DD-HH-MM-SS>/
#       ├── <robot>_ouster_<...>_0.pcap.zst
#       ├── <robot>_ouster_<...>_1.pcap.zst
#       └── ...
#
# This script decompresses the .pcap.zst files (in index order), then runs
# `pcap_to_mcap` over them to write a single pointcloud bag.
#
# `pcap_to_mcap` only ships when ouster_ros is built with -DBUILD_PCAP=ON.
#
# Usage:
#   pcap_to_mcap.sh [options] <pcap_session_dir>
#
# Options:
#   -n <namespace>   Robot namespace        (default: derived from session dir name)
#   -m <metadata>    ouster_metadata.json   (default: <session>/../ouster_metadata.json)
#   -o <output_bag>  Output bag directory   (default: <session>_pointcloud)
#   -t <point_type>  Point type             (default: original; RGB: color_point | xyzrgb)
#   -w <workspace>   ROS2 workspace to source if pcap_to_mcap isn't on the path
#                    (default: $ROS_WORKSPACE or /home/fieldai/Projects/fieldai-monorepo/ros2_ws)
#   -f               Overwrite output bag dir if it already exists
#   -k               Keep decompressed .pcap files (default: remove them on success)
#   -h               Show this help
#
# Any arguments after `--` are passed straight through to pcap_to_mcap, e.g.:
#   pcap_to_mcap.sh -t color_point /data/.../falcon38_ouster_... -- \
#       --min-range 0.5 --max-range 120 --organized 1 --destagger 1 --v-reduction 2
#
set -euo pipefail

# Print the leading comment block (everything after the shebang up to the first
# non-comment line), stripping the leading "# ".
usage() { awk 'NR>1 && /^#/{sub(/^# ?/,"");print;next} NR>1{exit}' "$0"; exit "${1:-0}"; }

NAMESPACE="" METADATA="" OUTPUT="" POINT_TYPE="original"
WORKSPACE="${ROS_WORKSPACE:-/home/fieldai/Projects/fieldai-monorepo/ros2_ws}"
FORCE=0 KEEP=0

while getopts ":n:m:o:t:w:fkh" opt; do
  case "$opt" in
    n) NAMESPACE="$OPTARG" ;;
    m) METADATA="$OPTARG" ;;
    o) OUTPUT="$OPTARG" ;;
    t) POINT_TYPE="$OPTARG" ;;
    w) WORKSPACE="$OPTARG" ;;
    f) FORCE=1 ;;
    k) KEEP=1 ;;
    h) usage 0 ;;
    \?) echo "Unknown option: -$OPTARG" >&2; usage 1 ;;
    :) echo "Option -$OPTARG requires an argument" >&2; usage 1 ;;
  esac
done
shift $((OPTIND - 1))

# Positional: the pcap session directory.
[[ $# -ge 1 ]] || { echo "error: missing <pcap_session_dir>" >&2; usage 1; }
SESSION="${1%/}"; shift

# An optional `--` separates the session dir from passthrough args; consume it
# so it isn't forwarded (pcap_to_mcap would treat it as end-of-options and
# silently ignore everything after it). Remaining args go straight to the tool.
[[ "${1:-}" == "--" ]] && shift
EXTRA_ARGS=("$@")

die() { echo "error: $*" >&2; exit 1; }

[[ -d "$SESSION" ]] || die "session dir not found: $SESSION"

BASE="$(basename "$SESSION")"
PARENT="$(dirname "$SESSION")"

# Derive defaults from the session dir name: <robot>_ouster_<timestamp>
[[ "$BASE" == *_ouster_* ]] || die "session dir name '$BASE' is not <robot>_ouster_<timestamp>; pass -n and -o explicitly"
ROBOT="${BASE%%_ouster_*}"
STAMP="${BASE#*_ouster_}"

[[ -n "$NAMESPACE" ]] || NAMESPACE="$ROBOT"
[[ -n "$METADATA"  ]] || METADATA="$PARENT/ouster_metadata.json"
[[ -n "$OUTPUT"    ]] || OUTPUT="$PARENT/${ROBOT}_lidar_pointcloud_${STAMP}"

[[ -f "$METADATA" ]] || die "metadata not found: $METADATA (override with -m)"

if [[ -e "$OUTPUT" ]]; then
  [[ "$FORCE" -eq 1 ]] || die "output bag already exists: $OUTPUT (use -f to overwrite)"
  echo ">> removing existing output: $OUTPUT"
  rm -rf "$OUTPUT"
fi

# Make pcap_to_mcap available (source the workspace only if it isn't already).
if ! command -v ros2 >/dev/null 2>&1 || ! ros2 pkg executables ouster_ros 2>/dev/null | grep -q pcap_to_mcap; then
  if [[ -f "$WORKSPACE/install/setup.bash" ]]; then
    echo ">> sourcing $WORKSPACE/install/setup.bash"
    # shellcheck disable=SC1091
    source "$WORKSPACE/install/setup.bash"
  fi
fi
ros2 pkg executables ouster_ros 2>/dev/null | grep -q pcap_to_mcap \
  || die "pcap_to_mcap not found. Rebuild with: colcon build --packages-select ouster_ros --cmake-args -DBUILD_PCAP=ON"

command -v zstd >/dev/null 2>&1 || die "zstd not found on PATH (needed to decompress .pcap.zst)"

# Collect .pcap.zst in index order (_0, _1, ..., _10); version sort keeps the
# numeric suffix ordered correctly since the filename prefix is constant.
mapfile -t ZSTS < <(find "$SESSION" -maxdepth 1 -name '*.pcap.zst' | sort -V)
[[ "${#ZSTS[@]}" -gt 0 ]] || die "no .pcap.zst files in $SESSION"

echo ">> session : $SESSION"
echo ">> robot   : $NAMESPACE"
echo ">> metadata: $METADATA"
echo ">> output  : $OUTPUT"
echo ">> type    : $POINT_TYPE"
echo ">> pcaps   : ${#ZSTS[@]} file(s)"

# Decompress each .pcap.zst -> .pcap (skip if already present). A truncated
# *last* file is tolerated and skipped, mirroring the recorder/pipeline, since
# the final capture file is the one most likely cut off at shutdown.
PCAPS=() DECOMPRESSED=()
last_idx=$(( ${#ZSTS[@]} - 1 ))
for i in "${!ZSTS[@]}"; do
  zst="${ZSTS[$i]}"
  pcap="${zst%.zst}"
  if [[ -f "$pcap" ]]; then
    echo ">> reuse  : $(basename "$pcap")"
  else
    echo ">> unzip  : $(basename "$zst")"
    if ! zstd -d -f -k -o "$pcap" "$zst" 2>/dev/null; then
      if [[ "$i" -eq "$last_idx" && "${#PCAPS[@]}" -gt 0 ]]; then
        echo ">> WARN   : $(basename "$zst") is corrupt/truncated — skipping (likely cut off at recording stop)" >&2
        rm -f "$pcap"
        continue
      fi
      die "failed to decompress $zst"
    fi
    DECOMPRESSED+=("$pcap")
  fi
  PCAPS+=("$pcap")
done
[[ "${#PCAPS[@]}" -gt 0 ]] || die "no usable pcap files after decompression"

# Remove decompressed files on exit unless -k was given.
cleanup() {
  if [[ "$KEEP" -eq 0 && "${#DECOMPRESSED[@]}" -gt 0 ]]; then
    echo ">> cleaning up ${#DECOMPRESSED[@]} decompressed .pcap file(s)"
    rm -f "${DECOMPRESSED[@]}"
  fi
}
trap cleanup EXIT

# Build the --pcap argument list.
PCAP_ARGS=()
for p in "${PCAPS[@]}"; do PCAP_ARGS+=(--pcap "$p"); done

echo ">> converting -> $OUTPUT"
ros2 run ouster_ros pcap_to_mcap \
  --metadata "$METADATA" \
  --output-bag "$OUTPUT" \
  --robot-namespace "$NAMESPACE" \
  --point-type "$POINT_TYPE" \
  "${PCAP_ARGS[@]}" \
  "${EXTRA_ARGS[@]}"

echo ">> done: $OUTPUT"
