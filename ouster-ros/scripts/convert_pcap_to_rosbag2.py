#!/usr/bin/env python3
"""Convert Ouster .pcap.zst captures in a dataset directory to pointcloud rosbags.

A recorded dataset stores raw Ouster UDP packets as rotated, zstd-compressed pcap
files (see core_logging/tcpdump_recorder.py), laid out as::

    <data_dir>/ouster/ouster_metadata.json
    <data_dir>/ouster/<robot>_ouster_<timestamp>/<robot>_ouster_<timestamp>_<idx>.pcap.zst

This script discovers every such session under ``data_dir``, decompresses its pcap
files (recovering a truncated trailing file when recording was killed mid-capture),
and runs the ``pcap_to_mcap`` binary once per session to produce::

    <data_dir>/rosbag2/<robot>_lidar_pointcloud_<timestamp>/

It is the single source of truth for pcap->pointcloud conversion: both the
``convert_pcap.launch.py`` launch file and the odometry-rerun automation drive it.
"""

from __future__ import annotations

import argparse
import logging
import re
import shutil
import struct
import subprocess
import sys
from pathlib import Path

log = logging.getLogger("convert_pcap")

_PCAP_GLOBAL_HEADER_SIZE = 24
_PCAP_RECORD_HEADER_SIZE = 16
_PCAP_MAGIC_LE = 0xA1B2C3D4
_PCAP_MAGIC_BE = 0xD4C3B2A1
_UINT32_SIZE = 4

# Session dir glob: <robot>_ouster_<YYYY>-... (mirrors tcpdump_recorder SESSION_DIR_PATTERN)
_SESSION_GLOB = "ouster/*_ouster_[0-9][0-9][0-9][0-9]-*"
_PCAP_IDX_RE = re.compile(r"_(\d+)\.pcap\.zst$")


class ProcessingError(RuntimeError):
    """Raised when a pcap file cannot be decompressed or converted."""


class PcapDecompressor:
    """Decompresses .pcap.zst files to .pcap using zstd.

    Falls back to streaming decompression for truncated zstd files, recovering
    all complete zstd blocks and aligning to pcap record boundaries.
    """

    def run(self, pcap_zst: Path) -> Path:
        pcap_path = pcap_zst.with_suffix("")  # .pcap.zst -> .pcap
        if pcap_path.exists():
            log.info("Reusing existing decompressed pcap: %s", pcap_path.name)
            return pcap_path

        # Write to .tmp then rename - avoids a partial file being reused on retry
        tmp_path = pcap_path.with_name(pcap_path.name + ".tmp")
        log.info("Decompressing %s -> %s", pcap_zst, pcap_path)
        try:
            subprocess.run(
                ["zstd", "-d", "-f", "-k", "-o", str(tmp_path), str(pcap_zst)],
                check=True,
            )
        except subprocess.CalledProcessError as e:
            tmp_path.unlink(missing_ok=True)
            if not self._recover_truncated_zst(pcap_zst, tmp_path):
                raise ProcessingError(f"Failed to decompress {pcap_zst}: {e}") from e

        if not tmp_path.exists():
            raise ProcessingError(f"zstd reported success but {tmp_path} is missing")
        tmp_path.rename(pcap_path)
        return pcap_path

    def _recover_truncated_zst(self, zst_path: Path, out_path: Path) -> bool:
        """Stream-decompress a truncated zstd file, keeping all complete zstd blocks.

        The output is truncated to the last complete pcap record boundary so
        downstream tools (pcap_to_mcap) never see a partial packet record.

        Returns True if a usable pcap was recovered, False otherwise.
        """
        try:
            import zstandard
        except ImportError:
            log.warning(
                "python3-zstandard not available; cannot recover truncated %s",
                zst_path.name,
            )
            return False

        dctx = zstandard.ZstdDecompressor()
        bytes_written = 0
        try:
            with open(zst_path, "rb") as fin, open(out_path, "wb") as fout:
                reader = dctx.stream_reader(fin, read_across_frames=True)
                while chunk := reader.read(262_144):
                    fout.write(chunk)
                    bytes_written += len(chunk)
        except zstandard.ZstdError:
            pass

        if bytes_written < _PCAP_GLOBAL_HEADER_SIZE:
            out_path.unlink(missing_ok=True)
            return False

        valid_end = self._find_last_complete_pcap_record(out_path)
        if valid_end is None:
            out_path.unlink(missing_ok=True)
            return False

        if valid_end < bytes_written:
            with open(out_path, "r+b") as f:
                f.truncate(valid_end)

        log.warning(
            "Recovered %d bytes from truncated %s (compressed size: %d bytes)",
            valid_end,
            zst_path.name,
            zst_path.stat().st_size,
        )
        return True

    @staticmethod
    def _find_last_complete_pcap_record(pcap_path: Path) -> int | None:
        """Return the byte offset just past the last complete pcap record.

        Returns None if the file doesn't contain a valid pcap global header or
        has zero complete records.
        """
        file_size = pcap_path.stat().st_size
        if file_size < _PCAP_GLOBAL_HEADER_SIZE:
            return None

        with open(pcap_path, "rb") as f:
            magic = struct.unpack("<I", f.read(_UINT32_SIZE))[0]
            if magic == _PCAP_MAGIC_LE:
                byte_order = "<"
            elif magic == _PCAP_MAGIC_BE:
                byte_order = ">"
            else:
                return None

            offset = _PCAP_GLOBAL_HEADER_SIZE
            while offset + _PCAP_RECORD_HEADER_SIZE <= file_size:
                f.seek(offset + 8)
                raw = f.read(_UINT32_SIZE)
                if len(raw) < _UINT32_SIZE:
                    break
                (incl_len,) = struct.unpack(f"{byte_order}I", raw)
                record_end = offset + _PCAP_RECORD_HEADER_SIZE + incl_len
                if record_end > file_size:
                    break
                offset = record_end

        return offset if offset > _PCAP_GLOBAL_HEADER_SIZE else None


def _resolve_pcap_to_mcap() -> str:
    """Resolve the path to the installed pcap_to_mcap binary.

    Exits with a clear message if the binary is missing — it is only built when
    ouster_ros is compiled with -DBUILD_PCAP=ON, which is off in default dev builds.
    """
    try:
        from ament_index_python.packages import get_package_prefix

        candidate = Path(get_package_prefix("ouster_ros")) / "lib" / "ouster_ros" / "pcap_to_mcap"
        if candidate.is_file():
            return str(candidate)
    except Exception:
        # Fall through to the actionable error below.
        pass

    binary = shutil.which("pcap_to_mcap")
    if binary:
        return binary

    log.error(
        "pcap_to_mcap binary not found. Rebuild ouster_ros with -DBUILD_PCAP=ON, e.g.:\n"
        "  colcon build --packages-up-to ouster_ros --cmake-args -DBUILD_PCAP=ON"
    )
    sys.exit(2)


def _find_session_dirs(data_dir: Path) -> list[Path]:
    """Find ouster session directories under the dataset directory."""
    sessions = [d for d in data_dir.rglob(_SESSION_GLOB) if d.is_dir()]
    return sorted(sessions, key=lambda p: p.name)


def _find_pcaps_in_session(session_dir: Path) -> list[Path]:
    """Find .pcap.zst files in a session directory, sorted by index suffix (_0, _1, ...)."""

    def _idx(p: Path) -> int:
        m = _PCAP_IDX_RE.search(p.name)
        return int(m.group(1)) if m else 0

    return sorted(session_dir.glob("*.pcap.zst"), key=_idx)


def _resolve_metadata(data_dir: Path, session_dir: Path) -> Path:
    """Locate ouster_metadata.json for a session.

    Prefers the metadata sitting beside the session (correct for nested mission
    directories), then falls back to the dataset-root ouster/, then the legacy
    pre-10.2 log/ location.
    """
    candidates = [
        session_dir.parent / "ouster_metadata.json",  # <...>/ouster/ouster_metadata.json
        data_dir / "ouster" / "ouster_metadata.json",
        data_dir / "log" / "ouster_metadata.json",  # legacy (pre-10.2)
    ]
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    raise FileNotFoundError(
        f"No ouster_metadata.json found for session '{session_dir.name}'. "
        f"Looked in: {', '.join(str(c) for c in candidates)}"
    )


def _output_bag_dir(data_dir: Path, session_dir: Path) -> Path:
    """Map a session dir to its pointcloud bag output dir.

    Input:  ouster/<robot>_ouster_<timestamp>/
    Output: rosbag2/<robot>_lidar_pointcloud_<timestamp>/
    """
    rosbag2_dir = data_dir / "rosbag2"
    rosbag2_dir.mkdir(parents=True, exist_ok=True)
    bag_name = session_dir.name.replace("_ouster_", "_lidar_pointcloud_")
    return rosbag2_dir / bag_name


def _build_convert_cmd(args: argparse.Namespace, pcaps: list[Path], metadata: Path, out_bag: Path) -> list[str]:
    cmd = [
        _resolve_pcap_to_mcap(),
        "--metadata", str(metadata),
        "--output-bag", str(out_bag),
        "--robot-namespace", args.robot_namespace,
        "--point-type", args.point_type,
        "--organized", "1" if args.organized else "0",
        "--destagger", "1" if args.destagger else "0",
        "--min-range", str(args.min_range),
        "--max-range", str(args.max_range),
        "--v-reduction", str(args.v_reduction),
        "--timestamp-mode", args.timestamp_mode,
        "--ptp-utc-tai-offset", str(args.ptp_utc_tai_offset),
    ]
    if args.mask_path:
        cmd += ["--mask-path", args.mask_path]
    for p in pcaps:
        cmd += ["--pcap", str(p)]
    return cmd


def _convert_session(args: argparse.Namespace, data_dir: Path, session_dir: Path) -> None:
    pcap_zst_files = _find_pcaps_in_session(session_dir)
    if not pcap_zst_files:
        raise ProcessingError(f"No pcap.zst files found in session '{session_dir}'.")

    metadata = _resolve_metadata(data_dir, session_dir)
    out_bag = _output_bag_dir(data_dir, session_dir)

    decompressor = PcapDecompressor()
    log.info("Decompressing %d pcap file(s) for %s", len(pcap_zst_files), session_dir.name)
    pcaps: list[Path] = []
    for i, p in enumerate(pcap_zst_files):
        try:
            pcaps.append(decompressor.run(p))
        except ProcessingError:
            if i == len(pcap_zst_files) - 1 and pcaps:
                log.warning("Skipping corrupt trailing pcap %s - likely truncated during recording", p.name)
            else:
                raise

    if out_bag.exists():
        log.info("Removing existing output bag before conversion: %s", out_bag)
        shutil.rmtree(out_bag)

    log.info("Converting %d pcap(s) -> %s", len(pcaps), out_bag.name)
    subprocess.run(_build_convert_cmd(args, pcaps, metadata, out_bag), check=True)

    if not any(out_bag.glob("*.mcap")):
        raise ProcessingError(f"No pointcloud bag generated for session '{session_dir.name}'.")


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--data-dir", required=True, type=Path, help="Dataset directory containing ouster/ captures")
    p.add_argument("--robot-namespace", required=True, help="Robot name for topic namespacing and frame ids")
    # pcap_to_mcap pass-through params (defaults match the pcap_to_mcap C++ defaults)
    p.add_argument("--point-type", default="original")
    p.add_argument("--organized", type=_str2bool, default=True)
    p.add_argument("--destagger", type=_str2bool, default=True)
    p.add_argument("--min-range", type=float, default=0.0)
    p.add_argument("--max-range", type=float, default=1000.0)
    p.add_argument("--v-reduction", type=int, default=1)
    p.add_argument("--mask-path", default="")
    p.add_argument("--timestamp-mode", default="TIME_FROM_PTP_1588")
    p.add_argument("--ptp-utc-tai-offset", type=int, default=0)
    return p.parse_args(argv)


def _str2bool(value: str) -> bool:
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def main(argv: list[str] | None = None) -> int:
    logging.basicConfig(level=logging.INFO, format="[convert_pcap] %(levelname)s: %(message)s")
    args = parse_args(argv)

    data_dir: Path = args.data_dir
    if not data_dir.is_dir():
        log.error("data_dir does not exist or is not a directory: %s", data_dir)
        return 1

    sessions = _find_session_dirs(data_dir)
    if not sessions:
        log.error("No ouster session directories (ouster/<robot>_ouster_<ts>/) found under %s", data_dir)
        return 1

    log.info("Found %d ouster session(s) under %s", len(sessions), data_dir)
    failures = 0
    for session in sessions:
        try:
            _convert_session(args, data_dir, session)
        except (ProcessingError, FileNotFoundError, subprocess.CalledProcessError) as e:
            failures += 1
            log.error("Failed to convert session '%s': %s", session.name, e)

    if failures:
        log.error("%d/%d session(s) failed to convert", failures, len(sessions))
        return 1
    log.info("Converted %d session(s) successfully", len(sessions))
    return 0


if __name__ == "__main__":
    sys.exit(main())
