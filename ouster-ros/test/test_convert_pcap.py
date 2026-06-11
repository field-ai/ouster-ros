"""Unit tests for scripts/convert_pcap.py (pcap -> pointcloud orchestration)."""

from __future__ import annotations

import argparse
import importlib.util
import struct
import subprocess
import sys
from pathlib import Path

import pytest

# Load scripts/convert_pcap.py (installed as a program, not an importable package).
_SCRIPT = Path(__file__).resolve().parent.parent / "scripts" / "convert_pcap.py"
_spec = importlib.util.spec_from_file_location("convert_pcap", _SCRIPT)
assert _spec and _spec.loader
convert_pcap = importlib.util.module_from_spec(_spec)
sys.modules["convert_pcap"] = convert_pcap
_spec.loader.exec_module(convert_pcap)

PcapDecompressor = convert_pcap.PcapDecompressor
ProcessingError = convert_pcap.ProcessingError
ROBOT_NAME = "robot1"
TS = "2024-01-01-12-00-00"


def _write_pcap(path: Path, num_records: int = 50, payload_size: int = 1000) -> int:
    with open(path, "wb") as f:
        f.write(struct.pack("<IHHiIII", 0xA1B2C3D4, 2, 4, 0, 0, 65535, 1))
        for i in range(num_records):
            payload = bytes((i * 7 + j * 13 + 37) % 256 for j in range(payload_size))
            f.write(struct.pack("<IIII", i, 0, payload_size, payload_size))
            f.write(payload)
    return num_records


def _count_pcap_records(pcap_path: Path) -> int:
    file_size = pcap_path.stat().st_size
    offset = 24
    count = 0
    with open(pcap_path, "rb") as f:
        while offset + 16 <= file_size:
            f.seek(offset + 8)
            raw = f.read(4)
            if len(raw) < 4:
                break
            (incl_len,) = struct.unpack("<I", raw)
            if offset + 16 + incl_len > file_size:
                break
            offset += 16 + incl_len
            count += 1
    return count


def _default_args(data_dir: Path) -> argparse.Namespace:
    return convert_pcap.parse_args(["--data-dir", str(data_dir), "--robot-namespace", ROBOT_NAME])


def _make_session(data_dir: Path, timestamp: str = TS, with_pcap: bool = True, with_metadata: bool = True) -> Path:
    session = data_dir / "ouster" / f"{ROBOT_NAME}_ouster_{timestamp}"
    session.mkdir(parents=True)
    if with_pcap:
        (session / f"{ROBOT_NAME}_ouster_{timestamp}_0.pcap.zst").touch()
    if with_metadata:
        (data_dir / "ouster" / "ouster_metadata.json").touch()
    return session


@pytest.fixture
def mock_decompress(mocker):
    def _decompress(cmd, **_kwargs):
        Path(cmd[cmd.index("-o") + 1]).touch()

    return mocker.patch.object(convert_pcap.subprocess, "run", side_effect=_decompress)


class TestPcapDecompressor:
    def test_decompresses_pcap_zst(self, tmp_path: Path, mock_decompress) -> None:
        pcap_zst = tmp_path / "file_0.pcap.zst"
        pcap_zst.touch()
        result = PcapDecompressor().run(pcap_zst)
        assert result == tmp_path / "file_0.pcap"
        assert result.exists()

    def test_reuses_existing_decompressed_file(self, tmp_path: Path, mock_decompress) -> None:
        pcap_zst = tmp_path / "file_0.pcap.zst"
        pcap_zst.touch()
        (tmp_path / "file_0.pcap").touch()
        PcapDecompressor().run(pcap_zst)
        mock_decompress.assert_not_called()

    def test_raises_on_subprocess_failure(self, tmp_path: Path, mocker) -> None:
        pcap_zst = tmp_path / "file_0.pcap.zst"
        pcap_zst.write_bytes(b"not zstd")
        mocker.patch.object(
            convert_pcap.subprocess, "run", side_effect=subprocess.CalledProcessError(1, "zstd")
        )
        with pytest.raises(ProcessingError):
            PcapDecompressor().run(pcap_zst)

    def test_recovers_truncated_zst(self, tmp_path: Path, mocker) -> None:
        zstandard = pytest.importorskip("zstandard")
        pcap_orig = tmp_path / "original.pcap"
        num_records = _write_pcap(pcap_orig, num_records=200, payload_size=2000)
        zst_path = tmp_path / "file_0.pcap.zst"
        with open(pcap_orig, "rb") as fin, open(zst_path, "wb") as fout:
            zstandard.ZstdCompressor().copy_stream(fin, fout)
        pcap_orig.unlink()
        data = zst_path.read_bytes()
        zst_path.write_bytes(data[: len(data) * 3 // 4])  # truncate

        mocker.patch.object(
            convert_pcap.subprocess, "run", side_effect=subprocess.CalledProcessError(1, "zstd")
        )
        result = PcapDecompressor().run(zst_path)
        assert result.exists()
        assert 0 < _count_pcap_records(result) <= num_records

    def test_recovery_rejects_non_zst(self, tmp_path: Path, mocker) -> None:
        pytest.importorskip("zstandard")
        zst_path = tmp_path / "file_0.pcap.zst"
        zst_path.write_bytes(b"this is not zstd data at all")
        mocker.patch.object(
            convert_pcap.subprocess, "run", side_effect=subprocess.CalledProcessError(1, "zstd")
        )
        with pytest.raises(ProcessingError):
            PcapDecompressor().run(zst_path)


class TestDiscovery:
    def test_finds_and_sorts_sessions(self, tmp_path: Path) -> None:
        _make_session(tmp_path, "2024-01-01-12-00-01")
        _make_session(tmp_path, "2024-01-01-12-00-00")
        sessions = convert_pcap._find_session_dirs(tmp_path)
        assert [s.name for s in sessions] == [
            f"{ROBOT_NAME}_ouster_2024-01-01-12-00-00",
            f"{ROBOT_NAME}_ouster_2024-01-01-12-00-01",
        ]

    def test_orders_pcaps_by_index(self, tmp_path: Path) -> None:
        session = tmp_path / "s"
        session.mkdir()
        for idx in (2, 0, 10, 1):
            (session / f"{ROBOT_NAME}_ouster_{TS}_{idx}.pcap.zst").touch()
        ordered = convert_pcap._find_pcaps_in_session(session)
        assert [p.name.rsplit("_", 1)[-1] for p in ordered] == [
            "0.pcap.zst",
            "1.pcap.zst",
            "2.pcap.zst",
            "10.pcap.zst",
        ]

    def test_output_bag_dir_mapping(self, tmp_path: Path) -> None:
        session = tmp_path / "ouster" / f"{ROBOT_NAME}_ouster_{TS}"
        out = convert_pcap._output_bag_dir(tmp_path, session)
        assert out == tmp_path / "rosbag2" / f"{ROBOT_NAME}_lidar_pointcloud_{TS}"


class TestMetadataResolution:
    def test_prefers_metadata_beside_session(self, tmp_path: Path) -> None:
        session = _make_session(tmp_path)
        beside = session.parent / "ouster_metadata.json"  # same as dataset/ouster here
        assert convert_pcap._resolve_metadata(tmp_path, session) == beside

    def test_falls_back_to_legacy_log_location(self, tmp_path: Path) -> None:
        session = _make_session(tmp_path, with_metadata=False)
        legacy = tmp_path / "log" / "ouster_metadata.json"
        legacy.parent.mkdir(parents=True)
        legacy.touch()
        assert convert_pcap._resolve_metadata(tmp_path, session) == legacy

    def test_raises_when_metadata_missing(self, tmp_path: Path) -> None:
        session = _make_session(tmp_path, with_metadata=False)
        with pytest.raises(FileNotFoundError):
            convert_pcap._resolve_metadata(tmp_path, session)


class TestConvertSession:
    @pytest.fixture
    def mock_run(self, mocker):
        """Decompress -> touch the -o target; pcap_to_mcap -> create an .mcap in the bag."""

        def _run(cmd, **_kwargs):
            if "-o" in cmd:
                Path(cmd[cmd.index("-o") + 1]).touch()
            elif "--output-bag" in cmd:
                bag = Path(cmd[cmd.index("--output-bag") + 1])
                bag.mkdir(parents=True, exist_ok=True)
                (bag / "out_0.mcap").touch()

        mocker.patch.object(convert_pcap, "_resolve_pcap_to_mcap", return_value="pcap_to_mcap")
        return mocker.patch.object(convert_pcap.subprocess, "run", side_effect=_run)

    def test_skips_corrupt_trailing_pcap(self, tmp_path: Path, mocker) -> None:
        session = _make_session(tmp_path)
        (session / f"{ROBOT_NAME}_ouster_{TS}_1.pcap.zst").touch()
        num = len(list(session.glob("*.pcap.zst")))

        calls: list[str] = []

        def _run(cmd, **_kwargs):
            if "-o" in cmd:
                calls.append(cmd[cmd.index("-o") + 1])
                if len(calls) == num:
                    raise subprocess.CalledProcessError(1, "zstd")  # trailing file corrupt
                Path(cmd[cmd.index("-o") + 1]).touch()
            elif "--output-bag" in cmd:
                bag = Path(cmd[cmd.index("--output-bag") + 1])
                bag.mkdir(parents=True, exist_ok=True)
                (bag / "out_0.mcap").touch()

        mocker.patch.object(convert_pcap, "_resolve_pcap_to_mcap", return_value="pcap_to_mcap")
        run = mocker.patch.object(convert_pcap.subprocess, "run", side_effect=_run)

        convert_pcap._convert_session(_default_args(tmp_path), tmp_path, session)

        convert_cmd = next(c.args[0] for c in run.call_args_list if "--output-bag" in c.args[0])
        pcap_args = [convert_cmd[i + 1] for i, v in enumerate(convert_cmd) if v == "--pcap"]
        assert len(pcap_args) == 1  # only the first, non-corrupt pcap

    def test_raises_on_corrupt_non_trailing_pcap(self, tmp_path: Path, mocker) -> None:
        session = _make_session(tmp_path)
        (session / f"{ROBOT_NAME}_ouster_{TS}_1.pcap.zst").touch()
        mocker.patch.object(
            convert_pcap.subprocess, "run", side_effect=subprocess.CalledProcessError(1, "zstd")
        )
        with pytest.raises(ProcessingError):
            convert_pcap._convert_session(_default_args(tmp_path), tmp_path, session)

    def test_converts_multiple_sessions(self, tmp_path: Path, mock_run) -> None:
        _make_session(tmp_path, "2024-01-01-12-00-00")
        _make_session(tmp_path, "2024-01-01-12-00-01")
        assert convert_pcap.main(["--data-dir", str(tmp_path), "--robot-namespace", ROBOT_NAME]) == 0
        bags = sorted((tmp_path / "rosbag2").glob("*_lidar_pointcloud_*"))
        assert len(bags) == 2
