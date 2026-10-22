"""Path helpers for the legacy first-run tools; no data paths depend on cwd."""
from pathlib import Path
import os


def writable_output(path: Path) -> Path:
    result = path.resolve()
    data_root = Path(__file__).resolve().parents[2] / "data"
    if result.is_relative_to(data_root.resolve()):
        raise ValueError("Do not regenerate inside tracked data/. Choose a new output/ or external work directory.")
    return result


def relative_reference(target: Path, parent: Path) -> str:
    # Cross-volume references cannot be relative on Windows. The new snapshot
    # resolver uses artifact IDs instead; generation needs a workspace per volume.
    try:
        return Path(os.path.relpath(target.resolve(), parent.resolve())).as_posix()
    except ValueError as error:
        raise ValueError("Keep input/output on the same volume; use shared_inputs.py --workspace first") from error
