# scripts/cli/ensure_meshes.py — download robot meshes from Hugging Face on demand

from __future__ import annotations

import contextlib
import re
import shutil
import time
from pathlib import Path

from .common import PROJECT_ROOT, echo_error, echo_info, echo_success, echo_warning

_MAX_RETRIES = 3
_RETRY_BACKOFF_SECS = (2, 5, 10)

REPO_ID = "pndbotics/pnd_models_teleop"

MESH_BASE_DIR = PROJECT_ROOT / "src" / "visualization" / "adam_description" / "urdf"
MESH_INSTALL_DIR = (
    PROJECT_ROOT / "install" / "adam_description" / "share" / "adam_description" / "urdf"
)
_ADAM_DESC_DIR = PROJECT_ROOT / "src" / "visualization" / "adam_description"
_PKG_PREFIX = "package://adam_description/"

ADAM_TYPE_TO_MESH_DIRS: dict[str, list[str]] = {
    "adam_u": ["adam_u"],
    "adam_pro": ["adam_pro"],
    "adam_sp": ["adam_sp"],
}

ALL_MESH_DIRS = [
    "adam_u",
    "adam_pro",
    "adam_sp",
    "adam_inspire",
    "adam_lite",
]

_MESH_EXTENSIONS = {".stl", ".dae", ".obj"}


# ---------------------------------------------------------------------------
# URDF / MJCF mesh reference parsing
# ---------------------------------------------------------------------------


def _resolve_mesh_refs(desc_file: Path) -> set[Path]:
    """Parse a URDF or MJCF XML and return absolute paths of referenced mesh files."""
    text = desc_file.read_text(errors="replace")
    refs: set[Path] = set()

    if desc_file.suffix == ".urdf":
        for m in re.finditer(r'filename="([^"]+)"', text):
            fn = m.group(1)
            if fn.startswith(_PKG_PREFIX):
                p = _ADAM_DESC_DIR / fn[len(_PKG_PREFIX) :]
                if p.suffix.lower() in _MESH_EXTENSIONS:
                    refs.add(p)

    elif desc_file.suffix == ".xml":
        md = re.search(r'meshdir="([^"]+)"', text)
        meshdir = md.group(1) if md else ""
        base = desc_file.parent / meshdir
        for m in re.finditer(r'<mesh\s[^>]*file="([^"]+)"', text):
            p = base / m.group(1)
            if p.suffix.lower() in _MESH_EXTENSIONS:
                refs.add(p)

    return refs


def _collect_expected_meshes(adam_type: str, base_dir: Path) -> set[Path]:
    """Return all mesh file paths referenced by URDFs/XMLs for *adam_type*."""
    dirs = ADAM_TYPE_TO_MESH_DIRS.get(adam_type, [adam_type])
    refs: set[Path] = set()
    for d in dirs:
        vdir = base_dir / d
        if not vdir.exists():
            continue
        for f in (*vdir.rglob("*.urdf"), *vdir.rglob("*.xml")):
            refs |= _resolve_mesh_refs(f)
    return refs


def _get_missing_meshes(adam_type: str, base_dir: Path) -> set[Path]:
    """Return mesh files referenced in descriptors but absent on disk."""
    return {p for p in _collect_expected_meshes(adam_type, base_dir) if not p.exists()}


def _variant_dirs_to_download(adam_type: str, base_dir: Path) -> list[str]:
    """Determine which variant directories contain missing meshes."""
    dirs = ADAM_TYPE_TO_MESH_DIRS.get(adam_type, [adam_type])

    missing = _get_missing_meshes(adam_type, base_dir)
    if not missing:
        return [d for d in dirs if not (base_dir / d).exists()]

    to_dl: set[str] = set()
    for p in missing:
        with contextlib.suppress(ValueError):
            to_dl.add(p.relative_to(base_dir).parts[0])
    for d in dirs:
        if not (base_dir / d).exists():
            to_dl.add(d)
    return sorted(to_dl)


# ---------------------------------------------------------------------------
# Download helpers
# ---------------------------------------------------------------------------


def _download_variant(variant: str, dest: Path, revision: str = "main") -> bool:
    """Download a single variant's meshes from Hugging Face with automatic retry."""
    try:
        from huggingface_hub import snapshot_download
    except ImportError:
        echo_error(
            "huggingface-hub is not installed. "
            "Run: pip install huggingface-hub  or  uv pip install huggingface-hub"
        )
        return False

    echo_info(f"Downloading meshes for {variant} from Hugging Face ({REPO_ID})...")
    last_err: Exception | None = None
    for attempt in range(_MAX_RETRIES):
        try:
            snapshot_download(
                repo_id=REPO_ID,
                repo_type="dataset",
                local_dir=str(dest),
                allow_patterns=[f"{variant}/**"],
                revision=revision,
            )
            return True
        except Exception as e:
            last_err = e
            if attempt < _MAX_RETRIES - 1:
                wait = _RETRY_BACKOFF_SECS[min(attempt, len(_RETRY_BACKOFF_SECS) - 1)]
                echo_warning(f"Download interrupted: {e}")
                echo_info(f"Retrying in {wait}s (attempt {attempt + 2}/{_MAX_RETRIES})...")
                time.sleep(wait)

    echo_error(f"Failed to download {variant} after {_MAX_RETRIES} attempts: {last_err}")
    return False


# ---------------------------------------------------------------------------
# Sync to install directory
# ---------------------------------------------------------------------------


def _sync_to_install(variant: str, src_base: Path, install_base: Path) -> None:
    """Copy mesh files from the source tree to the colcon install tree."""
    src_dir = src_base / variant
    dst_dir = install_base / variant
    if not src_dir.is_dir() or not install_base.is_dir():
        return
    for src_file in src_dir.rglob("*"):
        if not src_file.is_file():
            continue
        rel = src_file.relative_to(src_dir)
        dst_file = dst_dir / rel
        if dst_file.exists() and dst_file.stat().st_size == src_file.stat().st_size:
            continue
        dst_file.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src_file, dst_file)


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------


def ensure_meshes(
    adam_type: str,
    *,
    force: bool = False,
    revision: str = "main",
    base_dir: Path | None = None,
) -> bool:
    """Ensure mesh files for *adam_type* are present locally.

    Parses URDF/XML descriptors, checks every referenced mesh file on disk,
    and downloads only the variant directories that have missing files.
    """
    dest = base_dir or MESH_BASE_DIR

    if force:
        to_download = list(ADAM_TYPE_TO_MESH_DIRS.get(adam_type, [adam_type]))
    else:
        to_download = _variant_dirs_to_download(adam_type, dest)

    if not to_download:
        _sync_all_variants(adam_type, dest)
        return True

    missing = _get_missing_meshes(adam_type, dest)
    if missing:
        echo_info(f"{len(missing)} mesh files missing for {adam_type}.")

    all_ok = True
    for d in to_download:
        if not _download_variant(d, dest, revision=revision):
            all_ok = False

    missing = _get_missing_meshes(adam_type, dest)
    if missing:
        echo_error(f"Failed to download meshs. {len(missing)} mesh files missing for {adam_type}.")
        return False

    _sync_all_variants(adam_type, dest)
    return all_ok


def _sync_all_variants(adam_type: str, src_base: Path) -> None:
    """Sync all variant directories for *adam_type* to the install tree."""
    if not MESH_INSTALL_DIR.is_dir():
        return
    dirs = ADAM_TYPE_TO_MESH_DIRS.get(adam_type, [adam_type])
    for d in dirs:
        _sync_to_install(d, src_base, MESH_INSTALL_DIR)


def ensure_meshes_all(
    *, force: bool = False, revision: str = "main", base_dir: Path | None = None
) -> bool:
    """Download meshes for every known variant."""
    dest = base_dir or MESH_BASE_DIR
    all_ok = True
    for d in ALL_MESH_DIRS:
        if not force and not _variant_dirs_to_download(d, dest):
            echo_info(f"{d}: all mesh files present, skipping.")
            continue
        if not _download_variant(d, dest, revision=revision):
            all_ok = False
    return all_ok


def list_variants(base_dir: Path | None = None) -> None:
    """Print download status for each variant based on URDF/XML mesh references."""
    dest = base_dir or MESH_BASE_DIR

    refs_by_dir: dict[str, set[Path]] = {d: set() for d in ALL_MESH_DIRS}
    for d in ALL_MESH_DIRS:
        vdir = dest / d
        if not vdir.exists():
            continue
        for f in (*vdir.rglob("*.urdf"), *vdir.rglob("*.xml")):
            for ref in _resolve_mesh_refs(f):
                try:
                    target = ref.relative_to(dest).parts[0]
                    if target in refs_by_dir:
                        refs_by_dir[target].add(ref)
                except ValueError:
                    pass

    for d in ALL_MESH_DIRS:
        refs = refs_by_dir[d]
        variant_dir = dest / d

        if not variant_dir.exists():
            echo_warning(f"  {d}: not downloaded")
            continue

        if not refs:
            echo_info(f"  {d}: no mesh references found in descriptors")
            continue

        present = sum(1 for p in refs if p.exists())
        total = len(refs)
        if present == total:
            echo_success(f"  {d}: ready ({total}/{total} mesh files)")
        elif present > 0:
            echo_warning(f"  {d}: incomplete ({present}/{total} mesh files)")
        else:
            echo_warning(f"  {d}: not downloaded (0/{total} mesh files expected)")


def adam_type_from_launch_stem(stem: str) -> str | None:
    """Extract adam_type from a launch file stem like 'mink-adam_u-webvr'."""
    if stem.startswith("test_"):
        return None
    m = re.match(r"^[^-]+-([^-]+)-[^-]+(?:-.+)*$", stem)
    return m.group(1) if m else None
