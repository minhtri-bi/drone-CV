"""Build-only APT closure installer: exact versions and every archive SHA-256.

APT verifies signed repository metadata. No version fallback, distro upgrade,
unverified downloads, host writes or firmware rebuild is performed.
"""

import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess


CACHE = Path("/tmp/drone-cv-g1-debs")


def validate_packages(packages: list[dict]) -> list[str]:
    if not packages:
        raise ValueError("empty dependency lock")
    names = [entry["name"] for entry in packages]
    if len(set(names)) != len(names):
        raise ValueError("duplicate dependency names")
    for entry in packages:
        if not re.fullmatch(r"[a-z0-9][a-z0-9+.-]+", entry["name"]):
            raise ValueError("invalid package name")
        if not re.fullmatch(r"[a-zA-Z0-9.+:~_-]+", entry["version"]):
            raise ValueError("invalid package version")
        if not re.fullmatch(r"[a-f0-9]{64}", entry["sha256"]):
            raise ValueError("invalid archive hash")
        if entry["architecture"] not in ("amd64", "all"):
            raise ValueError("lock is only for linux/amd64")
    return [f"{entry['name']}={entry['version']}" for entry in packages]


def archive_identity(path: Path) -> tuple[str, str, str]:
    fields = subprocess.check_output(
        ["dpkg-deb", "-f", str(path), "Package", "Version", "Architecture"], text=True)
    values = dict(line.split(": ", 1) for line in fields.splitlines())
    return values["Package"], values["Version"], values["Architecture"]


def main() -> None:
    lock = json.loads(Path("/opt/drone_cv/g1/packages.lock.json").read_text())
    packages = lock["packages"]
    requests = validate_packages(packages)
    CACHE.joinpath("partial").mkdir(parents=True, exist_ok=True)
    subprocess.run(["apt-get", "update"], check=True)
    common = ["apt-get", "--yes", "--no-install-recommends", "--no-upgrade",
              "-o", f"Dir::Cache::archives={CACHE}"]
    subprocess.run([*common, "--download-only", "install", *requests], check=True)
    expected = {(p["name"], p["version"], p["architecture"]): p["sha256"] for p in packages}
    observed = set()
    for archive in CACHE.glob("*.deb"):
        identity = archive_identity(archive)
        if identity not in expected:
            raise ValueError(f"unlocked transitive dependency: {identity}")
        if hashlib.sha256(archive.read_bytes()).hexdigest() != expected[identity]:
            raise ValueError(f"archive hash mismatch: {identity}")
        observed.add(identity)
    if observed != set(expected):
        raise ValueError(f"missing locked archives: {set(expected)-observed}")
    subprocess.run([*common, "--no-download", "install", *requests], check=True)
    for package in packages:
        installed = subprocess.check_output(
            ["dpkg-query", "-W", "-f=${Version}", package["name"]], text=True)
        if installed != package["version"]:
            raise ValueError(f"installed version mismatch: {package['name']}")
    shutil.rmtree(CACHE)
    for child in Path("/var/lib/apt/lists").iterdir():
        if child.is_file():
            child.unlink()


if __name__ == "__main__":
    main()
