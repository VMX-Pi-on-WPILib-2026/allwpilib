#!/usr/bin/env python3
"""Builds the VMX-pi HAL Maven repository used by the VMX GradleRIO fork.

The repository holds edu.wpi.first.hal:hal-cpp and hal-java at a VMX version
(for example 2026.2.2-vmx.2):

  - linuxarm64 binaries (and the headers/sources zips): this tree's VMX-pi
    HAL (hal/src/main/native/mau), built with Gradle;
  - every other platform: the upstream WPILib binaries of the same release,
    so desktop simulation and other targets keep working unchanged.

libvmxpi_hal_cpp.so is NOT included: libwpiHal.so only references it, and the
robot provides it (vmxpi-hal package). Do not add it here.

Usage (from the allwpilib root, with a JDK that runs Gradle 8.14, e.g. 21):
  python3 vmx/build_vmx_hal_maven.py --vmx-version 2026.2.2-vmx.2
"""

import argparse
import os
import shutil
import subprocess
import sys
import urllib.request
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
UPSTREAM_URL = "https://frcmaven.wpi.edu/artifactory/api/download/release"
GROUP_PATH = "edu/wpi/first/hal"

# Classifiers of hal-cpp that come from upstream (everything but linuxarm64).
UPSTREAM_CLASSIFIERS = [
    f"{platform}{suffix}"
    for platform in ["linuxx86-64", "linuxarm32", "linuxathena", "osxuniversal",
                     "windowsx86-64", "windowsarm64"]
    for suffix in ["", "debug", "static", "staticdebug"]
]


def run_gradle(vmx_version: str, built_repo: Path, robot_libraries: bool) -> None:
    cmd = [
        str(ROOT / "gradlew"), ":hal:publishToMavenLocal",
        "-Ponlylinuxarm64", f"-PpublishVersion={vmx_version}", "-PskipJavaFormat",
        f"-Dmaven.repo.local={built_repo}", "--no-scan",
    ]
    if robot_libraries:
        cmd[2:2] = [":wpilibc:publishToMavenLocal", ":wpilibj:publishToMavenLocal"]
    print("+", " ".join(cmd), flush=True)
    subprocess.run(cmd, cwd=ROOT, check=True)


def download(url: str, dest: Path, cache: Path) -> None:
    cached = cache / url.rsplit("/", 1)[1]
    if not cached.exists():
        print(f"downloading {url}", flush=True)
        tmp = cached.with_suffix(cached.suffix + ".part")
        with urllib.request.urlopen(url, timeout=120) as response, open(tmp, "wb") as out:
            shutil.copyfileobj(response, out)
        tmp.rename(cached)
    if not zipfile.is_zipfile(cached):
        raise ValueError(f"Expected a ZIP archive: {cached}")
    shutil.copyfile(cached, dest)


def write_metadata(artifact_dir: Path, artifact: str, version: str) -> None:
    (artifact_dir / "maven-metadata.xml").write_text(
        '<?xml version="1.0" encoding="UTF-8"?>\n'
        "<metadata>\n"
        "  <groupId>edu.wpi.first.hal</groupId>\n"
        f"  <artifactId>{artifact}</artifactId>\n"
        "  <versioning>\n"
        f"    <release>{version}</release>\n"
        f"    <versions>\n      <version>{version}</version>\n    </versions>\n"
        "  </versioning>\n"
        "</metadata>\n")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--vmx-version", required=True, help="e.g. 2026.2.2-vmx.2")
    parser.add_argument("--upstream-version", help="WPILib release for the other platforms "
                        "(default: the VMX version without its -vmx.N suffix)")
    parser.add_argument("--output", type=Path,
                        default=Path.home() / "wpilib" / "2026" / "vmx" / "maven",
                        help="Maven repository to write into (default: ~/wpilib/2026/vmx/maven)")
    parser.add_argument("--work", type=Path, default=ROOT / "build" / "vmx-maven",
                        help="scratch directory (Gradle output and download cache)")
    parser.add_argument("--with-robot-libraries", action="store_true",
                        help="Also build and include custom wpilibc/wpilibj at the same version")
    parser.add_argument("--built-repo", type=Path,
                        help="Reuse this Gradle publication repository with --skip-build")
    parser.add_argument("--skip-build", action="store_true",
                        help="reuse the previous Gradle output in --work")
    args = parser.parse_args()

    vmx_version = args.vmx_version
    upstream_version = args.upstream_version or vmx_version.split("-vmx")[0]
    built_repo = args.built_repo or args.work / "built"
    cache = args.work / "cache"
    cache.mkdir(parents=True, exist_ok=True)

    if not args.skip_build:
        run_gradle(vmx_version, built_repo, args.with_robot_libraries)

    for artifact in ["hal-cpp", "hal-java"]:
        src = built_repo / GROUP_PATH / artifact / vmx_version
        if not src.is_dir():
            sys.exit(f"missing Gradle output {src}; run without --skip-build")
        dst = args.output / GROUP_PATH / artifact / vmx_version
        dst.mkdir(parents=True, exist_ok=True)
        for f in src.iterdir():
            shutil.copyfile(f, dst / f.name)
        write_metadata(dst.parent, artifact, vmx_version)

    if args.with_robot_libraries:
        for group, artifact in [("wpilibc", "wpilibc-cpp"), ("wpilibj", "wpilibj-java")]:
            relative = Path("edu/wpi/first") / group / artifact / vmx_version
            src = built_repo / relative
            if not src.is_dir():
                sys.exit(f"missing custom robot library {src}")
            shutil.copytree(src, args.output / relative, dirs_exist_ok=True)

    cpp_dst = args.output / GROUP_PATH / "hal-cpp" / vmx_version
    base = f"{UPSTREAM_URL}/{GROUP_PATH}/hal-cpp/{upstream_version}"
    for classifier in UPSTREAM_CLASSIFIERS:
        download(f"{base}/hal-cpp-{upstream_version}-{classifier}.zip",
                 cpp_dst / f"hal-cpp-{vmx_version}-{classifier}.zip", cache)

    print(f"VMX HAL {vmx_version} (other platforms from WPILib {upstream_version}) -> {args.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
