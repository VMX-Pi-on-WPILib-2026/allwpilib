#!/usr/bin/env python3
"""Compile/run VMX HAL host regression tests with SDK transport fakes and ASan/UBSan.

Requires C++20 GCC/Clang (CXX) and the SDK headers at vmxpi-hal/include, or
--sdk-include. No SDK binaries, Gradle installation, network or robot required.
"""
import argparse
import os
from pathlib import Path
import shlex
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--sdk-include', type=Path, default=root / 'vmxpi-hal/include')
args = parser.parse_args()
native = root / 'hal/src/main/native'
mau = native / 'mau'
llvm = root / 'wpiutil/src/main/native/thirdparty/llvm'
includes = [native / 'include', root / 'hal/src/generated/main/native/include',
            root / 'wpiutil/src/main/native/include', llvm / 'include',
            args.sdk_include, mau, mau / 'DriveStation/include']
sources = [Path(__file__).with_name('regression.cpp'),
           mau / 'DriveStation/DriverComms.cpp', mau / 'DriveStation/MauDriveData.cpp',
           *[mau / (name + '.cpp') for name in ['I2C', 'SPI', 'PWM', 'AnalogInput', 'AnalogTrigger']],
           native / 'cpp/handles/HandlesInternal.cpp', llvm / 'cpp/llvm/SmallVector.cpp']
with tempfile.TemporaryDirectory(prefix='vmx-hal-tests-') as work:
    binary = str(Path(work) / 'regression')
    cmd = [*shlex.split(os.environ.get('CXX', 'c++')), '-std=c++20', '-pthread', '-g', '-O1',
           '-DLLVM_ENABLE_EXCEPTIONS', '-ffunction-sections', '-fdata-sections',
           '-fsanitize=address,undefined', '-fno-omit-frame-pointer',
           *['-I' + str(p) for p in includes], *map(str, sources),
           '-Wl,--gc-sections,--wrap=open,--wrap=ioctl,--wrap=close', '-o', binary]
    subprocess.run(cmd, check=True, timeout=180)
    subprocess.run([binary], check=True, timeout=30,
                   env={**os.environ, 'UBSAN_OPTIONS': 'halt_on_error=1', 'ASAN_OPTIONS': 'detect_leaks=1'})

    buddy_binary = str(Path(work) / 'buddy-regression')
    buddy_cmd = [*shlex.split(os.environ.get('CXX', 'c++')), '-std=c++20', '-g', '-O1',
                 '-fsanitize=address,undefined', '-fno-omit-frame-pointer',
                 '-I' + str(mau / 'DriveStation/include'),
                 str(Path(__file__).with_name('buddy_pool_regression.cpp')),
                 str(mau / 'DriveStation/buddy.cpp'), '-o', buddy_binary]
    subprocess.run(buddy_cmd, check=True, timeout=60)
    subprocess.run([buddy_binary], check=True, timeout=30,
                   env={**os.environ, 'UBSAN_OPTIONS': 'halt_on_error=1', 'ASAN_OPTIONS': 'detect_leaks=1'})
print('PASS BuddyPool: randomized split/merge allocation stress')
