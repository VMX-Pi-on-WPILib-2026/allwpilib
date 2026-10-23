#!/usr/bin/env python3
"""Test the actual VMX IMU HAL, C++ class and Java/JNI with a fake VMX SDK.

Needs C++20, JDK17+ and locally built WPILib jars. No board or VMX SDK binaries.
The real SDK ABI is separately checked by the linuxarm64 Gradle build.
"""
import os
from pathlib import Path
import shlex
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[3]
TESTS = Path(__file__).parent
cxx = shlex.split(os.environ.get('CXX', 'c++'))
incs = [ROOT / p for p in (
    'hal/src/main/native/include', 'hal/src/generated/main/native/include',
    'hal/src/main/native/mau', 'wpilibc/src/main/native/include',
    'wpilibc/src/generated/main/native/include', 'wpimath/src/main/native/include',
    'wpiutil/src/main/native/include', 'wpiutil/src/main/native/thirdparty/llvm/include',
    'wpiutil/src/main/native/thirdparty/fmtlib/include',
    'wpiutil/src/main/native/thirdparty/json/include',
    'wpiutil/src/main/native/thirdparty/nanopb/include',
    'wpimath/src/generated/main/native/cpp',
    'wpimath/src/main/native/thirdparty/units/include',
    'wpimath/src/main/native/thirdparty/gcem/include',
    'wpimath/src/main/native/thirdparty/eigen/include',
)]
backend = ROOT / 'hal/src/main/native/mau/VMXIMU.cpp'
with tempfile.TemporaryDirectory(prefix='vmx-imu-tests-') as tmp:
    work = Path(tmp)
    (work / 'AHRS.h').write_bytes((TESTS / 'FakeAHRS.h').read_bytes())
    base = [*cxx, '-std=c++20', '-pthread', '-g', '-O1', '-Wall', '-Wextra',
            '-I' + str(work), *['-I' + str(p) for p in incs]]
    binary = work / 'regression'
    subprocess.run([*base, '-fsanitize=address,undefined', '-fno-omit-frame-pointer',
                    str(TESTS / 'regression.cpp'), str(backend),
                    str(ROOT / 'wpilibc/src/main/native/cpp/VMXIMU.cpp'),
                    '-o', str(binary)], check=True, timeout=120)
    subprocess.run([str(binary)], check=True, timeout=30,
                   env={**os.environ, 'ASAN_OPTIONS': 'detect_leaks=1',
                        'UBSAN_OPTIONS': 'halt_on_error=1'})
    print('PASS VMXIMU C++/HAL: errors, units, timestamps, resets, shared lifetime, buffers', flush=True)
    for platform in ['sim', 'athena']:
        stub_binary = work / ('unsupported-' + platform)
        subprocess.run([*base, str(TESTS / 'unsupported.cpp'),
                        str(ROOT / f'hal/src/main/native/{platform}/VMXIMU.cpp'),
                        '-o', str(stub_binary)], check=True, timeout=60)
        subprocess.run([str(stub_binary)], check=True, timeout=10)
    print('PASS VMXIMU non-VMX backends: explicit unavailable errors', flush=True)
    jars = list((Path.home() / '.gradle/caches/modules-2/files-2.1').glob(
        'edu.wpi.first.*/*/2026.2.2/**/*.jar'))
    for module in ['wpiutil', 'wpimath', 'wpiunits', 'hal', 'wpilibj', 'ntcore']:
        jars.extend((ROOT / module / 'build/libs').glob('*.jar'))
    jars.extend((Path.home() / '.gradle/caches/modules-2/files-2.1').glob(
        'com.fasterxml.jackson.core/*/*/**/*.jar'))
    classpath = os.pathsep.join(map(str, [
        *[ROOT / module / 'build/classes/java/main' for module in
          ['wpiutil', 'wpimath', 'wpiunits', 'hal', 'wpilibj', 'ntcore']],
        *[jar for jar in jars if not jar.name.endswith(('-sources.jar', '-javadoc.jar'))]]))
    jdk = Path(os.environ['JAVA_HOME'])
    subprocess.run([str(jdk / 'bin/javac'), '--release', '17', '-cp', classpath,
                    '-h', str(work), '-d', str(work),
                    str(ROOT / 'hal/src/main/java/edu/wpi/first/hal/VMXIMUJNI.java'),
                    str(ROOT / 'wpilibj/src/main/java/edu/wpi/first/wpilibj/VMXIMU.java'),
                    str(TESTS / 'IMURegression.java')], check=True, timeout=60)
    library = work / 'libimutest.so'
    subprocess.run([*base, '-shared', '-fPIC', '-I' + str(jdk / 'include'),
                    '-I' + str(jdk / 'include/linux'), str(TESTS / 'JNIFake.cpp'),
                    str(backend), str(ROOT / 'hal/src/main/native/cpp/jni/VMXIMUJNI.cpp'),
                    '-o', str(library)], check=True, timeout=120)
    subprocess.run([str(jdk / 'bin/java'), '-cp', str(work) + os.pathsep + classpath,
                    'IMURegression', str(library)], check=True, timeout=30)
