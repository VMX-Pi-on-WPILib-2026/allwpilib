#!/usr/bin/env python3
"""Test Studica compatibility without a robot. Needs C++20, JDK17+, WPILib jars.

C++ uses the real WPILib headers and transport fakes under ASan/UBSan.
Java uses the real WPILib jars and an injected CAN link/clock; no native loader.
--java-classpath can override automatic discovery in the local Gradle cache.
"""
import argparse
import os
from pathlib import Path
import shlex
import subprocess
import tempfile

ROOT=Path(__file__).resolve().parents[3]
TESTS=Path(__file__).parent
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--java-classpath')
parser.add_argument('--legacy-java-jar',type=Path)
args=parser.parse_args()
inc=[ROOT/p for p in (
    'wpilibc/src/main/native/include','wpilibc/src/generated/main/native/include',
    'hal/src/main/native/include','hal/src/generated/main/native/include',
    'wpimath/src/main/native/include','wpiutil/src/main/native/include',
    'wpiutil/src/main/native/thirdparty/llvm/include','wpiutil/src/main/native/thirdparty/fmtlib/include',
    'wpimath/src/main/native/thirdparty/gcem/include','wpimath/src/main/native/thirdparty/eigen/include','ntcore/src/main/native/include','wpilibc/src/main/native/cpp/studica','hal/src/main/native/mau')]
cpp=ROOT/'wpilibc/src/main/native/cpp/studica'
cxx=shlex.split(os.environ.get('CXX','c++'))
with tempfile.TemporaryDirectory(prefix='studica-tests-') as tmp:
    work=Path(tmp)
    subprocess.run([*cxx,'-std=c++20','-pthread','-Wall','-Wextra','-Werror','-fsyntax-only',
                    *['-I'+str(p) for p in inc],*map(str,cpp.glob('*.cpp')),str(TESTS/'LegacyApi.cpp')],check=True,timeout=120)
    sources=[TESTS/'regression.cpp',*[cpp/name for name in ('TitanDevice.cpp','TitanQuad.cpp','TitanQuadEncoder.cpp','Cobra.cpp')],
             ROOT/'wpilibc/src/main/native/cpp/motorcontrol/MotorController.cpp',ROOT/'hal/src/main/native/cpp/SerialPort.cpp']
    binary=work/'regression'
    subprocess.run([*cxx,'-std=c++20','-pthread','-g','-O1','-fsanitize=address,undefined',
                    '-fno-omit-frame-pointer',*['-I'+str(p) for p in inc],*map(str,sources),'-lutil','-o',str(binary)],
                   check=True,timeout=120)
    subprocess.run([str(binary)],check=True,timeout=30,
                   env={**os.environ,'ASAN_OPTIONS':'detect_leaks=1','UBSAN_OPTIONS':'halt_on_error=1'})
    cache=Path.home()/'.gradle/caches/modules-2/files-2.1'
    jars=list(cache.glob('edu.wpi.first.*/*/2026.2.2/**/*.jar'))
    jars+=list((Path.home()/'wpilib/2026/VMX2026/releases').glob('*/maven/edu/wpi/first/hal/hal-java/2026.2.2-vmx.2/*-vmx.2.jar'))
    classpath=args.java_classpath or os.pathsep.join(map(str,jars))
    java=ROOT/'wpilibj/src/main/java'
    files=[*java.glob('com/studica/frc/*.java'),java/'edu/wpi/first/wpilibj/SpeedController.java',TESTS/'StudicaRegression.java', java/'edu/wpi/first/wpilibj/SerialPort.java', ROOT/'hal/src/main/java/edu/wpi/first/hal/SerialPortJNI.java',java/'edu/wpi/first/wpilibj/smartdashboard/SendableBuilder.java']
    subprocess.run(['javac','--release','17','-cp',classpath,'-d',str(work),*map(str,files)],check=True,timeout=120)
    subprocess.run(['java','-cp',str(work)+os.pathsep+classpath,'com.studica.frc.StudicaRegression'],check=True,timeout=30)

    if args.legacy_java_jar:
        for cls in ('TitanQuad','TitanQuadEncoder','Cobra','Servo','ServoContinuous','Lidar'):
            name='com.studica.frc.'+cls
            old=subprocess.check_output(['javap','-classpath',str(args.legacy_java_jar),'-public',name],text=True)
            new=subprocess.check_output(['javap','-classpath',str(work),'-public',name],text=True)
            def methods(text):
                return {line.strip().replace('synchronized ','') for line in text.splitlines()
                        if '(' in line and line.strip().startswith('public ')}
            missing=methods(old)-methods(new)
            if missing:
                raise RuntimeError(f'Missing legacy Java signatures for {name}: {missing}')
        print('Original 1.1.8 public Java signatures retained for all six classes')
