#!/bin/bash

# 권한 부여: chmod +x android_build_script.sh
# 실행: ./android_build_script.sh

# 원하는 CMake 특정 버전을 명시하세요.
CMAKE_VERSION="3.22.1"
export PATH="$HOME/Library/Android/sdk/cmake/$CMAKE_VERSION/bin:$PATH"

# 현재 스크립트 경로로 이동
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# 빌드 디렉토리 생성 및 이동
rm -rf build-android
mkdir build-android && cd build-android

# NDK r28+ 이상으로 빌드 시 자동으로 16KB로 정렬되어 빌드
NDK=~/Library/Android/sdk/ndk/28.2.13676358
READELF="$NDK/toolchains/llvm/prebuilt/darwin-x86_64/bin/llvm-readelf"

# CMake 설정
cmake -DCMAKE_TOOLCHAIN_FILE=$NDK/build/cmake/android.toolchain.cmake \
      -DANDROID_ABI=arm64-v8a \
      -DANDROID_PLATFORM=android-24 \
      -DCMAKE_BUILD_TYPE=Release \
      -DEXERCISE_SEGMENT_BUILD_EXAMPLES=OFF \
      -DEXERCISE_SEGMENT_BUILD_STATIC=OFF \
      ..

# 빌드
cmake --build . -j

# 결과 .so를 플러그인으로 복사
# 프로젝트 환경에 맞게 경로 수정
cp libexercise_segment.so \
   /Users/ahndohyeon/Desktop/Company/Flutter/exercise_segment_flutter/android/src/main/jniLibs/arm64-v8a/