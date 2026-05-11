#!/bin/bash

# 권한 부여: chmod +x android_build_script.sh
# 실행: ./android_build_script.sh

# 원하는 CMake 특정 버전을 명시하세요.
CMAKE_VERSION="3.22.1"
export PATH="$HOME/Library/Android/sdk/cmake/$CMAKE_VERSION/bin:$PATH"

# 2. 경로 설정
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# 3. 빌드 디렉토리 초기화
rm -rf build-android
mkdir build-android && cd build-android

# NDK r28+ 이상으로 빌드 시 자동으로 16KB로 정렬되어 빌드
NDK=~/Library/Android/sdk/ndk/28.2.13676358

# 4. CMake 설정 (16KB 정렬 플래그 명시적 추가)
cmake -DCMAKE_TOOLCHAIN_FILE=$NDK/build/cmake/android.toolchain.cmake \
      -DANDROID_ABI=arm64-v8a \
      -DANDROID_PLATFORM=android-35 \
      -DCMAKE_BUILD_TYPE=Release \
      -DEXERCISE_SEGMENT_BUILD_EXAMPLES=OFF \
      -DEXERCISE_SEGMENT_BUILD_STATIC=OFF \
      -DCMAKE_SHARED_LINKER_FLAGS="-Wl,-z,max-page-size=16384 -Wl,-z,common-page-size=16384" \
      -DCMAKE_MODULE_LINKER_FLAGS="-Wl,-z,max-page-size=16384 -Wl,-z,common-page-size=16384" \
      ..

# 5. 빌드 및 복사
cmake --build . -j
cp libexercise_segment.so /Users/ahndohyeon/Desktop/Company/Flutter/exercise_segment_flutter/android/src/main/jniLibs/arm64-v8a/

# 6. 정렬 즉시 확인 (llvm-readelf 활용)
echo "--- 16KB Alignment Check ---"
$NDK/toolchains/llvm/prebuilt/darwin-x86_64/bin/llvm-readelf -l libexercise_segment.so | grep LOAD