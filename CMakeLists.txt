cmake_minimum_required(VERSION 3.16)
project(ExerciseSegmentAPI VERSION 1.0.0 LANGUAGES C)

# C 표준 설정
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# 컴파일러 플래그 설정
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -O3")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -g -DDEBUG")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -DNDEBUG")

# 플랫폼별 최적화
if(NOT CMAKE_CROSSCOMPILING)
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm64|aarch64")
        # Apple 계열(iOS/macOS)은 toolchain이 아키텍처 최적화를 처리하므로 추가 플래그를 강제하지 않음
        if(NOT APPLE)
            set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=armv8-a")
        endif()
    elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|amd64")
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=native")
    endif()
endif()

# iOS에서는 기본적으로 shared 라이브러리 빌드를 비활성화 (정적 라이브러리 사용 권장)
if(CMAKE_SYSTEM_NAME STREQUAL "iOS")
    set(EXERCISE_SEGMENT_BUILD_SHARED_DEFAULT OFF)
else()
    set(EXERCISE_SEGMENT_BUILD_SHARED_DEFAULT ON)
endif()

option(EXERCISE_SEGMENT_BUILD_SHARED "Build shared library (exercise_segment)" ${EXERCISE_SEGMENT_BUILD_SHARED_DEFAULT})
option(EXERCISE_SEGMENT_BUILD_STATIC "Build static library (exercise_segment_static)" ON)
if(CMAKE_CROSSCOMPILING)
    set(EXERCISE_SEGMENT_BUILD_EXAMPLES_DEFAULT OFF)
else()
    set(EXERCISE_SEGMENT_BUILD_EXAMPLES_DEFAULT ON)
endif()
option(EXERCISE_SEGMENT_BUILD_EXAMPLES "Build example executables" ${EXERCISE_SEGMENT_BUILD_EXAMPLES_DEFAULT})

# 라이브러리 생성 (정적 + 동적)
if(EXERCISE_SEGMENT_BUILD_STATIC)
    add_library(exercise_segment_static STATIC
        src/segment_core.c
        src/calibration.c
        src/pose_analysis.c
        src/math_utils.c
    )
endif()

if(EXERCISE_SEGMENT_BUILD_SHARED)
    add_library(exercise_segment SHARED
        src/segment_core.c
        src/calibration.c
        src/pose_analysis.c
        src/math_utils.c
    )
endif()

# 헤더 파일 경로 설정
if(TARGET exercise_segment_static)
    target_include_directories(exercise_segment_static PUBLIC include)
endif()
if(TARGET exercise_segment)
    target_include_directories(exercise_segment PUBLIC include)
endif()

# 라이브러리 링크 (수학 라이브러리)
if(TARGET exercise_segment_static)
    target_link_libraries(exercise_segment_static m)
endif()
if(TARGET exercise_segment)
    target_link_libraries(exercise_segment m)
endif()

# Android 16KB 페이지 크기 지원 (Android 15+ 기기 대응 및 Google Play 요구사항)
if(CMAKE_SYSTEM_NAME STREQUAL "Android" OR ANDROID)
    if(TARGET exercise_segment)
        message(STATUS ">>> [SUCCESS] Entering 16KB Alignment Block")
        set_target_properties(exercise_segment PROPERTIES 
            LINK_OPTIONS "LINKER:-z,max-page-size=16384"
            LINK_FLAGS "-Wl,-z,max-page-size=16384 -Wl,-z,common-page-size=16384"
        )
    else()
        message(STATUS ">>> [ERROR] Target 'exercise_segment' not found yet!")
    endif()
endif()

# 설치 설정
set(EXERCISE_SEGMENT_INSTALL_TARGETS "")
if(TARGET exercise_segment)
    list(APPEND EXERCISE_SEGMENT_INSTALL_TARGETS exercise_segment)
endif()
if(TARGET exercise_segment_static)
    list(APPEND EXERCISE_SEGMENT_INSTALL_TARGETS exercise_segment_static)
endif()
if(EXERCISE_SEGMENT_INSTALL_TARGETS)
    install(TARGETS ${EXERCISE_SEGMENT_INSTALL_TARGETS}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
    )
endif()

install(DIRECTORY include/
    DESTINATION include
    FILES_MATCHING PATTERN "*.h"
)

# 예제/도구는 기본적으로 네이티브 빌드에서만 켬(크로스컴파일에서는 OFF)
if(EXERCISE_SEGMENT_BUILD_EXAMPLES)
    if(TARGET exercise_segment_static)
        set(EXERCISE_SEGMENT_LINK_LIB exercise_segment_static)
    elseif(TARGET exercise_segment)
        set(EXERCISE_SEGMENT_LINK_LIB exercise_segment)
    else()
        message(FATAL_ERROR "No library target available to link examples.")
    endif()

    add_executable(example_basic examples/basic_example.c)
    target_link_libraries(example_basic ${EXERCISE_SEGMENT_LINK_LIB})

    add_executable(realtime_demo examples/realtime_demo.c)
    target_link_libraries(realtime_demo ${EXERCISE_SEGMENT_LINK_LIB})

    add_executable(smart_demo examples/smart_demo.c)
    target_link_libraries(smart_demo ${EXERCISE_SEGMENT_LINK_LIB})

    add_executable(joint_analysis_demo examples/joint_analysis_demo.c)
    target_link_libraries(joint_analysis_demo ${EXERCISE_SEGMENT_LINK_LIB})

    add_executable(test_mid_joint_analysis test_mid_joint_analysis.c)
    target_link_libraries(test_mid_joint_analysis ${EXERCISE_SEGMENT_LINK_LIB})
endif()

# 테스트 실행 파일들은 examples/ 디렉토리로 통합됨

# 테스트 실행 파일 생성 (옵션)
option(BUILD_TESTS "Build test executables" OFF)
if(BUILD_TESTS)
    add_executable(test_math_utils tests/test_math_utils.c)
    target_link_libraries(test_math_utils exercise_segment_static)
    
    add_executable(test_calibration tests/test_calibration.c)
    target_link_libraries(test_calibration exercise_segment_static)
    
    add_executable(test_pose_analysis tests/test_pose_analysis.c)
    target_link_libraries(test_pose_analysis exercise_segment_static)
endif()

# 패키지 설정
set(CPACK_PACKAGE_NAME "ExerciseSegmentAPI")
set(CPACK_PACKAGE_VERSION "1.0.0")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Exercise Segment Analysis API")
set(CPACK_PACKAGE_VENDOR "Exercise Segment API Team")

include(CPack)
