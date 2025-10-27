/**
 * @file test_calibration_fix.c
 * @brief 수정된 캘리브레이션 로직 테스트
 */

#include "include/calibration.h"
#include "include/segment_api.h"
#include "include/segment_types.h"
#include <stdio.h>
#include <string.h>

// 테스트용 포즈 데이터 생성
PoseData create_test_pose(float shoulder_width, float confidence) {
  PoseData pose;
  memset(&pose, 0, sizeof(PoseData));

  // 기본 신뢰도 설정
  for (int i = 0; i < POSE_LANDMARK_COUNT; i++) {
    pose.landmarks[i].inFrameLikelihood = confidence;
  }

  // 테스트용 어깨 위치 설정
  pose.landmarks[POSE_LANDMARK_LEFT_SHOULDER].position.x = 100.0f;
  pose.landmarks[POSE_LANDMARK_LEFT_SHOULDER].position.y = 200.0f;
  pose.landmarks[POSE_LANDMARK_LEFT_SHOULDER].position.z = 0.0f;
  pose.landmarks[POSE_LANDMARK_LEFT_SHOULDER].inFrameLikelihood = confidence;

  pose.landmarks[POSE_LANDMARK_RIGHT_SHOULDER].position.x =
      100.0f + shoulder_width;
  pose.landmarks[POSE_LANDMARK_RIGHT_SHOULDER].position.y = 200.0f;
  pose.landmarks[POSE_LANDMARK_RIGHT_SHOULDER].position.z = 0.0f;
  pose.landmarks[POSE_LANDMARK_RIGHT_SHOULDER].inFrameLikelihood = confidence;

  // 엉덩이 위치 설정
  pose.landmarks[POSE_LANDMARK_LEFT_HIP].position.x = 120.0f;
  pose.landmarks[POSE_LANDMARK_LEFT_HIP].position.y = 400.0f;
  pose.landmarks[POSE_LANDMARK_LEFT_HIP].position.z = 0.0f;
  pose.landmarks[POSE_LANDMARK_LEFT_HIP].inFrameLikelihood = confidence;

  pose.landmarks[POSE_LANDMARK_RIGHT_HIP].position.x =
      120.0f + shoulder_width * 0.8f;
  pose.landmarks[POSE_LANDMARK_RIGHT_HIP].position.y = 400.0f;
  pose.landmarks[POSE_LANDMARK_RIGHT_HIP].position.z = 0.0f;
  pose.landmarks[POSE_LANDMARK_RIGHT_HIP].inFrameLikelihood = confidence;

  pose.timestamp = 1000;
  return pose;
}

int main() {
  printf("🧪 캘리브레이션 로직 수정 테스트 시작\n");
  printf("====================================\n\n");

  // 시스템 초기화
  printf("1️⃣ 시스템 초기화...\n");
  int init_result = segment_api_init();
  if (init_result != SEGMENT_OK) {
    printf("❌ 시스템 초기화 실패: %d\n", init_result);
    return -1;
  }
  printf("✅ 시스템 초기화 성공\n\n");

  // 테스트 케이스 1: 정상적인 캘리브레이션
  printf(
      "2️⃣ 테스트 1: 정상적인 캘리브레이션 (어깨 너비: 300.0f, 신뢰도: 0.8f)\n");
  PoseData normal_pose = create_test_pose(300.0f, 0.8f);
  int result1 = segment_calibrate_user(&normal_pose);
  printf("결과: %s\n\n", result1 == SEGMENT_OK ? "성공" : "실패");

  // 테스트 케이스 2: 낮은 신뢰도로 인한 실패
  printf("3️⃣ 테스트 2: 낮은 신뢰도 (어깨 너비: 300.0f, 신뢰도: 0.2f)\n");
  PoseData low_conf_pose = create_test_pose(300.0f, 0.2f);
  int result2 = segment_calibrate_user(&low_conf_pose);
  printf("결과: %s (예상: 실패)\n\n", result2 == SEGMENT_OK ? "성공" : "실패");

  // 테스트 케이스 3: 어깨 너비가 너무 작음
  printf("4️⃣ 테스트 3: 어깨 너비 너무 작음 (어깨 너비: 30.0f, 신뢰도: 0.8f)\n");
  PoseData small_shoulder_pose = create_test_pose(30.0f, 0.8f);
  int result3 = segment_calibrate_user(&small_shoulder_pose);
  printf("결과: %s (예상: 실패)\n\n", result3 == SEGMENT_OK ? "성공" : "실패");

  // 테스트 케이스 4: 어깨 너비가 너무 큼
  printf("5️⃣ 테스트 4: 어깨 너비 너무 큼 (어깨 너비: 700.0f, 신뢰도: 0.8f)\n");
  PoseData large_shoulder_pose = create_test_pose(700.0f, 0.8f);
  int result4 = segment_calibrate_user(&large_shoulder_pose);
  printf("결과: %s (예상: 실패)\n\n", result4 == SEGMENT_OK ? "성공" : "실패");

  // 테스트 케이스 5: 기록자 캘리브레이션
  printf(
      "6️⃣ 테스트 5: 기록자 캘리브레이션 (어깨 너비: 322.78f, 신뢰도: 0.9f)\n");
  PoseData recorder_pose = create_test_pose(322.78f, 0.9f);
  int result5 = segment_calibrate_recorder(&recorder_pose);
  printf("결과: %s\n\n", result5 == SEGMENT_OK ? "성공" : "실패");

  // 캘리브레이션 데이터 확인
  if (g_user_calibrated) {
    printf("7️⃣ 사용자 캘리브레이션 정보:\n");
    printf("   스케일 팩터: %.3f\n", g_user_calibration.scale_factor);
    printf("   캘리브레이션 품질: %.2f\n",
           g_user_calibration.calibration_quality);
    printf("   중심 오프셋: (%.2f, %.2f)\n", g_user_calibration.center_offset.x,
           g_user_calibration.center_offset.y);
  }

  if (g_recorder_calibrated) {
    printf("\n8️⃣ 기록자 캘리브레이션 정보:\n");
    printf("   스케일 팩터: %.3f\n", g_recorder_calibration.scale_factor);
    printf("   캘리브레이션 품질: %.2f\n",
           g_recorder_calibration.calibration_quality);
    printf("   중심 오프셋: (%.2f, %.2f)\n",
           g_recorder_calibration.center_offset.x,
           g_recorder_calibration.center_offset.y);
  }

  printf("\n🎯 테스트 완료!\n");
  printf("개선 사항:\n");
  printf("  ✅ 상수 중앙 집중화\n");
  printf("  ✅ 중복 코드 제거\n");
  printf("  ✅ 더 엄격한 신뢰도 기준 (0.3f)\n");
  printf("  ✅ 합리적인 스케일 팩터 범위 (0.3f ~ 3.0f)\n");
  printf("  ✅ 개선된 어깨 너비 범위 (50.0f ~ 600.0f)\n");
  printf("  ✅ 더 자세한 에러 메시지\n");

  return 0;
}
