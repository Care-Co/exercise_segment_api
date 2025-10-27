/**
 * @file test_detailed_leg_comparison.c
 * @brief 다리 길이 변화를 상세히 확인하는 테스트
 */

#include "include/calibration.h"
#include "include/segment_api.h"
#include "include/segment_types.h"
#include <stdio.h>
#include <string.h>

// 사용자 포즈 생성 (다리가 긴 체형)
PoseData create_long_leg_user_pose() {
  PoseData pose;
  memset(&pose, 0, sizeof(PoseData));

  // 높은 신뢰도 설정
  for (int i = 0; i < POSE_LANDMARK_COUNT; i++) {
    pose.landmarks[i].inFrameLikelihood = 0.9f;
  }

  printf("📝 사용자 포즈 설정: 다리가 매우 긴 체형\n");

  // 어깨 (정상 어깨 너비 300)
  pose.landmarks[POSE_LANDMARK_LEFT_SHOULDER].position =
      (Point3D){200.0f, 300.0f, 0.0f};
  pose.landmarks[POSE_LANDMARK_RIGHT_SHOULDER].position =
      (Point3D){500.0f, 300.0f, 0.0f};

  // 팔꿈치 (짧은 상완 - 150)
  pose.landmarks[POSE_LANDMARK_LEFT_ELBOW].position =
      (Point3D){180.0f, 450.0f, 0.0f};
  pose.landmarks[POSE_LANDMARK_RIGHT_ELBOW].position =
      (Point3D){520.0f, 450.0f, 0.0f};

  // 손목 (짧은 전완 - 100)
  pose.landmarks[POSE_LANDMARK_LEFT_WRIST].position =
      (Point3D){170.0f, 550.0f, 0.0f};
  pose.landmarks[POSE_LANDMARK_RIGHT_WRIST].position =
      (Point3D){530.0f, 550.0f, 0.0f};

  // 엉덩이
  pose.landmarks[POSE_LANDMARK_LEFT_HIP].position =
      (Point3D){250.0f, 600.0f, 0.0f};
  pose.landmarks[POSE_LANDMARK_RIGHT_HIP].position =
      (Point3D){450.0f, 600.0f, 0.0f};

  // 무릎 (매우 긴 대퇴 - 400)
  pose.landmarks[POSE_LANDMARK_LEFT_KNEE].position =
      (Point3D){240.0f, 1000.0f, 0.0f};
  pose.landmarks[POSE_LANDMARK_RIGHT_KNEE].position =
      (Point3D){460.0f, 1000.0f, 0.0f};

  // 발목 (매우 긴 정강이 - 350)
  pose.landmarks[POSE_LANDMARK_LEFT_ANKLE].position =
      (Point3D){230.0f, 1350.0f, 0.0f};
  pose.landmarks[POSE_LANDMARK_RIGHT_ANKLE].position =
      (Point3D){470.0f, 1350.0f, 0.0f};

  // 기타 관절들 (손, 발 등)
  pose.landmarks[POSE_LANDMARK_LEFT_HEEL].position =
      (Point3D){225.0f, 1360.0f, 0.0f};
  pose.landmarks[POSE_LANDMARK_RIGHT_HEEL].position =
      (Point3D){475.0f, 1360.0f, 0.0f};
  pose.landmarks[POSE_LANDMARK_LEFT_FOOT_INDEX].position =
      (Point3D){235.0f, 1370.0f, 0.0f};
  pose.landmarks[POSE_LANDMARK_RIGHT_FOOT_INDEX].position =
      (Point3D){465.0f, 1370.0f, 0.0f};

  // 손가락들
  pose.landmarks[POSE_LANDMARK_LEFT_INDEX].position =
      (Point3D){165.0f, 570.0f, 0.0f};
  pose.landmarks[POSE_LANDMARK_RIGHT_INDEX].position =
      (Point3D){535.0f, 570.0f, 0.0f};
  pose.landmarks[POSE_LANDMARK_LEFT_PINKY].position =
      (Point3D){175.0f, 575.0f, 0.0f};
  pose.landmarks[POSE_LANDMARK_RIGHT_PINKY].position =
      (Point3D){525.0f, 575.0f, 0.0f};
  pose.landmarks[POSE_LANDMARK_LEFT_THUMB].position =
      (Point3D){160.0f, 565.0f, 0.0f};
  pose.landmarks[POSE_LANDMARK_RIGHT_THUMB].position =
      (Point3D){540.0f, 565.0f, 0.0f};

  // 얼굴 관절들
  pose.landmarks[POSE_LANDMARK_NOSE].position = (Point3D){350.0f, 250.0f, 0.0f};
  pose.landmarks[POSE_LANDMARK_LEFT_EYE_INNER].position =
      (Point3D){340.0f, 240.0f, 0.0f};
  pose.landmarks[POSE_LANDMARK_LEFT_EYE].position =
      (Point3D){335.0f, 240.0f, 0.0f};
  pose.landmarks[POSE_LANDMARK_LEFT_EYE_OUTER].position =
      (Point3D){330.0f, 240.0f, 0.0f};
  pose.landmarks[POSE_LANDMARK_RIGHT_EYE_INNER].position =
      (Point3D){360.0f, 240.0f, 0.0f};
  pose.landmarks[POSE_LANDMARK_RIGHT_EYE].position =
      (Point3D){365.0f, 240.0f, 0.0f};
  pose.landmarks[POSE_LANDMARK_RIGHT_EYE_OUTER].position =
      (Point3D){370.0f, 240.0f, 0.0f};
  pose.landmarks[POSE_LANDMARK_LEFT_EAR].position =
      (Point3D){320.0f, 250.0f, 0.0f};
  pose.landmarks[POSE_LANDMARK_RIGHT_EAR].position =
      (Point3D){380.0f, 250.0f, 0.0f};
  pose.landmarks[POSE_LANDMARK_MOUTH_LEFT].position =
      (Point3D){340.0f, 270.0f, 0.0f};
  pose.landmarks[POSE_LANDMARK_MOUTH_RIGHT].position =
      (Point3D){360.0f, 270.0f, 0.0f};

  pose.timestamp = 1000;

  printf("  - 예상 좌다리 전체: %.2f (대퇴 400 + 정강 350)\n", 400.0f + 350.0f);
  printf("  - 예상 우다리 전체: %.2f (대퇴 400 + 정강 350)\n", 400.0f + 350.0f);

  return pose;
}

// 목표 포즈 생성 (다리가 짧은 표준 체형)
PoseData create_short_leg_target_pose() {
  PoseData pose;
  memset(&pose, 0, sizeof(PoseData));

  // 높은 신뢰도 설정
  for (int i = 0; i < POSE_LANDMARK_COUNT; i++) {
    pose.landmarks[i].inFrameLikelihood = 0.9f;
  }

  printf("📝 목표 포즈 설정: 다리가 짧은 표준 체형\n");

  // 어깨 (동일한 어깨 너비 300)
  pose.landmarks[POSE_LANDMARK_LEFT_SHOULDER].position =
      (Point3D){200.0f, 300.0f, 0.0f};
  pose.landmarks[POSE_LANDMARK_RIGHT_SHOULDER].position =
      (Point3D){500.0f, 300.0f, 0.0f};

  // 팔꿈치 (긴 상완 - 200)
  pose.landmarks[POSE_LANDMARK_LEFT_ELBOW].position =
      (Point3D){150.0f, 500.0f, 0.0f};
  pose.landmarks[POSE_LANDMARK_RIGHT_ELBOW].position =
      (Point3D){550.0f, 500.0f, 0.0f};

  // 손목 (긴 전완 - 180)
  pose.landmarks[POSE_LANDMARK_LEFT_WRIST].position =
      (Point3D){120.0f, 680.0f, 0.0f};
  pose.landmarks[POSE_LANDMARK_RIGHT_WRIST].position =
      (Point3D){580.0f, 680.0f, 0.0f};

  // 엉덩이
  pose.landmarks[POSE_LANDMARK_LEFT_HIP].position =
      (Point3D){250.0f, 600.0f, 0.0f};
  pose.landmarks[POSE_LANDMARK_RIGHT_HIP].position =
      (Point3D){450.0f, 600.0f, 0.0f};

  // 무릎 (짧은 대퇴 - 200)
  pose.landmarks[POSE_LANDMARK_LEFT_KNEE].position =
      (Point3D){240.0f, 800.0f, 0.0f};
  pose.landmarks[POSE_LANDMARK_RIGHT_KNEE].position =
      (Point3D){460.0f, 800.0f, 0.0f};

  // 발목 (짧은 정강이 - 150)
  pose.landmarks[POSE_LANDMARK_LEFT_ANKLE].position =
      (Point3D){230.0f, 950.0f, 0.0f};
  pose.landmarks[POSE_LANDMARK_RIGHT_ANKLE].position =
      (Point3D){470.0f, 950.0f, 0.0f};

  // 기타 관절들
  pose.landmarks[POSE_LANDMARK_LEFT_HEEL].position =
      (Point3D){225.0f, 960.0f, 0.0f};
  pose.landmarks[POSE_LANDMARK_RIGHT_HEEL].position =
      (Point3D){475.0f, 960.0f, 0.0f};
  pose.landmarks[POSE_LANDMARK_LEFT_FOOT_INDEX].position =
      (Point3D){235.0f, 970.0f, 0.0f};
  pose.landmarks[POSE_LANDMARK_RIGHT_FOOT_INDEX].position =
      (Point3D){465.0f, 970.0f, 0.0f};

  // 손가락들
  pose.landmarks[POSE_LANDMARK_LEFT_INDEX].position =
      (Point3D){115.0f, 700.0f, 0.0f};
  pose.landmarks[POSE_LANDMARK_RIGHT_INDEX].position =
      (Point3D){585.0f, 700.0f, 0.0f};
  pose.landmarks[POSE_LANDMARK_LEFT_PINKY].position =
      (Point3D){125.0f, 705.0f, 0.0f};
  pose.landmarks[POSE_LANDMARK_RIGHT_PINKY].position =
      (Point3D){575.0f, 705.0f, 0.0f};
  pose.landmarks[POSE_LANDMARK_LEFT_THUMB].position =
      (Point3D){110.0f, 695.0f, 0.0f};
  pose.landmarks[POSE_LANDMARK_RIGHT_THUMB].position =
      (Point3D){590.0f, 695.0f, 0.0f};

  // 얼굴 관절들 (동일)
  pose.landmarks[POSE_LANDMARK_NOSE].position = (Point3D){350.0f, 250.0f, 0.0f};
  pose.landmarks[POSE_LANDMARK_LEFT_EYE_INNER].position =
      (Point3D){340.0f, 240.0f, 0.0f};
  pose.landmarks[POSE_LANDMARK_LEFT_EYE].position =
      (Point3D){335.0f, 240.0f, 0.0f};
  pose.landmarks[POSE_LANDMARK_LEFT_EYE_OUTER].position =
      (Point3D){330.0f, 240.0f, 0.0f};
  pose.landmarks[POSE_LANDMARK_RIGHT_EYE_INNER].position =
      (Point3D){360.0f, 240.0f, 0.0f};
  pose.landmarks[POSE_LANDMARK_RIGHT_EYE].position =
      (Point3D){365.0f, 240.0f, 0.0f};
  pose.landmarks[POSE_LANDMARK_RIGHT_EYE_OUTER].position =
      (Point3D){370.0f, 240.0f, 0.0f};
  pose.landmarks[POSE_LANDMARK_LEFT_EAR].position =
      (Point3D){320.0f, 250.0f, 0.0f};
  pose.landmarks[POSE_LANDMARK_RIGHT_EAR].position =
      (Point3D){380.0f, 250.0f, 0.0f};
  pose.landmarks[POSE_LANDMARK_MOUTH_LEFT].position =
      (Point3D){340.0f, 270.0f, 0.0f};
  pose.landmarks[POSE_LANDMARK_MOUTH_RIGHT].position =
      (Point3D){360.0f, 270.0f, 0.0f};

  pose.timestamp = 2000;

  printf("  - 예상 좌다리 전체: %.2f (대퇴 200 + 정강 150)\n", 200.0f + 150.0f);
  printf("  - 예상 우다리 전체: %.2f (대퇴 200 + 정강 150)\n", 200.0f + 150.0f);

  return pose;
}

int main() {
  printf("🦵 다리 길이 상세 비교 테스트\n");
  printf("===================================\n\n");

  // 시스템 초기화
  printf("1️⃣ 시스템 초기화...\n");
  int init_result = segment_api_init();
  if (init_result != SEGMENT_OK) {
    printf("❌ 시스템 초기화 실패: %d\n", init_result);
    return -1;
  }
  printf("✅ 시스템 초기화 성공\n\n");

  // 사용자 포즈 생성 및 캘리브레이션 (다리가 긴 체형)
  printf("2️⃣ 사용자 캘리브레이션 (다리가 긴 체형)...\n");
  PoseData user_pose = create_long_leg_user_pose();

  // 사용자 포즈 정보 표시
  printf("  사용자 포즈 준비 완료\n");

  // 사용자 캘리브레이션 수행
  int calib_result = segment_calibrate_user(&user_pose);
  if (calib_result != SEGMENT_OK) {
    printf("❌ 사용자 캘리브레이션 실패: %d\n", calib_result);
    return -1;
  }

  printf("\n==================================================\n");

  // 목표 포즈 생성 (다리가 짧은 표준 체형)
  printf("3️⃣ 목표 포즈 생성 (다리가 짧은 표준 체형)...\n");
  PoseData target_pose = create_short_leg_target_pose();

  printf("\n🎯 핵심 테스트: 목표 포즈를 사용자 비율로 조정\n");
  printf("기대 효과: 목표 포즈의 다리가 사용자처럼 길어져야 함!\n");
  printf("============================================================\n");

  // 목표 포즈를 사용자 비율에 맞게 조정
  PoseData adjusted_target_pose;
  int adjust_result = adjust_target_pose_to_user_proportions(
      &target_pose, &g_user_calibration, &adjusted_target_pose);

  if (adjust_result != SEGMENT_OK) {
    printf("❌ 목표 포즈 조정 실패: %d\n", adjust_result);
    return -1;
  }

  printf("\n🏁 최종 결과 요약:\n");
  printf("==========================================\n");
  printf("만약 조정이 제대로 작동했다면:\n");
  printf("  - 원본 목표 포즈 다리: 짧음 (~350)\n");
  printf("  - 조정된 목표 포즈 다리: 길어짐 (사용자 비율 적용, ~750 예상)\n");
  printf("  - 이제 파란색 점들(목표)이 초록색 점들(사용자)과 비슷한 다리 "
         "비율을 가짐!\n");
  printf("==========================================\n");

  return 0;
}
