/**
 * @file pose_analysis_simple.c
 * @brief 간단한 포즈 분석 구현 (ML Kit 33개 랜드마크 지원)
 * @author Exercise Segment API Team
 * @version 2.0.0
 */

#include "pose_analysis.h"
#include "math_utils.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

// 최소 신뢰도 임계값
#define MIN_CONFIDENCE_THRESHOLD 0.5f

float calculate_segment_progress(const PoseData *current_pose,
                                 const PoseData *start_pose,
                                 const PoseData *end_pose,
                                 const JointType *care_joints
                                 __attribute__((unused)),
                                 int care_joint_count __attribute__((unused))) {
  if (!current_pose || !start_pose || !end_pose) {
    return 0.0f;
  }

  // 골반 중심 기준 상대 좌표로 진행도 계산 (카메라 이동 무관)

  // 1. 각 포즈의 골반 중심점 계산
  Point3D current_hip_center = {
      (current_pose->landmarks[POSE_LANDMARK_LEFT_HIP].position.x +
       current_pose->landmarks[POSE_LANDMARK_RIGHT_HIP].position.x) /
          2.0f,
      (current_pose->landmarks[POSE_LANDMARK_LEFT_HIP].position.y +
       current_pose->landmarks[POSE_LANDMARK_RIGHT_HIP].position.y) /
          2.0f,
      (current_pose->landmarks[POSE_LANDMARK_LEFT_HIP].position.z +
       current_pose->landmarks[POSE_LANDMARK_RIGHT_HIP].position.z) /
          2.0f};

  Point3D start_hip_center = {
      (start_pose->landmarks[POSE_LANDMARK_LEFT_HIP].position.x +
       start_pose->landmarks[POSE_LANDMARK_RIGHT_HIP].position.x) /
          2.0f,
      (start_pose->landmarks[POSE_LANDMARK_LEFT_HIP].position.y +
       start_pose->landmarks[POSE_LANDMARK_RIGHT_HIP].position.y) /
          2.0f,
      (start_pose->landmarks[POSE_LANDMARK_LEFT_HIP].position.z +
       start_pose->landmarks[POSE_LANDMARK_RIGHT_HIP].position.z) /
          2.0f};

  Point3D end_hip_center = {
      (end_pose->landmarks[POSE_LANDMARK_LEFT_HIP].position.x +
       end_pose->landmarks[POSE_LANDMARK_RIGHT_HIP].position.x) /
          2.0f,
      (end_pose->landmarks[POSE_LANDMARK_LEFT_HIP].position.y +
       end_pose->landmarks[POSE_LANDMARK_RIGHT_HIP].position.y) /
          2.0f,
      (end_pose->landmarks[POSE_LANDMARK_LEFT_HIP].position.z +
       end_pose->landmarks[POSE_LANDMARK_RIGHT_HIP].position.z) /
          2.0f};

  // 2. 주요 관절들의 진행도 계산 (가중 평균 방식)
  // 많이 움직이는 관절에 더 큰 가중치를 부여
  float weighted_progress = 0.0f;
  float total_weight = 0.0f;
  float non_main_weight = 0.0f; // 주요 관절이 아닌 관절의 가중치 합
  int main_joint_progress_count = 0; // 움직이는 주요 관절 개수

  // 주요 관절들 (어깨, 팔꿈치, 손목, 무릎, 발목)
  JointType main_joints[] = {
      POSE_LANDMARK_LEFT_SHOULDER, POSE_LANDMARK_RIGHT_SHOULDER,
      POSE_LANDMARK_LEFT_ELBOW,    POSE_LANDMARK_RIGHT_ELBOW,
      POSE_LANDMARK_LEFT_WRIST,    POSE_LANDMARK_RIGHT_WRIST,
      POSE_LANDMARK_LEFT_KNEE,     POSE_LANDMARK_RIGHT_KNEE,
      POSE_LANDMARK_LEFT_ANKLE,    POSE_LANDMARK_RIGHT_ANKLE};

  int main_joint_count = sizeof(main_joints) / sizeof(main_joints[0]);

  for (int i = 0; i < main_joint_count; i++) {
    JointType joint = main_joints[i];

    // 신뢰도 확인
    if (current_pose->landmarks[joint].inFrameLikelihood <
            MIN_CONFIDENCE_THRESHOLD ||
        start_pose->landmarks[joint].inFrameLikelihood <
            MIN_CONFIDENCE_THRESHOLD ||
        end_pose->landmarks[joint].inFrameLikelihood <
            MIN_CONFIDENCE_THRESHOLD) {
      continue;
    }

    // 골반 중심 기준 상대 좌표 계산
    Point3D current_relative = {
        current_pose->landmarks[joint].position.x - current_hip_center.x,
        current_pose->landmarks[joint].position.y - current_hip_center.y,
        current_pose->landmarks[joint].position.z - current_hip_center.z};

    Point3D start_relative = {
        start_pose->landmarks[joint].position.x - start_hip_center.x,
        start_pose->landmarks[joint].position.y - start_hip_center.y,
        start_pose->landmarks[joint].position.z - start_hip_center.z};

    Point3D end_relative = {
        end_pose->landmarks[joint].position.x - end_hip_center.x,
        end_pose->landmarks[joint].position.y - end_hip_center.y,
        end_pose->landmarks[joint].position.z - end_hip_center.z};

    // 거리 계산
    float start_to_end = distance_3d(&start_relative, &end_relative);
    float current_to_end = distance_3d(&current_relative, &end_relative);

    float ratio;
    float weight;

    if (start_to_end > 10.0f) {
      // 움직이는 관절: 목표에 얼마나 가까워졌는지 계산
      ratio = 1.0f - (current_to_end / start_to_end);
      ratio = fmaxf(0.0f, ratio); // 음수 방지

      // 진행도 점수를 더욱 관대하게 (300% 보너스)
      ratio = fminf(1.0f, ratio * 3.0f);

      // 가중치 = 움직인 거리 (많이 움직인 관절이 더 중요)
      weight = start_to_end;
      main_joint_progress_count++;
    } else {
      // 거의 움직이지 않는 관절 (5px 이하)
      // → 이미 목표 위치에 있다고 간주 (100% 진행도)
      ratio = 1.0f;

      // 가중치를 작게 줘서 움직이는 관절에 비해 영향력 낮춤
      weight = 10.0f;
      non_main_weight += weight;
    }

    weighted_progress += ratio * weight;
    total_weight += weight;
  }

  // 아무 관절도 충분히 움직이지 않으면 0% 반환
  if (total_weight == 0.0f) {
    return 0.0f;
  }

  // 가중 평균 진행도 계산
  float progress = weighted_progress / total_weight;

  // 주요 관절이 아닌 관절에 20% 할당
  float non_main_progress = 0.2f; // 전체 진행도의 20%

  // 주요 관절의 진행도를 나머지 80%에 대해 계산
  float main_joint_ratio = (total_weight - non_main_weight) / total_weight;
  float main_joint_progress = progress * main_joint_ratio * 0.8f;

  // 최종 진행도 = 주요 관절 진행도(80%) + 비주요 관절 진행도(20%)
  float final_progress = main_joint_progress + non_main_progress;

  // 진행도에 30% 추가 (최대 1.0을 초과하지 않도록 제한)
  final_progress = final_progress * 1.3f;

  return fmaxf(0.0f, fminf(1.0f, final_progress));
}

bool is_segment_completed(const PoseData *current_pose,
                          const PoseData *end_pose,
                          const JointType *care_joints __attribute__((unused)),
                          int care_joint_count __attribute__((unused)),
                          float threshold) {
  if (!current_pose || !end_pose) {
    return false;
  }

  // 골반 중심 기준 상대적 완료 판단
  Point3D current_hip_center = {
      (current_pose->landmarks[POSE_LANDMARK_LEFT_HIP].position.x +
       current_pose->landmarks[POSE_LANDMARK_RIGHT_HIP].position.x) /
          2.0f,
      (current_pose->landmarks[POSE_LANDMARK_LEFT_HIP].position.y +
       current_pose->landmarks[POSE_LANDMARK_RIGHT_HIP].position.y) /
          2.0f,
      (current_pose->landmarks[POSE_LANDMARK_LEFT_HIP].position.z +
       current_pose->landmarks[POSE_LANDMARK_RIGHT_HIP].position.z) /
          2.0f};

  Point3D target_hip_center = {
      (end_pose->landmarks[POSE_LANDMARK_LEFT_HIP].position.x +
       end_pose->landmarks[POSE_LANDMARK_RIGHT_HIP].position.x) /
          2.0f,
      (end_pose->landmarks[POSE_LANDMARK_LEFT_HIP].position.y +
       end_pose->landmarks[POSE_LANDMARK_RIGHT_HIP].position.y) /
          2.0f,
      (end_pose->landmarks[POSE_LANDMARK_LEFT_HIP].position.z +
       end_pose->landmarks[POSE_LANDMARK_RIGHT_HIP].position.z) /
          2.0f};

  float total_distance = 0.0f;
  int joint_count = 0;

  // 주요 관절들
  JointType main_joints[] = {
      POSE_LANDMARK_LEFT_SHOULDER, POSE_LANDMARK_RIGHT_SHOULDER,
      POSE_LANDMARK_LEFT_ELBOW,    POSE_LANDMARK_RIGHT_ELBOW,
      POSE_LANDMARK_LEFT_KNEE,     POSE_LANDMARK_RIGHT_KNEE};

  for (int i = 0; i < 6; i++) {
    JointType joint = main_joints[i];

    // 골반 기준 상대 좌표
    Point3D current_relative = {
        current_pose->landmarks[joint].position.x - current_hip_center.x,
        current_pose->landmarks[joint].position.y - current_hip_center.y,
        current_pose->landmarks[joint].position.z - current_hip_center.z};

    Point3D target_relative = {
        end_pose->landmarks[joint].position.x - target_hip_center.x,
        end_pose->landmarks[joint].position.y - target_hip_center.y,
        end_pose->landmarks[joint].position.z - target_hip_center.z};

    float distance = distance_3d(&current_relative, &target_relative);
    total_distance += distance;
    joint_count++;
  }

  if (joint_count == 0) {
    return false;
  }

  float avg_distance = total_distance / joint_count;
  return avg_distance <= threshold;
}

float segment_calculate_similarity(const PoseData *current_pose,
                                   const PoseData *target_pose) {
  if (!current_pose || !target_pose) {
    return 0.0f;
  }

  // 골반 중심 기준 상대적 유사도 계산 (좌우 이동 무관)

  // 1. 골반 중심점 계산
  Point3D current_hip_center = {
      (current_pose->landmarks[POSE_LANDMARK_LEFT_HIP].position.x +
       current_pose->landmarks[POSE_LANDMARK_RIGHT_HIP].position.x) /
          2.0f,
      (current_pose->landmarks[POSE_LANDMARK_LEFT_HIP].position.y +
       current_pose->landmarks[POSE_LANDMARK_RIGHT_HIP].position.y) /
          2.0f,
      (current_pose->landmarks[POSE_LANDMARK_LEFT_HIP].position.z +
       current_pose->landmarks[POSE_LANDMARK_RIGHT_HIP].position.z) /
          2.0f};

  Point3D target_hip_center = {
      (target_pose->landmarks[POSE_LANDMARK_LEFT_HIP].position.x +
       target_pose->landmarks[POSE_LANDMARK_RIGHT_HIP].position.x) /
          2.0f,
      (target_pose->landmarks[POSE_LANDMARK_LEFT_HIP].position.y +
       target_pose->landmarks[POSE_LANDMARK_RIGHT_HIP].position.y) /
          2.0f,
      (target_pose->landmarks[POSE_LANDMARK_LEFT_HIP].position.z +
       target_pose->landmarks[POSE_LANDMARK_RIGHT_HIP].position.z) /
          2.0f};

  // 2. 주요 관절들의 평균 거리 계산
  float total_distance = 0.0f;
  int joint_count = 0;

  JointType main_joints[] = {
      POSE_LANDMARK_LEFT_SHOULDER, POSE_LANDMARK_RIGHT_SHOULDER,
      POSE_LANDMARK_LEFT_ELBOW,    POSE_LANDMARK_RIGHT_ELBOW,
      POSE_LANDMARK_LEFT_WRIST,    POSE_LANDMARK_RIGHT_WRIST,
      POSE_LANDMARK_LEFT_KNEE,     POSE_LANDMARK_RIGHT_KNEE,
      POSE_LANDMARK_LEFT_ANKLE,    POSE_LANDMARK_RIGHT_ANKLE};

  for (int i = 0; i < 10; i++) {
    JointType joint = main_joints[i];

    // 골반 중심 기준 상대 좌표
    Point3D current_relative = {
        current_pose->landmarks[joint].position.x - current_hip_center.x,
        current_pose->landmarks[joint].position.y - current_hip_center.y,
        current_pose->landmarks[joint].position.z - current_hip_center.z};

    Point3D target_relative = {
        target_pose->landmarks[joint].position.x - target_hip_center.x,
        target_pose->landmarks[joint].position.y - target_hip_center.y,
        target_pose->landmarks[joint].position.z - target_hip_center.z};

    float distance = distance_3d(&current_relative, &target_relative);
    total_distance += distance;
    joint_count++;
  }

  if (joint_count == 0) {
    return 0.0f;
  }

  // 3. 평균 거리를 유사도로 변환
  float avg_distance = total_distance / joint_count;

  // 거리 → 유사도 (500px 기준, 극단적으로 관대)
  float similarity = fmaxf(0.0f, 1.0f - (avg_distance / 500.0f));

  return similarity;
}

void calculate_correction_vectors(const PoseData *current_pose,
                                  const PoseData *target_pose,
                                  const JointType *care_joints
                                  __attribute__((unused)),
                                  int care_joint_count __attribute__((unused)),
                                  Point3D corrections[POSE_LANDMARK_COUNT]) {
  if (!current_pose || !target_pose || !corrections) {
    return;
  }

  // 모든 랜드마크에 대해 교정 벡터 계산
  for (int i = 0; i < POSE_LANDMARK_COUNT; i++) {
    // 신뢰도 확인
    if (current_pose->landmarks[i].inFrameLikelihood <
            MIN_CONFIDENCE_THRESHOLD ||
        target_pose->landmarks[i].inFrameLikelihood <
            MIN_CONFIDENCE_THRESHOLD) {
      corrections[i] = (Point3D){0.0f, 0.0f, 0.0f};
      continue;
    }

    // 교정 벡터 계산 (목표 - 현재)
    corrections[i].x = target_pose->landmarks[i].position.x -
                       current_pose->landmarks[i].position.x;
    corrections[i].y = target_pose->landmarks[i].position.y -
                       current_pose->landmarks[i].position.y;
    corrections[i].z = target_pose->landmarks[i].position.z -
                       current_pose->landmarks[i].position.z;

    // 벡터 정규화 제거 - 실제 거리 정보 유지
    // float magnitude = sqrtf(corrections[i].x * corrections[i].x +
    //                         corrections[i].y * corrections[i].y +
    //                         corrections[i].z * corrections[i].z);

    // if (magnitude > 0.0f) {
    //   corrections[i].x /= magnitude;
    //   corrections[i].y /= magnitude;
    //   corrections[i].z /= magnitude;
    // }
  }
}

// 관절 이름 매핑
static const char *get_joint_name(JointType joint) {
  switch (joint) {
  case POSE_LANDMARK_LEFT_SHOULDER:
    return "왼쪽 어깨";
  case POSE_LANDMARK_RIGHT_SHOULDER:
    return "오른쪽 어깨";
  case POSE_LANDMARK_LEFT_ELBOW:
    return "왼쪽 팔꿈치";
  case POSE_LANDMARK_RIGHT_ELBOW:
    return "오른쪽 팔꿈치";
  case POSE_LANDMARK_LEFT_WRIST:
    return "왼쪽 손목";
  case POSE_LANDMARK_RIGHT_WRIST:
    return "오른쪽 손목";
  case POSE_LANDMARK_LEFT_HIP:
    return "왼쪽 골반";
  case POSE_LANDMARK_RIGHT_HIP:
    return "오른쪽 골반";
  case POSE_LANDMARK_LEFT_KNEE:
    return "왼쪽 무릎";
  case POSE_LANDMARK_RIGHT_KNEE:
    return "오른쪽 무릎";
  case POSE_LANDMARK_LEFT_ANKLE:
    return "왼쪽 발목";
  case POSE_LANDMARK_RIGHT_ANKLE:
    return "오른쪽 발목";
  default:
    return "알 수 없는 관절";
  }
}

int analyze_exercise_joints(const PoseData *start_pose,
                            const PoseData *end_pose,
                            JointAnalysis *joint_analysis) {
  if (!start_pose || !end_pose || !joint_analysis) {
    return SEGMENT_ERROR_INVALID_PARAMETER;
  }

  printf("\n🔍 운동 관절 분석 시작...\n");
  printf("========================================\n");

  // 골반 중심점 계산
  Point3D start_hip_center = {
      (start_pose->landmarks[POSE_LANDMARK_LEFT_HIP].position.x +
       start_pose->landmarks[POSE_LANDMARK_RIGHT_HIP].position.x) /
          2.0f,
      (start_pose->landmarks[POSE_LANDMARK_LEFT_HIP].position.y +
       start_pose->landmarks[POSE_LANDMARK_RIGHT_HIP].position.y) /
          2.0f,
      (start_pose->landmarks[POSE_LANDMARK_LEFT_HIP].position.z +
       start_pose->landmarks[POSE_LANDMARK_RIGHT_HIP].position.z) /
          2.0f};

  Point3D end_hip_center = {
      (end_pose->landmarks[POSE_LANDMARK_LEFT_HIP].position.x +
       end_pose->landmarks[POSE_LANDMARK_RIGHT_HIP].position.x) /
          2.0f,
      (end_pose->landmarks[POSE_LANDMARK_LEFT_HIP].position.y +
       end_pose->landmarks[POSE_LANDMARK_RIGHT_HIP].position.y) /
          2.0f,
      (end_pose->landmarks[POSE_LANDMARK_LEFT_HIP].position.z +
       end_pose->landmarks[POSE_LANDMARK_RIGHT_HIP].position.z) /
          2.0f};

  // 주요 관절들 분석
  JointType main_joints[] = {
      POSE_LANDMARK_LEFT_SHOULDER, POSE_LANDMARK_RIGHT_SHOULDER,
      POSE_LANDMARK_LEFT_ELBOW,    POSE_LANDMARK_RIGHT_ELBOW,
      POSE_LANDMARK_LEFT_WRIST,    POSE_LANDMARK_RIGHT_WRIST,
      POSE_LANDMARK_LEFT_HIP,      POSE_LANDMARK_RIGHT_HIP,
      POSE_LANDMARK_LEFT_KNEE,     POSE_LANDMARK_RIGHT_KNEE,
      POSE_LANDMARK_LEFT_ANKLE,    POSE_LANDMARK_RIGHT_ANKLE};

  int main_joint_count = sizeof(main_joints) / sizeof(main_joints[0]);
  float max_distance = 0.0f;

  // 1단계: 모든 관절의 움직임 거리 계산
  for (int i = 0; i < main_joint_count; i++) {
    JointType joint = main_joints[i];

    // 신뢰도 확인
    if (start_pose->landmarks[joint].inFrameLikelihood <
            MIN_CONFIDENCE_THRESHOLD ||
        end_pose->landmarks[joint].inFrameLikelihood <
            MIN_CONFIDENCE_THRESHOLD) {
      joint_analysis[i].joint = joint;
      joint_analysis[i].movement_distance = 0.0f;
      joint_analysis[i].weight = 0.0f;
      joint_analysis[i].is_important = false;
      joint_analysis[i].joint_name = get_joint_name(joint);
      continue;
    }

    // 골반 중심 기준 상대 좌표 계산
    Point3D start_relative = {
        start_pose->landmarks[joint].position.x - start_hip_center.x,
        start_pose->landmarks[joint].position.y - start_hip_center.y,
        start_pose->landmarks[joint].position.z - start_hip_center.z};

    Point3D end_relative = {
        end_pose->landmarks[joint].position.x - end_hip_center.x,
        end_pose->landmarks[joint].position.y - end_hip_center.y,
        end_pose->landmarks[joint].position.z - end_hip_center.z};

    float distance = distance_3d(&start_relative, &end_relative);

    joint_analysis[i].joint = joint;
    joint_analysis[i].movement_distance = distance;
    joint_analysis[i].joint_name = get_joint_name(joint);

    if (distance > max_distance) {
      max_distance = distance;
    }
  }

  // 2단계: 중요 관절 판단 및 가중치 계산
  float important_threshold = max_distance * 0.3f; // 최대 움직임의 30% 이상
  int important_count = 0;

  // 관절별 중요도 분석 (로그 최소화)
  for (int i = 0; i < main_joint_count; i++) {
    float distance = joint_analysis[i].movement_distance;
    bool is_important = (distance >= important_threshold && distance > 6.0f);

    joint_analysis[i].is_important = is_important;

    if (is_important) {
      joint_analysis[i].weight = distance;
      important_count++;
    } else {
      joint_analysis[i].weight = 10.0f;
    }
  }

  printf("  - 주요 관절 %d개 식별됨 (임계값: %.1fpx)\n", important_count,
         important_threshold);

  return SEGMENT_OK;
}

void print_important_joints(const JointAnalysis *joint_analysis) {
  if (!joint_analysis) {
    return;
  }

  // 주요 관절 개수만 간단히 출력
  int important_count = 0;
  for (int i = 0; i < 12; i++) {
    if (joint_analysis[i].is_important) {
      important_count++;
    }
  }

  if (important_count == 0) {
    printf("  - 모든 관절 균등 분석\n");
  } else {
    printf("  - 핵심 관절 %d개 중심 분석\n", important_count);
  }
}

float calculate_progress_with_analysis(const PoseData *current_pose,
                                       const PoseData *start_pose,
                                       const PoseData *end_pose,
                                       const JointAnalysis *joint_analysis) {
  if (!current_pose || !start_pose || !end_pose || !joint_analysis) {
    return 0.0f;
  }

  // 골반 중심점 계산
  Point3D current_hip_center = {
      (current_pose->landmarks[POSE_LANDMARK_LEFT_HIP].position.x +
       current_pose->landmarks[POSE_LANDMARK_RIGHT_HIP].position.x) /
          2.0f,
      (current_pose->landmarks[POSE_LANDMARK_LEFT_HIP].position.y +
       current_pose->landmarks[POSE_LANDMARK_RIGHT_HIP].position.y) /
          2.0f,
      (current_pose->landmarks[POSE_LANDMARK_LEFT_HIP].position.z +
       current_pose->landmarks[POSE_LANDMARK_RIGHT_HIP].position.z) /
          2.0f};

  Point3D start_hip_center = {
      (start_pose->landmarks[POSE_LANDMARK_LEFT_HIP].position.x +
       start_pose->landmarks[POSE_LANDMARK_RIGHT_HIP].position.x) /
          2.0f,
      (start_pose->landmarks[POSE_LANDMARK_LEFT_HIP].position.y +
       start_pose->landmarks[POSE_LANDMARK_RIGHT_HIP].position.y) /
          2.0f,
      (start_pose->landmarks[POSE_LANDMARK_LEFT_HIP].position.z +
       start_pose->landmarks[POSE_LANDMARK_RIGHT_HIP].position.z) /
          2.0f};

  Point3D end_hip_center = {
      (end_pose->landmarks[POSE_LANDMARK_LEFT_HIP].position.x +
       end_pose->landmarks[POSE_LANDMARK_RIGHT_HIP].position.x) /
          2.0f,
      (end_pose->landmarks[POSE_LANDMARK_LEFT_HIP].position.y +
       end_pose->landmarks[POSE_LANDMARK_RIGHT_HIP].position.y) /
          2.0f,
      (end_pose->landmarks[POSE_LANDMARK_LEFT_HIP].position.z +
       end_pose->landmarks[POSE_LANDMARK_RIGHT_HIP].position.z) /
          2.0f};

  float weighted_progress = 0.0f;
  float total_weight = 0.0f;

  // 분석된 관절 정보를 사용하여 진행도 계산
  for (int i = 0; i < 12; i++) { // 주요 관절 12개
    JointAnalysis analysis = joint_analysis[i];

    // 신뢰도 확인
    if (current_pose->landmarks[analysis.joint].inFrameLikelihood <
            MIN_CONFIDENCE_THRESHOLD ||
        start_pose->landmarks[analysis.joint].inFrameLikelihood <
            MIN_CONFIDENCE_THRESHOLD ||
        end_pose->landmarks[analysis.joint].inFrameLikelihood <
            MIN_CONFIDENCE_THRESHOLD) {
      continue;
    }

    // 골반 중심 기준 상대 좌표 계산
    Point3D current_relative = {
        current_pose->landmarks[analysis.joint].position.x -
            current_hip_center.x,
        current_pose->landmarks[analysis.joint].position.y -
            current_hip_center.y,
        current_pose->landmarks[analysis.joint].position.z -
            current_hip_center.z};

    Point3D start_relative = {
        start_pose->landmarks[analysis.joint].position.x - start_hip_center.x,
        start_pose->landmarks[analysis.joint].position.y - start_hip_center.y,
        start_pose->landmarks[analysis.joint].position.z - start_hip_center.z};

    Point3D end_relative = {
        end_pose->landmarks[analysis.joint].position.x - end_hip_center.x,
        end_pose->landmarks[analysis.joint].position.y - end_hip_center.y,
        end_pose->landmarks[analysis.joint].position.z - end_hip_center.z};

    float start_to_end = distance_3d(&start_relative, &end_relative);
    float current_to_end = distance_3d(&current_relative, &end_relative);

    float ratio;
    float weight = analysis.weight; // 미리 분석된 가중치 사용

    if (analysis.is_important && start_to_end > 5.0f) {
      // 중요 관절: 정확한 진행도 계산
      ratio = 1.0f - (current_to_end / start_to_end);
      ratio = fmaxf(0.0f, ratio);
      ratio = fminf(1.0f, ratio * 3.0f); // 300% 보너스
    } else if (analysis.is_important) {
      // 중요 관절이지만 움직임이 적음
      ratio = 1.0f;
    } else {
      // 덜 중요한 관절: 더 관대한 유사도 체크
      ratio = (current_to_end < 80.0f) ? 1.0f : 0.7f;
    }

    weighted_progress += ratio * weight;
    total_weight += weight;
  }

  if (total_weight == 0.0f) {
    return 0.0f;
  }

  float progress = weighted_progress / total_weight;

  // 진행도에 20% 추가 (최대 1.0을 초과하지 않도록 제한)
  progress = progress * 1.3f;

  return fmaxf(0.0f, fminf(1.0f, progress));
}
