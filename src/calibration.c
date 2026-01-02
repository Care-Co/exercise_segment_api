/**
 * @file calibration.c
 * @brief 캘리브레이션 구현 (ML Kit 33개 랜드마크 지원)
 * @author Exercise Segment API Team
 * @version 2.0.0
 */

#include "../include/calibration.h"
#include "../include/math_utils.h"
#include "../include/segment_api.h"
#include "../include/segment_types.h"
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

// 전역 변수들은 calibration.h에서 extern 선언됨

// 관절 연결 관계 정의
JointConnection g_joint_connections[20];

// segment_calibrate_recorder는 segment_core.c에서 구현됨

/**
 * @brief 공통 캘리브레이션 로직 구현
 */
int segment_calibrate_common(const PoseData *base_pose,
                             CalibrationData *out_calibration,
                             const char *calibration_type) {
  if (!base_pose || !out_calibration) {
    printf("❌ %s 캘리브레이션 실패: 잘못된 매개변수\n", calibration_type);
    return SEGMENT_ERROR_INVALID_PARAMETER;
  }

  // 포즈 데이터 유효성 검사
  if (!segment_validate_pose(base_pose)) {
    printf("❌ %s 캘리브레이션 실패: 유효하지 않은 포즈 데이터\n",
           calibration_type);
    return SEGMENT_ERROR_INVALID_POSE;
  }

  // 필수 관절들의 신뢰도 확인 (개선된 기준)
  float leftShoulderConf =
      base_pose->landmarks[POSE_LANDMARK_LEFT_SHOULDER].inFrameLikelihood;
  float rightShoulderConf =
      base_pose->landmarks[POSE_LANDMARK_RIGHT_SHOULDER].inFrameLikelihood;
  float leftHipConf =
      base_pose->landmarks[POSE_LANDMARK_LEFT_HIP].inFrameLikelihood;
  float rightHipConf =
      base_pose->landmarks[POSE_LANDMARK_RIGHT_HIP].inFrameLikelihood;

  // 개선된 신뢰도 검사
  if (leftShoulderConf < CALIBRATION_MIN_CONFIDENCE ||
      rightShoulderConf < CALIBRATION_MIN_CONFIDENCE ||
      leftHipConf < CALIBRATION_MIN_CONFIDENCE ||
      rightHipConf < CALIBRATION_MIN_CONFIDENCE) {
    printf("❌ %s 캘리브레이션 실패: 신뢰도 부족 - 어깨(L:%.2f, R:%.2f), "
           "엉덩이(L:%.2f, R:%.2f) [최소: %.2f]\n",
           calibration_type, leftShoulderConf, rightShoulderConf, leftHipConf,
           rightHipConf, CALIBRATION_MIN_CONFIDENCE);
    return SEGMENT_ERROR_CALIBRATION_FAILED;
  }

  printf("STEP 1: %s 캘리브레이션 진행 (신뢰도 검증 완료)\n", calibration_type);

  // 어깨 너비 계산
  float user_shoulder_width =
      distance_3d(&base_pose->landmarks[POSE_LANDMARK_LEFT_SHOULDER].position,
                  &base_pose->landmarks[POSE_LANDMARK_RIGHT_SHOULDER].position);

  // 어깨 너비 유효성 검사 (개선된 범위)
  if (user_shoulder_width < CALIBRATION_MIN_SHOULDER_WIDTH ||
      user_shoulder_width > CALIBRATION_MAX_SHOULDER_WIDTH) {
    printf("❌ %s 캘리브레이션 실패: 어깨 너비 범위 초과 (%.2f) [범위: "
           "%.1f~%.1f]\n",
           calibration_type, user_shoulder_width,
           CALIBRATION_MIN_SHOULDER_WIDTH, CALIBRATION_MAX_SHOULDER_WIDTH);
    return SEGMENT_ERROR_CALIBRATION_FAILED;
  }

  printf("📏 %s 어깨 너비: %.2f\n", calibration_type, user_shoulder_width);

  // 스케일 팩터 계산 (개선된 상수 사용)
  out_calibration->scale_factor =
      user_shoulder_width / CALIBRATION_IDEAL_SHOULDER_WIDTH;

  // 스케일 팩터 유효성 검사 (개선된 범위)
  if (out_calibration->scale_factor < CALIBRATION_MIN_SCALE_FACTOR ||
      out_calibration->scale_factor > CALIBRATION_MAX_SCALE_FACTOR) {
    printf("❌ %s 캘리브레이션 실패: 스케일 팩터 범위 초과 (%.3f) [범위: "
           "%.1f~%.1f]\n",
           calibration_type, out_calibration->scale_factor,
           CALIBRATION_MIN_SCALE_FACTOR, CALIBRATION_MAX_SCALE_FACTOR);
    return SEGMENT_ERROR_CALIBRATION_FAILED;
  }

  printf("📐 %s 스케일 팩터: %.3f (정상 범위)\n", calibration_type,
         out_calibration->scale_factor);

  // 중심점 오프셋 계산
  Point3D user_center_3d = calculate_pose_center(base_pose);
  Point3D ideal_center_3d = calculate_pose_center(&g_ideal_base_pose);

  out_calibration->center_offset.x = ideal_center_3d.x - user_center_3d.x;
  out_calibration->center_offset.y = ideal_center_3d.y - user_center_3d.y;
  out_calibration->center_offset.z = 0.0f; // z는 0으로 설정

  // 캘리브레이션 완료 플래그 설정
  out_calibration->is_calibrated = true;
  out_calibration->calibration_quality = CALIBRATION_DEFAULT_QUALITY;

  // 관절별 길이 켈리브레이션 수행
  printf("\n🔧 %s 관절별 길이 켈리브레이션 시작...\n", calibration_type);
  int joint_result =
      segment_calibrate_joint_lengths(base_pose, out_calibration);
  if (joint_result != SEGMENT_OK) {
    printf("⚠️  %s 관절별 길이 켈리브레이션 실패, 기본 켈리브레이션만 적용\n",
           calibration_type);
  }

  printf("RESULT: %s 캘리브레이션 완료 (품질: %.2f)\n", calibration_type,
         out_calibration->calibration_quality);
  printf("  - 어깨너비: %.1f, 스케일: %.3f, 오프셋: (%.1f, %.1f)\n",
         user_shoulder_width, out_calibration->scale_factor,
         out_calibration->center_offset.x, out_calibration->center_offset.y);

  return SEGMENT_OK;
}

int segment_calibrate_user(const PoseData *base_pose) {
  // 공통 캘리브레이션 로직 사용
  int result =
      segment_calibrate_common(base_pose, &g_user_calibration, "사용자");

  if (result == SEGMENT_OK) {
    // 중요: 사용자 캘리브레이션 완료 플래그 설정
    g_user_calibrated = true;

    // 관절별 길이 정보 출력 (디버깅용)
    print_joint_lengths(&g_user_calibration);
  }

  return result;
}

/**
 * @brief 관절 트리에서 특정 관절 연결을 조정하는 헬퍼 함수
 */
static int adjust_joint_with_tree(PoseData *pose,
                                  const CalibrationData *calibration,
                                  PoseLandmarkType from_joint,
                                  PoseLandmarkType to_joint,
                                  const char *joint_name) {
  // 캘리브레이션 데이터에서 해당 관절 연결 찾기
  for (int i = 0; i < calibration->joint_lengths.count; i++) {
    const JointLength *joint_length = &calibration->joint_lengths.lengths[i];

    if (!joint_length->is_valid)
      continue;

    // 연결 인덱스로 관절 연결 정보 가져오기
    int conn_idx = joint_length->connection_index;
    if (conn_idx >= 20)
      continue; // 안전성 체크

    JointConnection *conn = &g_joint_connections[conn_idx];

    // 찾는 관절 연결인지 확인
    if (conn->from_joint == from_joint && conn->to_joint == to_joint) {

      // 🎯 기준 관절(from_joint) 고정: 기준 관절은 절대 변경하지 않음
      Point3D from_pos = pose->landmarks[from_joint].position;
      // 대상 관절(to_joint)의 현재 좌표 (이전 조정 결과가 반영된 최신 값)
      Point3D to_pos = pose->landmarks[to_joint].position;

      // 현재 방향 벡터 계산 (from_joint → to_joint, 2D 거리)
      float dx = to_pos.x - from_pos.x;
      float dy = to_pos.y - from_pos.y;
      float current_length = sqrtf(dx * dx + dy * dy);

      if (current_length <= 0.0f) {
        return 0;
      }

      // 🎯 캘리브레이션된 사용자 절대 길이 적용
      float target_length = joint_length->user_length;

      // 방향 벡터 정규화 (같은 방향 유지, 길이만 1로 정규화)
      float direction_x = dx / current_length;
      float direction_y = dy / current_length;

      // 🎯 기준 관절(from_pos)에서 같은 방향 벡터로 목표 길이만큼 떨어진 위치로
      // 조정 새로운 to_joint 위치 = from_pos + (정규화된 방향 벡터) * 목표 길이
      pose->landmarks[to_joint].position.x =
          from_pos.x + direction_x * target_length;
      pose->landmarks[to_joint].position.y =
          from_pos.y + direction_y * target_length;
      pose->landmarks[to_joint].position.z = to_pos.z; // Z축은 유지

      // 조정 후 길이 검증 (2D 거리로 검증)
      Point3D new_to_pos = pose->landmarks[to_joint].position;
      float dx_new = new_to_pos.x - from_pos.x;
      float dy_new = new_to_pos.y - from_pos.y;
      float actual_length = sqrtf(dx_new * dx_new + dy_new * dy_new);

      // 정강이 길이가 비정상적으로 짧아지는 경우 디버깅
      if (strstr(joint_name, "정강") && actual_length < 50.0f) {
        printf("  ⚠️ %s 길이 이상: 목표 %.1f → 실제 %.1f\n", joint_name,
               target_length, actual_length);
        printf("    from: (%.1f,%.1f,%.1f) to: (%.1f,%.1f,%.1f)\n", from_pos.x,
               from_pos.y, from_pos.z, new_to_pos.x, new_to_pos.y,
               new_to_pos.z);
      }

      // 조정 완료
      return 1; // 성공
    }
  }

  return 0; // 해당 관절 연결을 찾지 못함
}

/**
 * @brief 관절 트리 순회를 통한 스마트 포즈 피팅
 * 골반 중심을 기준으로 상체→팔, 다리 순서로 관절 길이를 조정
 * 캘리브레이션된 사용자 관절 길이를 목표 포즈에 직접 적용
 */
int smart_pose_fit(const PoseData *target_pose,
                   const CalibrationData *calibration,
                   PoseData *adjusted_pose) {
  if (!target_pose || !calibration || !adjusted_pose) {
    printf("❌ 스마트 포즈 피팅 실패: 잘못된 매개변수\n");
    return SEGMENT_ERROR_INVALID_PARAMETER;
  }

  if (!calibration->is_calibrated || calibration->joint_lengths.count == 0) {
    *adjusted_pose = *target_pose;
    return SEGMENT_OK;
  }

  // 원본 포즈 복사
  *adjusted_pose = *target_pose;

  // 🏗️ 관절 트리 순회: 골반 → 상체 → 팔들, 골반 → 다리들

  int adjustments = 0;

  // 2️⃣ 좌상체 (골반 → 어깨) 조정 (팔 조정 전에 수행)
  adjustments +=
      adjust_joint_with_tree(adjusted_pose, calibration, POSE_LANDMARK_LEFT_HIP,
                             POSE_LANDMARK_LEFT_SHOULDER, "좌상체");

  // 3️⃣ 우상체 (골반 → 어깨) 조정 (팔 조정 전에 수행)
  adjustments += adjust_joint_with_tree(adjusted_pose, calibration,
                                        POSE_LANDMARK_RIGHT_HIP,
                                        POSE_LANDMARK_RIGHT_SHOULDER, "우상체");

  // 4️⃣ 좌상완 (어깨 → 팔꿈치) 조정
  adjustments += adjust_joint_with_tree(adjusted_pose, calibration,
                                        POSE_LANDMARK_LEFT_SHOULDER,
                                        POSE_LANDMARK_LEFT_ELBOW, "좌상완");

  // 5️⃣ 좌전완 (팔꿈치 → 손목) 조정 (상완 조정 후)
  adjustments += adjust_joint_with_tree(adjusted_pose, calibration,
                                        POSE_LANDMARK_LEFT_ELBOW,
                                        POSE_LANDMARK_LEFT_WRIST, "좌전완");

  // 6️⃣ 우상완 (어깨 → 팔꿈치) 조정
  adjustments += adjust_joint_with_tree(adjusted_pose, calibration,
                                        POSE_LANDMARK_RIGHT_SHOULDER,
                                        POSE_LANDMARK_RIGHT_ELBOW, "우상완");

  // 7️⃣ 우전완 (팔꿈치 → 손목) 조정
  adjustments += adjust_joint_with_tree(adjusted_pose, calibration,
                                        POSE_LANDMARK_RIGHT_ELBOW,
                                        POSE_LANDMARK_RIGHT_WRIST, "우전완");

  // 8️⃣ 좌대퇴 (골반 → 무릎) 조정
  adjustments +=
      adjust_joint_with_tree(adjusted_pose, calibration, POSE_LANDMARK_LEFT_HIP,
                             POSE_LANDMARK_LEFT_KNEE, "좌대퇴");

  // 9️⃣ 좌정강 (무릎 → 발목) 조정 (대퇴 조정 후!)
  adjustments += adjust_joint_with_tree(adjusted_pose, calibration,
                                        POSE_LANDMARK_LEFT_KNEE,
                                        POSE_LANDMARK_LEFT_ANKLE, "좌정강");

  // 🔟 우대퇴 (골반 → 무릎) 조정
  adjustments += adjust_joint_with_tree(adjusted_pose, calibration,
                                        POSE_LANDMARK_RIGHT_HIP,
                                        POSE_LANDMARK_RIGHT_KNEE, "우대퇴");

  // 1️⃣1️⃣ 우정강 (무릎 → 발목) 조정 (대퇴 조정 후!)
  adjustments += adjust_joint_with_tree(adjusted_pose, calibration,
                                        POSE_LANDMARK_RIGHT_KNEE,
                                        POSE_LANDMARK_RIGHT_ANKLE, "우정강");

  return SEGMENT_OK;
}

CalibrationData *get_calibration_data(void) {
  if (!g_recorder_calibrated) {
    return NULL;
  }
  return &g_recorder_calibration;
}

bool is_calibrated(void) {
  return g_recorder_calibrated && g_recorder_calibration.is_calibrated;
}

void reset_calibration(void) {
  memset(&g_recorder_calibration, 0, sizeof(CalibrationData));
  g_recorder_calibrated = false;
}

int segment_calibrate(const PoseData *base_pose,
                      CalibrationData *out_calibration) {
  // 기본 캘리브레이션 함수 (기록자용)
  int result = segment_calibrate_recorder(base_pose);
  if (result == SEGMENT_OK && out_calibration) {
    *out_calibration = g_recorder_calibration;
  }
  return result;
}

bool segment_validate_calibration(const CalibrationData *calibration) {
  if (!calibration) {
    return false;
  }

  // 캘리브레이션 완료 여부 확인
  if (!calibration->is_calibrated) {
    return false;
  }

  // 스케일 팩터 유효성 검사
  if (calibration->scale_factor <= 0.1f || calibration->scale_factor >= 10.0f) {
    return false;
  }

  // 캘리브레이션 품질 검사
  if (calibration->calibration_quality < 0.5f) {
    return false;
  }

  return true;
}

float calculate_pose_scale_factor(const PoseData *pose) {
  if (!pose) {
    return 1.0f;
  }

  // 어깨 너비를 기준으로 스케일 팩터 계산
  float shoulder_width =
      distance_3d(&pose->landmarks[POSE_LANDMARK_LEFT_SHOULDER].position,
                  &pose->landmarks[POSE_LANDMARK_RIGHT_SHOULDER].position);

  if (shoulder_width <= 0.0f) {
    return 1.0f;
  }

  // 개선된 상수 사용
  return CALIBRATION_IDEAL_SHOULDER_WIDTH / shoulder_width;
}

Point3D calculate_pose_center(const PoseData *pose) {
  Point3D center = {0.0f, 0.0f, 0.0f};

  if (!pose) {
    return center;
  }

  // 모든 랜드마크의 평균 위치 계산
  for (int i = 0; i < POSE_LANDMARK_COUNT; i++) {
    center.x += pose->landmarks[i].position.x;
    center.y += pose->landmarks[i].position.y;
    center.z += pose->landmarks[i].position.z;
  }

  center.x /= POSE_LANDMARK_COUNT;
  center.y /= POSE_LANDMARK_COUNT;
  center.z /= POSE_LANDMARK_COUNT;

  return center;
}

// MARK: - 관절별 길이 켈리브레이션 함수들

int initialize_joint_connections(void) {
  // 주요 관절 연결 관계 정의 (인체 해부학적 구조 기반)
  g_joint_connections[0] = (JointConnection){
      POSE_LANDMARK_LEFT_SHOULDER, POSE_LANDMARK_LEFT_ELBOW, "좌상완"};
  g_joint_connections[1] = (JointConnection){
      POSE_LANDMARK_LEFT_ELBOW, POSE_LANDMARK_LEFT_WRIST, "좌전완"};
  g_joint_connections[2] = (JointConnection){
      POSE_LANDMARK_RIGHT_SHOULDER, POSE_LANDMARK_RIGHT_ELBOW, "우상완"};
  g_joint_connections[3] = (JointConnection){
      POSE_LANDMARK_RIGHT_ELBOW, POSE_LANDMARK_RIGHT_WRIST, "우전완"};

  g_joint_connections[4] = (JointConnection){POSE_LANDMARK_LEFT_HIP,
                                             POSE_LANDMARK_LEFT_KNEE, "좌대퇴"};
  g_joint_connections[5] = (JointConnection){
      POSE_LANDMARK_LEFT_KNEE, POSE_LANDMARK_LEFT_ANKLE, "좌정강"};
  g_joint_connections[6] = (JointConnection){
      POSE_LANDMARK_RIGHT_HIP, POSE_LANDMARK_RIGHT_KNEE, "우대퇴"};
  g_joint_connections[7] = (JointConnection){
      POSE_LANDMARK_RIGHT_KNEE, POSE_LANDMARK_RIGHT_ANKLE, "우정강"};

  g_joint_connections[8] = (JointConnection){
      POSE_LANDMARK_LEFT_HIP, POSE_LANDMARK_LEFT_SHOULDER, "좌상체"};
  g_joint_connections[9] = (JointConnection){
      POSE_LANDMARK_RIGHT_HIP, POSE_LANDMARK_RIGHT_SHOULDER, "우상체"};

  g_joint_connections[10] = (JointConnection){
      POSE_LANDMARK_LEFT_SHOULDER, POSE_LANDMARK_RIGHT_SHOULDER, "어깨너비"};
  g_joint_connections[11] = (JointConnection){
      POSE_LANDMARK_LEFT_HIP, POSE_LANDMARK_RIGHT_HIP, "골반너비"};

  g_joint_connections[12] = (JointConnection){
      POSE_LANDMARK_NOSE, POSE_LANDMARK_LEFT_SHOULDER, "목-좌어깨"};
  g_joint_connections[13] = (JointConnection){
      POSE_LANDMARK_NOSE, POSE_LANDMARK_RIGHT_SHOULDER, "목-우어깨"};

  g_joint_connections[14] = (JointConnection){
      POSE_LANDMARK_LEFT_ANKLE, POSE_LANDMARK_LEFT_HEEL, "좌발길이"};
  g_joint_connections[15] = (JointConnection){
      POSE_LANDMARK_RIGHT_ANKLE, POSE_LANDMARK_RIGHT_HEEL, "우발길이"};

  g_joint_connections[16] = (JointConnection){
      POSE_LANDMARK_LEFT_WRIST, POSE_LANDMARK_LEFT_INDEX, "좌손길이"};
  g_joint_connections[17] = (JointConnection){
      POSE_LANDMARK_RIGHT_WRIST, POSE_LANDMARK_RIGHT_INDEX, "우손길이"};

  g_joint_connections[18] = (JointConnection){
      POSE_LANDMARK_LEFT_ANKLE, POSE_LANDMARK_LEFT_FOOT_INDEX, "좌발가락"};
  g_joint_connections[19] = (JointConnection){
      POSE_LANDMARK_RIGHT_ANKLE, POSE_LANDMARK_RIGHT_FOOT_INDEX, "우발가락"};

  return 20; // 총 20개 연결
}

float calculate_joint_distance(const PoseData *pose,
                               PoseLandmarkType from_joint,
                               PoseLandmarkType to_joint) {
  if (!pose) {
    return -1.0f;
  }

  // 관절 인덱스 유효성 검사
  if (from_joint >= POSE_LANDMARK_COUNT || to_joint >= POSE_LANDMARK_COUNT) {
    return -1.0f;
  }

  // 신뢰도 검사 제거 - 모든 관절 측정
  // float from_conf = pose->landmarks[from_joint].inFrameLikelihood;
  // float to_conf = pose->landmarks[to_joint].inFrameLikelihood;

  // if (from_conf < 0.3f || to_conf < 0.3f) {
  //   return -1.0f; // 신뢰도가 너무 낮음
  // }

  // 2D 거리 계산 (Z축 제외 - 깊이 정보 부정확)
  Point3D from_pos = pose->landmarks[from_joint].position;
  Point3D to_pos = pose->landmarks[to_joint].position;

  float dx = to_pos.x - from_pos.x;
  float dy = to_pos.y - from_pos.y;
  // Z축 제외: float dz = to_pos.z - from_pos.z;

  return sqrtf(dx * dx + dy * dy);
}

int segment_calibrate_joint_lengths(const PoseData *base_pose,
                                    CalibrationData *out_calibration) {
  if (!base_pose || !out_calibration) {
    return SEGMENT_ERROR_INVALID_PARAMETER;
  }

  // 관절 연결 관계 초기화
  int connection_count = initialize_joint_connections();

  // 관절 길이 켈리브레이션 초기화
  out_calibration->joint_lengths.count = 0;

  // 관절별 길이 측정 (로그 최소화)
  int success_count = 0;

  for (int i = 0; i < connection_count; i++) {
    JointConnection *conn = &g_joint_connections[i];

    // 사용자 관절 길이 계산
    float user_length =
        calculate_joint_distance(base_pose, conn->from_joint, conn->to_joint);

    if (user_length < 0.0f) {
      continue;
    }

    // 디버깅: 다리 관절의 실제 좌표 출력
    if (conn->from_joint == POSE_LANDMARK_LEFT_KNEE &&
        conn->to_joint == POSE_LANDMARK_LEFT_ANKLE) {
      Point3D knee_pos = base_pose->landmarks[POSE_LANDMARK_LEFT_KNEE].position;
      Point3D ankle_pos =
          base_pose->landmarks[POSE_LANDMARK_LEFT_ANKLE].position;
      printf("🔍 좌정강 캘리브레이션 좌표:\n");
      printf("  무릎: (%.1f, %.1f, %.1f)\n", knee_pos.x, knee_pos.y,
             knee_pos.z);
      printf("  발목: (%.1f, %.1f, %.1f)\n", ankle_pos.x, ankle_pos.y,
             ankle_pos.z);
      printf("  계산된 길이: %.1f\n", user_length);
    }

    // 이상적 관절 길이 계산 (표준 포즈 기준)
    float ideal_length = calculate_joint_distance(
        &g_ideal_base_pose, conn->from_joint, conn->to_joint);

    if (ideal_length <= 0.0f) {
      continue;
    }

    // 스케일 팩터 계산
    float scale_factor = user_length / ideal_length;

    // 스케일 팩터 유효성 검사 (개선된 범위)
    if (scale_factor < CALIBRATION_MIN_SCALE_FACTOR ||
        scale_factor > CALIBRATION_MAX_SCALE_FACTOR) {
      continue;
    }

    // 관절 길이 정보 저장
    JointLength *joint_length =
        &out_calibration->joint_lengths
             .lengths[out_calibration->joint_lengths.count];
    joint_length->connection_index = i; // 연결 인덱스 저장
    joint_length->ideal_length = ideal_length;
    joint_length->user_length = user_length;
    joint_length->scale_factor = scale_factor;
    joint_length->is_valid = true;

    out_calibration->joint_lengths.count++;
    success_count++;
  }

  printf("  - 관절별 비율 계산 완료: %d/%d개 성공\n", success_count,
         connection_count);

  return SEGMENT_OK;
}

int apply_joint_length_calibration(const PoseData *original_pose,
                                   const CalibrationData *calibration,
                                   PoseData *calibrated_pose) {
  if (!original_pose || !calibration || !calibrated_pose) {
    return SEGMENT_ERROR_INVALID_PARAMETER;
  }

  // 원본 포즈 복사
  *calibrated_pose = *original_pose;

  // 관절별 길이 켈리브레이션 적용
  for (int i = 0; i < calibration->joint_lengths.count; i++) {
    const JointLength *joint_length = &calibration->joint_lengths.lengths[i];

    if (!joint_length->is_valid) {
      continue;
    }

    // 해당 관절 연결에 대한 스케일링 적용
    // (실제 구현에서는 더 정교한 변환 필요)
    // 여기서는 간단히 전체 스케일 팩터를 사용
    PoseLandmarkType from_joint = g_joint_connections[i].from_joint;
    PoseLandmarkType to_joint = g_joint_connections[i].to_joint;

    // 관절 위치를 중심점 기준으로 스케일링
    Point3D center = calculate_pose_center(original_pose);

    // 시작 관절 스케일링
    calibrated_pose->landmarks[from_joint].position.x =
        center.x +
        (original_pose->landmarks[from_joint].position.x - center.x) *
            joint_length->scale_factor;
    calibrated_pose->landmarks[from_joint].position.y =
        center.y +
        (original_pose->landmarks[from_joint].position.y - center.y) *
            joint_length->scale_factor;

    // 끝 관절 스케일링
    calibrated_pose->landmarks[to_joint].position.x =
        center.x + (original_pose->landmarks[to_joint].position.x - center.x) *
                       joint_length->scale_factor;
    calibrated_pose->landmarks[to_joint].position.y =
        center.y + (original_pose->landmarks[to_joint].position.y - center.y) *
                       joint_length->scale_factor;
  }

  return SEGMENT_OK;
}

void print_joint_lengths(const CalibrationData *calibration) {
  if (!calibration) {
    printf("❌ 켈리브레이션 데이터가 없습니다.\n");
    return;
  }

  printf("\n관절별 길이 켈리브레이션 정보:\n");

  printf("총 %d개 관절 연결이 켈리브레이션되었습니다.\n",
         calibration->joint_lengths.count);
}

/**
 * @brief 포즈의 주요 관절 길이를 상세 로깅
 */
void debug_log_pose_lengths(const PoseData *pose, const char *label) {
  if (!pose || !label) {
    return;
  }

  printf("\n🔍 [%s] 상세 관절 길이 분석:\n", label);
  printf("================================\n");

  // 팔 길이
  float left_upper_arm = calculate_joint_distance(
      pose, POSE_LANDMARK_LEFT_SHOULDER, POSE_LANDMARK_LEFT_ELBOW);
  float left_forearm = calculate_joint_distance(pose, POSE_LANDMARK_LEFT_ELBOW,
                                                POSE_LANDMARK_LEFT_WRIST);
  float right_upper_arm = calculate_joint_distance(
      pose, POSE_LANDMARK_RIGHT_SHOULDER, POSE_LANDMARK_RIGHT_ELBOW);
  float right_forearm = calculate_joint_distance(
      pose, POSE_LANDMARK_RIGHT_ELBOW, POSE_LANDMARK_RIGHT_WRIST);

  printf("🦾 팔 길이:\n");
  printf("  좌상완: %.2f | 좌전완: %.2f | 좌팔전체: %.2f\n", left_upper_arm,
         left_forearm, left_upper_arm + left_forearm);
  printf("  우상완: %.2f | 우전완: %.2f | 우팔전체: %.2f\n", right_upper_arm,
         right_forearm, right_upper_arm + right_forearm);

  // 다리 길이 (핵심!)
  float left_thigh = calculate_joint_distance(pose, POSE_LANDMARK_LEFT_HIP,
                                              POSE_LANDMARK_LEFT_KNEE);
  float left_shin = calculate_joint_distance(pose, POSE_LANDMARK_LEFT_KNEE,
                                             POSE_LANDMARK_LEFT_ANKLE);
  float right_thigh = calculate_joint_distance(pose, POSE_LANDMARK_RIGHT_HIP,
                                               POSE_LANDMARK_RIGHT_KNEE);
  float right_shin = calculate_joint_distance(pose, POSE_LANDMARK_RIGHT_KNEE,
                                              POSE_LANDMARK_RIGHT_ANKLE);

  printf("🦵 다리 길이 (중요!):\n");
  printf("  좌대퇴: %.2f | 좌정강: %.2f | 좌다리전체: %.2f\n", left_thigh,
         left_shin, left_thigh + left_shin);
  printf("  우대퇴: %.2f | 우정강: %.2f | 우다리전체: %.2f\n", right_thigh,
         right_shin, right_thigh + right_shin);

  // 몸통
  float left_torso = calculate_joint_distance(pose, POSE_LANDMARK_LEFT_SHOULDER,
                                              POSE_LANDMARK_LEFT_HIP);
  float right_torso = calculate_joint_distance(
      pose, POSE_LANDMARK_RIGHT_SHOULDER, POSE_LANDMARK_RIGHT_HIP);
  float shoulder_width = calculate_joint_distance(
      pose, POSE_LANDMARK_LEFT_SHOULDER, POSE_LANDMARK_RIGHT_SHOULDER);
  float hip_width = calculate_joint_distance(pose, POSE_LANDMARK_LEFT_HIP,
                                             POSE_LANDMARK_RIGHT_HIP);

  printf("🏃 몸통:\n");
  printf("  좌상체: %.2f | 우상체: %.2f\n", left_torso, right_torso);
  printf("  어깨너비: %.2f | 골반너비: %.2f\n", shoulder_width, hip_width);

  printf("================================\n");
}

/**
 * @brief 두 포즈 간 길이 비교 상세 로깅
 */
void debug_compare_pose_lengths(const PoseData *pose1, const char *label1,
                                const PoseData *pose2, const char *label2) {
  if (!pose1 || !pose2 || !label1 || !label2) {
    return;
  }

  printf("\n📊 [%s] vs [%s] 비교 분석:\n", label1, label2);
  printf("=========================================\n");

  // 다리 길이 비교 (가장 중요!)
  printf("🦵 다리 길이 변화:\n");

  float pose1_left_thigh = calculate_joint_distance(
      pose1, POSE_LANDMARK_LEFT_HIP, POSE_LANDMARK_LEFT_KNEE);
  float pose1_left_shin = calculate_joint_distance(
      pose1, POSE_LANDMARK_LEFT_KNEE, POSE_LANDMARK_LEFT_ANKLE);
  float pose1_left_total = pose1_left_thigh + pose1_left_shin;

  float pose2_left_thigh = calculate_joint_distance(
      pose2, POSE_LANDMARK_LEFT_HIP, POSE_LANDMARK_LEFT_KNEE);
  float pose2_left_shin = calculate_joint_distance(
      pose2, POSE_LANDMARK_LEFT_KNEE, POSE_LANDMARK_LEFT_ANKLE);
  float pose2_left_total = pose2_left_thigh + pose2_left_shin;

  float pose1_right_thigh = calculate_joint_distance(
      pose1, POSE_LANDMARK_RIGHT_HIP, POSE_LANDMARK_RIGHT_KNEE);
  float pose1_right_shin = calculate_joint_distance(
      pose1, POSE_LANDMARK_RIGHT_KNEE, POSE_LANDMARK_RIGHT_ANKLE);
  float pose1_right_total = pose1_right_thigh + pose1_right_shin;

  float pose2_right_thigh = calculate_joint_distance(
      pose2, POSE_LANDMARK_RIGHT_HIP, POSE_LANDMARK_RIGHT_KNEE);
  float pose2_right_shin = calculate_joint_distance(
      pose2, POSE_LANDMARK_RIGHT_KNEE, POSE_LANDMARK_RIGHT_ANKLE);
  float pose2_right_total = pose2_right_thigh + pose2_right_shin;

  printf("  좌다리 전체: %.2f → %.2f (변화: %+.2f, 비율: %.3f)\n",
         pose1_left_total, pose2_left_total,
         pose2_left_total - pose1_left_total,
         pose1_left_total > 0 ? pose2_left_total / pose1_left_total : 0.0f);
  printf("    - 좌대퇴: %.2f → %.2f (변화: %+.2f)\n", pose1_left_thigh,
         pose2_left_thigh, pose2_left_thigh - pose1_left_thigh);
  printf("    - 좌정강: %.2f → %.2f (변화: %+.2f)\n", pose1_left_shin,
         pose2_left_shin, pose2_left_shin - pose1_left_shin);

  printf("  우다리 전체: %.2f → %.2f (변화: %+.2f, 비율: %.3f)\n",
         pose1_right_total, pose2_right_total,
         pose2_right_total - pose1_right_total,
         pose1_right_total > 0 ? pose2_right_total / pose1_right_total : 0.0f);
  printf("    - 우대퇴: %.2f → %.2f (변화: %+.2f)\n", pose1_right_thigh,
         pose2_right_thigh, pose2_right_thigh - pose1_right_thigh);
  printf("    - 우정강: %.2f → %.2f (변화: %+.2f)\n", pose1_right_shin,
         pose2_right_shin, pose2_right_shin - pose1_right_shin);

  // 팔 길이 비교
  printf("\n🦾 팔 길이 변화:\n");

  float pose1_left_arm =
      calculate_joint_distance(pose1, POSE_LANDMARK_LEFT_SHOULDER,
                               POSE_LANDMARK_LEFT_ELBOW) +
      calculate_joint_distance(pose1, POSE_LANDMARK_LEFT_ELBOW,
                               POSE_LANDMARK_LEFT_WRIST);
  float pose2_left_arm =
      calculate_joint_distance(pose2, POSE_LANDMARK_LEFT_SHOULDER,
                               POSE_LANDMARK_LEFT_ELBOW) +
      calculate_joint_distance(pose2, POSE_LANDMARK_LEFT_ELBOW,
                               POSE_LANDMARK_LEFT_WRIST);

  float pose1_right_arm =
      calculate_joint_distance(pose1, POSE_LANDMARK_RIGHT_SHOULDER,
                               POSE_LANDMARK_RIGHT_ELBOW) +
      calculate_joint_distance(pose1, POSE_LANDMARK_RIGHT_ELBOW,
                               POSE_LANDMARK_RIGHT_WRIST);
  float pose2_right_arm =
      calculate_joint_distance(pose2, POSE_LANDMARK_RIGHT_SHOULDER,
                               POSE_LANDMARK_RIGHT_ELBOW) +
      calculate_joint_distance(pose2, POSE_LANDMARK_RIGHT_ELBOW,
                               POSE_LANDMARK_RIGHT_WRIST);

  printf("  좌팔 전체: %.2f → %.2f (변화: %+.2f, 비율: %.3f)\n", pose1_left_arm,
         pose2_left_arm, pose2_left_arm - pose1_left_arm,
         pose1_left_arm > 0 ? pose2_left_arm / pose1_left_arm : 0.0f);
  printf("  우팔 전체: %.2f → %.2f (변화: %+.2f, 비율: %.3f)\n",
         pose1_right_arm, pose2_right_arm, pose2_right_arm - pose1_right_arm,
         pose1_right_arm > 0 ? pose2_right_arm / pose1_right_arm : 0.0f);

  printf("=========================================\n");
}
