/**
 * @file calibration.h
 * @brief 캘리브레이션 관련 함수들 선언
 * @author Exercise Segment API Team
 * @version 1.0.0
 */

#ifndef CALIBRATION_H
#define CALIBRATION_H

#include "segment_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 사용자의 기본 포즈로 개인화 캘리브레이션 수행
 * @param base_pose 사용자가 자연스럽게 서있는 자세의 포즈 데이터
 * @param out_calibration 계산된 캘리브레이션 결과를 저장할 구조체
 * @return SEGMENT_OK 성공, 음수 에러 코드
 */
int segment_calibrate(const PoseData *base_pose,
                      CalibrationData *out_calibration);

/**
 * @brief 두 포즈 간 캘리브레이션 데이터 계산
 * @param user_pose 사용자 포즈
 * @param reference_pose 참조 포즈 (이상적 포즈)
 * @param out_calibration 계산된 캘리브레이션 결과를 저장할 구조체
 * @return SEGMENT_OK 성공, 음수 에러 코드
 */
int segment_calibrate_between_poses(const PoseData *user_pose,
                                    const PoseData *reference_pose,
                                    CalibrationData *out_calibration);

/**
 * @brief 캘리브레이션 데이터 유효성 검사
 * @param calibration 검사할 캘리브레이션 데이터
 * @return true 유효, false 무효
 */
bool segment_validate_calibration(const CalibrationData *calibration);

/**
 * @brief 캘리브레이션을 포즈에 적용
 * @param original_pose 원본 포즈
 * @param calibration 캘리브레이션 데이터
 * @param calibrated_pose 캘리브레이션된 포즈를 저장할 구조체
 * @return SEGMENT_OK 성공, 음수 에러 코드
 */
int apply_calibration_to_pose(const PoseData *original_pose,
                              const CalibrationData *calibration,
                              PoseData *calibrated_pose);

/**
 * @brief 포즈의 스케일 팩터 계산
 * @param pose 포즈 데이터
 * @return 스케일 팩터
 */
float calculate_pose_scale_factor(const PoseData *pose);

/**
 * @brief 현재 캘리브레이션 데이터 가져오기
 * @return 캘리브레이션 데이터 포인터 (없으면 NULL)
 */
CalibrationData *get_calibration_data(void);

/**
 * @brief 포즈의 중심점 계산
 * @param pose 포즈 데이터
 * @return 중심점
 */
Point3D calculate_pose_center(const PoseData *pose);

/**
 * @brief 캘리브레이션 상태 확인
 * @return true 캘리브레이션 완료, false 미완료
 */
bool is_calibrated(void);

/**
 * @brief 캘리브레이션 데이터 초기화
 */
void reset_calibration(void);

/**
 * @brief 관절 간 길이 계산
 * @param pose 포즈 데이터
 * @param from_joint 시작 관절
 * @param to_joint 끝 관절
 * @return 관절 간 거리 (실패시 -1.0f)
 */
float calculate_joint_distance(const PoseData *pose,
                               PoseLandmarkType from_joint,
                               PoseLandmarkType to_joint);

/**
 * @brief 관절별 길이 켈리브레이션 수행
 * @param base_pose 사용자 기본 포즈
 * @param out_calibration 계산된 켈리브레이션 결과
 * @return SEGMENT_OK 성공, 음수 에러 코드
 */
int segment_calibrate_joint_lengths(const PoseData *base_pose,
                                    CalibrationData *out_calibration);

/**
 * @brief 관절별 길이 켈리브레이션 적용
 * @param original_pose 원본 포즈
 * @param calibration 켈리브레이션 데이터
 * @param calibrated_pose 켈리브레이션된 포즈
 * @return SEGMENT_OK 성공, 음수 에러 코드
 */
int apply_joint_length_calibration(const PoseData *original_pose,
                                   const CalibrationData *calibration,
                                   PoseData *calibrated_pose);

/**
 * @brief 관절 연결 관계 초기화
 * @return 관절 연결 개수
 */
int initialize_joint_connections(void);

/**
 * @brief 관절별 길이 정보 출력 (디버깅용)
 * @param calibration 켈리브레이션 데이터
 */
void print_joint_lengths(const CalibrationData *calibration);

/**
 * @brief 공통 캘리브레이션 로직 (내부 함수)
 * @param base_pose 사용자의 기본 포즈 데이터
 * @param out_calibration 계산된 캘리브레이션 결과를 저장할 구조체
 * @param calibration_type 캘리브레이션 타입 ("기록자" 또는 "사용자")
 * @return SEGMENT_OK 성공, 음수 에러 코드
 */
int segment_calibrate_common(const PoseData *base_pose,
                             CalibrationData *out_calibration,
                             const char *calibration_type);

/**
 * @brief 목표 포즈를 사용자 신체 비율에 맞게 조정
 * @param target_pose 원본 목표 포즈
 * @param calibration 사용자 캘리브레이션 데이터
 * @param adjusted_pose 조정된 목표 포즈를 저장할 구조체
 * @return SEGMENT_OK 성공, 음수 에러 코드
 */
int smart_pose_fit(const PoseData *target_pose,
                   const CalibrationData *calibration, PoseData *adjusted_pose);

// 캘리브레이션 상수 정의
#define CALIBRATION_IDEAL_SHOULDER_WIDTH                                       \
  322.78f // 이상적 어깨 너비 (실제 데이터 기반)
#define CALIBRATION_MIN_CONFIDENCE 0.3f // 최소 신뢰도 임계값 (더 엄격한 기준)
#define CALIBRATION_MIN_SHOULDER_WIDTH 50.0f  // 최소 어깨 너비
#define CALIBRATION_MAX_SHOULDER_WIDTH 600.0f // 최대 어깨 너비
#define CALIBRATION_MIN_SCALE_FACTOR 0.3f     // 최소 스케일 팩터
#define CALIBRATION_MAX_SCALE_FACTOR 3.0f     // 최대 스케일 팩터
#define CALIBRATION_DEFAULT_QUALITY 0.95f // 기본 캘리브레이션 품질

// 전역 변수 extern 선언
extern CalibrationData g_recorder_calibration; // A(기록자) 캘리브레이션 데이터
extern bool g_recorder_calibrated; // A(기록자) 캘리브레이션 완료 플래그
extern CalibrationData g_user_calibration; // B(사용자) 캘리브레이션 데이터
extern bool g_user_calibrated; // B(사용자) 캘리브레이션 완료 플래그
extern PoseData g_ideal_base_pose;              // 이상적 기본 포즈
extern JointConnection g_joint_connections[20]; // 관절 연결 관계

#ifdef __cplusplus
}
#endif

#endif // CALIBRATION_H
