/**
 * @file    motor_control.c
 * @brief   카메라 트래킹 기반 3축 서보모터 제어 + 재밍 신호 지향 시스템
 *
 * 구조:
 *   - 카메라로부터 목표물 픽셀 좌표 입력 (bounding box 중심)
 *   - 픽셀 오차 → Pan/Tilt/Roll 각도 오차 변환
 *   - PID 제어기로 서보모터 PWM 출력 계산
 *   - 재밍 신호 발사 여부 판단 (락온 임계값 이내일 때만 활성화)
 *
 * 빌드:
 *   gcc -o motor_control motor_control.c -lm
 * 실행:
 *   ./motor_control
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <time.h>

/* ─────────────────────────────────────────────
 * 1. 상수 및 매크로 정의
 * ───────────────────────────────────────────── */
#define PI                  3.14159265358979323846

/* 카메라 해상도 */
#define CAM_WIDTH           1920
#define CAM_HEIGHT          1080
#define CAM_HFOV_DEG        90.0   /* 수평 화각 (degree) */
#define CAM_VFOV_DEG        60.0   /* 수직 화각 (degree) */

/* 서보모터 PWM 범위 (µs) */
#define SERVO_PWM_MIN_US    500    /* -90° */
#define SERVO_PWM_MAX_US    2500   /* +90° */
#define SERVO_PWM_CENTER_US 1500   /* 0°   */

/* 서보모터 물리 각도 한계 */
#define PAN_ANGLE_MIN      -180.0
#define PAN_ANGLE_MAX       180.0
#define TILT_ANGLE_MIN     -90.0
#define TILT_ANGLE_MAX      90.0
#define ROLL_ANGLE_MIN     -45.0
#define ROLL_ANGLE_MAX      45.0

/* 락온(Lock-On) 임계값: 픽셀 오차 이하이면 재밍 활성화 */
#define LOCKON_THRESHOLD_PX  20
/* 최소 연속 락온 프레임 수 */
#define LOCKON_HOLD_FRAMES   5

/* PID 샘플링 주기 (초) */
#define DT                  0.02   /* 50 Hz 제어 루프 */

/* 시뮬레이션 최대 스텝 */
#define MAX_SIM_STEPS       200

/* ─────────────────────────────────────────────
 * 2. 데이터 구조체 정의
 * ───────────────────────────────────────────── */

/** 픽셀 좌표 */
typedef struct {
    double x;   /* 0 ~ CAM_WIDTH  */
    double y;   /* 0 ~ CAM_HEIGHT */
} PixelCoord;

/** 목표물 트래킹 상태 */
typedef struct {
    PixelCoord  bbox_center;    /* 바운딩박스 중심 좌표 */
    double      bbox_w;         /* 바운딩박스 너비 */
    double      bbox_h;         /* 바운딩박스 높이 */
    int         detected;       /* 목표물 감지 여부 (1=감지, 0=미감지) */
    double      confidence;     /* 트래킹 신뢰도 (0.0~1.0) */
} TrackingState;

/** PID 제어기 상태 */
typedef struct {
    double kp, ki, kd;          /* PID 이득 */
    double integral;            /* 적분 누적값 */
    double prev_error;          /* 이전 오차 */
    double output_min;          /* 출력 하한 */
    double output_max;          /* 출력 상한 */
} PIDController;

/** 서보모터 상태 */
typedef struct {
    double current_angle;       /* 현재 각도 (degree) */
    double target_angle;        /* 목표 각도 (degree) */
    int    pwm_us;              /* PWM 펄스폭 (µs) */
    double angle_min;
    double angle_max;
} ServoMotor;

/** 3축 짐벌 시스템 */
typedef struct {
    ServoMotor  pan;            /* Yaw축  */
    ServoMotor  tilt;           /* Pitch축 */
    ServoMotor  roll;           /* Roll축 */
    PIDController pid_pan;
    PIDController pid_tilt;
    PIDController pid_roll;
} GimbalSystem;

/** 재밍 발사 상태 */
typedef struct {
    int     lockon_frames;      /* 연속 락온 프레임 수 */
    int     is_firing;          /* 현재 재밍 발사 중 여부 */
    double  pan_at_fire;        /* 발사 시 Pan 각도  */
    double  tilt_at_fire;       /* 발사 시 Tilt 각도 */
    double  roll_at_fire;       /* 발사 시 Roll 각도 */
    int     total_fire_count;   /* 누적 발사 횟수 */
} JammingFireState;

/* ─────────────────────────────────────────────
 * 3. PID 제어기
 * ───────────────────────────────────────────── */

/**
 * @brief PID 제어기 초기화
 */
void pid_init(PIDController *pid, double kp, double ki, double kd,
              double out_min, double out_max) {
    pid->kp        = kp;
    pid->ki        = ki;
    pid->kd        = kd;
    pid->integral  = 0.0;
    pid->prev_error = 0.0;
    pid->output_min = out_min;
    pid->output_max = out_max;
}

/**
 * @brief PID 업데이트: 오차 → 제어 출력 계산
 * @param pid    PID 제어기
 * @param error  현재 오차
 * @param dt     샘플링 주기 (초)
 * @return 제어 출력 (각도 보정량 degree)
 */
double pid_update(PIDController *pid, double error, double dt) {
    pid->integral += error * dt;
    /* Anti-windup: 적분 클램핑 */
    if (pid->integral > pid->output_max / pid->ki)
        pid->integral = pid->output_max / pid->ki;
    if (pid->integral < pid->output_min / pid->ki)
        pid->integral = pid->output_min / pid->ki;

    double derivative = (error - pid->prev_error) / dt;
    double output = pid->kp * error
                  + pid->ki * pid->integral
                  + pid->kd * derivative;

    pid->prev_error = error;

    /* 출력 클램핑 */
    if (output > pid->output_max) output = pid->output_max;
    if (output < pid->output_min) output = pid->output_min;

    return output;
}

void pid_reset(PIDController *pid) {
    pid->integral   = 0.0;
    pid->prev_error = 0.0;
}

/* ─────────────────────────────────────────────
 * 4. 서보모터 유틸리티
 * ───────────────────────────────────────────── */

/**
 * @brief 서보모터 초기화
 */
void servo_init(ServoMotor *s, double angle_min, double angle_max) {
    s->current_angle = 0.0;
    s->target_angle  = 0.0;
    s->pwm_us        = SERVO_PWM_CENTER_US;
    s->angle_min     = angle_min;
    s->angle_max     = angle_max;
}

/**
 * @brief 각도(degree) → PWM(µs) 변환
 *        angle_min → PWM_MIN, angle_max → PWM_MAX 선형 매핑
 */
int angle_to_pwm(const ServoMotor *s, double angle) {
    double ratio = (angle - s->angle_min) / (s->angle_max - s->angle_min);
    int pwm = (int)(SERVO_PWM_MIN_US + ratio * (SERVO_PWM_MAX_US - SERVO_PWM_MIN_US));
    if (pwm < SERVO_PWM_MIN_US) pwm = SERVO_PWM_MIN_US;
    if (pwm > SERVO_PWM_MAX_US) pwm = SERVO_PWM_MAX_US;
    return pwm;
}

/**
 * @brief 서보모터 목표 각도 설정 및 PWM 갱신
 */
void servo_set_angle(ServoMotor *s, double angle) {
    if (angle < s->angle_min) angle = s->angle_min;
    if (angle > s->angle_max) angle = s->angle_max;
    s->target_angle  = angle;
    s->current_angle = angle;   /* 실제 하드웨어에서는 피드백 루프 적용 */
    s->pwm_us        = angle_to_pwm(s, angle);
}

/* ─────────────────────────────────────────────
 * 5. 짐벌 시스템 초기화
 * ───────────────────────────────────────────── */

/**
 * @brief 3축 짐벌 시스템 초기화
 */
void gimbal_init(GimbalSystem *g) {
    servo_init(&g->pan,  PAN_ANGLE_MIN,  PAN_ANGLE_MAX);
    servo_init(&g->tilt, TILT_ANGLE_MIN, TILT_ANGLE_MAX);
    servo_init(&g->roll, ROLL_ANGLE_MIN, ROLL_ANGLE_MAX);

    /* PID 이득: 출력 = 각도 보정량(degree) */
    pid_init(&g->pid_pan,  0.15, 0.005, 0.08, -30.0, 30.0);
    pid_init(&g->pid_tilt, 0.15, 0.005, 0.08, -30.0, 30.0);
    pid_init(&g->pid_roll, 0.10, 0.003, 0.05, -10.0, 10.0);

    printf("[GIMBAL] 3축 서보 짐벌 시스템 초기화 완료\n");
    printf("         Pan  : %.0f° ~ %.0f°\n", PAN_ANGLE_MIN,  PAN_ANGLE_MAX);
    printf("         Tilt : %.0f° ~ %.0f°\n", TILT_ANGLE_MIN, TILT_ANGLE_MAX);
    printf("         Roll : %.0f° ~ %.0f°\n", ROLL_ANGLE_MIN, ROLL_ANGLE_MAX);
}

/* ─────────────────────────────────────────────
 * 6. 카메라 좌표 → 각도 오차 변환
 * ───────────────────────────────────────────── */

/**
 * @brief 픽셀 오차 → Pan/Tilt 각도 오차 변환
 *        화면 중심 기준 오차 픽셀을 화각(FOV)에 비례하여 각도로 변환
 * @param px_err_x  수평 픽셀 오차 (양수 = 오른쪽)
 * @param px_err_y  수직 픽셀 오차 (양수 = 아래)
 * @param pan_err   [출력] Pan 각도 오차 (degree)
 * @param tilt_err  [출력] Tilt 각도 오차 (degree)
 */
void pixel_error_to_angle(double px_err_x, double px_err_y,
                          double *pan_err, double *tilt_err) {
    *pan_err  = (px_err_x / (CAM_WIDTH  / 2.0)) * (CAM_HFOV_DEG / 2.0);
    *tilt_err = (px_err_y / (CAM_HEIGHT / 2.0)) * (CAM_VFOV_DEG / 2.0);
}

/**
 * @brief 바운딩박스 크기 → Roll 보정값 계산
 *        목표물 기울기 추정 (bbox 비율 기반 휴리스틱)
 * @param bbox_w  바운딩박스 너비
 * @param bbox_h  바운딩박스 높이
 * @return Roll 오차 추정값 (degree)
 */
double estimate_roll_error(double bbox_w, double bbox_h) {
    double aspect = bbox_w / (bbox_h + 1e-6);
    /* 정방형(1.0)에서 멀수록 Roll 보정 필요 */
    return (aspect - 1.0) * 5.0;   /* 계수는 하드웨어 캘리브레이션 필요 */
}

/* ─────────────────────────────────────────────
 * 7. 메인 제어 루프
 * ───────────────────────────────────────────── */

/**
 * @brief 단일 제어 스텝: 트래킹 상태 → 짐벌 각도 업데이트
 * @param g      짐벌 시스템
 * @param track  현재 트래킹 상태
 * @param fire   재밍 발사 상태
 * @return 픽셀 오차의 유클리드 거리 (락온 판단용)
 */
double gimbal_update(GimbalSystem *g, const TrackingState *track,
                     JammingFireState *fire) {

    if (!track->detected) {
        /* 목표물 미감지: 모터 정지, 재밍 해제 */
        pid_reset(&g->pid_pan);
        pid_reset(&g->pid_tilt);
        pid_reset(&g->pid_roll);
        fire->lockon_frames = 0;
        fire->is_firing     = 0;
        return -1.0;
    }

    /* 화면 중심 기준 픽셀 오차 */
    double cx = CAM_WIDTH  / 2.0;
    double cy = CAM_HEIGHT / 2.0;
    double px_err_x = track->bbox_center.x - cx;
    double px_err_y = track->bbox_center.y - cy;
    double pixel_dist = sqrt(px_err_x * px_err_x + px_err_y * px_err_y);

    /* 픽셀 오차 → 각도 오차 변환 */
    double pan_err, tilt_err;
    pixel_error_to_angle(px_err_x, px_err_y, &pan_err, &tilt_err);
    double roll_err = estimate_roll_error(track->bbox_w, track->bbox_h);

    /* PID 제어 출력 계산 */
    double pan_out  = pid_update(&g->pid_pan,  pan_err,  DT);
    double tilt_out = pid_update(&g->pid_tilt, tilt_err, DT);
    double roll_out = pid_update(&g->pid_roll, roll_err, DT);

    /* 서보모터 각도 업데이트 */
    servo_set_angle(&g->pan,  g->pan.current_angle  + pan_out);
    servo_set_angle(&g->tilt, g->tilt.current_angle + tilt_out);
    servo_set_angle(&g->roll, g->roll.current_angle + roll_out);

    /* ── 락온(Lock-On) 판단 ── */
    if (pixel_dist <= LOCKON_THRESHOLD_PX) {
        fire->lockon_frames++;
    } else {
        fire->lockon_frames = 0;
        fire->is_firing     = 0;
    }

    /* LOCKON_HOLD_FRAMES 연속 프레임 이상 락온 → 재밍 발사 */
    if (fire->lockon_frames >= LOCKON_HOLD_FRAMES && !fire->is_firing) {
        fire->is_firing      = 1;
        fire->pan_at_fire    = g->pan.current_angle;
        fire->tilt_at_fire   = g->tilt.current_angle;
        fire->roll_at_fire   = g->roll.current_angle;
        fire->total_fire_count++;
        printf("[FIRE ] 🔴 재밍 신호 발사! Pan=%.2f° Tilt=%.2f° Roll=%.2f° (총 %d회)\n",
               fire->pan_at_fire, fire->tilt_at_fire, fire->roll_at_fire,
               fire->total_fire_count);
    }

    return pixel_dist;
}

/* ─────────────────────────────────────────────
 * 8. 시뮬레이션용 목표물 움직임 생성
 * ───────────────────────────────────────────── */

/**
 * @brief 시뮬레이션: 목표물이 화면을 가로지르는 궤적 생성
 * @param step    현재 시뮬레이션 스텝
 * @param track   [출력] 트래킹 상태
 */
void simulate_target(int step, TrackingState *track) {
    /* 목표물: 정현파 궤적으로 이동 */
    double t = step * DT;
    track->detected        = 1;
    track->confidence      = 0.92;
    track->bbox_center.x   = CAM_WIDTH  / 2.0 + 400.0 * cos(0.3 * t);
    track->bbox_center.y   = CAM_HEIGHT / 2.0 + 200.0 * sin(0.5 * t);
    track->bbox_w          = 80.0 + 10.0 * sin(0.2 * t);
    track->bbox_h          = 60.0 + 8.0  * cos(0.2 * t);

    /* 40~60 스텝 구간은 목표물 소실 시뮬레이션 */
    if (step >= 40 && step < 60) {
        track->detected   = 0;
        track->confidence = 0.0;
    }
}

/* ─────────────────────────────────────────────
 * 9. 상태 출력
 * ───────────────────────────────────────────── */

void print_status(int step, const GimbalSystem *g,
                  const TrackingState *track,
                  const JammingFireState *fire,
                  double pixel_dist) {

    if (!track->detected) {
        printf("[%04d] ⚠ 목표물 미감지 - 모터 대기\n", step);
        return;
    }

    const char *fire_str = fire->is_firing ? "🔴 FIRING" : "⬜ STANDBY";
    printf("[%04d] %s | "
           "Pan=%6.2f°(%4dµs) Tilt=%6.2f°(%4dµs) Roll=%5.2f°(%4dµs) | "
           "Err=%.1fpx | Lock=%d\n",
           step, fire_str,
           g->pan.current_angle,  g->pan.pwm_us,
           g->tilt.current_angle, g->tilt.pwm_us,
           g->roll.current_angle, g->roll.pwm_us,
           pixel_dist, fire->lockon_frames);
}

/* ─────────────────────────────────────────────
 * 10. main
 * ───────────────────────────────────────────── */

int main(int argc, char *argv[]) {

    printf("╔══════════════════════════════════════════════════╗\n");
    printf("║   목표물 지향 서보 짐벌 + 재밍 신호 발사 시스템  ║\n");
    printf("║   3축 (Pan/Tilt/Roll) · PID 제어 · 카메라 트래킹 ║\n");
    printf("╚══════════════════════════════════════════════════╝\n\n");

    /* 시스템 초기화 */
    GimbalSystem     gimbal;
    JammingFireState fire = {0};
    TrackingState    track;

    gimbal_init(&gimbal);

    printf("\n[SIM ] 시뮬레이션 시작 (%d 스텝, %.0f Hz)\n\n",
           MAX_SIM_STEPS, 1.0 / DT);

    /* 메인 제어 루프 */
    for (int step = 0; step < MAX_SIM_STEPS; step++) {

        /* 카메라 트래킹 결과 입력 (시뮬레이션) */
        simulate_target(step, &track);

        /* 짐벌 제어 업데이트 */
        double pixel_dist = gimbal_update(&gimbal, &track, &fire);

        /* 10 스텝마다 상태 출력 */
        if (step % 10 == 0 || fire.is_firing) {
            print_status(step, &gimbal, &track, &fire, pixel_dist);
        }
    }

    /* 최종 요약 */
    printf("\n[결과] ──────────────────────────────────────────\n");
    printf("       총 스텝          : %d\n",   MAX_SIM_STEPS);
    printf("       재밍 발사 횟수   : %d회\n", fire.total_fire_count);
    printf("       최종 Pan 각도    : %.2f°\n", gimbal.pan.current_angle);
    printf("       최종 Tilt 각도   : %.2f°\n", gimbal.tilt.current_angle);
    printf("       최종 Roll 각도   : %.2f°\n", gimbal.roll.current_angle);
    printf("───────────────────────────────────────────────────\n");

    return EXIT_SUCCESS;
}
