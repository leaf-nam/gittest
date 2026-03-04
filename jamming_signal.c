#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#define SAMPLE_RATE     1000000   // 1 MHz 샘플링 레이트
#define DEFAULT_FREQ    433920000 // 433.92 MHz (기본 재밍 주파수)
#define DEFAULT_DURATION 5.0     // 기본 지속 시간 (초)
#define DEFAULT_AMPLITUDE 1.0    // 기본 진폭
#define PI              3.14159265358979323846

typedef struct {
    double frequency;   // 재밍 주파수 (Hz)
    double duration;    // 지속 시간 (초)
    double amplitude;   // 진폭 (0.0 ~ 1.0)
    int    sample_rate; // 샘플링 레이트 (Hz)
} JammingConfig;

typedef struct {
    double *samples;    // 신호 샘플 배열
    int     size;       // 샘플 개수
} SignalBuffer;

/**
 * @brief 재밍 신호 버퍼를 초기화합니다.
 * @param config 재밍 설정
 * @return 초기화된 SignalBuffer (실패 시 samples = NULL)
 */
SignalBuffer init_signal_buffer(const JammingConfig *config) {
    SignalBuffer buf;
    buf.size = (int)(config->sample_rate * config->duration);
    buf.samples = (double *)malloc(buf.size * sizeof(double));
    if (!buf.samples) {
        fprintf(stderr, "[ERROR] 메모리 할당 실패\n");
        buf.size = 0;
    }
    return buf;
}

/**
 * @brief 연속파(CW) 재밍 신호를 생성합니다.
 *        y[n] = A * sin(2π * f * n / Fs)
 * @param buf    출력 버퍼
 * @param config 재밍 설정
 */
void generate_cw_jamming(SignalBuffer *buf, const JammingConfig *config) {
    double phase_increment = 2.0 * PI * config->frequency / config->sample_rate;
    for (int i = 0; i < buf->size; i++) {
        buf->samples[i] = config->amplitude * sin(phase_increment * i);
    }
    printf("[INFO] CW 재밍 신호 생성 완료: %.2f MHz, %.1f초, 샘플 %d개\n",
           config->frequency / 1e6, config->duration, buf->size);
}

/**
 * @brief 광대역 노이즈 재밍 신호를 생성합니다. (AWGN 기반)
 * @param buf    출력 버퍼
 * @param config 재밍 설정
 */
void generate_noise_jamming(SignalBuffer *buf, const JammingConfig *config) {
    for (int i = 0; i < buf->size; i++) {
        double u1 = (rand() + 1.0) / (RAND_MAX + 1.0);
        double u2 = (rand() + 1.0) / (RAND_MAX + 1.0);
        double gaussian = sqrt(-2.0 * log(u1)) * cos(2.0 * PI * u2);
        buf->samples[i] = config->amplitude * gaussian;
    }
    printf("[INFO] 노이즈 재밍 신호 생성 완료: %.1f초, 샘플 %d개\n",
           config->duration, buf->size);
}

/**
 * @brief 주파수 도약(Sweep) 재밍 신호를 생성합니다.
 * @param buf       출력 버퍼
 * @param config    재밍 설정
 * @param bandwidth 도약 대역폭 (Hz)
 */
void generate_sweep_jamming(SignalBuffer *buf, const JammingConfig *config, double bandwidth) {
    double f_start = config->frequency - bandwidth / 2.0;
    double f_end   = config->frequency + bandwidth / 2.0;
    double phase = 0.0;
    for (int i = 0; i < buf->size; i++) {
        double t = (double)i / config->sample_rate;
        double freq = f_start + (f_end - f_start) * t / config->duration;
        phase += 2.0 * PI * freq / config->sample_rate;
        buf->samples[i] = config->amplitude * sin(phase);
    }
    printf("[INFO] Sweep 재밍 신호 생성 완료: %.2f ~ %.2f MHz, %.1f초\n",
           f_start / 1e6, f_end / 1e6, config->duration);
}

/**
 * @brief 신호를 CSV 파일로 저장합니다.
 * @param buf      신호 버퍼
 * @param filename 출력 파일명
 * @param max_samples 저장할 최대 샘플 수 (0이면 전체)
 */
void save_signal_to_csv(const SignalBuffer *buf, const char *filename, int max_samples) {
    FILE *fp = fopen(filename, "w");
    if (!fp) {
        fprintf(stderr, "[ERROR] 파일 열기 실패: %s\n", filename);
        return;
    }
    fprintf(fp, "index,amplitude\n");
    int count = (max_samples > 0 && max_samples < buf->size) ? max_samples : buf->size;
    for (int i = 0; i < count; i++) {
        fprintf(fp, "%d,%.6f\n", i, buf->samples[i]);
    }
    fclose(fp);
    printf("[INFO] 신호 저장 완료: %s (%d 샘플)\n", filename, count);
}

/**
 * @brief 신호 통계를 출력합니다.
 * @param buf 신호 버퍼
 */
void print_signal_stats(const SignalBuffer *buf) {
    double sum = 0.0, max_val = buf->samples[0], min_val = buf->samples[0];
    for (int i = 0; i < buf->size; i++) {
        sum += buf->samples[i];
        if (buf->samples[i] > max_val) max_val = buf->samples[i];
        if (buf->samples[i] < min_val) min_val = buf->samples[i];
    }
    double mean = sum / buf->size;

    double variance = 0.0;
    for (int i = 0; i < buf->size; i++) {
        variance += (buf->samples[i] - mean) * (buf->samples[i] - mean);
    }
    double rms = sqrt(variance / buf->size);

    printf("------ 신호 통계 ------\n");
    printf("  샘플 수  : %d\n", buf->size);
    printf("  최대값   : %.6f\n", max_val);
    printf("  최소값   : %.6f\n", min_val);
    printf("  평균     : %.6f\n", mean);
    printf("  RMS      : %.6f\n", rms);
    printf("-----------------------\n");
}

/**
 * @brief 신호 버퍼 메모리를 해제합니다.
 */
void free_signal_buffer(SignalBuffer *buf) {
    if (buf->samples) {
        free(buf->samples);
        buf->samples = NULL;
        buf->size = 0;
    }
}

int main(int argc, char *argv[]) {
    printf("===== 재밍 신호 생성기 =====\n");

    JammingConfig config = {
        .frequency   = DEFAULT_FREQ,
        .duration    = DEFAULT_DURATION,
        .amplitude   = DEFAULT_AMPLITUDE,
        .sample_rate = SAMPLE_RATE
    };

    // 커맨드라인 인수 파싱
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-f") == 0 && i + 1 < argc)
            config.frequency = atof(argv[++i]);
        else if (strcmp(argv[i], "-d") == 0 && i + 1 < argc)
            config.duration = atof(argv[++i]);
        else if (strcmp(argv[i], "-a") == 0 && i + 1 < argc)
            config.amplitude = atof(argv[++i]);
        else if (strcmp(argv[i], "-h") == 0) {
            printf("사용법: %s [-f 주파수Hz] [-d 지속시간s] [-a 진폭0~1]\n", argv[0]);
            printf("  예시: %s -f 433920000 -d 3.0 -a 0.8\n", argv[0]);
            return 0;
        }
    }

    printf("[설정] 주파수: %.2f MHz | 지속시간: %.1f초 | 진폭: %.2f\n",
           config.frequency / 1e6, config.duration, config.amplitude);

    // 1. CW 재밍 신호 생성
    SignalBuffer cw_buf = init_signal_buffer(&config);
    if (!cw_buf.samples) return EXIT_FAILURE;
    generate_cw_jamming(&cw_buf, &config);
    print_signal_stats(&cw_buf);
    save_signal_to_csv(&cw_buf, "cw_jamming.csv", 1000);
    free_signal_buffer(&cw_buf);

    // 2. 노이즈 재밍 신호 생성
    SignalBuffer noise_buf = init_signal_buffer(&config);
    if (!noise_buf.samples) return EXIT_FAILURE;
    generate_noise_jamming(&noise_buf, &config);
    print_signal_stats(&noise_buf);
    save_signal_to_csv(&noise_buf, "noise_jamming.csv", 1000);
    free_signal_buffer(&noise_buf);

    // 3. Sweep 재밍 신호 생성 (±5 MHz 대역)
    SignalBuffer sweep_buf = init_signal_buffer(&config);
    if (!sweep_buf.samples) return EXIT_FAILURE;
    generate_sweep_jamming(&sweep_buf, &config, 10e6);
    print_signal_stats(&sweep_buf);
    save_signal_to_csv(&sweep_buf, "sweep_jamming.csv", 1000);
    free_signal_buffer(&sweep_buf);

    printf("===== 완료 =====\n");
    return EXIT_SUCCESS;
}
