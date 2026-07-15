#include <Arduino.h>
#include <WiFi.h>
#include "driver/adc.h"
#include "driver/i2s.h"
#include "soc/syscon_reg.h"
#include "soc/syscon_struct.h"
#include "soc/sens_reg.h"
#include "soc/sens_struct.h"
#include "LinAlg.h"

// ── Mode selection ────────────────────────────────────────────────────────────
// MODE_TEST      → outputs "v0,v1,v2,v3\n" (raw ADC counts) for mic_calibration.py
// MODE_PRODUCTION → outputs "az_degrees\n" (0–360) for visualizer.py
#define MODE_PRODUCTION 0
#define MODE_TEST       1
#define MODE            MODE_PRODUCTION

// ── Tunable constants ─────────────────────────────────────────────────────────
static constexpr int    SAMPLES         = 300;
// I2S streams all 4 channels interleaved at I2S_SAMPLE_RATE total.
// Each mic gets I2S_SAMPLE_RATE/4 = 20 kHz → 50 µs per mic-sample.
static constexpr int    I2S_SAMPLE_RATE = 80000;
static constexpr int    I2S_DMA_BUF_LEN = 256;
static constexpr int    I2S_DMA_BUF_CNT = 4;

// XCORR_RANGE: at 20 kHz/mic (80 kHz total), max TDOA across 17.7 cm diagonal ≈ 10.3 samples. Use 12.
static constexpr int    XCORR_RANGE     = 12;

// VOLTAGE: per-mic 12-bit ADC count at idle. Calibrate with MODE_TEST first.
static constexpr int    VOLTAGE[4]      = {1919, 1913, 1915, 1925};

// THRESHOLD: single-sample abs deviation that fires the recording trigger.
static constexpr int    THRESHOLD       = 400;

static constexpr double SPEED_SOUND     = 343.0;
static constexpr bool   INCL_EDGE       = false;
// NCC_THRESHOLD: minimum normalized cross-correlation peak to accept a TDOA estimate.
// Raise if random-direction readings persist; lower if real sounds are being rejected.
static constexpr float  NCC_THRESHOLD   = 0.15f;

// CH_SKEW_US: within-cycle sampling delay per channel (12.5 µs/slot at 80 kHz).
// Scan order CH0→CH3→CH6→CH7 means mic1 is sampled 12.5 µs after mic0, etc.
static constexpr double CH_SKEW_US[4]  = {0.0, 12.5, 25.0, 37.5};

// ── ADC wiring ────────────────────────────────────────────────────────────────
// GPIO 36 (VP) = ADC1_CH0  → MIC 0
// GPIO 39 (VN) = ADC1_CH3  → MIC 1
// GPIO 34      = ADC1_CH6  → MIC 2
// GPIO 35      = ADC1_CH7  → MIC 3
static const adc1_channel_t MIC_CH[4] = {
    ADC1_CHANNEL_0, ADC1_CHANNEL_3, ADC1_CHANNEL_6, ADC1_CHANNEL_7,
};
// Reverse lookup: ADC1 channel index (0–7) → mic index, -1 = unused
static const int CH_TO_MIC[8] = {0, -1, -1, 1, -1, -1, 2, 3};

// ── Microphone geometry (meters, 12.5 cm square) ─────────────────────────────
static const Vector MIC_LOC[4] = {
    { 0.0625, -0.0625, 0},
    { 0.0625,  0.0625, 0},
    {-0.0625,  0.0625, 0},
    {-0.0625, -0.0625, 0},
};
static const int MIC_PAIRS[6][2] = {{0,1},{0,2},{0,3},{1,2},{1,3},{2,3}};

// ── Sample buffers ────────────────────────────────────────────────────────────
static int          reads[4][SAMPLES];
static unsigned int ts[4][SAMPLES];       // pre-filled uniform 100 µs spacing
static int          fill[4]    = {0};
static bool         recording     = false;
static bool         align_to_ch0  = false;
static uint16_t     dma_buf[I2S_DMA_BUF_LEN];

// ── Internal types ────────────────────────────────────────────────────────────
struct AnnotatedVec {
    Vector v; bool edge;
    AnnotatedVec() : v(), edge(true) {}
    AnnotatedVec(Vector _v, bool _e) : v(_v), edge(_e) {}
};
struct Cone {
    Vector n; double s;
    Cone() : n(), s(0) {}
    Cone(Vector _n, double _s) : n(_n), s(_s) {}
};
static Cone cones[6];

// ── Cross-correlation TDOA ────────────────────────────────────────────────────
// Returns time offset in seconds. Negative = a received the signal first.
static double xcorr_tdoa(const int* a, const unsigned int* a_ts, int va,
                          const int* b, const unsigned int* b_ts, int vb) {
    double time_step = 1e-6 * (((long)a_ts[SAMPLES-1] - a_ts[0]) +
                                ((long)b_ts[SAMPLES-1] - b_ts[0])) / (2.0 * (SAMPLES - 1));

    static long xcorr[2 * XCORR_RANGE + 1];
    for (int lag = -XCORR_RANGE; lag <= XCORR_RANGE; lag++) {
        long corr = 0;
        int  lo   = (lag >= 0) ? lag     : 0;
        int  hi   = (lag >= 0) ? SAMPLES : SAMPLES + lag;
        for (int i = lo; i < hi; i++)
            corr += (int32_t)(a[i] - va) * (int32_t)(b[i - lag] - vb);
        xcorr[lag + XCORR_RANGE] = corr;
    }

    int pk = 0;
    for (int i = 1; i <= 2 * XCORR_RANGE; i++)
        if (xcorr[i] > xcorr[pk]) pk = i;

    // NCC quality gate: peak must be large relative to both signal energies.
    // Filters noisy-but-positive peaks that give wrong TDOA on low-SNR recordings.
    {
        float ea = 0.0f, eb = 0.0f;
        for (int i = 0; i < SAMPLES; i++) {
            float da = (float)(a[i] - va), db = (float)(b[i] - vb);
            ea += da*da; eb += db*db;
        }
        if (ea < 1.0f || eb < 1.0f ||
            (float)xcorr[pk] / (sqrtf(ea) * sqrtf(eb)) < NCC_THRESHOLD)
            return NAN;
    }

    double interp = pk;
    if (pk > 0 && pk < 2 * XCORR_RANGE) {
        double fa = xcorr[pk-1], fb = xcorr[pk], fc = xcorr[pk+1];
        double denom = fa - 2.0*fb + fc;
        if (fabs(denom) > 1.0)
            interp = pk + 0.5*(fa - fc) / denom;
    }

    return time_step * (interp - XCORR_RANGE);
}

// ── Cone geometry ─────────────────────────────────────────────────────────────
static inline int sign_of(double x) { return (x > 0) - (x < 0); }

// Returns the tangent of the cone's half-angle (slope = adjacent/opposite).
// Returns 0 if dt is effectively zero (source equidistant → skip this pair).
static double cone_slope(double dt, double dist) {
    if (fabs(dt) < 1e-9) return 0.0;   // guard: avoids sqrt(dist²)/0
    double ct = SPEED_SOUND * fabs(dt);
    return (dist <= ct) ? 0.0 : sqrt(dist*dist - ct*ct) / ct;
}

static AnnotatedVec cone_intersect(const Cone& c1, const Cone& c2) {
    double alpha = atan(c1.s);
    double beta  = atan(c2.s);
    double theta = acos(fmax(-1.0, fmin(1.0, dot(c1.n, c2.n))));
    Vector ax    = normalize(cross(c2.n, c1.n));
    double z     = cos(alpha);
    double bz1   = cos(theta - beta);
    double bz2   = cos(theta + beta);

    if ((bz1 > z) == (bz2 > z)) {
        double diffs[4] = {
            fabs((theta-beta) - alpha), fabs((theta+beta) - alpha),
            fabs((theta-beta) + alpha), fabs((theta+beta) + alpha),
        };
        int mi = 0;
        for (int i = 1; i < 4; i++) if (diffs[i] < diffs[mi]) mi = i;
        double ang = (mi == 0) ?  beta + ((theta-beta)-alpha)/2.0
                   : (mi == 1) ? -beta + ((theta+beta)-alpha)/2.0
                   : (mi == 2) ?  beta + ((theta-beta)+alpha)/2.0
                   :             -beta + ((theta+beta)+alpha)/2.0;
        return AnnotatedVec(matrix_multiplication(rotaion_matrix_around_axis(ax, -ang), c2.n), true);
    }

    Vector w     = Vector(0, sin(theta - beta), cos(theta - beta));
    Vector nv    = Vector(0, sin(theta),        cos(theta));
    double denom = w.z - w.y*nv.y*nv.z - nv.z*nv.z*w.z;
    if (fabs(denom) < 1e-9) return AnnotatedVec();
    double ang   = acos(fmax(-1.0, fmin(1.0, (z - w.y*nv.y*nv.z - nv.z*nv.z*w.z) / denom)));
    Vector base  = matrix_multiplication(rotaion_matrix_around_axis(ax, beta), c2.n);
    Vector d1    = matrix_multiplication(rotaion_matrix_around_axis(c2.n,  ang), base);
    Vector d2    = matrix_multiplication(rotaion_matrix_around_axis(c2.n, -ang), base);
    return AnnotatedVec(d1.z > 0 ? d1 : d2, false);
}

static Vector sound_direction() {
    for (int i = 0; i < 6; i++) {
        int a = MIC_PAIRS[i][0], b = MIC_PAIRS[i][1];
        double dt = xcorr_tdoa(reads[a], ts[a], VOLTAGE[a], reads[b], ts[b], VOLTAGE[b]);
        dt -= (CH_SKEW_US[b] - CH_SKEW_US[a]) * 1e-6;
        if (isnan(dt))                              { cones[i] = Cone(); continue; } // uncorrelated
        if (fabs(dt) < 1e-9)                        { cones[i] = Cone(); continue; } // equidistant
        double dist = magnitude(MIC_LOC[b] - MIC_LOC[a]);
        if (SPEED_SOUND * fabs(dt) >= dist)         { cones[i] = Cone(); continue; } // impossible TDOA
        cones[i] = Cone(normalize(MIC_LOC[b] - MIC_LOC[a]) * sign_of(dt),
                        cone_slope(dt, dist));
    }

    // Sum valid cone normals → guide vector pointing toward the source hemisphere.
    // Each cone normal already points toward the closer mic (= source side), so
    // the sum is a stable, geometry-free reference for resolving the 180° ambiguity.
    Vector guide(0, 0, 0);
    for (int i = 0; i < 6; i++)
        if (magnitude(cones[i].n) > 0.5) guide = guide + cones[i].n;

    Vector candidates[13];
    int n = 0;
    for (int i = 0; i < 5; i++) {
        for (int j = i + 1; j < 6; j++) {
            if ((i == 0 && j == 5) || (i == 1 && j == 4)) continue; // co-axial pairs
            if (magnitude(cones[i].n) < 0.5 || magnitude(cones[j].n) < 0.5) continue;
            if (dot(cones[i].n, cones[j].n) > 0.94) continue; // < 20° apart → ill-conditioned
            AnnotatedVec av = cone_intersect(cones[i], cones[j]);
            if (!av.edge || INCL_EDGE) candidates[n++] = av.v;
        }
    }

    if (n == 0) return Vector(0, 0, 1);

    // Single-pass consensus: flip any candidate pointing against the guide, then sum.
    // Faster than the old two-pass and more reliable because the guide is derived
    // directly from which mic received first rather than from the candidates themselves.
    Vector sum(0, 0, 0);
    for (int i = 0; i < n; i++) {
        Vector v = candidates[i];
        if (dot(v, guide) < 0.0) v = v * -1;
        sum = sum + v;
    }

    return normalize(sum);
}

// ── Arduino entry points ──────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_OFF);

    for (int i = 0; i < 4; i++)
        adc1_config_channel_atten(MIC_CH[i], ADC_ATTEN_DB_11);

    // Pre-fill ts with uniform 100 µs spacing (10 kHz per mic at 40 kHz total)
    const unsigned int PERIOD_US = 4 * 1000000UL / I2S_SAMPLE_RATE; // = 100
    for (int j = 0; j < SAMPLES; j++)
        for (int i = 0; i < 4; i++)
            ts[i][j] = j * PERIOD_US;

    const i2s_config_t i2s_cfg = {
        .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
        .sample_rate          = I2S_SAMPLE_RATE,
        .bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format       = I2S_CHANNEL_FMT_ONLY_RIGHT,
        .communication_format = I2S_COMM_FORMAT_I2S_MSB,
        .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count        = I2S_DMA_BUF_CNT,
        .dma_buf_len          = I2S_DMA_BUF_LEN,
        .use_apll             = false,
        .tx_desc_auto_clear   = false,
        .fixed_mclk           = 0,
    };
    i2s_driver_install(I2S_NUM_0, &i2s_cfg, 0, NULL);
    i2s_set_adc_mode(ADC_UNIT_1, MIC_CH[0]);

    // Enable all 4 pads BEFORE i2s_adc_enable
    SENS.sar_meas_start1.sar1_en_pad = (1<<0)|(1<<3)|(1<<6)|(1<<7);

    i2s_adc_enable(I2S_NUM_0);

    // i2s_adc_enable resets PATT_LEN to 0. Overwrite the scan pattern AFTER enable.
    // Entry format: [7:4]=channel, [3:2]=bit_width (3=12-bit), [1:0]=atten (3=11dB)
    SYSCON.saradc_ctrl.sar1_patt_len = 3; // 4 entries (len = count - 1)
    SYSCON.saradc_sar1_patt_tab[0] =
        ((uint32_t)((0<<4)|(3<<2)|3) << 24) |  // CH0 = mic0
        ((uint32_t)((3<<4)|(3<<2)|3) << 16) |  // CH3 = mic1
        ((uint32_t)((6<<4)|(3<<2)|3) <<  8) |  // CH6 = mic2
        ((uint32_t)((7<<4)|(3<<2)|3) <<  0);   // CH7 = mic3
    SYSCON.saradc_ctrl.sar1_patt_p_clear = 1;
    SYSCON.saradc_ctrl.sar1_patt_p_clear = 0;

    Serial.println("ready");
}

#if MODE == MODE_TEST
// ── Test loop: outputs "v0,v1,v2,v3\n" per DMA buffer for mic_calibration.py ─
void loop() {
    size_t bytes_read;
    i2s_read(I2S_NUM_0, dma_buf, sizeof(dma_buf), &bytes_read, portMAX_DELAY);
    const int n = (int)(bytes_read / 2);

    int latest[4] = {-1, -1, -1, -1};
    for (int s = 0; s < n; s++) {
        const uint16_t raw = dma_buf[s];
        const int ch = (raw >> 12) & 0xF;
        if (ch > 7) continue;
        const int m = CH_TO_MIC[ch];
        if (m < 0) continue;
        latest[m] = (int)(raw & 0xFFF);
    }
    if (latest[0] < 0 || latest[1] < 0 || latest[2] < 0 || latest[3] < 0) return;
    Serial.printf("%d,%d,%d,%d\n", latest[0], latest[1], latest[2], latest[3]);
}

#else
// ── Production loop: outputs "az_degrees\n" (0–360) for visualizer.py ────────
void loop() {
    size_t bytes_read;
    i2s_read(I2S_NUM_0, dma_buf, sizeof(dma_buf), &bytes_read, portMAX_DELAY);
    const int n = (int)(bytes_read / 2);

    for (int s = 0; s < n; s++) {
        const uint16_t raw = dma_buf[s];
        const int ch = (raw >> 12) & 0xF;
        if (ch > 7) continue;
        const int m = CH_TO_MIC[ch];
        if (m < 0) continue;
        const int val = (int)(raw & 0xFFF);

        if (!recording) {
            if (abs(val - VOLTAGE[m]) > THRESHOLD) {
                recording    = true;
                align_to_ch0 = true;
                for (int i = 0; i < 4; i++) fill[i] = 0;
            }
            continue;
        }

        // Skip samples until the next CH0 so all mics start at the same scan position
        if (align_to_ch0) {
            if (m == 0) align_to_ch0 = false;
            else continue;
        }

        if (fill[m] < SAMPLES)
            reads[m][fill[m]++] = val;
    }

    if (!recording) return;
    for (int i = 0; i < 4; i++) if (fill[i] < SAMPLES) return;

    recording = false;
    Vector dir = sound_direction();
    double az  = atan2(dir.y, dir.x) * 180.0 / M_PI;
    if (az < 0.0) az += 360.0;   // convert (-180,180] → [0,360)
    Serial.printf("%.1f\n", az);
}
#endif
