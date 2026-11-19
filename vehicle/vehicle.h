#ifndef VEHICLE_H
#define VEHICLE_H
#include "../common/link.h"

typedef struct {
    double sweep_step_deg, row_step_deg;
    double acquire_level, release_level, peak_margin; /* acquire_level qualifies final lock */
    double refine_start_deg, refine_min_deg, refine_factor;
    unsigned refine_max_rounds, weak_rounds;
    double probe_deg, recover_step_deg, arrival_tolerance_deg;
    unsigned recover_radius_points, sample_count;
    uint32_t settle_ms, sample_window_ms, move_timeout_ms;
    double air_search_deg, air_refine_deg, air_min_deg;
    unsigned air_max_rounds;
    uint32_t air_recheck_ms; /* 0 keeps a healthy link in TRACK */
    double near_step_deg; /* spacing of wider peak verification grid */
    double detect_level;  /* fallback for receivers with only numeric RSSI */
    bool fast_relock; /* enable with main-lobe-calibrated acquire/release levels */
    uint32_t track_weak_ms; /* sustained fresh weak frames; 0 uses round checks only */
    bool fast_acquire; /* strong, calibrated main-lobe threshold permits a shorter peak check */
    double realign_drop_db; /* severe RSSI threshold = release_level - this margin */
    uint32_t realign_weak_ms, realign_ack_ms; /* zero disables the corresponding guard */
    double pair_margin; /* opposite-point difference in dB; 0 disables sub-step fitting */
    double rx_half_power_deg; /* measured one-sided -3 dB angle; 0 disables calibrated peak estimates */
} VehicleConfig;

extern const VehicleConfig vehicle_defaults;
double vehicle_realign_threshold(const VehicleConfig *cfg);

typedef struct {
    LinkInput link;
    bool rssi_valid;
    uint32_t sample_ms; /* effective measurement time after receiver delay correction */
    double rssi;
    uint32_t session; /* nonzero vehicle boot/session identifier */
    AirReply air;
    uint32_t air_rx_ms;
    double rough_heading_deg; /* true heading of vehicle base; acquisition only */
    uint32_t heading_ms;
    bool heading_valid;
    bool has_beacon_flag, beacon_present; /* per-frame carrier/beacon indication */
    bool request_realign; /* one control tick pulse from operator or supervisor */
} VehicleInput;

typedef enum { REALIGN_NONE, REALIGN_SIGNAL, REALIGN_ACK, REALIGN_REQUEST } RealignReason;

#define TRACK_HISTORY_SIZE 24
typedef struct { float az, el, level; uint32_t stamp; } TrackSample;

typedef struct {
    VehicleConfig cfg;
    double refine_span;
    Mode mode;
    int point, best_point, sweep_row;
    unsigned refine_rounds, weak_rounds;
    unsigned strong_crosses;
    double center_az, center_el; /* absolute base angles after beacon capture */
    double search_yaw; /* geographic to base yaw offset while acquiring */
    double center_score, best_score;
    double cross_score[5];
    int prior_slope[2]; /* previous tracking cross, azimuth and elevation */
    double last_track_az_step, last_track_el_step;
    TrackSample track_history[TRACK_HISTORY_SIZE];
    unsigned track_count, track_next;
    bool probe_el_first;
    uint32_t last_track_fit_ms;
    Command command;
    bool commanded, settling;
    uint32_t moved_ms, stable_ms, last_sample_ms;
    unsigned samples, detected_samples;
    bool point_seen, refined, confirm_lock;
    double sum;
    AirRequest request; /* publish periodically to the aircraft */
    double air_az, air_el, air_span;
    double trial_az, trial_el, trial_score; /* retained center while verifying fitted TX trim */
    bool trial_trim;
    unsigned air_rounds, air_search_point;
    bool ground_refresh; /* one local RX cross between slow LoRa TX probe rounds */
    uint32_t last_air_cal_ms;
    uint32_t air_unready_ms;
    bool air_waiting;
    bool ever_locked, fast_probe, monitoring_lock, weak_stream;
    double peak_above_gate; /* last confirmed center relative to its range-dependent gate */
    double confirmed_air_az, confirmed_air_el;
    bool verify_previous_trim;
    unsigned rx_verify_retries;
    unsigned low_center_rounds;
    bool retry_center, missed_beacon;
    double saved_az, saved_el, saved_air_az, saved_air_el;
    uint32_t weak_since_ms, weak_sample_ms;
    RealignReason realign_reason; /* nonzero only on the tick that preempted a stage */
    bool have_guard_sample, severe_stream, sweep_known_weak, reject_severe_candidate;
    bool waiting_match;
    uint32_t guard_sample_ms, severe_since_ms, unmatched_since_ms;
} Vehicle;

void vehicle_init(Vehicle *v);
/* Applies a validated copy and restarts acquisition. On error leaves v intact. */
bool vehicle_configure(Vehicle *v, const VehicleConfig *cfg);
Command vehicle_step(Vehicle *v, uint32_t now, const VehicleInput *in);
#endif
