#ifndef ANTENNA_LINK_H
#define ANTENNA_LINK_H
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    double east, north, up; /* m/s, at the position measurement epoch */
    bool valid;
} Velocity;

typedef struct {
    double lat_deg, lon_deg, height_m;
    uint32_t stamp_ms;  /* measurement time expressed on local monotonic clock */
    bool valid;
} Fix;

typedef struct {
    double az_deg, el_deg; /* encoder pitch 0..90: air down, ground up */
    uint32_t stamp_ms;
    bool valid;
} GimbalAngles;

typedef struct {
    double az_zero_deg, el_zero_deg; /* encoder readings looking forward and level */
    bool reverse_az, reverse_el; /* optional mounting correction; standard devices use false */
} GimbalFrame;

/* LoRa application payloads. Serialize fields explicitly in the board adapter. */
typedef struct {
    uint32_t session, point;
    double az_trim, el_trim; /* degrees added to geographic TX line of sight */
} AirRequest;
typedef struct { uint32_t session, point; bool arrived; } AirReply;
#define AIR_TRIM_LIMIT_DEG 20.0
#define AIR_ARRIVAL_DEG 0.35
#define LORA_PERIOD_MS 1000u
#define COORD_MAX_AGE_MS (2u*LORA_PERIOD_MS+500u)

typedef struct {
    Fix local, peer;
    bool enabled, local_ok, peer_ok;
    uint32_t peer_status_ms;
    GimbalAngles actual;
    GimbalFrame frame;
    uint32_t lead_ms; /* extra measured actuator/command latency, zero = predict to now */
} LinkInput;

typedef enum { IDLE, WAIT_DATA, FAULT, POINT, SWEEP, REFINE, TRACK, RECOVER, CAL_AIR, NEAR, VERIFY_PEAK } Mode;
typedef struct {
    Mode mode;
    bool valid;
    double az_deg, el_deg; /* controller outputs are absolute native gimbal angles */
} Command;

#define NAV_MAX_AGE_MS (2u*LORA_PERIOD_MS+500u)
#define ENCODER_MAX_AGE_MS 100u
#define MAX_PREDICTION_MS (LORA_PERIOD_MS+200u)
Command link_status(uint32_t now, const LinkInput *in);
double wrap360(double angle);
double shortest_angle(double angle);
double prediction_seconds(uint32_t age_ms, uint32_t lead_ms);
bool encoder_ready(uint32_t now, const LinkInput *in);
Command gimbal_command(Command body, const LinkInput *in);
/* Internal motion estimate, never a LoRa input field. */
Velocity estimate_velocity(const Fix *current, const Fix *previous);
Command moving_line_of_sight(uint32_t now, const LinkInput *in, Velocity local, Velocity peer);
/* Convert one raw GGA latitude or longitude field and its hemisphere. */
bool gga_degrees(double raw, char hemisphere, double *degrees);
Command line_of_sight(uint32_t now, const LinkInput *in);
#endif
