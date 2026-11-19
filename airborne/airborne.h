#ifndef AIRBORNE_H
#define AIRBORNE_H
#include "../common/link.h"
typedef struct {
    LinkInput link;
    double true_heading_deg;  /* heading of antenna base, includes mounting yaw */
    uint32_t heading_ms;
    bool heading_valid;
    double pitch_deg, roll_deg; /* nose up and right wing down; zero for a level base */
    bool rates_valid;
    double heading_rate, pitch_rate, roll_rate; /* Euler angle derivatives, deg/s */
    AirRequest request;
    uint32_t request_rx_ms; /* local timestamp of last accepted fresh LoRa frame */
} AirInput;
/* One zero-initialized object per aircraft. Retain across takeoff and landing. */
typedef struct { Fix fix; Velocity estimate; } MotionSample;
typedef struct { MotionSample local, peer; } AirHistory;
void airborne_track_init(AirHistory *history);
Command airborne_track_step(AirHistory *history, uint32_t now, const AirInput *in, AirReply *reply);
Command airborne_point_step(uint32_t now, const AirInput *in, AirReply *reply);
Command airborne_step(uint32_t now, const AirInput *in);
#endif
