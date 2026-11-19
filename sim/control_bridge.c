/* Host-only adapter. The two firmware algorithms are compiled unchanged. */
#include "../airborne/airborne.h"
#include "../vehicle/vehicle.h"
#include <stdlib.h>
#include <math.h>

#ifdef _WIN32
#define API __declspec(dllexport)
#else
#define API
#endif

typedef struct { double lat, lon, height, heading; } Pose;
typedef struct {
    double az, el;
    int valid, mode;
} Target;
typedef struct {
    Target air, ground;
    int phase, point, samples, ack, settling, confirm, rx_probe, reason;
    int holding;
    double acquire, release, severe;
    uint32_t severe_ms, ack_timeout_ms;
    double track_center, track_best, center_az, center_el;
} Readout;

typedef struct {
    AirHistory history;
    AirInput air;
    AirReply reply;
    VehicleInput ground;
    Vehicle scanner;
    bool fixed_thresholds;
    double nominal;
    double track_center, track_best;
} Control;

API Control *control_new(void) {
    Control *c=calloc(1,sizeof(*c));
    if (!c) return NULL;
    vehicle_init(&c->scanner);
    c->scanner.cfg.fast_relock=true;
    c->scanner.cfg.fast_acquire=true;
    c->scanner.cfg.recover_radius_points=1;
    c->scanner.cfg.track_weak_ms=250;
    c->scanner.cfg.rx_half_power_deg=6; /* half-power half-angle of the simulated receiver */
    c->air.link.enabled=c->ground.link.enabled=true;
    c->air.link.local_ok=c->ground.link.local_ok=true;
    c->ground.session=1;
    c->ground.has_beacon_flag=true;
    c->ground.rssi_valid=true;
    return c;
}

API void control_free(Control *c) { free(c); }

static void range_thresholds(Control *c) {
    c->scanner.cfg.acquire_level=fmax(-112.0,c->nominal-2.0);
    c->scanner.cfg.release_level=fmax(-115.0,c->nominal-3.0);
}

API int control_thresholds(Control *c, int fixed, double detect, double severe,
                           double release, double acquire, Readout *out) {
    VehicleConfig *cfg=&c->scanner.cfg;
    if (fixed) {
        if (!(detect>=-120 && detect<=release && severe>=-120 && severe<release &&
              release<acquire && acquire<=0))
            return 0;
        cfg->detect_level=detect;
        cfg->realign_drop_db=release-severe;
        cfg->release_level=release;
        cfg->acquire_level=acquire;
    } else {
        cfg->detect_level=vehicle_defaults.detect_level;
        cfg->realign_drop_db=vehicle_defaults.realign_drop_db;
        range_thresholds(c);
    }
    c->fixed_thresholds=fixed!=0;
    out->acquire=cfg->acquire_level;
    out->release=cfg->release_level;
    out->severe=vehicle_realign_threshold(cfg);
    return 1;
}

static Target target(Command c) {
    return (Target){c.az_deg,c.el_deg,c.valid,c.mode};
}

API void control_step(Control *c, uint32_t now, const Pose *air, const Pose *ground,
                      const double *axes, double rssi, int detected, double on_axis_level,
                      int request_realign, Readout *out) {
    if (now%100==0) {
        c->air.link.local=(Fix){air->lat,air->lon,air->height,now,true};
        c->ground.link.local=(Fix){ground->lat,ground->lon,ground->height,now,true};
        c->air.heading_valid=c->ground.heading_valid=true;
        c->air.heading_ms=c->ground.heading_ms=now;
        c->air.true_heading_deg=air->heading;
        c->ground.rough_heading_deg=wrap360(ground->heading+3.0);
    }
    if (now%LORA_PERIOD_MS==0) {
        /* Budget for the range of this navigation exchange, no pointing truth.
           Carrier detection remains independent of the tighter lock gate. */
        c->nominal=on_axis_level;
        if (!c->fixed_thresholds) range_thresholds(c);
        c->air.link.peer=c->ground.link.local;
        c->ground.link.peer=c->air.link.local;
        c->air.link.peer_ok=c->ground.link.peer_ok=true;
        c->air.link.peer_status_ms=c->ground.link.peer_status_ms=now;
        c->air.request=c->scanner.request;
        c->ground.air=c->reply;
        c->air.request_rx_ms=c->ground.air_rx_ms=now;
    }
    c->air.link.actual=(GimbalAngles){axes[0],axes[1],now,true};
    c->ground.link.actual=(GimbalAngles){axes[2],axes[3],now,true};
    c->ground.beacon_present=detected!=0;
    c->ground.request_realign=request_realign!=0;
    c->ground.sample_ms=now;
    c->ground.rssi=rssi;

    Vehicle *v=&c->scanner;
    Mode before=v->mode;
    int old_point=v->point;
    bool confirming=v->confirm_lock;
    bool monitoring_weak=v->monitoring_lock && v->weak_stream;
    bool retry_center=v->retry_center;
    bool refined=v->refined;
    double old_az=v->center_az, old_el=v->center_el;
    out->air=target(airborne_track_step(&c->history,now,&c->air,&c->reply));
    out->ground=target(vehicle_step(v,now,&c->ground));
    if (before!=TRACK || v->mode!=TRACK) {
        c->track_center=c->track_best=-121;
    } else if (old_point==4 && v->point==0) {
        c->track_center=c->track_best=v->cross_score[0];
        for (int i=1;i<5;++i) {
            if (v->cross_score[i]<=-121) { c->track_best=-121; break; }
            c->track_best=fmax(c->track_best,v->cross_score[i]);
        }
    }
    out->track_center=c->track_center;
    out->track_best=c->track_best;
    out->center_az=v->center_az;
    out->center_el=v->center_el;
    out->phase=v->mode;
    out->point=v->point;
    out->samples=(int)v->samples;
    out->settling=v->settling;
    out->ack=c->ground.air.arrived && c->ground.air.session==v->request.session &&
             c->ground.air.point==v->request.point && now-c->ground.air_rx_ms<=COORD_MAX_AGE_MS;
    out->confirm=v->confirm_lock;
    out->rx_probe=v->ground_refresh && v->mode==CAL_AIR;
    out->holding=v->monitoring_lock;
    out->acquire=v->cfg.acquire_level;
    out->release=v->cfg.release_level;
    out->severe=vehicle_realign_threshold(&v->cfg);
    out->severe_ms=v->cfg.realign_weak_ms;
    out->ack_timeout_ms=v->cfg.realign_ack_ms;
    /* Disambiguate edges which have multiple causes in the actual C state machine. */
    out->reason=0;
    if (before==REFINE && v->mode==VERIFY_PEAK) out->reason=v->refined ? 1 : 2;
    if (before==VERIFY_PEAK && v->mode==REFINE) {
        out->reason=(refined || old_az!=v->center_az || old_el!=v->center_el) ? 3 : 6;
    }
    if (before==CAL_AIR && v->mode==VERIFY_PEAK)
        out->reason=confirming ? 4 : (v->air_rounds>=v->cfg.air_max_rounds ? 7 : 5);
    if (before==REFINE && v->mode==CAL_AIR && v->confirm_lock) out->reason=8;
    if (before==REFINE && v->mode==CAL_AIR && !v->confirm_lock) out->reason=11;
    if (before==TRACK && v->mode==CAL_AIR && !v->cfg.air_recheck_ms) out->reason=17;
    if (before==RECOVER && v->mode==NEAR) out->reason=12;
    if (before==CAL_AIR && v->mode==RECOVER && !monitoring_weak && c->ground.beacon_present)
        out->reason=13;
    if (retry_center && !v->retry_center && v->mode==REFINE) out->reason=9;
    if (monitoring_weak && !v->weak_stream && v->mode==RECOVER &&
        now-v->weak_since_ms>=c->scanner.cfg.track_weak_ms &&
        v->weak_sample_ms==now) out->reason=10;
    switch (v->realign_reason) {
    case REALIGN_SIGNAL: out->reason=14; break;
    case REALIGN_ACK: out->reason=15; break;
    case REALIGN_REQUEST: out->reason=16; break;
    default: break;
    }
}

