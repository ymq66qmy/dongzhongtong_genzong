#include "airborne.h"
#include <math.h>

void airborne_track_init(AirHistory *history) {
    *history=(AirHistory){0};
}

static Command aim(uint32_t now, const AirInput *in, AirReply *reply, Command c) {
    *reply=(AirReply){0};
    if (!c.valid) return c;
    if (!encoder_ready(now,&in->link) || !in->heading_valid || !isfinite(in->true_heading_deg) ||
        !isfinite(in->pitch_deg) || !isfinite(in->roll_deg) ||
        now-in->heading_ms>NAV_MAX_AGE_MS) return (Command){WAIT_DATA,false,0,0};
    const AirRequest *r=&in->request;
    bool requested=r->session && now-in->request_rx_ms<=COORD_MAX_AGE_MS &&
                   isfinite(r->az_trim) && fabs(r->az_trim)<=AIR_TRIM_LIMIT_DEG &&
                   isfinite(r->el_trim) && fabs(r->el_trim)<=AIR_TRIM_LIMIT_DEG;
    if (requested) {
        c.az_deg=wrap360(c.az_deg+r->az_trim);
        c.el_deg=fmax(-90.0,fmin(90.0,c.el_deg+r->el_trim));
    }
    const double rad=0.01745329251994329577;
    double heading=in->true_heading_deg, pitch=in->pitch_deg, roll=in->roll_deg;
    if (in->rates_valid && isfinite(in->heading_rate) && isfinite(in->pitch_rate) &&
        isfinite(in->roll_rate)) {
        double dt=prediction_seconds(now-in->heading_ms,in->link.lead_ms);
        heading+=in->heading_rate*dt;
        pitch+=in->pitch_rate*dt;
        roll+=in->roll_rate*dt;
    }
    double yaw=wrap360(c.az_deg-heading)*rad, el=c.el_deg*rad;
    pitch*=rad;
    roll*=rad;
    /* Inverse yaw, pitch, roll rotation into body forward/right/down. */
    double along=cos(el)*cos(yaw), side=cos(el)*sin(yaw), down=-sin(el);
    double forward=cos(pitch)*along-sin(pitch)*down;
    double tilted_down=sin(pitch)*along+cos(pitch)*down;
    double right=cos(roll)*side+sin(roll)*tilted_down;
    double body_down=-sin(roll)*side+cos(roll)*tilted_down;
    c.az_deg=wrap360(atan2(right,forward)/rad);
    /* Mechanical zero points along the horizon; the airborne face tilts down. */
    c.el_deg=fmax(0.0,atan2(body_down,hypot(forward,right))/rad);
    c=gimbal_command(c,&in->link);
    if (requested) {
        bool arrived=fabs(shortest_angle(c.az_deg-in->link.actual.az_deg))<=AIR_ARRIVAL_DEG &&
                     fabs(c.el_deg-in->link.actual.el_deg)<=AIR_ARRIVAL_DEG;
        *reply=(AirReply){r->session,r->point,arrived};
    }
    return c;
}

Command airborne_point_step(uint32_t now, const AirInput *in, AirReply *reply) {
    return aim(now,in,reply,line_of_sight(now,&in->link));
}

Command airborne_step(uint32_t now, const AirInput *in) {
    AirReply reply;
    return airborne_point_step(now,in,&reply);
}

/* Repeated 10 ms control ticks reuse the last accepted fix and its estimate.
   A duplicate or reordered radio frame must not become another difference. */
static Fix motion_fix(MotionSample *history, Fix incoming, uint32_t now) {
    Fix *last=&history->fix;
    if (!incoming.valid || now-incoming.stamp_ms>NAV_MAX_AGE_MS ||
        !isfinite(incoming.lat_deg) || fabs(incoming.lat_deg)>90 ||
        !isfinite(incoming.lon_deg) || fabs(incoming.lon_deg)>180 ||
        !isfinite(incoming.height_m)) {
        if (last->valid && incoming.valid && now-last->stamp_ms<=NAV_MAX_AGE_MS &&
            incoming.stamp_ms-last->stamp_ms>=0x80000000u) return *last;
        history->estimate=(Velocity){0};
        last->valid=false;
        incoming.valid=false;
        return incoming;
    }
    if (last->valid && (incoming.stamp_ms==last->stamp_ms ||
        incoming.stamp_ms-last->stamp_ms>=0x80000000u)) return *last;
    if (last->valid && incoming.stamp_ms-last->stamp_ms<50u) return incoming;
    history->estimate=estimate_velocity(&incoming,last);
    *last=incoming;
    return incoming;
}

Command airborne_track_step(AirHistory *history, uint32_t now, const AirInput *in, AirReply *reply) {
    AirInput current=*in;
    current.link.local=motion_fix(&history->local,in->link.local,now);
    current.link.peer=motion_fix(&history->peer,in->link.peer,now);
    return aim(now,&current,reply,moving_line_of_sight(now,&current.link,
               history->local.estimate,history->peer.estimate));
}
