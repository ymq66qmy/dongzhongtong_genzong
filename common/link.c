#include "link.h"
#include <math.h>
#define RAD (0.01745329251994329577)

double wrap360(double angle) {
    double x = fmod(angle, 360.0);
    if (x<0) x+=360.0;
    return x>=360.0 ? 0.0 : x;
}

bool gga_degrees(double raw, char hemisphere, double *degrees) {
    bool lat=hemisphere=='N' || hemisphere=='S';
    bool lon=hemisphere=='E' || hemisphere=='W';
    if (!degrees || (!lat && !lon) || !isfinite(raw) || raw<0) return false;
    double whole=floor(raw/100), minutes=raw-100*whole;
    double value=whole+minutes/60;
    if (minutes>=60 || value>(lat ? 90 : 180)) return false;
    *degrees=(hemisphere=='S' || hemisphere=='W') ? -value : value;
    return true;
}

double shortest_angle(double angle) { return wrap360(angle+180.0)-180.0; }

double prediction_seconds(uint32_t age_ms, uint32_t lead_ms) {
    return fmin((double)age_ms+lead_ms, MAX_PREDICTION_MS)*0.001;
}

bool encoder_ready(uint32_t now, const LinkInput *in) {
    return in->actual.valid && now-in->actual.stamp_ms<=ENCODER_MAX_AGE_MS &&
           isfinite(in->actual.az_deg) && isfinite(in->actual.el_deg) &&
           in->actual.el_deg>=0.0 && in->actual.el_deg<=90.0 &&
           isfinite(in->frame.az_zero_deg) && isfinite(in->frame.el_zero_deg);
}

Command gimbal_command(Command body, const LinkInput *in) {
    const GimbalFrame *f=&in->frame;
    double az=f->az_zero_deg+(f->reverse_az ? -body.az_deg : body.az_deg);
    body.az_deg=wrap360(az);
    body.el_deg=f->el_zero_deg+(f->reverse_el ? -body.el_deg : body.el_deg);
    body.el_deg=fmax(0.0,fmin(90.0,body.el_deg));
    return body;
}

static bool fresh(uint32_t now, Fix p) {
    return p.valid && now - p.stamp_ms <= NAV_MAX_AGE_MS &&
           isfinite(p.lat_deg) && fabs(p.lat_deg) <= 90 &&
           isfinite(p.lon_deg) && fabs(p.lon_deg) <= 180 && isfinite(p.height_m);
}

typedef struct { double x, y, z; } Vec;
static Vec ecef(Fix p) {
    double lat = p.lat_deg * RAD, lon = p.lon_deg * RAD;
    double s = sin(lat), n = 6378137.0 / sqrt(1 - 0.0066943799901413165*s*s);
    double h = (n + p.height_m) * cos(lat);
    return (Vec){h*cos(lon), h*sin(lon), (n*(1-0.0066943799901413165)+p.height_m)*s};
}

Velocity estimate_velocity(const Fix *current, const Fix *previous) {
    uint32_t dt=current->stamp_ms-previous->stamp_ms;
    if (dt<50 || dt>NAV_MAX_AGE_MS || !fresh(current->stamp_ms,*current) ||
        !fresh(current->stamp_ms,*previous)) {
        return (Velocity){0};
    }
    Vec a=ecef(*previous), b=ecef(*current);
    double scale=1000.0/dt;
    double dx=(b.x-a.x)*scale, dy=(b.y-a.y)*scale, dz=(b.z-a.z)*scale;
    double lat=current->lat_deg*RAD, lon=current->lon_deg*RAD;
    double radial=cos(lon)*dx+sin(lon)*dy;
    return (Velocity){-sin(lon)*dx+cos(lon)*dy,
        -sin(lat)*radial+cos(lat)*dz,cos(lat)*radial+sin(lat)*dz,true};
}

static Vec predicted(Fix fix, Velocity v, uint32_t now, uint32_t lead_ms) {
    Vec p=ecef(fix);
    if (!v.valid || !isfinite(v.east) || !isfinite(v.north) || !isfinite(v.up)) return p;
    double dt=prediction_seconds(now-fix.stamp_ms,lead_ms);
    double lat=fix.lat_deg*RAD, lon=fix.lon_deg*RAD;
    double radial=cos(lat)*v.up-sin(lat)*v.north;
    p.x+=dt*(cos(lon)*radial-sin(lon)*v.east);
    p.y+=dt*(sin(lon)*radial+cos(lon)*v.east);
    p.z+=dt*(sin(lat)*v.up+cos(lat)*v.north);
    return p;
}

Command link_status(uint32_t now, const LinkInput *in) {
    Command out = {IDLE, false, 0, 0};
    if (!in->enabled) return out;
    out.mode = FAULT;
    if (!in->local_ok) return out;
    out.mode = WAIT_DATA;
    if (now - in->peer_status_ms > NAV_MAX_AGE_MS) return out;
    if (!in->peer_ok) { out.mode = FAULT; return out; }
    return (Command){POINT,true,0,0};
}

Command moving_line_of_sight(uint32_t now, const LinkInput *in, Velocity local, Velocity peer) {
    Command out=link_status(now,in);
    if (!out.valid) return out;
    out=(Command){WAIT_DATA,false,0,0};
    if (!fresh(now,in->local) || !fresh(now,in->peer)) return out;
    Vec a=predicted(in->local,local,now,in->lead_ms);
    Vec b=predicted(in->peer,peer,now,in->lead_ms);
    double dx=b.x-a.x, dy=b.y-a.y, dz=b.z-a.z;
    double lat=in->local.lat_deg*RAD, lon=in->local.lon_deg*RAD;
    double radial=cos(lon)*dx + sin(lon)*dy;
    double east=-sin(lon)*dx + cos(lon)*dy;
    double north=-sin(lat)*radial + cos(lat)*dz;
    double up=cos(lat)*radial + sin(lat)*dz;
    double horizontal=hypot(east,north);
    if (!isfinite(horizontal) || !isfinite(up) || hypot(horizontal,up)<1.0) return out;
    return (Command){POINT,true,wrap360(atan2(east,north)/RAD),atan2(up,horizontal)/RAD};
}

Command line_of_sight(uint32_t now, const LinkInput *in) {
    return moving_line_of_sight(now,in,(Velocity){0},(Velocity){0});
}
