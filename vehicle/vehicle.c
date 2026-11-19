#include "vehicle.h"
#include <math.h>

typedef struct { double az, el; } Offset;
#define NO_BEACON_SCORE (-121.0)

const VehicleConfig vehicle_defaults={
    .sweep_step_deg=10, .row_step_deg=6, .acquire_level=-88, .release_level=-96,
    .peak_margin=0.45, .refine_start_deg=4, .refine_min_deg=1, .refine_factor=0.5,
    .refine_max_rounds=12, .weak_rounds=2, .probe_deg=1.25, .recover_step_deg=5,
    .arrival_tolerance_deg=0.25, .recover_radius_points=2, .sample_count=5,
    .settle_ms=40, .sample_window_ms=110, .move_timeout_ms=4000,
    .air_search_deg=8, .air_refine_deg=4, .air_min_deg=1,
    .air_max_rounds=15, .air_recheck_ms=0, .near_step_deg=4, .detect_level=-116,
    .realign_drop_db=24, .realign_weak_ms=250, .realign_ack_ms=6000, .pair_margin=0.12
};

static int sweep_points(const VehicleConfig *cfg) {
    return (int)ceil(360.0/cfg->sweep_step_deg);
}

static Offset cross_point(int i, double span) {
    static const int cross[5][2] = {{0,0},{1,0},{-1,0},{0,1},{0,-1}};
    return (Offset){cross[i][0]*span,cross[i][1]*span};
}

static int cross_index(const Vehicle *v, int point) {
    static const int elevation_first[5]={0,3,4,1,2};
    return v->mode==TRACK && v->probe_el_first ? elevation_first[point] : point;
}

static Offset offset(const Vehicle *v, int i) {
    const VehicleConfig *cfg=&v->cfg;
    Mode mode=v->mode;
    if (mode==CAL_AIR) return v->ground_refresh ? cross_point(i,cfg->probe_deg) : (Offset){0,0};
    if (mode==NEAR) return (Offset){0,0};
    if (mode==VERIFY_PEAK) {
        /* Center first, then every remaining point of the wider 5x5 grid. */
        if (!i) return (Offset){0,0};
        int k=i<=12 ? i-1 : i;
        int row=k/5, col=k%5;
        if (row%2) col=4-col;
        return (Offset){(col-2)*cfg->near_step_deg,(row-2)*cfg->near_step_deg};
    }
    if (mode == SWEEP) return (Offset){i*(360.0/sweep_points(cfg)),0};
    if (mode == RECOVER) {
        int r=(int)cfg->recover_radius_points, width=2*r+1;
        if (cfg->fast_relock && v->ever_locked) {
            if (!i) return (Offset){0,0};
            /* Walk successive square rings, one nearest-neighbor step at a time. */
            int ring=(int)ceil((sqrt(1.0+i)-1.0)/2.0);
            int side=2*ring, k=i-(side-1)*(side-1);
            int x=ring, y=1-ring+k%side;
            switch (k/side) {
            case 1: x=ring-1-k%side; y=ring; break;
            case 2: x=-ring; y=ring-1-k%side; break;
            case 3: x=1-ring+k%side; y=-ring; break;
            }
            return (Offset){x*cfg->recover_step_deg,y*cfg->recover_step_deg};
        }
        int row=i/width, col=i%width;
        if (row%2) col=width-1-col;
        return (Offset){(col-r)*cfg->recover_step_deg,(row-r)*cfg->recover_step_deg};
    }
    double span = mode == TRACK ? cfg->probe_deg : v->refine_span;
    return cross_point(cross_index(v,i),span);
}

static double limit_el(double angle) {
    return fmin(90.0,fmax(0.0,angle));
}

static void start_pattern(Vehicle *v, Mode mode) {
    if (mode==RECOVER && v->monitoring_lock && v->cfg.fast_relock) {
        v->saved_az=v->center_az;
        v->saved_el=v->center_el;
        v->saved_air_az=v->air_az;
        v->saved_air_el=v->air_el;
        v->retry_center=true;
        v->missed_beacon=false;
    }
    v->mode=mode;
    if (mode!=CAL_AIR) v->trial_trim=v->verify_previous_trim=false;
    if (mode!=TRACK) v->prior_slope[0]=v->prior_slope[1]=0;
    if (mode!=TRACK) v->last_track_az_step=v->last_track_el_step=0;
    if (mode!=TRACK) v->low_center_rounds=0;
    if (mode!=TRACK) { v->track_count=0; v->probe_el_first=false; }
    if (mode!=TRACK && mode!=CAL_AIR) v->monitoring_lock=false;
    v->point=0;
    v->best_point=0;
    v->best_score=NO_BEACON_SCORE;
    v->center_score=NO_BEACON_SCORE;
    v->commanded=false;
    if (mode==NEAR) {
        v->air_az=v->air_el=0;
        v->air_search_point=0;
        v->sweep_row=0;
        v->center_az=v->center_el=0;
        v->request.session=0;
    }
    if (mode==SWEEP) {
        static const double rows[]={0,1,-1,2,-2};
        v->center_az=0;
        v->center_el=rows[v->sweep_row]*v->cfg.row_step_deg;
    }
}

static void restart(Vehicle *v, VehicleConfig cfg) {
    *v=(Vehicle){.cfg=cfg};
    start_pattern(v,NEAR);
}

void vehicle_init(Vehicle *v) { restart(v,vehicle_defaults); }

static bool between(double value, double lo, double hi) {
    return isfinite(value) && value>=lo && value<=hi;
}

bool vehicle_configure(Vehicle *v, const VehicleConfig *c) {
    if (!v || !c || !between(c->sweep_step_deg,0.1,180) || !between(c->row_step_deg,0,45) ||
        !between(c->acquire_level,-120,0) || !between(c->release_level,-120,0) ||
        c->release_level>=c->acquire_level || c->release_level<=-120 ||
        !between(c->peak_margin,0,120) || !between(c->refine_start_deg,0.01,90) ||
        !between(c->refine_min_deg,0.01,c->refine_start_deg) ||
        !(c->refine_factor>0 && c->refine_factor<1) ||
        !between(c->probe_deg,0.01,90) || !between(c->recover_step_deg,0.01,90) ||
        !between(c->arrival_tolerance_deg,0.001,5) || c->recover_radius_points<1 ||
        c->recover_radius_points>32 || !c->weak_rounds || c->weak_rounds>1000 ||
        !c->refine_max_rounds || c->refine_max_rounds>1000 ||
        !c->sample_count || c->sample_count>1000 || c->settle_ms>60000 ||
        c->sample_window_ms<c->sample_count*10u || c->sample_window_ms>60000 ||
        !c->move_timeout_ms || c->move_timeout_ms>60000 ||
        !between(c->air_search_deg,0.01,AIR_TRIM_LIMIT_DEG) ||
        !between(c->air_refine_deg,0.01,AIR_TRIM_LIMIT_DEG) ||
        !between(c->air_min_deg,0.01,c->air_refine_deg) ||
        !c->air_max_rounds || c->air_max_rounds>1000 ||
        !between(c->near_step_deg,0.01,90) || !between(c->detect_level,-120,c->release_level) ||
        !between(c->realign_drop_db,0,120) || !between(c->pair_margin,0,120) ||
        !between(c->rx_half_power_deg,0,90)) return false;
    restart(v,*c);
    return true;
}

static void apply_peak(Vehicle *v) {
    Offset d=offset(v,v->best_point);
    v->center_az=wrap360(v->center_az+d.az);
    v->center_el=limit_el(v->center_el+d.el);
}

static double trim_limit(double x) {
    return fmax(-AIR_TRIM_LIMIT_DEG,fmin(AIR_TRIM_LIMIT_DEG,x));
}

static Offset air_offset(const Vehicle *v, int i) {
    Offset d=v->mode==CAL_AIR && !v->ground_refresh && !v->confirm_lock ?
             cross_point(i,v->air_span) : (Offset){0,0};
    return (Offset){trim_limit(v->air_az+d.az),trim_limit(v->air_el+d.el)};
}

static void start_air_cal(Vehicle *v) {
    v->trial_trim=false;
    v->verify_previous_trim=false;
    v->confirm_lock=false;
    v->ground_refresh=false;
    v->air_rounds=0;
    v->air_span=v->cfg.air_refine_deg;
    start_pattern(v,CAL_AIR);
}

static void start_refine(Vehicle *v) {
    v->refined=false;
    v->strong_crosses=0;
    v->refine_rounds=0;
    v->fast_probe=v->cfg.fast_acquire || (v->cfg.fast_relock && v->ever_locked);
    v->refine_span=v->fast_probe ? v->cfg.probe_deg : v->cfg.refine_start_deg;
    start_pattern(v,REFINE);
}

static bool stronger_neighbor(const Vehicle *v) {
    return v->best_score!=NO_BEACON_SCORE && (v->center_score==NO_BEACON_SCORE ||
           v->best_score>v->center_score+v->cfg.peak_margin);
}

/* Same-direction corrections in successive beacon crosses indicate a moving
   peak. Lead by at most one measured step; stop on reversal or no correction. */
static double motion_lead(double step, double previous) {
    return step*previous>0 ? copysign(fmin(fabs(step),fabs(previous)),step) : 0;
}

/* For a calibrated main lobe P(x)=Pmax-k*(x-peak)^2, opposite probes
   differ by 4*k*span*(peak-center). Normalize azimuth by cos(elevation).
   Only use complete, strong windows; the ordinary search handles sidelobes. */
static bool fit_rx_peak(const Vehicle *v, double el, double span, Offset *step) {
    const VehicleConfig *cfg=&v->cfg;
    if (cfg->rx_half_power_deg<=0 || cfg->pair_margin<=0 ||
        v->cross_score[0]<cfg->acquire_level-4.0) return false;
    double k=3.0/(cfg->rx_half_power_deg*cfg->rx_half_power_deg);
    double horizontal=cos(el*0.017453292519943295);
    bool fitted=false;
    for (int axis=0;axis<2;++axis) {
        double pos=v->cross_score[1+axis*2], neg=v->cross_score[2+axis*2];
        if (pos==NO_BEACON_SCORE || neg==NO_BEACON_SCORE ||
            fabs(pos-neg)<=2*cfg->pair_margin ||
            (axis && (el<span || el+span>90)) || (!axis && horizontal<0.25)) continue;
        double slope=axis ? k : k*horizontal*horizontal;
        double correction=(pos-neg)/(4*slope*span);
        double bound=2*span;
        correction=fmax(-bound,fmin(bound,correction));
        if (axis) step->el=correction;
        else step->az=correction;
        fitted=true;
    }
    return fitted;
}

/* Near a smooth maximum, fit a parabola through each symmetric triplet.
   Only use complete windows. A clipped mechanical probe is not symmetric. */
static bool adjust_cross_center(Vehicle *v, bool tx, double span) {
    double center=v->cross_score[0];
    double floor=v->mode==TRACK ? v->cfg.release_level : v->cfg.acquire_level;
    if (v->cfg.pair_margin<=0 || center<floor) return false;
    double *az=tx ? &v->air_az : &v->center_az;
    double *el=tx ? &v->air_el : &v->center_el;
    bool moved=false;
    for (int axis=0;axis<2;++axis) {
        double pos=v->cross_score[1+2*axis], neg=v->cross_score[2+2*axis];
        double diff=pos-neg, curvature=2*center-pos-neg;
        double coord=axis ? *el : *az;
        bool clipped=tx ? fabs(coord)+span>AIR_TRIM_LIMIT_DEG :
                     axis && (coord<span || coord+span>90);
        int sign=0;
        if (!clipped && pos!=NO_BEACON_SCORE && neg!=NO_BEACON_SCORE &&
            curvature>v->cfg.pair_margin && fabs(diff)>v->cfg.pair_margin)
            sign=diff>0 ? 1 : -1;
        if (v->mode==TRACK) {
            int previous=v->prior_slope[axis];
            v->prior_slope[axis]=sign;
            if (sign!=previous) continue;
        } else if (fabs(diff)<=2*v->cfg.pair_margin) continue;
        if (!sign) continue;
        double step=0.5*span*fmax(-1.0,fmin(1.0,diff/curvature));
        if (fabs(step)<v->cfg.arrival_tolerance_deg) continue;
        if (axis) *el+=step;
        else *az=tx ? *az+step : wrap360(*az+step);
        moved=true;
        v->prior_slope[axis]=0;
    }
    return moved;
}

static void finish_air_round(Vehicle *v, uint32_t now) {
    if (v->trial_trim) {
        v->trial_trim=false;
        if (v->samples!=v->cfg.sample_count ||
            v->center_score<v->cfg.acquire_level ||
            v->center_score<v->trial_score+v->cfg.pair_margin) {
            v->air_az=v->trial_az;
            v->air_el=v->trial_el;
            start_pattern(v,CAL_AIR); /* reacquire the saved center and confirm it */
            return;
        }
    }
    if (v->best_score==NO_BEACON_SCORE) { start_pattern(v,RECOVER); return; }
    if (v->confirm_lock) {
        v->confirm_lock=false;
        if (v->verify_previous_trim &&
            (v->samples!=v->cfg.sample_count || v->detected_samples!=v->cfg.sample_count ||
             v->center_score-v->cfg.acquire_level<v->peak_above_gate-v->cfg.peak_margin)) {
            if (v->rx_verify_retries<2 && v->detected_samples==v->cfg.sample_count &&
                v->center_score>=v->cfg.release_level) {
                ++v->rx_verify_retries;
                start_refine(v);
            } else start_air_cal(v);
            return;
        }
        if (v->samples==v->cfg.sample_count && v->center_score>=v->cfg.acquire_level) {
            v->last_air_cal_ms=now;
            v->confirmed_air_az=v->air_az;
            v->confirmed_air_el=v->air_el;
            v->peak_above_gate=v->center_score-v->cfg.acquire_level;
            v->weak_rounds=0;
            v->ever_locked=v->monitoring_lock=true;
            v->rx_verify_retries=0;
            v->retry_center=false;
            v->weak_stream=false;
            v->weak_sample_ms=now;
            start_pattern(v,TRACK);
        } else start_pattern(v,VERIFY_PEAK);
        return;
    }
    bool improved=stronger_neighbor(v);
    if (v->ground_refresh) {
        if (improved) apply_peak(v);
        else if (v->cfg.fast_acquire) adjust_cross_center(v,false,v->cfg.probe_deg);
        v->ground_refresh=false;
        start_pattern(v,CAL_AIR);
        return;
    }
    if (improved) {
        Offset best=air_offset(v,v->best_point);
        v->air_az=best.az;
        v->air_el=best.el;
    }
    if (!improved && (v->air_span<=v->cfg.air_min_deg || v->cfg.fast_acquire) &&
        v->center_score>=v->cfg.acquire_level) {
        v->trial_az=v->air_az;
        v->trial_el=v->air_el;
        v->trial_score=v->center_score;
        v->trial_trim=v->cfg.fast_acquire && adjust_cross_center(v,true,v->air_span);
        v->confirm_lock=true;
        start_pattern(v,CAL_AIR);
        return;
    }
    if ((!improved && v->air_span<=v->cfg.air_min_deg) ||
        ++v->air_rounds>=v->cfg.air_max_rounds) {
        /* A persistently weak local maximum cannot qualify the calibrated link.
           Search around it instead of alternating TX calibration and the same grid. */
        start_pattern(v,v->cfg.fast_acquire && v->best_score<v->cfg.acquire_level ?
                      RECOVER : VERIFY_PEAK);
        return;
    }
    if (!improved) v->air_span=fmax(v->cfg.air_min_deg,v->air_span*v->cfg.refine_factor);
    v->ground_refresh=true;
    start_pattern(v,CAL_AIR);
}

static void next_coarse_pair(Vehicle *v) {
    static const int grid[9][2]={{0,0},{1,0},{-1,0},{0,1},{0,-1},
                                 {1,1},{-1,1},{1,-1},{-1,-1}};
    v->air_search_point=(v->air_search_point+1)%9;
    if (!v->air_search_point) v->sweep_row=(v->sweep_row+1)%5;
    v->air_az=grid[v->air_search_point][0]*v->cfg.air_search_deg;
    v->air_el=grid[v->air_search_point][1]*v->cfg.air_search_deg;
}

static bool acquiring(Mode mode) { return mode==NEAR || mode==SWEEP; }

/* Measure the RSSI slope during the transit between probes as well as at
   their endpoints. Fit level+k*(x*x+y*y) to encoder angles and time.
   Removing a linear time trend separates common fading from angular gain. */
static void follow_encoder_samples(Vehicle *v, const VehicleInput *in, uint32_t now) {
    const VehicleConfig *cfg=&v->cfg;
    const GimbalAngles *enc=&in->link.actual;
    if (cfg->rx_half_power_deg<=0 || cfg->pair_margin<=0) return;
    uint32_t age=now-in->sample_ms, enc_age=now-enc->stamp_ms;
    if (!in->rssi_valid || !between(in->rssi,fmax(-120,cfg->release_level-3),0) ||
        (in->has_beacon_flag && !in->beacon_present) || age>30 || enc_age>30 ||
        fabs((double)age-enc_age)>10) {
        v->track_count=0;
        return;
    }
    if (v->track_count) {
        unsigned last=(v->track_next+TRACK_HISTORY_SIZE-1)%TRACK_HISTORY_SIZE;
        uint32_t gap=in->sample_ms-v->track_history[last].stamp;
        if (gap<40 || gap>=0x80000000u) return;
        if (gap>100) v->track_count=0;
    }
    const GimbalFrame *f=&in->link.frame;
    double az=wrap360((enc->az_deg-f->az_zero_deg)*(f->reverse_az ? -1 : 1));
    double el=(enc->el_deg-f->el_zero_deg)*(f->reverse_el ? -1 : 1);
    v->track_history[v->track_next]=(TrackSample){(float)az,(float)el,(float)in->rssi,in->sample_ms};
    v->track_next=(v->track_next+1)%TRACK_HISTORY_SIZE;
    if (v->track_count<TRACK_HISTORY_SIZE) ++v->track_count;
    if (v->track_count<TRACK_HISTORY_SIZE || now-v->last_track_fit_ms<200 || in->rssi<cfg->release_level ||
        in->rssi-cfg->acquire_level>v->peak_above_gate-cfg->peak_margin ||
        (cfg->air_recheck_ms && now-v->last_air_cal_ms>=cfg->air_recheck_ms)) return;
    v->last_track_fit_ms=now;
    double horizontal=cos(v->center_el*0.017453292519943295);
    if (horizontal<0.25) return;
    double k=3.0/(cfg->rx_half_power_deg*cfg->rx_half_power_deg);
    double sx=0,sy=0,st=0,sz=0,xx=0,yy=0,tt=0,xy=0,xt=0,yt=0,xz=0,yz=0,tz=0;
    for (unsigned i=0;i<TRACK_HISTORY_SIZE;++i) {
        const TrackSample *s=&v->track_history[i];
        double x=shortest_angle(s->az-v->center_az)*horizontal, y=s->el-v->center_el;
        double t=(now-s->stamp)*0.001, z=s->level-in->rssi+k*(x*x+y*y);
        sx+=x; sy+=y; st+=t; sz+=z;
        xx+=x*x; yy+=y*y; tt+=t*t; xy+=x*y; xt+=x*t; yt+=y*t;
        xz+=x*z; yz+=y*z; tz+=t*z;
    }
    const double n=TRACK_HISTORY_SIZE;
    tt-=st*st/n; xt-=sx*st/n; yt-=sy*st/n; tz-=st*sz/n;
    if (tt<=0) return;
    xx-=sx*sx/n+xt*xt/tt; yy-=sy*sy/n+yt*yt/tt;
    xy-=sx*sy/n+xt*yt/tt;
    xz-=sx*sz/n+xt*tz/tt; yz-=sy*sz/n+yt*tz/tt;
    double det=xx*yy-xy*xy;
    /* Minimum residual variance in squared degrees after removing the time trend.
       A straight single-axis traverse only identifies that axis. */
    double min_spread=n*0.025;
    double dx=0,dy=0;
    if (det>min_spread*fmax(xx,yy)) {
        dx=(yy*xz-xy*yz)/det; dy=(xx*yz-xy*xz)/det;
    } else if (xx>min_spread && yy<0.1*min_spread) dx=xz/xx;
    else if (yy>min_spread && xx<0.1*min_spread) dy=yz/yy;
    else return;
    if (hypot(dx,dy)*cfg->probe_deg<2*cfg->pair_margin) return;
    double bound=cfg->probe_deg;
    dx=fmax(-bound,fmin(bound,0.45*dx/(2*k*horizontal)));
    dy=fmax(-bound,fmin(bound,0.45*dy/(2*k)));
    v->center_az=wrap360(v->center_az+dx);
    v->center_el=limit_el(v->center_el+dy);
    v->probe_el_first=!v->probe_el_first;
    start_pattern(v,TRACK);
}

static void finish_point(Vehicle *v, double score, const GimbalFrame *f, uint32_t now) {
    const VehicleConfig *cfg=&v->cfg;
    Mode mode=v->mode;
    if ((mode==REFINE || mode==TRACK || mode==CAL_AIR) && v->point<5)
        v->cross_score[cross_index(v,v->point)]=v->detected_samples==cfg->sample_count ? score : NO_BEACON_SCORE;
    if (v->point==0) v->center_score=score;
    if (score>v->best_score) { v->best_score=score; v->best_point=v->point; }
    v->commanded=false;
    if (acquiring(mode) && score!=NO_BEACON_SCORE) {
        /* Latch the sampled native target. Navigation has no path to the RX
           command again until a complete local search confirms loss. */
        v->center_az=wrap360((v->command.az_deg-f->az_zero_deg)*(f->reverse_az ? -1 : 1));
        v->center_el=limit_el((v->command.el_deg-f->el_zero_deg)*(f->reverse_el ? -1 : 1));
        start_refine(v);
        return;
    }
    if (mode==NEAR) {
        v->sweep_known_weak=true; /* this geometric direction was already tried */
        start_pattern(v,SWEEP);
        return;
    }
    if (mode==VERIFY_PEAK && cfg->fast_acquire && v->samples==cfg->sample_count &&
        score>=cfg->acquire_level && stronger_neighbor(v)) {
        apply_peak(v);
        start_refine(v);
        return;
    }
    if (mode==RECOVER && cfg->fast_relock && v->ever_locked &&
        v->samples==cfg->sample_count && (score>=cfg->acquire_level ||
        (!v->point && cfg->rx_half_power_deg>0 && v->detected_samples==cfg->sample_count &&
         score>=cfg->release_level-3.0) ||
        (score>=cfg->acquire_level-8.0 && stronger_neighbor(v)))) {
        apply_peak(v);
        start_refine(v);
        return;
    }
    int width=(int)cfg->recover_radius_points*2+1;
    int count=5;
    if (mode==SWEEP) count=sweep_points(cfg);
    if (mode==RECOVER) count=width*width;
    if (mode==VERIFY_PEAK) count=25;
    if (mode==CAL_AIR && v->confirm_lock) count=1;
    if (++v->point<count) return;
    if (mode==CAL_AIR) { finish_air_round(v,now); return; }

    if (mode==SWEEP) {
        next_coarse_pair(v);
        start_pattern(v,SWEEP);
        return;
    }
    if (mode==RECOVER) {
        if (v->best_score==NO_BEACON_SCORE ||
            (cfg->fast_acquire && v->best_score<cfg->acquire_level-8.0)) {
            start_pattern(v,NEAR);
            return;
        }
        apply_peak(v);
        v->weak_rounds=0;
        start_refine(v);
        return;
    }

    if (mode==TRACK) {
        if (v->best_score<cfg->release_level) ++v->weak_rounds;
        else v->weak_rounds=0;
        if (v->weak_rounds>=cfg->weak_rounds) { start_pattern(v,RECOVER); return; }
    }
    bool improved=stronger_neighbor(v);
    if (mode==VERIFY_PEAK) {
        if (v->best_score==NO_BEACON_SCORE) start_pattern(v,RECOVER);
        else if (improved) { apply_peak(v); start_refine(v); }
        else if (!v->refined) start_refine(v);
        else start_air_cal(v);
        return;
    }
    double old_az=v->center_az, old_el=v->center_el;
    if (improved && (mode!=TRACK || v->best_score>=cfg->release_level)) apply_peak(v);
    if (mode==TRACK && improved) v->prior_slope[0]=v->prior_slope[1]=0;
    if (!improved && (mode==TRACK || (mode==REFINE && cfg->fast_acquire)))
        improved=adjust_cross_center(v,false,mode==TRACK ? cfg->probe_deg : v->refine_span);
    if (mode==TRACK) {
        double az_step=shortest_angle(v->center_az-old_az), el_step=v->center_el-old_el;
        /* During motion the center was sampled earlier than both side points.
           A strong opposite-side contrast can identify drift before the best
           side exceeds that earlier center by the normal peak margin. */
        if (v->cross_score[0]>=cfg->release_level) {
            for (int axis=0;axis<2;++axis) {
                double p=v->cross_score[1+2*axis], n=v->cross_score[2+2*axis];
                if (p==NO_BEACON_SCORE || n==NO_BEACON_SCORE ||
                    fabs(p-n)<=2*fmax(cfg->peak_margin,cfg->pair_margin) ||
                    fmax(p,n)<v->cross_score[0]-cfg->pair_margin) continue;
                double step=copysign(cfg->probe_deg,p-n);
                if (axis) el_step=limit_el(old_el+step)-old_el;
                else az_step=step;
            }
        }
        Offset fitted={az_step,el_step};
        bool calibrated=fit_rx_peak(v,old_el,cfg->probe_deg,&fitted);
        az_step=fitted.az;
        el_step=fitted.el;
        double lead=calibrated ? 0.5 : 1.0;
        v->center_az=wrap360(old_az+az_step+lead*motion_lead(az_step,v->last_track_az_step));
        v->center_el=limit_el(old_el+el_step+lead*motion_lead(el_step,v->last_track_el_step));
        improved=az_step!=0 || el_step!=0;
        v->last_track_az_step=az_step;
        v->last_track_el_step=el_step;
    }

    if (mode==REFINE) {
        Offset fitted={shortest_angle(v->center_az-old_az),v->center_el-old_el};
        if (fit_rx_peak(v,old_el,v->refine_span,&fitted)) {
            v->center_az=wrap360(old_az+fitted.az);
            v->center_el=limit_el(old_el+fitted.el);
            improved=fabs(fitted.az)>cfg->arrival_tolerance_deg ||
                     fabs(fitted.el)>cfg->arrival_tolerance_deg;
        }
        if (v->best_score==NO_BEACON_SCORE) { start_pattern(v,RECOVER); return; }
        /* A moving peak need not have a stationary encoder center. If two
           complete near-peak crosses agree, verify the fitted point there. */
        bool complete_cross=true;
        for (int i=0;i<5;++i)
            if (v->cross_score[i]==NO_BEACON_SCORE) complete_cross=false;
        bool near_peak=complete_cross && cfg->fast_acquire && cfg->rx_half_power_deg>0 &&
                       v->cross_score[0]>=cfg->acquire_level &&
                       v->best_score<=v->center_score+cfg->peak_margin;
        v->strong_crosses=near_peak ? v->strong_crosses+1 : 0;
        /* The measured peak can keep moving throughout an otherwise strong
           RX cross. Re-test the fitted center before starting a slow TX sweep. */
        if (cfg->fast_relock && cfg->rx_half_power_deg>0 && v->ever_locked &&
            v->air_az==v->confirmed_air_az && v->air_el==v->confirmed_air_el &&
            improved && complete_cross && v->center_score>=cfg->release_level &&
            v->best_score-cfg->acquire_level>=v->peak_above_gate-cfg->peak_margin) {
            start_air_cal(v);
            v->confirm_lock=v->verify_previous_trim=true;
            return;
        }
        if ((!improved || v->strong_crosses>=2) && (v->fast_probe || cfg->fast_acquire) &&
            v->samples==cfg->sample_count && v->center_score>=cfg->acquire_level) {
            start_air_cal(v);
            /* New links still probe both TX axes. A previously calibrated link
               may reuse its trim, subject to a fresh on-center confirmation. */
            v->confirm_lock=v->ever_locked && cfg->fast_relock &&
                v->center_score-cfg->acquire_level>=v->peak_above_gate-2*cfg->pair_margin;
            if (!v->confirm_lock && near_peak && cfg->fast_relock && v->ever_locked &&
                v->air_az==v->confirmed_air_az && v->air_el==v->confirmed_air_el) {
                v->confirm_lock=v->verify_previous_trim=true;
            }
            return;
        }
        if (v->fast_probe) {
            if (v->center_score<cfg->acquire_level || ++v->refine_rounds>=cfg->refine_max_rounds) {
                v->fast_probe=false;
                v->refine_rounds=0;
                v->refine_span=cfg->refine_start_deg;
            }
            start_pattern(v,REFINE);
            return;
        }
        if (!improved && v->center_score!=NO_BEACON_SCORE) {
            if (v->refine_span<=cfg->refine_min_deg) {
                v->weak_rounds=0;
                v->refined=true;
                if (cfg->fast_relock && v->ever_locked && v->center_score>=cfg->acquire_level &&
                    v->center_score-cfg->acquire_level>=v->peak_above_gate-2*cfg->pair_margin) {
                    start_air_cal(v);
                    v->confirm_lock=true;
                    return;
                }
                start_pattern(v,VERIFY_PEAK);
                return;
            } else {
                v->refine_span=fmax(cfg->refine_min_deg,v->refine_span*cfg->refine_factor);
            }
        }
        if (++v->refine_rounds>=cfg->refine_max_rounds) mode=VERIFY_PEAK;
    }
    if (mode==TRACK && cfg->fast_acquire && cfg->pair_margin>0) {
        double strength=v->cross_score[0]-cfg->acquire_level;
        double loss=v->peak_above_gate-strength;
        if (!improved && !v->prior_slope[0] && !v->prior_slope[1] && strength>=0 &&
            loss>fmax(cfg->peak_margin,2*cfg->pair_margin)) {
            if (++v->low_center_rounds>=2) { start_air_cal(v); return; }
        } else v->low_center_rounds=0;
        if (!improved && strength>v->peak_above_gate) v->peak_above_gate=strength;
    }
    if (mode==TRACK && cfg->air_recheck_ms && v->best_score>=cfg->acquire_level &&
        now-v->last_air_cal_ms>=cfg->air_recheck_ms) { start_air_cal(v); return; }
    start_pattern(v,mode);
}

double vehicle_realign_threshold(const VehicleConfig *cfg) {
    return fmax(-120.0,cfg->release_level-cfg->realign_drop_db);
}

static RealignReason realign_check(Vehicle *v, uint32_t now, const VehicleInput *in, bool matched) {
    if (in->request_realign) return REALIGN_REQUEST;
    if (!v->cfg.realign_ack_ms || matched || !v->request.session || v->mode==NEAR) {
        v->waiting_match=false;
    } else if (!v->waiting_match) {
        v->waiting_match=true;
        v->unmatched_since_ms=now;
    } else if (now-v->unmatched_since_ms>=v->cfg.realign_ack_ms) {
        return REALIGN_ACK;
    }

    uint32_t gap=in->sample_ms-v->guard_sample_ms;
    if (!v->cfg.realign_weak_ms || now-in->sample_ms>30u ||
        (v->have_guard_sample && (!gap || gap>=0x80000000u))) return REALIGN_NONE;
    bool absent=in->has_beacon_flag && !in->beacon_present;
    bool usable=in->rssi_valid && between(in->rssi,-120,0);
    double threshold=vehicle_realign_threshold(&v->cfg);
    bool severe=absent || (usable && in->rssi<=threshold);
    if (!v->severe_stream || gap>30u) v->severe_since_ms=in->sample_ms;
    v->have_guard_sample=true;
    v->guard_sample_ms=in->sample_ms;
    v->severe_stream=severe;
    double resume_level=fmin(v->cfg.release_level,threshold+3.0);
    if (usable && !absent && in->rssi>resume_level) {
        v->sweep_known_weak=false;
        v->reject_severe_candidate=false;
    }
    if (severe && v->mode!=NEAR && !(v->mode==SWEEP && v->sweep_known_weak) &&
        in->sample_ms-v->severe_since_ms>=v->cfg.realign_weak_ms) return REALIGN_SIGNAL;
    return REALIGN_NONE;
}

Command vehicle_step(Vehicle *v, uint32_t now, const VehicleInput *in) {
    bool geometric=acquiring(v->mode);
    Command geo=geometric ? line_of_sight(now,&in->link) : link_status(now,&in->link);
    const VehicleConfig *cfg=&v->cfg;
    if (!geo.valid) { restart(v,*cfg); return geo; }
    if (!encoder_ready(now,&in->link) || !in->session) {
        restart(v,*cfg);
        return (Command){WAIT_DATA,false,0,0};
    }

    if (v->request.session && v->request.session!=in->session) restart(v,*cfg);
    bool matched=in->air.session==in->session && in->air.point==v->request.point &&
                 v->request.session==in->session && now-in->air_rx_ms<=COORD_MAX_AGE_MS;
    RealignReason reason=realign_check(v,now,in,matched);
    if (reason!=REALIGN_NONE) {
        bool ever_locked=v->ever_locked;
        double peak_above_gate=v->peak_above_gate;
        double confirmed_az=v->confirmed_air_az, confirmed_el=v->confirmed_air_el;
        bool retry=reason==REALIGN_SIGNAL && v->monitoring_lock &&
                   v->cfg.fast_relock && in->has_beacon_flag && !in->beacon_present;
        double old_az=v->center_az, old_el=v->center_el;
        double old_air_az=v->air_az, old_air_el=v->air_el;
        restart(v,*cfg); /* drop old trim, peak, samples and outstanding point token */
        v->ever_locked=ever_locked;
        v->peak_above_gate=peak_above_gate;
        v->confirmed_air_az=confirmed_az;
        v->confirmed_air_el=confirmed_el;
        if (retry) {
            /* A blackout at a confirmed peak permits one historical probe only
               after the fresh geometric trial and a genuine signal return. */
            v->retry_center=v->missed_beacon=true;
            v->saved_az=old_az; v->saved_el=old_el;
            v->saved_air_az=old_air_az; v->saved_air_el=old_air_el;
        }
        v->reject_severe_candidate=reason==REALIGN_SIGNAL;
        matched=false;
    }
    v->realign_reason=reason;
    /* Observe fresh receiver frames even while a servo or LoRa ACK is pending.
       A short dip while probing must not be counted as a sustained loss. */
    bool probe_fade=v->mode==TRACK && in->rssi_valid && in->rssi>=cfg->release_level-
                    (cfg->acquire_level-cfg->release_level) &&
                    (!in->has_beacon_flag || in->beacon_present);
    bool checking_center=(v->mode==TRACK && !probe_fade) ||
                         (v->point==0 && matched && in->air.arrived && v->settling);
    if (!checking_center) v->weak_stream=false;
    if (cfg->track_weak_ms && v->monitoring_lock && checking_center && now-in->sample_ms<=30u &&
        (int32_t)(in->sample_ms-v->weak_sample_ms)>0) {
        bool weak=!in->rssi_valid || !isfinite(in->rssi) || in->rssi<cfg->release_level ||
                  in->rssi>0 || (in->has_beacon_flag && !in->beacon_present);
        if (!v->weak_stream || in->sample_ms-v->weak_sample_ms>30u)
            v->weak_since_ms=in->sample_ms;
        v->weak_sample_ms=in->sample_ms;
        v->weak_stream=weak;
        if (weak && in->sample_ms-v->weak_since_ms>=cfg->track_weak_ms) {
            v->weak_stream=false;
            start_pattern(v,RECOVER);
        }
    }
    if (v->retry_center && now-in->sample_ms<=30u &&
        (v->mode==RECOVER || acquiring(v->mode))) {
        bool seen=!v->reject_severe_candidate && in->rssi_valid && between(in->rssi,-120,0) &&
                  (in->has_beacon_flag ? in->beacon_present : in->rssi>=cfg->detect_level);
        if (!seen) v->missed_beacon=true;
        else if (v->missed_beacon && v->mode==NEAR) {
            /* The latest geometric aim is already in motion. A detected skirt
               on that approach must not send the servo back to a stale peak. */
            v->retry_center=false;
        } else if (v->missed_beacon) {
            /* One retry at the previous peak after an interruption. The same
               fresh cross and on-center lock window are still mandatory. */
            v->retry_center=false;
            v->center_az=v->saved_az;
            v->center_el=v->saved_el;
            v->air_az=v->saved_air_az;
            v->air_el=v->saved_air_el;
            start_refine(v);
        }
    }
    bool air_ready=matched && in->air.arrived;
    if (v->mode==TRACK && air_ready) follow_encoder_samples(v,in,now);
    else v->track_count=0;
    if (!matched || air_ready) v->air_waiting=false;
    else if (!v->air_waiting) {
        v->air_waiting=true;
        v->air_unready_ms=now;
    } else if (now-v->air_unready_ms>=cfg->move_timeout_ms) {
        /* A healthy moving platform may still be slewing. Reissue this
           point with a new token; hardware faults come from device status. */
        v->request.session=0;
        v->air_waiting=false;
    }
    if (v->commanded && !air_ready) {
        v->settling=false;
        v->moved_ms=now;
    }
    if (v->commanded && air_ready) {
        const GimbalAngles *actual=&in->link.actual;
        double az_error=shortest_angle(v->command.az_deg-actual->az_deg);
        if (now-actual->stamp_ms>=now-v->moved_ms ||
            fabs(az_error)>cfg->arrival_tolerance_deg ||
            fabs(v->command.el_deg-actual->el_deg)>cfg->arrival_tolerance_deg) {
            v->settling=false;
            if (now-v->moved_ms>=cfg->move_timeout_ms) {
                v->moved_ms=now;
            }
        } else if (!v->settling) {
            v->settling=true;
            v->stable_ms=now;
            v->last_sample_ms=now;
            v->samples=0;
            v->detected_samples=0;
            v->point_seen=false;
            v->sum=0;
        } else if (now-v->stable_ms>=cfg->settle_ms) {
            uint32_t age=now-in->sample_ms;
            uint32_t advance_ms=in->sample_ms-v->last_sample_ms;
            if (advance_ms>0 && advance_ms<0x80000000u && age<=30u &&
                age<=now-v->stable_ms-cfg->settle_ms) {
                v->last_sample_ms=in->sample_ms;
                ++v->samples;
                bool usable=in->rssi_valid && isfinite(in->rssi) &&
                            in->rssi>=-120 && in->rssi<=0;
                bool detected=usable && (in->has_beacon_flag ? in->beacon_present :
                                         in->rssi>=cfg->detect_level);
                if (v->reject_severe_candidate) detected=false;
                v->point_seen|=detected;
                if (detected) ++v->detected_samples;
                v->sum+=detected ? in->rssi : -120;
            }
            if ((acquiring(v->mode) && v->point_seen) || v->samples==cfg->sample_count ||
                now-v->stable_ms>=cfg->settle_ms+cfg->sample_window_ms) {
                double score=v->point_seen ? (v->sum-120.0*(cfg->sample_count-v->samples)) /
                                             cfg->sample_count : NO_BEACON_SCORE;
                finish_point(v,score,&in->link.frame,now);
            }
        }
    }
    Offset trim=air_offset(v,v->point);
    if (v->request.session!=in->session || trim.az!=v->request.az_trim ||
        trim.el!=v->request.el_trim) {
        v->request=(AirRequest){in->session,now,trim.az,trim.el};
        air_ready=false;
    }

    if (acquiring(v->mode) && !geometric) {
        geo=line_of_sight(now,&in->link);
        if (!geo.valid) return geo;
    }
    if (!v->commanded) {
        if ((v->mode==SWEEP || v->mode==NEAR) && v->point==0) {
            const GimbalFrame *f=&in->link.frame;
            double body_az=(in->link.actual.az_deg-f->az_zero_deg)*(f->reverse_az ? -1 : 1);
            v->search_yaw=wrap360(body_az-geo.az_deg);
        }
        v->moved_ms=now;
        v->settling=false;
        v->commanded=true;
    }
    Offset d=offset(v,v->point);
    Command body={v->mode,true,wrap360(v->center_az+d.az),limit_el(v->center_el+d.el)};
    if (acquiring(v->mode)) {
        if (v->mode==NEAR && in->heading_valid && isfinite(in->rough_heading_deg) &&
            now-in->heading_ms<=NAV_MAX_AGE_MS) v->search_yaw=-in->rough_heading_deg;
        body.az_deg=wrap360(geo.az_deg+v->search_yaw+d.az);
        body.el_deg=limit_el(geo.el_deg+v->center_el+d.el);
    }
    v->command=gimbal_command(body,&in->link);
    if (!air_ready && v->mode!=CAL_AIR) v->command.mode=POINT;
    return v->command;
}
