/* Build together with both controllers. Optional argv[1] = CSV trace. */
#include "../vehicle/vehicle.h"
#include "../airborne/airborne.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#define RAD 0.01745329251994329577

static double diff(double a,double b) { return wrap360(a-b+180)-180; }
static double move(double error) { return fmin(0.9,fmax(-0.9,error)); }

typedef struct { double az,el; } Axes;
static bool advance(Axes *a, Command cmd) {
    if (!cmd.valid) return false;
    a->az=wrap360(a->az+move(diff(cmd.az_deg,a->az)));
    a->el+=move(cmd.el_deg-a->el);
    return fabs(diff(a->az,cmd.az_deg))<0.05 && fabs(a->el-cmd.el_deg)<0.05;
}

static void check_inputs(void) {
    assert(shortest_angle(1-359)==2 && shortest_angle(359-1)==-2);
    assert(wrap360(-1e-15)<360);
    double degrees;
    assert(gga_degrees(3430.0,'S',&degrees) && degrees==-34.5);
    assert(!gga_degrees(10960.0,'E',&degrees));
    Fix a={0,179.999,100,UINT32_MAX-20,true};
    Fix b={0,-179.999,100,UINT32_MAX-20,true};
    AirInput in={.link={.local=a,.peer=b,.enabled=true,.local_ok=true,.peer_ok=true,
        .peer_status_ms=UINT32_MAX-20,.actual={359,0,10,true}},.true_heading_deg=90,
        .heading_ms=UINT32_MAX-20,.heading_valid=true};
    Command c=airborne_step(10,&in);
    assert(c.valid && fabs(diff(c.az_deg,0))<0.001 && fabs(c.el_deg)<0.01);
    in.link.frame=(GimbalFrame){3,12,true,true};
    c=airborne_step(10,&in);
    assert(fabs(c.az_deg-3)<0.001 && fabs(c.el_deg-12)<0.01);
    in.link.actual.valid=false;
    assert(airborne_step(10,&in).mode==WAIT_DATA);
    in.link.actual.valid=true;
    in.link.frame=(GimbalFrame){0};
    Fix prev={0,0,100,100,true}, next={0,0.00001,100,300,true};
    assert(estimate_velocity(&next,&prev).valid && fabs(estimate_velocity(&next,&prev).east-5.566)<0.01);
    assert(!estimate_velocity(&prev,&next).valid);
    Vehicle probe;
    VehicleConfig cfg=vehicle_defaults;
    cfg.refine_start_deg=6;
    cfg.sample_count=4;
    assert(vehicle_configure(&probe,&cfg) && probe.cfg.refine_start_deg==6);
    cfg.release_level=cfg.acquire_level;
    assert(!vehicle_configure(&probe,&cfg) && probe.cfg.sample_count==4);
    in.pitch_deg=10;
    assert(fabs(airborne_step(10,&in).el_deg-10)<0.01);
    assert(airborne_step(3000,&in).mode==WAIT_DATA);
    in.heading_valid=false;
    assert(!airborne_step(10,&in).valid);
    in.link.local_ok=false;
    assert(airborne_step(10,&in).mode==FAULT);
    in.link.enabled=false;
    assert(airborne_step(10,&in).mode==IDLE);
}

static void check_pitch_convention(void) {
    /* At the equator this longitude step spans 100 m on level ground. */
    const double lon=100.0/(6378137.0*RAD);
    const Fix aircraft[]={{0,lon,0,0,true},{0,lon,100,0,true},
                          {0,0,100,0,true},{0,0,-100,0,true}};
    const double expected[]={0,45,90,0};
    Fix ground={.valid=true};
    for (unsigned i=0;i<sizeof(expected)/sizeof(expected[0]);++i) {
        AirInput air={.heading_valid=true,.link={.enabled=true,.local_ok=true,.peer_ok=true,
                      .local=aircraft[i],.peer=ground,.actual={.valid=true}}};
        VehicleInput car={.session=1,.link={.enabled=true,.local_ok=true,.peer_ok=true,
                          .local=ground,.peer=aircraft[i],.actual={.valid=true}}};
        Vehicle v;
        vehicle_init(&v);
        Command a=airborne_step(0,&air), g=vehicle_step(&v,0,&car);
        assert(a.valid && g.valid && fabs(a.el_deg-expected[i])<0.01 &&
               fabs(g.el_deg-expected[i])<0.01);
        assert(a.el_deg>=0 && a.el_deg<=90 && g.el_deg>=0 && g.el_deg<=90);
        air.link.actual.el_deg=-1;
        car.link.actual.el_deg=91;
        assert(airborne_step(0,&air).mode==WAIT_DATA && vehicle_step(&v,0,&car).mode==WAIT_DATA);
    }
}

static void check_scan_config(void) {
    Vehicle scanner;
    VehicleConfig cfg=vehicle_defaults;
    cfg.sweep_step_deg=90;
    cfg.refine_start_deg=6;
    cfg.refine_min_deg=1.5;
    cfg.refine_max_rounds=3;
    cfg.peak_margin=1;
    cfg.settle_ms=0;
    cfg.sample_count=1;
    cfg.acquire_level=-65;
    assert(vehicle_configure(&scanner,&cfg));
    VehicleInput in={.link={.enabled=true,.local_ok=true,.peer_ok=true,
        .local={.valid=true},.peer={.lat_deg=0.01,.height_m=50,.valid=true},
        .frame={30,15,true,true},.actual={5,0,0,true}},
        .session=1,.rssi_valid=true,.rssi=-70};
    Command cmd={0};
    int levels=0;
    for (uint32_t now=0;now<10000;now+=10) {
        in.link.local.stamp_ms=in.link.peer.stamp_ms=in.link.peer_status_ms=now;
        in.link.actual.stamp_ms=in.sample_ms=now;
        if (cmd.valid) {
            in.link.actual.az_deg=cmd.az_deg;
            in.link.actual.el_deg=cmd.el_deg;
        }
        in.air=(AirReply){scanner.request.session,scanner.request.point,true};
        in.air_rx_ms=now;
        if (now<200) {
            in.air.point=scanner.request.point+1; /* delayed unrelated ACK */
        }
        if (now>=500) in.rssi=-60;
        cmd=vehicle_step(&scanner,now,&in);
        if (now<200) assert(scanner.point==0 && scanner.samples==0);
        if (now==0) assert(fabs(cmd.az_deg-5)<0.001);
        if (now<500) assert(cmd.mode!=TRACK && scanner.mode!=SWEEP);
        if (cmd.mode==REFINE) {
            if (scanner.refine_span==6) levels|=1;
            if (scanner.refine_span==3) levels|=2;
            if (scanner.refine_span==1.5) levels|=4;
        }
        if (cmd.mode==TRACK) {
            assert(levels==7);
            Command stale=vehicle_step(&scanner,now+ENCODER_MAX_AGE_MS+1,&in);
            assert(stale.mode==WAIT_DATA && !stale.valid && scanner.cfg.acquire_level==-65);
            return;
        }
    }
    assert(0 && "custom refinement did not converge");
}

static const char *mode_name(Mode mode) {
    static const char *names[]={"IDLE","WAIT_DATA","FAULT","POINT","SWEEP","REFINE",
                                "TRACK","RECOVER","CAL_AIR","NEAR","VERIFY_PEAK"};
    return names[mode];
}

static void check_recovery_thresholds(void) {
    Vehicle v;
    VehicleInput in={.link={.enabled=true,.local_ok=true,.peer_ok=true,
        .local={.valid=true},.peer={.lat_deg=0.01,.valid=true},.actual={.valid=true}},
        .session=1,.rssi_valid=true,.rssi=-92};
    vehicle_init(&v);
    v.mode=RECOVER;
    v.center_el=15;
    v.air_az=3;
    for (uint32_t now=0;now<15000;now+=10) {
        in.link.local.stamp_ms=in.link.peer.stamp_ms=in.link.peer_status_ms=now;
        in.link.actual.stamp_ms=in.air_rx_ms=in.sample_ms=now;
        in.air=(AirReply){v.request.session,v.request.point,now>=6000};
        Command cmd=vehicle_step(&v,now,&in);
        assert(cmd.valid && v.air_az==3);
        if (now<6000) assert(v.point==0 && v.mode==RECOVER);
        if (v.mode==REFINE) return; /* -92 is detectable; search again before locking */
        in.link.actual.az_deg=cmd.az_deg;
        in.link.actual.el_deg=cmd.el_deg;
    }
    assert(0 && "local recovery did not resume tracking");
}

/* At the geometric bearing a local sidelobe has its own maximum. Its stronger
   neighbor is eight degrees away, beyond a single fine cross. */
static void check_lobes(bool receiver_flag, bool strong_main) {
    Vehicle v;
    vehicle_init(&v);
    /* Exercise the wide-grid peak comparator in isolation, including its sweep
       through the receiver floor. Global interrupt behavior is tested separately. */
    v.cfg.realign_weak_ms=0;
    VehicleInput in={.session=1,.link={.local={.valid=true},.peer={.lat_deg=.01,.valid=true},
        .enabled=true,.local_ok=true,.peer_ok=true,.actual={.valid=true}},
        .rssi_valid=true,.has_beacon_flag=receiver_flag,.beacon_present=true};
    bool refined=false, verified=false, compared_main=false, calibrated=false;
    for (uint32_t now=0;now<40000;now+=10) {
        in.link.local.stamp_ms=in.link.peer.stamp_ms=in.link.peer_status_ms=now;
        in.link.actual.stamp_ms=in.sample_ms=in.air_rx_ms=now;
        in.air=(AirReply){v.request.session,v.request.point,true};
        double x=shortest_angle(in.link.actual.az_deg), y=in.link.actual.el_deg;
        double side=receiver_flag ? -119 : -85;
        double main_peak=strong_main ? -64 : -100;
        if (!strong_main) side=-112;
        in.rssi=fmax(-120,fmax(side-2*(x*x+y*y),
                     main_peak-2*((x-8)*(x-8)+y*y)));
        Command cmd=vehicle_step(&v,now,&in);
        assert(cmd.valid && v.mode!=SWEEP);
        if (v.mode==REFINE) refined=true;
        if (v.mode==VERIFY_PEAK) {
            verified=true;
            if (fabs(x-8)<0.1 && fabs(y)<0.1) compared_main=true;
        }
        if (v.mode==CAL_AIR) { assert(verified && compared_main); calibrated=true; }
        if (cmd.mode==TRACK) {
            assert(strong_main && calibrated && fabs(shortest_angle(v.center_az-8))<1.1);
            return;
        }
        in.link.actual.az_deg=cmd.az_deg;
        in.link.actual.el_deg=cmd.el_deg;
    }
    assert(!strong_main && refined && calibrated);
}

static void check_receiver_indication(void) {
    Vehicle v;
    vehicle_init(&v);
    VehicleInput in={.session=9,.link={.enabled=true,.local_ok=true,.peer_ok=true,
        .local={.valid=true},.peer={.lat_deg=0.01,.valid=true},.actual={.valid=true}},
        .rssi_valid=true,.has_beacon_flag=true,.rssi=-60};
    bool swept=false;
    for (uint32_t now=0;now<3000;now+=10) {
        in.link.actual.stamp_ms=in.link.peer.stamp_ms=in.link.local.stamp_ms=now;
        in.link.peer_status_ms=in.air_rx_ms=in.sample_ms=now;
        in.air=(AirReply){v.request.session,v.request.point,true};
        if (now>=500) { in.beacon_present=true; in.rssi=-119; }
        Command c=vehicle_step(&v,now,&in);
        if (v.mode==SWEEP) swept=true;
        if (v.mode==REFINE) { assert(swept && now>=500); return; }
        in.link.actual.az_deg=c.az_deg;
        in.link.actual.el_deg=c.el_deg;
    }
    assert(0 && "receiver indication did not interrupt the circular scan");
}

static void run_flight(bool early, FILE *csv) {
    const double takeoff=early ? 5.0 : 45.0;
    Vehicle v;
    vehicle_init(&v);
    AirHistory history={0};
    VehicleInput ground={.session=481,.heading_valid=true,.rough_heading_deg=early ? 160 : 92};
    AirInput air={.heading_valid=true};
    AirReply reply={0};
    Command gc={0},ac={0};
    Axes rx={60,0},tx={10,0};
    bool swept=false,local_search=false,restarted=false,restored=false;
    double lock=-1, relock=-1, sum=0, hold_sq=0, predict_sq=0;
    unsigned tracked=0, predicted=0;
    Mode old=IDLE;
    for (uint32_t now=0;now<285000;now+=10) {
        double t=now*0.001, f=fmax(0,t-takeoff);
        double east=220+55*sin(f/40), north=450+0.5*f, up=fmin(1.8*f,65);
        double heading=52+5*sin(f/80), base=90+0.4*sin(t/18);
        double az=atan2(east,north)/RAD, el=atan2(up,hypot(east,north))/RAD;
        double true_rx=wrap360(az-base);
        Fix u={north/110574.3,(east+4)/111319.5,42+up,now,true};
        Fix g={0,0,40,now,true};
        if (now%100==0) {
            air.link.local=u;
            ground.link.local=g;
            air.true_heading_deg=heading;
            air.heading_ms=ground.heading_ms=now;
        }
        /* One bidirectional request/response exchange per second. A missing
           exchange drops both telemetry and pointing coordination. */
        if (now%LORA_PERIOD_MS==0 && now!=78000) {
            air.link.peer=ground.link.local;
            ground.link.peer=air.link.local;
            air.link.peer_status_ms=ground.link.peer_status_ms=now;
            air.link.peer_ok=ground.link.peer_ok=true;
            air.request=v.request;
            air.request_rx_ms=now;
            ground.air=reply;
            ground.air_rx_ms=now;
        }
        air.link.enabled=ground.link.enabled=true;
        air.link.local_ok=ground.link.local_ok=true;
        advance(&tx,ac);
        advance(&rx,gc);
        ground.link.actual=(GimbalAngles){rx.az,rx.el,now,true};
        air.link.actual=(GimbalAngles){tx.az,tx.el,now,true};
        double err=hypot(diff(rx.az,true_rx),rx.el-el);
        double txerr=hypot(diff(tx.az,az+180-heading),tx.el-el);
        bool blackout=(t>=95 && t<95.1) || (t>=145 && t<183) || (early && t<6);
        ground.rssi_valid=!blackout;
        ground.rssi=blackout ? -120 : fmax(-120,-63-.25*err*err-.19*txerr*txerr+0.17*sin(503*t));
        ground.sample_ms=now;
        ac=airborne_track_step(&history,now,&air,&reply);
        if (t>takeoff+3 && f<30 && now%100!=0) {
            AirInput neutral=air;
            neutral.request.session=0;
            AirHistory copy=history;
            AirReply dummy;
            /* Fresh position with the same heading sample and mounting model isolates
               position prediction from geodetic approximation and heading latency. */
            AirInput ideal=neutral;
            ideal.link.local=u;
            ideal.link.peer=g;
            double target=airborne_step(now,&ideal).az_deg;
            double hold=airborne_step(now,&neutral).az_deg;
            double pred=airborne_track_step(&copy,now,&neutral,&dummy).az_deg;
            hold_sq+=pow(diff(hold,target),2);
            predict_sq+=pow(diff(pred,target),2);
            predicted++;
        }
        Mode before=v.mode;
        if (t==takeoff) assert(early ? before==SWEEP : before==TRACK);
        Vehicle shadow=v;
        gc=vehicle_step(&v,now,&ground);
        if ((before==REFINE || before==CAL_AIR || before==TRACK || before==VERIFY_PEAK || before==RECOVER) &&
            v.mode!=NEAR && v.mode!=SWEEP) {
            VehicleInput spoof=ground;
            spoof.link.peer.lon_deg+=0.3;
            spoof.link.peer.height_m+=800;
            spoof.rough_heading_deg+=90;
            Command twin=vehicle_step(&shadow,now,&spoof);
            assert(twin.mode==gc.mode && twin.az_deg==gc.az_deg && twin.el_deg==gc.el_deg);
        }
        assert(ac.valid && gc.valid);
        if (now==79000) assert(v.mode!=NEAR && v.mode!=SWEEP);
        if (gc.mode!=old && !early) printf("%6.2f s %s\n",t,mode_name(gc.mode));
        old=gc.mode;
        if (lock<0 && v.mode==SWEEP) swept=true;
        if (gc.mode==TRACK && lock<0) lock=t;
        if (!early && t>=takeoff && t<145) assert(v.mode!=NEAR && v.mode!=SWEEP);
        if (t>=145 && t<183) {
            if(v.mode==RECOVER) local_search=true;
            if(local_search && v.mode==NEAR) restarted=true;
        }
        if (t>=183 && gc.mode==TRACK && !restored) { restored=true; relock=t; }
        if (t>=95 && t<95.1) assert(v.mode!=NEAR && v.mode!=SWEEP);
        if (t>=80 && t<145 && gc.mode==TRACK) { sum+=err*err; tracked++; }
        if (csv) fprintf(csv,"%.2f,%s,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                         t,mode_name(gc.mode),ground.rssi,true_rx,rx.az,el,rx.el,err,txerr,
                         v.request.az_trim,v.request.el_trim);
    }
    printf("%s: takeoff %.0f, first TRACK %.2f, sweep %d, relock %.2f, RX RMS %.3f, "
           "air hold/predict %.5f/%.5f deg\n",early?"early":"ground",takeoff,lock,swept,
           relock,sqrt(sum/tracked),sqrt(hold_sq/predicted),sqrt(predict_sq/predicted));
    assert(lock>=0 && (early ? lock>takeoff && swept : lock<takeoff && !swept));
    assert(local_search && restarted && restored && tracked && sqrt(sum/tracked)<4);
    assert(predicted && predict_sq<hold_sq);
}

static void check_realign_interrupts(void) {
    const Mode phases[]={SWEEP,REFINE,VERIFY_PEAK,CAL_AIR,TRACK,RECOVER};
    for (unsigned i=0;i<sizeof(phases)/sizeof(phases[0]);++i) {
        for (int trigger=0;trigger<3;++trigger) {
            Vehicle v;
            vehicle_init(&v);
            v.mode=phases[i];
            v.air_az=4;
            v.cfg.release_level=-58.2;
            VehicleInput in={.session=9,.link={.enabled=true,.local_ok=true,.peer_ok=true,
                .local={.valid=true},.peer={.lat_deg=.002,.valid=true},.actual={.valid=true}},
                .rssi_valid=true,.has_beacon_flag=true,.beacon_present=true};
            RealignReason expected=trigger==0 ? REALIGN_SIGNAL : trigger==1 ? REALIGN_ACK : REALIGN_REQUEST;
            uint32_t t;
            for (t=1000;t<=7100;t+=10) {
                in.link.actual.stamp_ms=in.link.local.stamp_ms=in.link.peer.stamp_ms=t;
                in.link.peer_status_ms=in.sample_ms=t;
                in.rssi=trigger==0 ? -115.6 : -55;
                in.request_realign=trigger==2 && t==1100;
                Command cmd=vehicle_step(&v,t,&in);
                assert(cmd.valid);
                if (v.realign_reason!=REALIGN_NONE) {
                    assert(v.realign_reason==expected && v.mode==NEAR && v.point==0);
                    assert(v.request.az_trim==0 && v.request.el_trim==0 && v.request.point==t);
                    break;
                }
                assert(v.mode==phases[i]);
            }
            assert(t==(trigger==0 ? 1250u : trigger==1 ? 7010u : 1100u));
        }
    }
}

int main(int argc,char **argv) {
    check_realign_interrupts();
    check_inputs();
    check_pitch_convention();
    check_scan_config();
    check_recovery_thresholds();
    check_lobes(false,true);
    check_lobes(true,true);
    check_lobes(false,false);
    check_receiver_indication();
    FILE *csv=argc>1 ? fopen(argv[1],"w") : NULL;
    if (argc>1 && !csv) return 2;
    if (csv) fprintf(csv,"time_s,mode,rssi,az_true,az_actual,el_true,el_actual,error_deg,air_error_deg,tx_az_trim,tx_el_trim\n");
    run_flight(false,csv);
    if (csv) fclose(csv);
    run_flight(true,NULL);
    return 0;
}
