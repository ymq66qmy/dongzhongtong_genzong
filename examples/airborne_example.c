#include "antenna_examples.h"

static AirHistory history;
static Fix peer_fix;
static AirRequest peer_request;
static uint32_t peer_rx_ms;
static bool peer_ok;
static bool peer_received;

/* Call once at boot. Keep this state across takeoff and landing. */
void air_example_init(void)
{
    airborne_track_init(&history);
    peer_received = false;
}

/* Call for each newly accepted LoRa packet. fix.stamp_ms is the measurement
 * time converted to the aircraft clock. received_ms is the local RX time.
 * Serialize RX updates and control ticks or copy through a mailbox. */
void air_example_receive(Fix fix, bool healthy, AirRequest request, uint32_t received_ms)
{
    peer_fix = fix;
    peer_ok = healthy;
    peer_request = request;
    peer_rx_ms = received_ms;
    peer_received = true;
}

/* Call every 10 ms. local supplies the local GPS, encoder, attitude and flags.
 * Retain reply for the next 1 s LoRa send, together with local position/status. */
Command air_example_tick(uint32_t now, const AirInput *local, AirReply *reply)
{
    AirInput input = *local;
    input.link.peer = peer_received ? peer_fix : (Fix){0};
    input.link.peer_ok = peer_received && peer_ok;
    input.link.peer_status_ms = peer_received ? peer_rx_ms : now - (NAV_MAX_AGE_MS + 1u);
    input.request = peer_received ? peer_request : (AirRequest){0};
    input.request_rx_ms = peer_received ? peer_rx_ms : 0;
    return airborne_track_step(&history, now, &input, reply);
}

#ifdef ANTENNA_EXAMPLE_DEMO
#include <stdio.h>

int main(void)
{
    uint32_t now = 1000;
    AirInput local = {
        .link = {
            .enabled = true, .local_ok = true,
            .local = {31.2330, 121.4780, 155.0, now, true},
            .actual = {0, 0, now, true}
        },
        .true_heading_deg = 215,
        .heading_ms = now,
        .heading_valid = true
    };
    AirReply reply;
    air_example_init();
    air_example_receive((Fix){31.2294, 121.4748, 30.0, now, true},
                        true, (AirRequest){.session = 1, .point = 7}, now);
    Command cmd = air_example_tick(now, &local, &reply);
    if (!cmd.valid)
        return 1;

    /* On STM32, send these native target angles to the servo driver. */
    printf("Air target: az=%.2f el=%.2f mode=%d\n", cmd.az_deg, cmd.el_deg, (int)cmd.mode);
    printf("LoRa reply: session=%lu point=%lu arrived=%d\n",
           (unsigned long)reply.session, (unsigned long)reply.point, reply.arrived);
    return (reply.session == 1 && reply.point == 7) ? 0 : 1;
}
#endif
