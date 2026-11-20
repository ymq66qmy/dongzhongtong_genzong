#include "antenna_examples.h"

static Vehicle scanner;
static uint32_t vehicle_session;
static Fix peer_fix;
static AirReply peer_reply;
static uint32_t peer_rx_ms;
static bool peer_ok;
static bool peer_received;

/* Supply a new nonzero session at boot. Do not reinitialize during tracking. */
void vehicle_example_init(uint32_t session)
{
    vehicle_init(&scanner);
    vehicle_session = session;
    peer_received = false;
}

/* Accept each new, decoded LoRa packet once, using the vehicle clock.
 * Serialize RX updates and control ticks or copy through a mailbox. */
void vehicle_example_receive(Fix fix, bool healthy, AirReply reply, uint32_t received_ms)
{
    peer_fix = fix;
    peer_ok = healthy;
    peer_reply = reply;
    peer_rx_ms = received_ms;
    peer_received = true;
}

/* Call every 10 ms. local supplies local GPS, encoder, RSSI and flags.
 * Retain request for the next 1 s LoRa send, together with local position/status. */
Command vehicle_example_tick(uint32_t now, const VehicleInput *local, AirRequest *request)
{
    VehicleInput input = *local;
    input.session = vehicle_session;
    input.link.peer = peer_received ? peer_fix : (Fix){0};
    input.link.peer_ok = peer_received && peer_ok;
    input.link.peer_status_ms = peer_received ? peer_rx_ms : now - (NAV_MAX_AGE_MS + 1u);
    input.air = peer_received ? peer_reply : (AirReply){0};
    input.air_rx_ms = peer_received ? peer_rx_ms : 0;
    Command cmd = vehicle_step(&scanner, now, &input);
    *request = scanner.request;
    return cmd;
}

#ifdef ANTENNA_EXAMPLE_DEMO
#include <stdio.h>

int main(void)
{
    uint32_t now = 1000;
    VehicleInput local = {
        .link = {
            .enabled = true, .local_ok = true,
            .local = {31.2294, 121.4748, 30.0, now, true},
            .actual = {0, 0, now, true}
        },
        .rssi = -105,
        .rssi_valid = true,
        .sample_ms = now,
        .has_beacon_flag = true,
        .beacon_present = true,
        .heading_valid = false
    };
    AirRequest request;
    vehicle_example_init(1);
    /* The first aircraft packet carries a position but has no reply yet. */
    vehicle_example_receive((Fix){31.2330, 121.4780, 155.0, now, true},
                            true, (AirReply){0}, now);
    Command cmd = vehicle_example_tick(now, &local, &request);
    if (!cmd.valid)
        return 1;

    /* On STM32, send these native target angles to the servo driver. */
    printf("Vehicle target: az=%.2f el=%.2f mode=%d\n", cmd.az_deg, cmd.el_deg, (int)cmd.mode);
    printf("LoRa request: session=%lu point=%lu az_trim=%.2f el_trim=%.2f\n",
           (unsigned long)request.session, (unsigned long)request.point,
           request.az_trim, request.el_trim);
    return request.session == 1 ? 0 : 1;
}
#endif
