#ifndef ANTENNA_EXAMPLES_H
#define ANTENNA_EXAMPLES_H

#include "../airborne/airborne.h"
#include "../vehicle/vehicle.h"

void air_example_init(void);
void air_example_receive(Fix fix, bool healthy, AirRequest request, uint32_t received_ms);
Command air_example_tick(uint32_t now, const AirInput *local, AirReply *reply);

void vehicle_example_init(uint32_t session);
void vehicle_example_receive(Fix fix, bool healthy, AirReply reply, uint32_t received_ms);
Command vehicle_example_tick(uint32_t now, const VehicleInput *local, AirRequest *request);

#endif
