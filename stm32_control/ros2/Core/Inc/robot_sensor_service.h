#ifndef ROBOT_SENSOR_SERVICE_H
#define ROBOT_SENSOR_SERVICE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "robot_sensor.h"


typedef struct
{
    RobotSensorRegistry *registry;
    RobotProtocolContext *protocol;
} RobotSensorService;


void RobotSensorService_Init(
    RobotSensorService *service,
    RobotSensorRegistry *registry,
    RobotProtocolContext *protocol
);

RobotStatusCode RobotSensorService_Handle(
    RobotSensorService *service,
    const RobotProtocolMessage *message
);


#ifdef __cplusplus
}
#endif

#endif
