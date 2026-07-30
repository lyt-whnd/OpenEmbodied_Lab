#include "robot_dispatcher.h"

#include <stddef.h>
#include <string.h>


static bool dispatcher_transmit(
    const uint8_t *data,
    uint16_t length,
    void *user_context
)
{
    RobotDispatcher *dispatcher =
        (RobotDispatcher *)user_context;

    if (
        dispatcher == NULL ||
        dispatcher->transmit_handler == NULL
    )
    {
        return false;
    }

    return dispatcher->transmit_handler(
        data,
        length,
        dispatcher->transport_context
    );
}


static void protocol_message_received(
    const RobotProtocolMessage *message,
    const uint8_t *raw,
    uint16_t raw_length,
    void *user_context
)
{
    RobotDispatcher *dispatcher =
        (RobotDispatcher *)user_context;

    (void)RobotDispatcher_OnMessage(
        dispatcher,
        message,
        raw,
        raw_length
    );
}


void RobotDispatcher_Init(
    RobotDispatcher *dispatcher,
    RobotMotionController *motion,
    RobotProtocolTransmitHandler transmit_handler,
    void *transport_context,
    uint32_t now_ms
)
{
    if (dispatcher == NULL)
    {
        return;
    }

    memset(dispatcher, 0, sizeof(*dispatcher));
    dispatcher->motion = motion;
    dispatcher->transmit_handler = transmit_handler;
    dispatcher->transport_context = transport_context;
    dispatcher->current_time_ms = now_ms;

    RobotProtocol_Init(
        &dispatcher->protocol,
        dispatcher_transmit,
        protocol_message_received,
        dispatcher
    );
}


void RobotDispatcher_SetTime(
    RobotDispatcher *dispatcher,
    uint32_t now_ms
)
{
    if (dispatcher == NULL)
    {
        return;
    }

    dispatcher->current_time_ms = now_ms;
}


RobotStatusCode RobotDispatcher_OnMessage(
    RobotDispatcher *dispatcher,
    const RobotProtocolMessage *message,
    const uint8_t *raw,
    uint16_t raw_length
)
{
    RobotStatusCode status;

    if (dispatcher == NULL || message == NULL)
    {
        return ROBOT_STATUS_NOT_IMPLEMENTED;
    }

    /*
     * Raw bytes are intentionally exposed for future diagnostics/forwarding.
     * Stage 1 routes only the decoded view and does not copy them.
     */
    (void)raw;
    (void)raw_length;

    if (dispatcher->motion != NULL)
    {
        RobotMotion_Process(
            dispatcher->motion,
            dispatcher->current_time_ms
        );
    }

    if (
        message->service ==
        ROBOT_SERVICE_MOTION
    )
    {
        status = RobotMotion_HandleMessage(
            dispatcher->motion,
            message
        );
    }
    else
    {
        status = ROBOT_STATUS_UNKNOWN_SERVICE;
        ++dispatcher->unsupported_service_count;
    }

    ++dispatcher->dispatched_message_count;

    if (
        (
            message->flags &
            ROBOT_FLAG_ACK_REQUIRED
        ) != 0U &&
        !RobotProtocol_SendResponse(
            &dispatcher->protocol,
            message,
            status
        )
    )
    {
        ++dispatcher->response_error_count;
    }

    return status;
}


RobotProtocolContext *RobotDispatcher_GetProtocol(
    RobotDispatcher *dispatcher
)
{
    if (dispatcher == NULL)
    {
        return NULL;
    }

    return &dispatcher->protocol;
}
