#include "robot_dispatcher.h"

#include <stddef.h>
#include <string.h>

#include "robot_reliable.h"
#include "robot_service_registry.h"


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

static RobotStatusCode dispatch_application(
    RobotDispatcher *dispatcher,
    const RobotProtocolMessage *message
)
{
    if (message->service == ROBOT_SERVICE_MOTION)
    {
        return RobotMotion_HandleMessage(
            dispatcher->motion,
            message
        );
    }
    if (message->service == ROBOT_SERVICE_SENSOR)
    {
        return RobotSensorService_Handle(
            &dispatcher->sensor_service,
            message
        );
    }

    ++dispatcher->unsupported_service_count;
    return ROBOT_STATUS_UNKNOWN_SERVICE;
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
    RobotResultCache_Init(&dispatcher->result_cache);

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


void RobotDispatcher_SetSensorRegistry(
    RobotDispatcher *dispatcher,
    RobotSensorRegistry *registry
)
{
    if (dispatcher == NULL)
    {
        return;
    }
    RobotSensorService_Init(
        &dispatcher->sensor_service,
        registry,
        &dispatcher->protocol
    );
}


RobotStatusCode RobotDispatcher_OnMessage(
    RobotDispatcher *dispatcher,
    const RobotProtocolMessage *message,
    const uint8_t *raw,
    uint16_t raw_length
)
{
    RobotStatusCode status;
    const RobotMessagePolicy *policy;

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

    policy = RobotServiceRegistry_Lookup(
        message->service,
        message->opcode
    );

    if (policy == NULL)
    {
        status = (
            RobotServiceRegistry_HasService(message->service)
            ? ROBOT_STATUS_UNKNOWN_OPCODE
            : ROBOT_STATUS_UNKNOWN_SERVICE
        );
        ++dispatcher->unsupported_service_count;
    }
    else if (
        policy->qos == ROBOT_QOS_RELIABLE &&
        (
            message->flags &
            ROBOT_FLAG_ACK_REQUIRED
        ) != 0U &&
        (
            message->flags &
            ROBOT_FLAG_RESPONSE
        ) == 0U
    )
    {
        RobotReliableRequest request;

        if (RobotReliable_DecodeRequest(message, &request))
        {
            RobotResultCacheEntry cached;
            RobotResultCacheLookup lookup =
                RobotResultCache_Find(
                    &dispatcher->result_cache,
                    message->src,
                    message->service,
                    message->opcode,
                    request.epoch,
                    request.request_id,
                    dispatcher->current_time_ms,
                    &cached
                );

            if (lookup == ROBOT_RESULT_CACHE_FOUND)
            {
                ++dispatcher->duplicate_request_count;

                if (
                    !RobotReliable_SendResult(
                        &dispatcher->protocol,
                        message,
                        &request,
                        cached.stage,
                        (RobotStatusCode)cached.status
                    )
                )
                {
                    ++dispatcher->response_error_count;
                }

                return (RobotStatusCode)cached.status;
            }

            if (lookup == ROBOT_RESULT_CACHE_CONFLICT)
            {
                ++dispatcher->invalid_reliable_count;

                if (
                    !RobotReliable_SendResult(
                        &dispatcher->protocol,
                        message,
                        &request,
                        ROBOT_RESULT_STAGE_FAILED,
                        ROBOT_STATUS_REQUEST_ID_CONFLICT
                    )
                )
                {
                    ++dispatcher->response_error_count;
                }

                return ROBOT_STATUS_REQUEST_ID_CONFLICT;
            }

            if (
                !RobotReliable_SendResult(
                    &dispatcher->protocol,
                    message,
                    &request,
                    ROBOT_RESULT_STAGE_RECEIVED,
                    ROBOT_STATUS_OK
                )
            )
            {
                ++dispatcher->response_error_count;
            }

            RobotProtocolMessage application = *message;
            application.payload = request.payload;
            application.payload_length =
                request.payload_length;
            status = dispatch_application(
                dispatcher,
                &application
            );

            RobotResultStage stage = (
                status == ROBOT_STATUS_OK
                ? ROBOT_RESULT_STAGE_APPLIED
                : ROBOT_RESULT_STAGE_FAILED
            );

            RobotResultCache_Store(
                &dispatcher->result_cache,
                message->src,
                message->service,
                message->opcode,
                request.epoch,
                request.request_id,
                stage,
                (uint16_t)status,
                dispatcher->current_time_ms
            );

            if (
                !RobotReliable_SendResult(
                    &dispatcher->protocol,
                    message,
                    &request,
                    stage,
                    status
                )
            )
            {
                ++dispatcher->response_error_count;
            }

            ++dispatcher->dispatched_message_count;
            return status;
        }

        /*
         * Empty-payload reliable commands are the frozen Stage-1 wire
         * representation. Keep accepting them until capability negotiation
         * can require reliable schema version 1.
         */
        if (message->payload_length != 0U)
        {
            ++dispatcher->invalid_reliable_count;
            status = ROBOT_STATUS_BAD_LENGTH;
        }
        else
        {
            status = dispatch_application(
                dispatcher,
                message
            );
        }
    }
    else
    {
        status = dispatch_application(
            dispatcher,
            message
        );
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
