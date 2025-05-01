#include <open62541/server.h>
#include <open62541/server_config_default.h>
#include <open62541/plugin/log_stdout.h>
#include <signal.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <time.h>
#include <string.h>

#define PI 3.14159265
#define DEFAULT_CYCLE_TIME_MS 100

// Flow control valve structure
typedef struct {
    struct {
        double control_signal;
        double upstream_pressure;
        double kv;
        int valve_characteristic;
    } config;

    struct {
        double valve_opening;
        double flow;
    } state;

    struct {
        double stiction_threshold;
        double dead_time_seconds;
        double hysteresis_percent;
        double positioner_error_percent;
        double last_control_signal;
        double last_update_time;
    } error;
} FlowControlValve;

// Globals
FlowControlValve flow_control_valve;
volatile bool running = true;
UA_Server *server;

void stopHandler(int sign) {
    running = false;
}

void FlowControlValve_Init(FlowControlValve *valve) {
    if (!valve) return;

    memset(valve, 0, sizeof(FlowControlValve));
    valve->config.control_signal = 50.0;
    valve->config.upstream_pressure = 5.0;
    valve->config.kv = 10.0;
    valve->config.valve_characteristic = 1;

    valve->state.valve_opening = valve->config.control_signal;
    valve->state.flow = 0.0;

    valve->error.stiction_threshold = 0.5;
    valve->error.dead_time_seconds = 0.0;
    valve->error.hysteresis_percent = 0.0;
    valve->error.positioner_error_percent = 0.0;
    valve->error.last_control_signal = valve->config.control_signal;
    valve->error.last_update_time = 0.0;
}

void FlowControlValve_Update(FlowControlValve *valve, uint32_t cycle_time_ms) {
    if (!valve) return;

    double now = (double)clock() / CLOCKS_PER_SEC;
    double control_signal = fmin(fmax(valve->config.control_signal, 0.0), 100.0);

    if (now - valve->error.last_update_time < valve->error.dead_time_seconds)
        return;

    valve->error.last_update_time = now;

    if (fabs(control_signal - valve->error.last_control_signal) < valve->error.stiction_threshold)
        control_signal = valve->error.last_control_signal;

    double hysteresis = 0.0;
    if (control_signal > valve->error.last_control_signal)
        hysteresis = valve->error.hysteresis_percent;
    else if (control_signal < valve->error.last_control_signal)
        hysteresis = -valve->error.hysteresis_percent;

    valve->error.last_control_signal = control_signal;
    control_signal += hysteresis;
    control_signal = fmin(fmax(control_signal, 0.0), 100.0);

    valve->state.valve_opening = control_signal * (1.0 + valve->error.positioner_error_percent / 100.0);
    valve->state.valve_opening = fmin(fmax(valve->state.valve_opening, 0.0), 100.0);

    double f_opening = 0.0;
    if (valve->config.valve_characteristic == 0)
        f_opening = valve->state.valve_opening / 100.0;
    else {
        double R = 50.0;
        f_opening = (pow(R, valve->state.valve_opening / 100.0) - 1.0) / (R - 1.0);
    }

    double Cv_eff = valve->config.kv * f_opening;
    double delta_p = valve->config.upstream_pressure - 1.0;
    valve->state.flow = Cv_eff * sqrt(delta_p);
}

static void assignIfMatch(UA_QualifiedName *browseName, const char *name,
                         const UA_DataValue *data, const UA_DataType *type,
                         void *target) {
    UA_String compareName = UA_STRING(name);
    if (UA_String_equal(&browseName->name, &compareName) &&
        data->value.type == type &&
        data->hasValue && UA_Variant_isScalar(&data->value)) {
        memcpy(target, data->value.data, type->memSize);
    }
}

static void onConfigChanged(UA_Server *server,
                            const UA_NodeId *sessionId, void *sessionContext,
                            const UA_NodeId *nodeId, void *nodeContext,
                            const UA_NumericRange *range,
                            const UA_DataValue *data) {
    if (!server || !nodeId || !data || !data->hasValue || !UA_Variant_isScalar(&data->value)) {
        return;
    }

    UA_QualifiedName browseName;
    UA_StatusCode retval = UA_Server_readBrowseName(server, *nodeId, &browseName);
    if (retval != UA_STATUSCODE_GOOD) {
        return;
    }

    assignIfMatch(&browseName, "ControlSignal", data, &UA_TYPES[UA_TYPES_DOUBLE], &flow_control_valve.config.control_signal);
    assignIfMatch(&browseName, "UpstreamPressure", data, &UA_TYPES[UA_TYPES_DOUBLE], &flow_control_valve.config.upstream_pressure);
    assignIfMatch(&browseName, "Kv", data, &UA_TYPES[UA_TYPES_DOUBLE], &flow_control_valve.config.kv);
    assignIfMatch(&browseName, "ValveCharacteristic", data, &UA_TYPES[UA_TYPES_INT32], &flow_control_valve.config.valve_characteristic);

    assignIfMatch(&browseName, "StictionThreshold", data, &UA_TYPES[UA_TYPES_DOUBLE], &flow_control_valve.error.stiction_threshold);
    assignIfMatch(&browseName, "DeadTime", data, &UA_TYPES[UA_TYPES_DOUBLE], &flow_control_valve.error.dead_time_seconds);
    assignIfMatch(&browseName, "Hysteresis", data, &UA_TYPES[UA_TYPES_DOUBLE], &flow_control_valve.error.hysteresis_percent);
    assignIfMatch(&browseName, "PositionerError", data, &UA_TYPES[UA_TYPES_DOUBLE], &flow_control_valve.error.positioner_error_percent);

    UA_QualifiedName_clear(&browseName);
}

static void addVariableWithCallback(UA_Server *server, UA_NodeId parentNode,
                                     const char *nodeIdStr, const char *displayName,
                                     void *value, const UA_DataType *type) {
    UA_VariableAttributes attr = UA_VariableAttributes_default;
    attr.displayName = UA_LOCALIZEDTEXT("en-US", displayName);
    attr.accessLevel = UA_ACCESSLEVELMASK_READ | UA_ACCESSLEVELMASK_WRITE;
    UA_Variant_setScalar(&attr.value, value, type);

    UA_Server_addVariableNode(server, UA_NODEID_STRING(1, nodeIdStr), parentNode,
                              UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT),
                              UA_QUALIFIEDNAME(1, nodeIdStr),
                              UA_NODEID_NUMERIC(0, UA_NS0ID_BASEDATAVARIABLETYPE),
                              attr, NULL, NULL);

    UA_ValueCallback callback = {.onRead = NULL, .onWrite = onConfigChanged};
    UA_Server_setVariableNode_valueCallback(server, UA_NODEID_STRING(1, nodeIdStr), callback);
}

static void addFlowControlValveObject(UA_Server *server) {
    UA_Server_addObjectNode(server, UA_NODEID_STRING(1, "FlowControlValve"),
        UA_NODEID_NUMERIC(0, UA_NS0ID_OBJECTSFOLDER),
        UA_NODEID_NUMERIC(0, UA_NS0ID_ORGANIZES),
        UA_QUALIFIEDNAME(1, "FlowControlValve"),
        UA_NODEID_NUMERIC(0, UA_NS0ID_BASEOBJECTTYPE),
        UA_ObjectAttributes_default, NULL, NULL);

    UA_Server_addObjectNode(server, UA_NODEID_STRING(1, "Configuration"),
        UA_NODEID_STRING(1, "FlowControlValve"),
        UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT),
        UA_QUALIFIEDNAME(1, "Configuration"),
        UA_NODEID_NUMERIC(0, UA_NS0ID_FOLDERTYPE),
        UA_ObjectAttributes_default, NULL, NULL);

    addVariableWithCallback(server, UA_NODEID_STRING(1, "Configuration"), "ControlSignal", "Control Signal", &flow_control_valve.config.control_signal, &UA_TYPES[UA_TYPES_DOUBLE]);
    addVariableWithCallback(server, UA_NODEID_STRING(1, "Configuration"), "UpstreamPressure", "Upstream Pressure", &flow_control_valve.config.upstream_pressure, &UA_TYPES[UA_TYPES_DOUBLE]);
    addVariableWithCallback(server, UA_NODEID_STRING(1, "Configuration"), "Kv", "Kv", &flow_control_valve.config.kv, &UA_TYPES[UA_TYPES_DOUBLE]);
    addVariableWithCallback(server, UA_NODEID_STRING(1, "Configuration"), "ValveCharacteristic", "Valve Characteristic", &flow_control_valve.config.valve_characteristic, &UA_TYPES[UA_TYPES_INT32]);

    UA_Server_addObjectNode(server, UA_NODEID_STRING(1, "Errors"),
        UA_NODEID_STRING(1, "FlowControlValve"),
        UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT),
        UA_QUALIFIEDNAME(1, "Errors"),
        UA_NODEID_NUMERIC(0, UA_NS0ID_FOLDERTYPE),
        UA_ObjectAttributes_default, NULL, NULL);

    addVariableWithCallback(server, UA_NODEID_STRING(1, "Errors"), "StictionThreshold", "Stiction Threshold", &flow_control_valve.error.stiction_threshold, &UA_TYPES[UA_TYPES_DOUBLE]);
    addVariableWithCallback(server, UA_NODEID_STRING(1, "Errors"), "DeadTime", "Dead Time (s)", &flow_control_valve.error.dead_time_seconds, &UA_TYPES[UA_TYPES_DOUBLE]);
    addVariableWithCallback(server, UA_NODEID_STRING(1, "Errors"), "Hysteresis", "Hysteresis (%)", &flow_control_valve.error.hysteresis_percent, &UA_TYPES[UA_TYPES_DOUBLE]);
    addVariableWithCallback(server, UA_NODEID_STRING(1, "Errors"), "PositionerError", "Positioner Error (%)", &flow_control_valve.error.positioner_error_percent, &UA_TYPES[UA_TYPES_DOUBLE]);

    UA_Server_addObjectNode(server, UA_NODEID_STRING(1, "Status"),
        UA_NODEID_STRING(1, "FlowControlValve"),
        UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT),
        UA_QUALIFIEDNAME(1, "Status"),
        UA_NODEID_NUMERIC(0, UA_NS0ID_FOLDERTYPE),
        UA_ObjectAttributes_default, NULL, NULL);

    UA_VariableAttributes statusAttr = UA_VariableAttributes_default;
    statusAttr.displayName = UA_LOCALIZEDTEXT("en-US", "ValveOpening");
    statusAttr.accessLevel = UA_ACCESSLEVELMASK_READ;
    UA_Variant_setScalar(&statusAttr.value, &flow_control_valve.state.valve_opening, &UA_TYPES[UA_TYPES_DOUBLE]);
    UA_Server_addVariableNode(server, UA_NODEID_STRING(1, "ValveOpening"), UA_NODEID_STRING(1, "Status"), UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT), UA_QUALIFIEDNAME(1, "ValveOpening"), UA_NODEID_NUMERIC(0, UA_NS0ID_BASEDATAVARIABLETYPE), statusAttr, NULL, NULL);

    statusAttr.displayName = UA_LOCALIZEDTEXT("en-US", "Flow");
    UA_Variant_setScalar(&statusAttr.value, &flow_control_valve.state.flow, &UA_TYPES[UA_TYPES_DOUBLE]);
    UA_Server_addVariableNode(server, UA_NODEID_STRING(1, "Flow"), UA_NODEID_STRING(1, "Status"), UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT), UA_QUALIFIEDNAME(1, "Flow"), UA_NODEID_NUMERIC(0, UA_NS0ID_BASEDATAVARIABLETYPE), statusAttr, NULL, NULL);
}

int main(void) {
    signal(SIGINT, stopHandler);
    signal(SIGTERM, stopHandler);

    FlowControlValve_Init(&flow_control_valve);
    server = UA_Server_new();
    UA_ServerConfig_setDefault(UA_Server_getConfig(server));

    addFlowControlValveObject(server);
    printf("OPC UA Flow Control Valve Server running at opc.tcp://localhost:4840\n");

    if (UA_Server_run_startup(server) != UA_STATUSCODE_GOOD) {
        UA_Server_delete(server);
        return EXIT_FAILURE;
    }

    while (running) {
        UA_Server_run_iterate(server, true);
        FlowControlValve_Update(&flow_control_valve, DEFAULT_CYCLE_TIME_MS);

        UA_Variant value;
        UA_Variant_init(&value);
        UA_Variant_setScalar(&value, &flow_control_valve.state.valve_opening, &UA_TYPES[UA_TYPES_DOUBLE]);
        UA_Server_writeValue(server, UA_NODEID_STRING(1, "ValveOpening"), value);

        UA_Variant_setScalar(&value, &flow_control_valve.state.flow, &UA_TYPES[UA_TYPES_DOUBLE]);
        UA_Server_writeValue(server, UA_NODEID_STRING(1, "Flow"), value);

#ifdef _WIN32
        Sleep(DEFAULT_CYCLE_TIME_MS);
#else
        usleep(DEFAULT_CYCLE_TIME_MS * 1000);
#endif
    }

    UA_Server_run_shutdown(server);
    UA_Server_delete(server);
    return EXIT_SUCCESS;
}
