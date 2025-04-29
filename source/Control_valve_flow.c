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

// Transmitter data structure
typedef struct {
    struct {
        double control_signal;      // Control signal (0–100%)
        double upstream_pressure;   // Upstream pressure (bar)
        double kv;                  // Valve sizing coefficient (Kv)
        int valve_characteristic;   // Valve characteristic (0 = Linear, 1 = Equal Percentage)
    } config;

    struct {
        double valve_opening;       // Actual valve opening (%)
        double flow;                // Flow (m³/h)
    } state;
} FlowControlValve;

// Global variables
FlowControlValve flow_control_valve;
volatile bool running = true;
UA_Server *server;

// Function to handle graceful shutdown
void stopHandler(int sign) {
    running = false;
}

// Initialize the Flow Control Valve parameters
void FlowControlValve_Init(FlowControlValve *valve) {
    if (!valve) return;

    memset(valve, 0, sizeof(FlowControlValve));

    valve->config.control_signal = 50.0;           // Default control signal: 50%
    valve->config.upstream_pressure = 5.0;         // Default upstream pressure: 5.0 bar
    valve->config.kv = 10.0;                       // Default Kv value
    valve->config.valve_characteristic = 1;        // Default to Equal Percentage

    valve->state.valve_opening = valve->config.control_signal;  // Default opening
    valve->state.flow = 0.0;  // Initialize flow to zero
}

// Compute the flow based on given parameters
void FlowControlValve_Update(FlowControlValve *valve, uint32_t cycle_time_ms) {
    if (!valve) return;

    // Calculate valve opening (clamped between 0 and 100)
    valve->state.valve_opening = fmin(fmax(valve->config.control_signal, 0.0), 100.0);

    // Calculate Cv_eff based on valve characteristic (Linear or Equal Percentage)
    double f_opening = 0.0;
    if (valve->config.valve_characteristic == 0) {  // Linear characteristic
        f_opening = valve->state.valve_opening / 100.0;
    } else if (valve->config.valve_characteristic == 1) {  // Equal Percentage characteristic
        double R = 50.0;  // Standard R value for Equal Percentage
        f_opening = (pow(R, valve->state.valve_opening / 100.0) - 1.0) / (R - 1.0);
    }

    double Cv_eff = valve->config.kv * f_opening;

    // Calculate the pressure differential (ΔP)
    double delta_p = valve->config.upstream_pressure - 1.0;  // Assume downstream pressure = 1.0 bar

    // Calculate flow (m³/h)
    valve->state.flow = Cv_eff * sqrt(delta_p);
}

// Node change callback for configurable parameters
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
        UA_QualifiedName_clear(&browseName);
        return;
    }

    UA_String controlSignalStr = UA_STRING("ControlSignal");
    UA_String upstreamPressureStr = UA_STRING("UpstreamPressure");
    UA_String kvStr = UA_STRING("Kv");
    UA_String valveCharacteristicStr = UA_STRING("ValveCharacteristic");

    if (UA_String_equal(&browseName.name, &controlSignalStr)) {
        if (data->value.type == &UA_TYPES[UA_TYPES_DOUBLE]) {
            flow_control_valve.config.control_signal = *(UA_Double*)data->value.data;
        }
    } else if (UA_String_equal(&browseName.name, &upstreamPressureStr)) {
        if (data->value.type == &UA_TYPES[UA_TYPES_DOUBLE]) {
            flow_control_valve.config.upstream_pressure = *(UA_Double*)data->value.data;
        }
    } else if (UA_String_equal(&browseName.name, &kvStr)) {
        if (data->value.type == &UA_TYPES[UA_TYPES_DOUBLE]) {
            flow_control_valve.config.kv = *(UA_Double*)data->value.data;
        }
    } else if (UA_String_equal(&browseName.name, &valveCharacteristicStr)) {
        if (data->value.type == &UA_TYPES[UA_TYPES_INT32]) {
            flow_control_valve.config.valve_characteristic = *(UA_Int32*)data->value.data;
        }
    }

    UA_QualifiedName_clear(&browseName);
}

// Add a variable with a callback for changes
static void addVariableWithCallback(UA_Server *server, UA_NodeId parentNode,
                                     const char *nodeIdStr, const char *displayName,
                                     void *value, const UA_DataType *type) {
    UA_VariableAttributes attr = UA_VariableAttributes_default;
    attr.displayName = UA_LOCALIZEDTEXT("en-US", displayName);
    attr.accessLevel = UA_ACCESSLEVELMASK_READ | UA_ACCESSLEVELMASK_WRITE;
    UA_Variant_setScalar(&attr.value, value, type);

    UA_NodeId nodeId = UA_NODEID_STRING(1, nodeIdStr);
    UA_QualifiedName browseName = UA_QUALIFIEDNAME(1, nodeIdStr);

    UA_Server_addVariableNode(server, nodeId, parentNode,
                              UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT),
                              browseName, UA_NODEID_NUMERIC(0, UA_NS0ID_BASEDATAVARIABLETYPE),
                              attr, NULL, NULL);

    UA_ValueCallback callback = {.onRead = NULL, .onWrite = onConfigChanged};
    UA_Server_setVariableNode_valueCallback(server, nodeId, callback);
}

// Add the transmitter object and configure its nodes
static void addFlowControlValveObject(UA_Server *server) {
    UA_ObjectAttributes objAttr = UA_ObjectAttributes_default;
    objAttr.displayName = UA_LOCALIZEDTEXT("en-US", "FlowControlValve");

    UA_NodeId valveId = UA_NODEID_STRING(1, "FlowControlValve");
    UA_QualifiedName valveName = UA_QUALIFIEDNAME(1, "FlowControlValve");

    UA_Server_addObjectNode(server, valveId,
                            UA_NODEID_NUMERIC(0, UA_NS0ID_OBJECTSFOLDER),
                            UA_NODEID_NUMERIC(0, UA_NS0ID_ORGANIZES),
                            valveName,
                            UA_NODEID_NUMERIC(0, UA_NS0ID_BASEOBJECTTYPE),
                            objAttr, NULL, NULL);

    // Configuration folder
    UA_NodeId configFolderId = UA_NODEID_STRING(1, "Configuration");
    UA_ObjectAttributes configFolderAttr = UA_ObjectAttributes_default;
    configFolderAttr.displayName = UA_LOCALIZEDTEXT("en-US", "Configuration");

    UA_Server_addObjectNode(server, configFolderId, valveId,
                            UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT),
                            UA_QUALIFIEDNAME(1, "Configuration"),
                            UA_NODEID_NUMERIC(0, UA_NS0ID_FOLDERTYPE),
                            configFolderAttr, NULL, NULL);

    // Add configurable parameters
    addVariableWithCallback(server, configFolderId, "ControlSignal", "Control Signal",
                            &flow_control_valve.config.control_signal, &UA_TYPES[UA_TYPES_DOUBLE]);
    addVariableWithCallback(server, configFolderId, "UpstreamPressure", "Upstream Pressure",
                            &flow_control_valve.config.upstream_pressure, &UA_TYPES[UA_TYPES_DOUBLE]);
    addVariableWithCallback(server, configFolderId, "Kv", "Kv",
                            &flow_control_valve.config.kv, &UA_TYPES[UA_TYPES_DOUBLE]);
    addVariableWithCallback(server, configFolderId, "ValveCharacteristic", "Valve Characteristic",
                            &flow_control_valve.config.valve_characteristic, &UA_TYPES[UA_TYPES_INT32]);

    // Status folder
    UA_NodeId statusFolderId = UA_NODEID_STRING(1, "Status");
    UA_ObjectAttributes statusFolderAttr = UA_ObjectAttributes_default;
    statusFolderAttr.displayName = UA_LOCALIZEDTEXT("en-US", "Status");

    UA_Server_addObjectNode(server, statusFolderId, valveId,
                            UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT),
                            UA_QUALIFIEDNAME(1, "Status"),
                            UA_NODEID_NUMERIC(0, UA_NS0ID_FOLDERTYPE),
                            statusFolderAttr, NULL, NULL);

    // Add Valve Opening and Flow variables
    UA_VariableAttributes statusAttr = UA_VariableAttributes_default;
    statusAttr.displayName = UA_LOCALIZEDTEXT("en-US", "ValveOpening");
    statusAttr.accessLevel = UA_ACCESSLEVELMASK_READ;
    UA_Variant_setScalar(&statusAttr.value, &flow_control_valve.state.valve_opening, &UA_TYPES[UA_TYPES_DOUBLE]);

    UA_Server_addVariableNode(server, UA_NODEID_STRING(1, "ValveOpening"), statusFolderId,
                              UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT),
                              UA_QUALIFIEDNAME(1, "ValveOpening"),
                              UA_NODEID_NUMERIC(0, UA_NS0ID_BASEDATAVARIABLETYPE),
                              statusAttr, NULL, NULL);

    statusAttr.displayName = UA_LOCALIZEDTEXT("en-US", "Flow");
    UA_Variant_setScalar(&statusAttr.value, &flow_control_valve.state.flow, &UA_TYPES[UA_TYPES_DOUBLE]);

    UA_Server_addVariableNode(server, UA_NODEID_STRING(1, "Flow"), statusFolderId,
                              UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT),
                              UA_QUALIFIEDNAME(1, "Flow"),
                              UA_NODEID_NUMERIC(0, UA_NS0ID_BASEDATAVARIABLETYPE),
                              statusAttr, NULL, NULL);
}

int main(void) {
    signal(SIGINT, stopHandler);
    signal(SIGTERM, stopHandler);

    FlowControlValve_Init(&flow_control_valve);

    server = UA_Server_new();
    UA_ServerConfig_setDefault(UA_Server_getConfig(server));

    addFlowControlValveObject(server);

    printf("OPC UA Flow Control Valve Server running at opc.tcp://localhost:4840\n");

    UA_StatusCode status = UA_Server_run_startup(server);
    if (status != UA_STATUSCODE_GOOD) {
        UA_Server_delete(server);
        return EXIT_FAILURE;
    }

    while (running) {
        UA_Server_run_iterate(server, true);
        FlowControlValve_Update(&flow_control_valve, DEFAULT_CYCLE_TIME_MS);

        UA_Variant value;
        UA_Variant_init(&value);

        // Update Valve Opening
        UA_Variant_setScalar(&value, &flow_control_valve.state.valve_opening, &UA_TYPES[UA_TYPES_DOUBLE]);
        UA_Server_writeValue(server, UA_NODEID_STRING(1, "ValveOpening"), value);

        // Update Flow
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
