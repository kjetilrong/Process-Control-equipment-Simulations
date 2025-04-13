#include <open62541/server.h>
#include <open62541/server_config_default.h>
#include <signal.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#ifdef _WIN32
#include <windows.h> // For Sleep
#else
#include <unistd.h> // For usleep
#endif

// ==================== SVB FUNCTION BLOCK IMPLEMENTATION ====================
typedef enum {
    VALVE_CLOSED,
    VALVE_OPENING,
    VALVE_OPEN,
    VALVE_CLOSING,
    VALVE_FAULT
} ValveState;

typedef enum {
    SOLENOID_ESD, // Emergency Shutdown
    SOLENOID_PSD, // Process Shutdown
    SOLENOID_PCS  // Process Control System
} SolenoidType;

typedef struct {
    // Configuration
    struct {
        uint8_t solenoid_count;
        bool esd_latching;
        uint32_t travel_time_ms;
    } param;

    // Internal State
    struct {
        ValveState current_state;
        ValveState target_state;
        uint32_t state_timer;
        bool esd_latched;
        bool solenoids_energized[3];
    } state;

    // I/O Terminals
    struct {
        bool solenoid_cmds[3];
        bool ls_open;
        bool ls_close;
        bool reset_cmd;
        bool solenoid_outputs[3];
        bool valve_moving;
        bool fault;
    } io;
} OnOffValve;

// Global Variables
OnOffValve valve;
volatile bool running = true;

// Valve Initialization
void Valve_Init(OnOffValve *valve) {
    memset(valve, 0, sizeof(OnOffValve));
    valve->param.solenoid_count = 3; // ESD, PSD, PCS
    valve->param.travel_time_ms = 5000; // Default: 5 seconds
    valve->state.current_state = VALVE_CLOSED;
    valve->state.target_state = VALVE_CLOSED;
}

// Convert Valve State to String
const char* Valve_StateToString(ValveState state) {
    switch (state) {
        case VALVE_CLOSED: return "CLOSED";
        case VALVE_OPENING: return "OPENING";
        case VALVE_OPEN: return "OPEN";
        case VALVE_CLOSING: return "CLOSING";
        case VALVE_FAULT: return "FAULT";
        default: return "UNKNOWN";
    }
}

// Valve State Update Logic
void Valve_Update(OnOffValve *valve, uint32_t cycle_time_ms) {
    // Check if all solenoids are energized
    bool all_solenoids_energized = true;
    for (uint32_t i = 0; i < valve->param.solenoid_count; i++) {
        if (!valve->io.solenoid_cmds[i]) {
            all_solenoids_energized = false;
            break;
        }
    }

    // Update state based on current commands and inputs
    switch (valve->state.current_state) {
        case VALVE_CLOSED:
            if (all_solenoids_energized) {
                valve->state.current_state = VALVE_OPENING;
                valve->state.state_timer = 0;
                valve->io.valve_moving = true;
            }
            break;

        case VALVE_OPENING:
            valve->state.state_timer += cycle_time_ms;
            if (valve->state.state_timer >= valve->param.travel_time_ms) {
                valve->state.current_state = VALVE_OPEN;
                valve->io.valve_moving = false;
            }
            break;

        case VALVE_OPEN:
            if (!all_solenoids_energized) {
                valve->state.current_state = VALVE_CLOSING;
                valve->state.state_timer = 0;
                valve->io.valve_moving = true;
            }
            break;

        case VALVE_CLOSING:
            valve->state.state_timer += cycle_time_ms;
            if (valve->state.state_timer >= valve->param.travel_time_ms) {
                valve->state.current_state = VALVE_CLOSED;
                valve->io.valve_moving = false;
            }
            break;

        case VALVE_FAULT:
            if (valve->io.reset_cmd) {
                valve->state.current_state = VALVE_CLOSED;
                valve->io.fault = false;
                valve->io.reset_cmd = false;
            }
            break;

        default:
            valve->state.current_state = VALVE_FAULT;
            valve->io.fault = true;
            break;
    }
}

// Value Callback for Solenoid Nodes
static void onValueChanged(UA_Server *server,
                           const UA_NodeId *sessionId, void *sessionContext,
                           const UA_NodeId *nodeId, void *nodeContext,
                           const UA_NumericRange *range,
                           const UA_DataValue *data) {
    if (data->hasValue && UA_Variant_isScalar(&data->value) &&
        data->value.type == &UA_TYPES[UA_TYPES_BOOLEAN]) {
        bool newValue = *(bool *)data->value.data;

        // Define NodeIds for comparison (local variables)
        const UA_NodeId solenoidESDNodeId = UA_NODEID_STRING(1, "SolenoidESD");
        const UA_NodeId solenoidPSDNodeId = UA_NODEID_STRING(1, "SolenoidPSD");
        const UA_NodeId solenoidPCSNodeId = UA_NODEID_STRING(1, "SolenoidPCS");

        // Determine which solenoid was updated
        if (UA_NodeId_equal(nodeId, &solenoidESDNodeId)) {
            valve.io.solenoid_cmds[SOLENOID_ESD] = newValue;
        } else if (UA_NodeId_equal(nodeId, &solenoidPSDNodeId)) {
            valve.io.solenoid_cmds[SOLENOID_PSD] = newValue;
        } else if (UA_NodeId_equal(nodeId, &solenoidPCSNodeId)) {
            valve.io.solenoid_cmds[SOLENOID_PCS] = newValue;
        }
    }
}

// Add Variable Node with Value Callback
static void addVariableNodeWithCallback(UA_Server *server, UA_NodeId parentNode,
                                        const char *nodeName, const char *displayName,
                                        void *value, const UA_DataType *type) {
    UA_VariableAttributes attr = UA_VariableAttributes_default;
    attr.displayName = UA_LOCALIZEDTEXT("en-US", displayName);
    attr.accessLevel = UA_ACCESSLEVELMASK_READ | UA_ACCESSLEVELMASK_WRITE;
    UA_Variant_setScalar(&attr.value, value, type);

    // Add the variable node
    UA_NodeId nodeId = UA_NODEID_STRING(1, nodeName);
    UA_Server_addVariableNode(server, nodeId, parentNode,
                              UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT),
                              UA_QUALIFIEDNAME(1, nodeName),
                              UA_NODEID_NUMERIC(0, UA_NS0ID_BASEDATAVARIABLETYPE),
                              attr, NULL, NULL);

    // Register the value callback
    UA_ValueCallback callback = {.onRead = NULL, .onWrite = onValueChanged};
    UA_Server_setVariableNode_valueCallback(server, nodeId, callback);
}

// Add Valve Object to OPC UA Server
static void addValveObject(UA_Server *server) {
    // Create valve object
    UA_NodeId valveNodeId = UA_NODEID_STRING(1, "SVBValve");
    UA_QualifiedName valveName = UA_QUALIFIEDNAME(1, "SVBValve");

    // Initialize ObjectAttributes for the valve object
    UA_ObjectAttributes valveObjAttr = UA_ObjectAttributes_default;
    valveObjAttr.displayName = UA_LOCALIZEDTEXT("en-US", "SVBValve");

    UA_Server_addObjectNode(server, valveNodeId, UA_NODEID_NUMERIC(0, UA_NS0ID_OBJECTSFOLDER),
                            UA_NODEID_NUMERIC(0, UA_NS0ID_ORGANIZES), valveName,
                            UA_NODEID_NUMERIC(0, UA_NS0ID_BASEOBJECTTYPE), valveObjAttr, NULL, NULL);

    // Add parameters folder
    UA_NodeId paramsNodeId = UA_NODEID_STRING(1, "Parameters");

    // Initialize ObjectAttributes for the parameters folder
    UA_ObjectAttributes paramsObjAttr = UA_ObjectAttributes_default;
    paramsObjAttr.displayName = UA_LOCALIZEDTEXT("en-US", "Parameters");

    UA_Server_addObjectNode(server, paramsNodeId, valveNodeId,
                            UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT),
                            UA_QUALIFIEDNAME(1, "Parameters"),
                            UA_NODEID_NUMERIC(0, UA_NS0ID_FOLDERTYPE), paramsObjAttr, NULL, NULL);

    // Add control folder
    UA_NodeId controlNodeId = UA_NODEID_STRING(1, "Control");

    // Initialize ObjectAttributes for the control folder
    UA_ObjectAttributes controlObjAttr = UA_ObjectAttributes_default;
    controlObjAttr.displayName = UA_LOCALIZEDTEXT("en-US", "Control");

    UA_Server_addObjectNode(server, controlNodeId, valveNodeId,
                            UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT),
                            UA_QUALIFIEDNAME(1, "Control"),
                            UA_NODEID_NUMERIC(0, UA_NS0ID_FOLDERTYPE), controlObjAttr, NULL, NULL);

    // Add status folder
    UA_NodeId statusNodeId = UA_NODEID_STRING(1, "Status");

    // Initialize ObjectAttributes for the status folder
    UA_ObjectAttributes statusObjAttr = UA_ObjectAttributes_default;
    statusObjAttr.displayName = UA_LOCALIZEDTEXT("en-US", "Status");

    UA_Server_addObjectNode(server, statusNodeId, valveNodeId,
                            UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT),
                            UA_QUALIFIEDNAME(1, "Status"),
                            UA_NODEID_NUMERIC(0, UA_NS0ID_FOLDERTYPE), statusObjAttr, NULL, NULL);

    // Add parameters
    addVariableNodeWithCallback(server, paramsNodeId, "TravelTime", "Travel Time (ms)",
                                &valve.param.travel_time_ms, &UA_TYPES[UA_TYPES_UINT32]);
    addVariableNodeWithCallback(server, paramsNodeId, "ESDLatching", "ESD Latching",
                                &valve.param.esd_latching, &UA_TYPES[UA_TYPES_BOOLEAN]);

    // Add control variables with callbacks
    addVariableNodeWithCallback(server, controlNodeId, "SolenoidESD", "Solenoid ESD",
                                &valve.io.solenoid_cmds[SOLENOID_ESD], &UA_TYPES[UA_TYPES_BOOLEAN]);
    addVariableNodeWithCallback(server, controlNodeId, "SolenoidPSD", "Solenoid PSD",
                                &valve.io.solenoid_cmds[SOLENOID_PSD], &UA_TYPES[UA_TYPES_BOOLEAN]);
    addVariableNodeWithCallback(server, controlNodeId, "SolenoidPCS", "Solenoid PCS",
                                &valve.io.solenoid_cmds[SOLENOID_PCS], &UA_TYPES[UA_TYPES_BOOLEAN]);
    addVariableNodeWithCallback(server, controlNodeId, "ResetLatch", "Reset Latch",
                                &valve.io.reset_cmd, &UA_TYPES[UA_TYPES_BOOLEAN]);

    // Add status variables
    addVariableNodeWithCallback(server, statusNodeId, "ValveState", "Valve State",
                                (void *)Valve_StateToString(valve.state.current_state), &UA_TYPES[UA_TYPES_STRING]);
    addVariableNodeWithCallback(server, statusNodeId, "LimitSwitchOpen", "Limit Switch Open",
                                &valve.io.ls_open, &UA_TYPES[UA_TYPES_BOOLEAN]);
    addVariableNodeWithCallback(server, statusNodeId, "LimitSwitchClose", "Limit Switch Close",
                                &valve.io.ls_close, &UA_TYPES[UA_TYPES_BOOLEAN]);
    addVariableNodeWithCallback(server, statusNodeId, "ValveMoving", "Valve Moving",
                                &valve.io.valve_moving, &UA_TYPES[UA_TYPES_BOOLEAN]);
    addVariableNodeWithCallback(server, statusNodeId, "Fault", "Fault Status",
                                &valve.io.fault, &UA_TYPES[UA_TYPES_BOOLEAN]);
}

// Signal Handler for Graceful Shutdown
void stopHandler(int sign) {
    running = false;
}

// Main Function
// Main Function
int main(void) {
    signal(SIGINT, stopHandler);
    signal(SIGTERM, stopHandler);

    // Initialize valve
    Valve_Init(&valve);

    // Create OPC UA server
    printf("Initializing server...\n");
    UA_Server *server = UA_Server_new();
    UA_ServerConfig *config = UA_Server_getConfig(server);

    // Set default configuration
    UA_ServerConfig_setDefault(config);

    // Bind to all interfaces (optional)
    // config->customHostname = UA_STRING("0.0.0.0");

    printf("Server initialized.\n");

    // Add SVB valve object
    addValveObject(server);

    printf("Server running at opc.tcp://0.0.0.0:4840\n");
    printf("Browse path: Objects->SVBValve\n");
    printf(" - Parameters: TravelTime, ESDLatching\n");
    printf(" - Control: SolenoidESD, SolenoidPSD, SolenoidPCS, ResetLatch\n");
    printf(" - Status: ValveState, LimitSwitchOpen, LimitSwitchClose, ValveMoving, Fault\n");

    // Start the server
    UA_StatusCode status = UA_Server_run_startup(server);
    if (status != UA_STATUSCODE_GOOD) {
        printf("Failed to start server. Status code: %s\n", UA_StatusCode_name(status));
        UA_Server_delete(server);
        return EXIT_FAILURE;
    }

    // Register the server with a Local Discovery Server (LDS) if needed
    //UA_String discoveryServerUrl = UA_STRING("opc.tcp://localhost:4840");
    //UA_StatusCode regStatus = UA_Server_register_discovery(server, NULL, discoveryServerUrl);
    //if (regStatus != UA_STATUSCODE_GOOD) {
    //    printf("Failed to register with discovery server. Status code: %s\n", UA_StatusCode_name(regStatus));
    //} else {
    //    printf("Registered with discovery server at %.*s\n", (int)discoveryServerUrl.length, discoveryServerUrl.data);
   // }

    // Run the server in a custom loop
    while (running) {
        // Process the server's main loop
        UA_Server_run_iterate(server, true);

        // Update the valve state periodically
        Valve_Update(&valve, 100);

        // Optionally, log the current state for debugging
        printf("Valve State: %s, Moving: %d, Fault: %d\n",
               Valve_StateToString(valve.state.current_state),
               valve.io.valve_moving,
               valve.io.fault);

        // Update the ValveState node in the OPC UA server
        UA_String stateString = UA_STRING_ALLOC(Valve_StateToString(valve.state.current_state));
        UA_Variant value;
        UA_Variant_init(&value);
        UA_Variant_setScalar(&value, &stateString, &UA_TYPES[UA_TYPES_STRING]);
        UA_Server_writeValue(server, UA_NODEID_STRING(1, "ValveState"), value);
        UA_String_clear(&stateString);






  // Update the ValveMoving node in the OPC UA server
    UA_Boolean moving = valve.io.valve_moving;
    UA_Variant_init(&value);
    UA_Variant_setScalar(&value, &moving, &UA_TYPES[UA_TYPES_BOOLEAN]);
    UA_Server_writeValue(server, UA_NODEID_STRING(1, "ValveMoving"), value);

    // Update the LimitSwitchOpen node in the OPC UA server
    UA_Boolean ls_open = valve.io.ls_open;
    UA_Variant_init(&value);
    UA_Variant_setScalar(&value, &ls_open, &UA_TYPES[UA_TYPES_BOOLEAN]);
    UA_Server_writeValue(server, UA_NODEID_STRING(1, "LimitSwitchOpen"), value);

      // Update the LimitSwitchClose node in the OPC UA server
    UA_Boolean ls_close = valve.io.ls_close;
    UA_Variant_init(&value);
    UA_Variant_setScalar(&value, &ls_close, &UA_TYPES[UA_TYPES_BOOLEAN]);
    UA_Server_writeValue(server, UA_NODEID_STRING(1, "LimitSwitchClose"), value);


        // Sleep for a fixed cycle time (e.g., 100 ms)
#ifdef _WIN32
        Sleep(100); // Sleep for 100 milliseconds (Windows)
#else
        usleep(100 * 1000); // Sleep for 100 milliseconds (Linux/macOS)
#endif
    }

    // Deregister the server from the discovery server (optional but recommended)
 //   UA_Server_deregisterDiscovery(server, NULL, discoveryServerUrl);

    // Shutdown the server
    UA_Server_run_shutdown(server);
    UA_Server_delete(server);

    return EXIT_SUCCESS;
}