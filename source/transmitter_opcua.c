#include <open62541/server.h>
#include <open62541/server_config_default.h>
#include <signal.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <time.h>

#define PI 3.14159265
#define DEFAULT_CYCLE_TIME_MS 100
//Compile : gcc transmitter_opcua.c -o tans10 -lopen62541 -lmbedtls -lmbedx509 -lmbedcrypto -lpthread -lm

// Transmitter data structure
typedef struct {
    // Configuration
    struct {
        double min_range;
        double max_range;
        double min_scale;
        double max_scale;
        double step_size;
        bool simulation_active;
        bool sine_wave;
        bool sawtooth_wave;
        bool overflow;
        bool underflow;
    } config;

    // Runtime data
    struct {
        double current_value;
        double simulation_time;
        bool fault;
    } state;
} Transmitter;

// Global variables
Transmitter transmitter;
volatile bool running = true;
UA_Server *server;

// Signal handler for graceful shutdown
void stopHandler(int sign) {
    running = false;
}

// Initialize transmitter with default values
void Transmitter_Init(Transmitter *tx) {
    memset(tx, 0, sizeof(Transmitter));
    
    tx->config.min_range = 0.0;
    tx->config.max_range = 100.0;
    tx->config.min_scale = -5.0;
    tx->config.max_scale = 105.0;
    tx->config.step_size = 1.0;
    tx->config.simulation_active = false;
    tx->config.sine_wave = false;
    tx->config.sawtooth_wave = true; // Default to sawtooth
    tx->config.overflow = false;
    tx->config.underflow = false;
    
    tx->state.current_value = 0.0;
    tx->state.simulation_time = 0.0;
    tx->state.fault = false;
}

// Update transmitter simulation
void Transmitter_Update(Transmitter *tx, uint32_t cycle_time_ms) {
    if (!tx->config.simulation_active) return;

    double time_step = (double)cycle_time_ms / 1000.0;
    tx->state.simulation_time += time_step;

    if (tx->config.overflow) {
        tx->state.current_value = tx->config.max_scale;
        return;
    }

    if (tx->config.underflow) {
        tx->state.current_value = tx->config.min_scale;
        return;
    }

    if (tx->config.sine_wave) {
        // Sine wave between min_range and max_range
        tx->state.current_value = tx->config.min_range + 
            ((tx->config.max_range - tx->config.min_range) / 2.0) * 
            (1.0 + sin(2 * PI * 0.1 * tx->state.simulation_time));
    } 
    else if (tx->config.sawtooth_wave) {
        // Sawtooth wave
        double period = 10.0; // 10 second period
        double phase = fmod(tx->state.simulation_time, period) / period;
        tx->state.current_value = tx->config.min_range + 
            (tx->config.max_range - tx->config.min_range) * phase;
    }
    else {
        // Linear increase/decrease with step size
        static bool increasing = true;
        if (increasing) {
            tx->state.current_value += tx->config.step_size;
            if (tx->state.current_value >= tx->config.max_range) {
                increasing = false;
                tx->state.current_value = tx->config.max_range;
            }
        } else {
            tx->state.current_value -= tx->config.step_size;
            if (tx->state.current_value <= tx->config.min_range) {
                increasing = true;
                tx->state.current_value = tx->config.min_range;
            }
        }
    }

    // Check for fault conditions
    tx->state.fault = (tx->state.current_value < tx->config.min_scale || 
                       tx->state.current_value > tx->config.max_scale);
}

// Callback for when configuration values are changed
static void onConfigChanged(UA_Server *server,
                           const UA_NodeId *sessionId, void *sessionContext,
                           const UA_NodeId *nodeId, void *nodeContext,
                           const UA_NumericRange *range,
                           const UA_DataValue *data) {
    if (!data->hasValue || !UA_Variant_isScalar(&data->value)) return;

    // Get the node name for comparison
    UA_String nodeName = UA_STRING_NULL;
    UA_Server_readBrowseName(server, *nodeId, &nodeName);

    // Create comparison strings
    UA_String stepSizeStr = UA_STRING("StepSize");
    UA_String simActiveStr = UA_STRING("SimulationActive");
    UA_String sineWaveStr = UA_STRING("SineWave");
    UA_String sawtoothStr = UA_STRING("SawtoothWave");
    UA_String overflowStr = UA_STRING("Overflow");
    UA_String underflowStr = UA_STRING("Underflow");

    if (UA_String_equal(&nodeName, &stepSizeStr)) {
        if (data->value.type == &UA_TYPES[UA_TYPES_DOUBLE]) {
            double newStep = *(double*)data->value.data;
            if (newStep >= 0.1 && newStep <= 10.0) {
                transmitter.config.step_size = newStep;
            }
        }
    }
    else if (UA_String_equal(&nodeName, &simActiveStr)) {
        if (data->value.type == &UA_TYPES[UA_TYPES_BOOLEAN]) {
            transmitter.config.simulation_active = *(bool*)data->value.data;
        }
    }
    else if (UA_String_equal(&nodeName, &sineWaveStr)) {
        if (data->value.type == &UA_TYPES[UA_TYPES_BOOLEAN]) {
            transmitter.config.sine_wave = *(bool*)data->value.data;
            if (transmitter.config.sine_wave) {
                transmitter.config.sawtooth_wave = false;
            }
        }
    }
    else if (UA_String_equal(&nodeName, &sawtoothStr)) {
        if (data->value.type == &UA_TYPES[UA_TYPES_BOOLEAN]) {
            transmitter.config.sawtooth_wave = *(bool*)data->value.data;
            if (transmitter.config.sawtooth_wave) {
                transmitter.config.sine_wave = false;
            }
        }
    }
    else if (UA_String_equal(&nodeName, &overflowStr)) {
        if (data->value.type == &UA_TYPES[UA_TYPES_BOOLEAN]) {
            transmitter.config.overflow = *(bool*)data->value.data;
            if (transmitter.config.overflow) {
                transmitter.config.underflow = false;
            }
        }
    }
    else if (UA_String_equal(&nodeName, &underflowStr)) {
        if (data->value.type == &UA_TYPES[UA_TYPES_BOOLEAN]) {
            transmitter.config.underflow = *(bool*)data->value.data;
            if (transmitter.config.underflow) {
                transmitter.config.overflow = false;
            }
        }
    }

    UA_String_clear(&nodeName);
}

// Add a variable node with callback
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

    // Set value callback
    UA_ValueCallback callback = {.onRead = NULL, .onWrite = onConfigChanged};
    UA_Server_setVariableNode_valueCallback(server, nodeId, callback);
}

// Add the transmitter object to the server
static void addTransmitterObject(UA_Server *server) {
    // Create transmitter object
    UA_ObjectAttributes objAttr = UA_ObjectAttributes_default;
    objAttr.displayName = UA_LOCALIZEDTEXT("en-US", "Transmitter");
    
    UA_NodeId transmitterId = UA_NODEID_STRING(1, "Transmitter");
    UA_QualifiedName transmitterName = UA_QUALIFIEDNAME(1, "Transmitter");
    
    UA_Server_addObjectNode(server, transmitterId,
                           UA_NODEID_NUMERIC(0, UA_NS0ID_OBJECTSFOLDER),
                           UA_NODEID_NUMERIC(0, UA_NS0ID_ORGANIZES),
                           transmitterName,
                           UA_NODEID_NUMERIC(0, UA_NS0ID_BASEOBJECTTYPE),
                           objAttr, NULL, NULL);

    // Add configuration folder
    UA_NodeId configFolderId = UA_NODEID_STRING(1, "Configuration");
    UA_ObjectAttributes configFolderAttr = UA_ObjectAttributes_default;
    configFolderAttr.displayName = UA_LOCALIZEDTEXT("en-US", "Configuration");
    
    UA_Server_addObjectNode(server, configFolderId, transmitterId,
                           UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT),
                           UA_QUALIFIEDNAME(1, "Configuration"),
                           UA_NODEID_NUMERIC(0, UA_NS0ID_FOLDERTYPE),
                           configFolderAttr, NULL, NULL);

    // Add status folder
    UA_NodeId statusFolderId = UA_NODEID_STRING(1, "Status");
    UA_ObjectAttributes statusFolderAttr = UA_ObjectAttributes_default;
    statusFolderAttr.displayName = UA_LOCALIZEDTEXT("en-US", "Status");
    
    UA_Server_addObjectNode(server, statusFolderId, transmitterId,
                           UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT),
                           UA_QUALIFIEDNAME(1, "Status"),
                           UA_NODEID_NUMERIC(0, UA_NS0ID_FOLDERTYPE),
                           statusFolderAttr, NULL, NULL);

    // Add configuration parameters with callbacks
    addVariableWithCallback(server, configFolderId, "StepSize", "Step Size",
                          &transmitter.config.step_size, &UA_TYPES[UA_TYPES_DOUBLE]);
    
    addVariableWithCallback(server, configFolderId, "SimulationActive", "Simulation Active",
                          &transmitter.config.simulation_active, &UA_TYPES[UA_TYPES_BOOLEAN]);
    
    addVariableWithCallback(server, configFolderId, "SineWave", "Sine Wave",
                          &transmitter.config.sine_wave, &UA_TYPES[UA_TYPES_BOOLEAN]);
    
    addVariableWithCallback(server, configFolderId, "SawtoothWave", "Sawtooth Wave",
                          &transmitter.config.sawtooth_wave, &UA_TYPES[UA_TYPES_BOOLEAN]);
    
    addVariableWithCallback(server, configFolderId, "Overflow", "Overflow",
                          &transmitter.config.overflow, &UA_TYPES[UA_TYPES_BOOLEAN]);
    
    addVariableWithCallback(server, configFolderId, "Underflow", "Underflow",
                          &transmitter.config.underflow, &UA_TYPES[UA_TYPES_BOOLEAN]);

    // Add status variables (read-only)
    UA_VariableAttributes statusAttr = UA_VariableAttributes_default;
    statusAttr.displayName = UA_LOCALIZEDTEXT("en-US", "CurrentValue");
    statusAttr.accessLevel = UA_ACCESSLEVELMASK_READ;
    UA_Variant_setScalar(&statusAttr.value, &transmitter.state.current_value, &UA_TYPES[UA_TYPES_DOUBLE]);
    
    UA_Server_addVariableNode(server, UA_NODEID_STRING(1, "CurrentValue"), statusFolderId,
                            UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT),
                            UA_QUALIFIEDNAME(1, "CurrentValue"),
                            UA_NODEID_NUMERIC(0, UA_NS0ID_BASEDATAVARIABLETYPE),
                            statusAttr, NULL, NULL);

    statusAttr.displayName = UA_LOCALIZEDTEXT("en-US", "Fault");
    UA_Variant_setScalar(&statusAttr.value, &transmitter.state.fault, &UA_TYPES[UA_TYPES_BOOLEAN]);
    
    UA_Server_addVariableNode(server, UA_NODEID_STRING(1, "Fault"), statusFolderId,
                            UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT),
                            UA_QUALIFIEDNAME(1, "Fault"),
                            UA_NODEID_NUMERIC(0, UA_NS0ID_BASEDATAVARIABLETYPE),
                            statusAttr, NULL, NULL);
}

int main(void) {
    signal(SIGINT, stopHandler);
    signal(SIGTERM, stopHandler);

    // Initialize transmitter
    Transmitter_Init(&transmitter);

    // Create and configure server
    server = UA_Server_new();
    UA_ServerConfig_setDefault(UA_Server_getConfig(server));

    // Add transmitter object
    addTransmitterObject(server);

    printf("OPC UA Transmitter Server running at opc.tcp://localhost:4840\n");
    printf("Browse path: Objects->Transmitter\n");
    printf(" - Configuration: StepSize, SimulationActive, SineWave, SawtoothWave, Overflow, Underflow\n");
    printf(" - Status: CurrentValue, Fault\n");

    // Start server
    UA_StatusCode status = UA_Server_run_startup(server);
    if (status != UA_STATUSCODE_GOOD) {
        UA_Server_delete(server);
        return status;
    }

    // Main loop
    while (running) {
        UA_Server_run_iterate(server, true);
        Transmitter_Update(&transmitter, DEFAULT_CYCLE_TIME_MS);

        // Update the current value in the server
        UA_Variant value;
        UA_Variant_init(&value);
        UA_Variant_setScalar(&value, &transmitter.state.current_value, &UA_TYPES[UA_TYPES_DOUBLE]);
        UA_Server_writeValue(server, UA_NODEID_STRING(1, "CurrentValue"), value);

        // Update fault status
        UA_Variant_setScalar(&value, &transmitter.state.fault, &UA_TYPES[UA_TYPES_BOOLEAN]);
        UA_Server_writeValue(server, UA_NODEID_STRING(1, "Fault"), value);

#ifdef _WIN32
        Sleep(DEFAULT_CYCLE_TIME_MS);
#else
        usleep(DEFAULT_CYCLE_TIME_MS * 1000);
#endif
    }

    // Cleanup
    UA_Server_run_shutdown(server);
    UA_Server_delete(server);
    return 0;
}