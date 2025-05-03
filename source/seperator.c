#include <open62541/server.h>
#include <open62541/server_config_default.h>
#include <open62541/plugin/log_stdout.h>
#include <signal.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <time.h>
#include <string.h>

#define DEFAULT_CYCLE_TIME_MS 100

// Physical constants
#define GAS_CONSTANT 8.314       // J/mol·K
#define TEMPERATURE 300.0        // K (27°C)
#define GAS_MOLAR_MASS 0.029     // kg/mol (approximate for natural gas)
#define GAMMA 1.4                // Specific heat ratio (Cp/Cv)
#define CRITICAL_PRESSURE_RATIO pow(2/(GAMMA+1), GAMMA/(GAMMA-1)) // ≈0.528 for air

// --- Separator Model ---
typedef struct {
    // Config (adjustable via OPC UA)
    struct {
        double Q_in_oil;
        double Q_in_water;
        double Q_in_gas;
        double valve_oil;
        double valve_water;
        double valve_gas;
    } config;

    // State (read-only via OPC UA)
    struct {
        double h_oil;
        double h_water;
        double pressure;
    } state;

    // Constants
    double area;
    double total_volume;
    double Cd;
    double A_valve_liquid;
    double A_valve_gas;
    double gas_mass;
    double ambient_pressure;
} SeparatorSimulator;

// Globals
SeparatorSimulator separator;
volatile bool running = true;
UA_Server *server;

void stopHandler(int sign) {
    running = false;
}

void Separator_Init(SeparatorSimulator *sep) {
    // Steady-state defaults
    sep->config.Q_in_oil = 0.05;      // m³/s
    sep->config.Q_in_water = 0.03;    // m³/s
    sep->config.Q_in_gas = 0.1;       // m³/s (increased for compressible gas)
    
    sep->config.valve_oil = 45.0;     // % opening
    sep->config.valve_water = 35.0;   // % opening
    sep->config.valve_gas = 25.0;     // % opening (more sensitive with new equations)
    
    // Initial state
    sep->state.h_oil = 0.5;           // m
    sep->state.h_water = 0.5;         // m
    sep->state.pressure = 150000.0;   // Pa (1.5 bar)
    
    // Physical parameters
    sep->area = 10.0;                 // m²
    sep->total_volume = 50.0;         // m³
    sep->Cd = 0.6;                    // Discharge coefficient
    sep->A_valve_liquid = 0.01;       // m²
    sep->A_valve_gas = 0.005;         // m²
    sep->ambient_pressure = 101325.0; // Pa (1 atm)
    
    // Initialize gas mass
    double initial_gas_volume = sep->total_volume - sep->area * 
                              (sep->state.h_oil + sep->state.h_water);
    sep->gas_mass = (sep->state.pressure * initial_gas_volume) * 
                   GAS_MOLAR_MASS / (GAS_CONSTANT * TEMPERATURE);
}

void Separator_Update(SeparatorSimulator *sep, uint32_t cycle_time_ms) {
    double dt = cycle_time_ms / 1000.0;
    const double g = 9.81;

    // 1. Update liquid levels (existing Torricelli's law calculations)
    double valve_oil_coeff = sep->config.valve_oil / 100.0;
    double valve_water_coeff = sep->config.valve_water / 100.0;

    double Q_out_oil = sep->Cd * sep->A_valve_liquid * valve_oil_coeff * sqrt(2 * g * sep->state.h_oil);
    double Q_out_water = sep->Cd * sep->A_valve_liquid * valve_water_coeff * sqrt(2 * g * sep->state.h_water);

    sep->state.h_oil += (sep->config.Q_in_oil - Q_out_oil) / sep->area * dt;
    sep->state.h_water += (sep->config.Q_in_water - Q_out_water) / sep->area * dt;

    // Clamp heights
    double max_height = sep->total_volume / sep->area;
    sep->state.h_oil = fmin(fmax(sep->state.h_oil, 0.0), max_height);
    sep->state.h_water = fmin(fmax(sep->state.h_water, 0.0), max_height - sep->state.h_oil);

    // 2. Calculate current gas volume
    double V_gas = sep->total_volume - sep->area * (sep->state.h_oil + sep->state.h_water);
    
    // 3. Calculate gas outflow (compressible flow equation)
    double valve_gas_coeff = sep->config.valve_gas / 100.0;
    double P_ratio = sep->ambient_pressure / sep->state.pressure;
    
    double Q_out_gas;
    if (P_ratio <= CRITICAL_PRESSURE_RATIO) {
        // Critical flow (choked)
        Q_out_gas = sep->Cd * sep->A_valve_gas * valve_gas_coeff * 
                   sqrt(GAMMA * sep->state.pressure / GAS_MOLAR_MASS * 
                   pow(2/(GAMMA+1), (GAMMA+1)/(GAMMA-1)));
    } else {
        // Subcritical flow
        Q_out_gas = sep->Cd * sep->A_valve_gas * valve_gas_coeff * 
                   sqrt(2 * sep->state.pressure / GAS_MOLAR_MASS * 
                   (GAMMA/(GAMMA-1)) * 
                   (pow(P_ratio, 2/GAMMA) - pow(P_ratio, (GAMMA+1)/GAMMA)));
    }

    // 4. Update gas mass (convert Q_in_gas from volumetric to mass flow)
    double Q_in_gas_mass = sep->config.Q_in_gas * sep->state.pressure * GAS_MOLAR_MASS / 
                          (GAS_CONSTANT * TEMPERATURE);
    sep->gas_mass += (Q_in_gas_mass - Q_out_gas * GAS_MOLAR_MASS) * dt;

    // 5. Calculate new pressure (ideal gas law)
    sep->state.pressure = (sep->gas_mass * GAS_CONSTANT * TEMPERATURE) / 
                         (V_gas * GAS_MOLAR_MASS);

    // Ensure pressure doesn't drop below ambient
    sep->state.pressure = fmax(sep->state.pressure, sep->ambient_pressure);
}

// --- OPC UA Callbacks ---
static void onConfigChanged(UA_Server *server, const UA_NodeId *sessionId,
                            void *sessionContext, const UA_NodeId *nodeId,
                            void *nodeContext, const UA_NumericRange *range,
                            const UA_DataValue *data) {
    if (!data || !data->hasValue || !UA_Variant_isScalar(&data->value)) return;

    UA_QualifiedName browseName;
    UA_Server_readBrowseName(server, *nodeId, &browseName);

    UA_String q_in_oil_str = UA_STRING("Q_in_oil");
    UA_String q_in_water_str = UA_STRING("Q_in_water");
    UA_String q_in_gas_str = UA_STRING("Q_in_gas");
    UA_String valve_oil_str = UA_STRING("valve_oil");
    UA_String valve_water_str = UA_STRING("valve_water");
    UA_String valve_gas_str = UA_STRING("valve_gas");

    if (UA_String_equal(&browseName.name, &q_in_oil_str))
        separator.config.Q_in_oil = *(UA_Double*)data->value.data;
    else if (UA_String_equal(&browseName.name, &q_in_water_str))
        separator.config.Q_in_water = *(UA_Double*)data->value.data;
    else if (UA_String_equal(&browseName.name, &q_in_gas_str))
        separator.config.Q_in_gas = *(UA_Double*)data->value.data;
    else if (UA_String_equal(&browseName.name, &valve_oil_str))
        separator.config.valve_oil = *(UA_Double*)data->value.data;
    else if (UA_String_equal(&browseName.name, &valve_water_str))
        separator.config.valve_water = *(UA_Double*)data->value.data;
    else if (UA_String_equal(&browseName.name, &valve_gas_str))
        separator.config.valve_gas = *(UA_Double*)data->value.data;

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

static void addSeparatorObject(UA_Server *server) {
    UA_Server_addObjectNode(server, UA_NODEID_STRING(1, "Separator"),
                            UA_NODEID_NUMERIC(0, UA_NS0ID_OBJECTSFOLDER),
                            UA_NODEID_NUMERIC(0, UA_NS0ID_ORGANIZES),
                            UA_QUALIFIEDNAME(1, "Separator"),
                            UA_NODEID_NUMERIC(0, UA_NS0ID_BASEOBJECTTYPE),
                            UA_ObjectAttributes_default, NULL, NULL);

    UA_Server_addObjectNode(server, UA_NODEID_STRING(1, "Config"),
                            UA_NODEID_STRING(1, "Separator"),
                            UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT),
                            UA_QUALIFIEDNAME(1, "Config"),
                            UA_NODEID_NUMERIC(0, UA_NS0ID_FOLDERTYPE),
                            UA_ObjectAttributes_default, NULL, NULL);

    addVariableWithCallback(server, UA_NODEID_STRING(1, "Config"), "Q_in_oil", "Oil Inflow", &separator.config.Q_in_oil, &UA_TYPES[UA_TYPES_DOUBLE]);
    addVariableWithCallback(server, UA_NODEID_STRING(1, "Config"), "Q_in_water", "Water Inflow", &separator.config.Q_in_water, &UA_TYPES[UA_TYPES_DOUBLE]);
    addVariableWithCallback(server, UA_NODEID_STRING(1, "Config"), "Q_in_gas", "Gas Inflow", &separator.config.Q_in_gas, &UA_TYPES[UA_TYPES_DOUBLE]);
    addVariableWithCallback(server, UA_NODEID_STRING(1, "Config"), "valve_oil", "Oil Valve", &separator.config.valve_oil, &UA_TYPES[UA_TYPES_DOUBLE]);
    addVariableWithCallback(server, UA_NODEID_STRING(1, "Config"), "valve_water", "Water Valve", &separator.config.valve_water, &UA_TYPES[UA_TYPES_DOUBLE]);
    addVariableWithCallback(server, UA_NODEID_STRING(1, "Config"), "valve_gas", "Gas Valve", &separator.config.valve_gas, &UA_TYPES[UA_TYPES_DOUBLE]);

    UA_Server_addObjectNode(server, UA_NODEID_STRING(1, "State"),
                            UA_NODEID_STRING(1, "Separator"),
                            UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT),
                            UA_QUALIFIEDNAME(1, "State"),
                            UA_NODEID_NUMERIC(0, UA_NS0ID_FOLDERTYPE),
                            UA_ObjectAttributes_default, NULL, NULL);

    // h_oil
    UA_VariableAttributes oilAttr = UA_VariableAttributes_default;
    oilAttr.displayName = UA_LOCALIZEDTEXT("en-US", "h_oil");
    oilAttr.accessLevel = UA_ACCESSLEVELMASK_READ;
    oilAttr.minimumSamplingInterval = 100.0;
    UA_Variant_setScalar(&oilAttr.value, &separator.state.h_oil, &UA_TYPES[UA_TYPES_DOUBLE]);
    UA_Server_addVariableNode(server, UA_NODEID_STRING(1, "h_oil"),
                              UA_NODEID_STRING(1, "State"),
                              UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT),
                              UA_QUALIFIEDNAME(1, "h_oil"),
                              UA_NODEID_NUMERIC(0, UA_NS0ID_BASEDATAVARIABLETYPE),
                              oilAttr, NULL, NULL);

    // h_water
    UA_VariableAttributes waterAttr = UA_VariableAttributes_default;
    waterAttr.displayName = UA_LOCALIZEDTEXT("en-US", "h_water");
    waterAttr.accessLevel = UA_ACCESSLEVELMASK_READ;
    waterAttr.minimumSamplingInterval = 100.0;
    UA_Variant_setScalar(&waterAttr.value, &separator.state.h_water, &UA_TYPES[UA_TYPES_DOUBLE]);
    UA_Server_addVariableNode(server, UA_NODEID_STRING(1, "h_water"),
                              UA_NODEID_STRING(1, "State"),
                              UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT),
                              UA_QUALIFIEDNAME(1, "h_water"),
                              UA_NODEID_NUMERIC(0, UA_NS0ID_BASEDATAVARIABLETYPE),
                              waterAttr, NULL, NULL);

    // pressure
    UA_VariableAttributes pressureAttr = UA_VariableAttributes_default;
    pressureAttr.displayName = UA_LOCALIZEDTEXT("en-US", "pressure");
    pressureAttr.accessLevel = UA_ACCESSLEVELMASK_READ;
    pressureAttr.minimumSamplingInterval = 100.0;
    UA_Variant_setScalar(&pressureAttr.value, &separator.state.pressure, &UA_TYPES[UA_TYPES_DOUBLE]);
    UA_Server_addVariableNode(server, UA_NODEID_STRING(1, "pressure"),
                              UA_NODEID_STRING(1, "State"),
                              UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT),
                              UA_QUALIFIEDNAME(1, "pressure"),
                              UA_NODEID_NUMERIC(0, UA_NS0ID_BASEDATAVARIABLETYPE),
                              pressureAttr, NULL, NULL);
}

int main(void) {
    signal(SIGINT, stopHandler);
    signal(SIGTERM, stopHandler);

    Separator_Init(&separator);
    server = UA_Server_new();
    UA_ServerConfig_setDefault(UA_Server_getConfig(server));

    addSeparatorObject(server);
    printf("OPC UA Separator Server running at opc.tcp://localhost:4840\n");

    UA_Server_run_startup(server);
    while (running) {
        UA_Server_run_iterate(server, true);
        Separator_Update(&separator, DEFAULT_CYCLE_TIME_MS);

        UA_Variant value;

        UA_Variant_setScalar(&value, &separator.state.h_oil, &UA_TYPES[UA_TYPES_DOUBLE]);
        UA_Server_writeValue(server, UA_NODEID_STRING(1, "h_oil"), value);

        UA_Variant_setScalar(&value, &separator.state.h_water, &UA_TYPES[UA_TYPES_DOUBLE]);
        UA_Server_writeValue(server, UA_NODEID_STRING(1, "h_water"), value);

        UA_Variant_setScalar(&value, &separator.state.pressure, &UA_TYPES[UA_TYPES_DOUBLE]);
        UA_Server_writeValue(server, UA_NODEID_STRING(1, "pressure"), value);

#ifdef _WIN32
        Sleep(DEFAULT_CYCLE_TIME_MS);
#else
        usleep(DEFAULT_CYCLE_TIME_MS * 1000);
#endif
    }

    UA_Server_run_shutdown(server);
    UA_Server_delete(server);
    return 0;
}
