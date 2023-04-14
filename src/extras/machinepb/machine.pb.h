/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.6 */

#ifndef PB_MACHINE_MACHINE_PB_H_INCLUDED
#define PB_MACHINE_MACHINE_PB_H_INCLUDED
#include "../nanopb/pb.h"

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

/* Enum definitions */
/* used in requests */
typedef enum _machine_SolenoidValve { 
    machine_SolenoidValve_VALVE_UNDEFINED = 0, 
    machine_SolenoidValve_VALVE_DRAIN = 1, 
    machine_SolenoidValve_VALVE_WATER = 2, 
    machine_SolenoidValve_VALVE_MILK = 3, 
    machine_SolenoidValve_VALVE_AIR = 4 
} machine_SolenoidValve;

typedef enum _machine_Mode { 
    machine_Mode_UNDEFINED_MODE = 0, 
    machine_Mode_MANUAL = 1, 
    machine_Mode_AUTONOMOUS = 2 
} machine_Mode;

typedef enum _machine_Status { 
    machine_Status_UNDEFINED_STATUS = 0, 
    machine_Status_ERROR = 1, 
    machine_Status_E_STOP_ACTIVE = 5, 
    machine_Status_SLEEPING = 6, 
    machine_Status_SHUTTING_DOWN = 9, 
    machine_Status_WAKING_UP = 10, 
    machine_Status_CALIBRATING = 20, 
    machine_Status_IDLE_STATIONARY = 30, 
    machine_Status_IDLE_MOVING = 31, 
    machine_Status_RINSING_PIPETTE = 40, 
    machine_Status_DISPENSING = 50, 
    machine_Status_WAITING_FOR_DISPENSE = 55, 
    machine_Status_COLLECTING = 60, 
    machine_Status_NAVIGATING_IK = 70, 
    machine_Status_NAVIGATING_OUTER = 75 
} machine_Status;

typedef enum _machine_FluidType { 
    machine_FluidType_FLUID_UNDEFINED = 0, 
    machine_FluidType_FLUID_DRAIN = 1, 
    machine_FluidType_FLUID_WATER = 2, 
    machine_FluidType_FLUID_MILK = 3 
} machine_FluidType;

/* Struct definitions */
typedef struct _machine_StateReportList { 
    pb_callback_t StateReports;
} machine_StateReportList;

typedef struct _machine_CollectionRequest { 
    bool completed;
    uint64_t request_number;
    uint64_t vial_number;
    float volume_ul;
} machine_CollectionRequest;

typedef struct _machine_FluidDetails { 
    float bowl_fluid_level_ml;
} machine_FluidDetails;

typedef struct _machine_FluidRequest { 
    machine_FluidType fluidType;
    float volume_ml;
    bool complete;
    /* if true, open drain while request is taking place
 (e.g. for rinsing with water) */
    bool open_drain;
} machine_FluidRequest;

typedef struct _machine_MovementDetails { 
    /* ik target from -1 to 1 */
    float target_x_unit;
    /* ik target from -1 to 1 */
    float target_y_unit;
    /* ik z target in mm */
    float target_z_ik;
    /* fk target in degrees */
    float target_ring_deg;
    /* fk target in degrees */
    float target_yaw_deg; /* @exclude not adding current positions here, because
 these reports are sent when something changes, not
 regularly. Regular info can come through a controls
 / realtime report. */
} machine_MovementDetails;

typedef struct _machine_PipetteState { 
    bool spent;
    uint32_t vial_held;
    float volume_target_ul;
    /* incremented every time a dispense is requested */
    uint32_t dispense_request_number;
} machine_PipetteState;

typedef struct _machine_SessionStatus { 
    uint64_t id;
    bool paused;
    bool complete;
    bool production;
    uint64_t production_id;
} machine_SessionStatus;

typedef struct _machine_StreamStatus { 
    bool live;
} machine_StreamStatus;

typedef struct _machine_StateReport { 
    /* timestamp in microseconds since unix epoch, UTC. Added
 by gateway since firmware doesn't know real time. */
    uint64_t timestamp_unix_micros;
    /* incremented on startup, currently 1 byte */
    uint64_t startup_counter;
    machine_Mode mode;
    machine_Status status;
    /* Useful for synchronisation with footage */
    bool lights_on;
    bool has_pipette_state;
    machine_PipetteState pipette_state;
    bool has_collection_request;
    machine_CollectionRequest collection_request;
    bool has_movement_details;
    machine_MovementDetails movement_details;
    bool has_fluid_request;
    machine_FluidRequest fluid_request;
    bool has_fluid_details;
    machine_FluidDetails fluid_details;
    /* the following are populated by the backend, useful in post-processing */
    bool paused;
    pb_callback_t timestamp_readable;
} machine_StateReport;


/* Helper constants for enums */
#define _machine_SolenoidValve_MIN machine_SolenoidValve_VALVE_UNDEFINED
#define _machine_SolenoidValve_MAX machine_SolenoidValve_VALVE_AIR
#define _machine_SolenoidValve_ARRAYSIZE ((machine_SolenoidValve)(machine_SolenoidValve_VALVE_AIR+1))

#define _machine_Mode_MIN machine_Mode_UNDEFINED_MODE
#define _machine_Mode_MAX machine_Mode_AUTONOMOUS
#define _machine_Mode_ARRAYSIZE ((machine_Mode)(machine_Mode_AUTONOMOUS+1))

#define _machine_Status_MIN machine_Status_UNDEFINED_STATUS
#define _machine_Status_MAX machine_Status_NAVIGATING_OUTER
#define _machine_Status_ARRAYSIZE ((machine_Status)(machine_Status_NAVIGATING_OUTER+1))

#define _machine_FluidType_MIN machine_FluidType_FLUID_UNDEFINED
#define _machine_FluidType_MAX machine_FluidType_FLUID_MILK
#define _machine_FluidType_ARRAYSIZE ((machine_FluidType)(machine_FluidType_FLUID_MILK+1))


#ifdef __cplusplus
extern "C" {
#endif

/* Initializer values for message structs */
#define machine_PipetteState_init_default        {0, 0, 0, 0}
#define machine_CollectionRequest_init_default   {0, 0, 0, 0}
#define machine_MovementDetails_init_default     {0, 0, 0, 0, 0}
#define machine_FluidRequest_init_default        {_machine_FluidType_MIN, 0, 0, 0}
#define machine_FluidDetails_init_default        {0}
#define machine_StateReport_init_default         {0, 0, _machine_Mode_MIN, _machine_Status_MIN, 0, false, machine_PipetteState_init_default, false, machine_CollectionRequest_init_default, false, machine_MovementDetails_init_default, false, machine_FluidRequest_init_default, false, machine_FluidDetails_init_default, 0, {{NULL}, NULL}}
#define machine_StateReportList_init_default     {{{NULL}, NULL}}
#define machine_SessionStatus_init_default       {0, 0, 0, 0, 0}
#define machine_StreamStatus_init_default        {0}
#define machine_PipetteState_init_zero           {0, 0, 0, 0}
#define machine_CollectionRequest_init_zero      {0, 0, 0, 0}
#define machine_MovementDetails_init_zero        {0, 0, 0, 0, 0}
#define machine_FluidRequest_init_zero           {_machine_FluidType_MIN, 0, 0, 0}
#define machine_FluidDetails_init_zero           {0}
#define machine_StateReport_init_zero            {0, 0, _machine_Mode_MIN, _machine_Status_MIN, 0, false, machine_PipetteState_init_zero, false, machine_CollectionRequest_init_zero, false, machine_MovementDetails_init_zero, false, machine_FluidRequest_init_zero, false, machine_FluidDetails_init_zero, 0, {{NULL}, NULL}}
#define machine_StateReportList_init_zero        {{{NULL}, NULL}}
#define machine_SessionStatus_init_zero          {0, 0, 0, 0, 0}
#define machine_StreamStatus_init_zero           {0}

/* Field tags (for use in manual encoding/decoding) */
#define machine_StateReportList_StateReports_tag 1
#define machine_CollectionRequest_completed_tag  1
#define machine_CollectionRequest_request_number_tag 2
#define machine_CollectionRequest_vial_number_tag 3
#define machine_CollectionRequest_volume_ul_tag  4
#define machine_FluidDetails_bowl_fluid_level_ml_tag 1
#define machine_FluidRequest_fluidType_tag       1
#define machine_FluidRequest_volume_ml_tag       2
#define machine_FluidRequest_complete_tag        3
#define machine_FluidRequest_open_drain_tag      4
#define machine_MovementDetails_target_x_unit_tag 1
#define machine_MovementDetails_target_y_unit_tag 2
#define machine_MovementDetails_target_z_ik_tag  5
#define machine_MovementDetails_target_ring_deg_tag 10
#define machine_MovementDetails_target_yaw_deg_tag 11
#define machine_PipetteState_spent_tag           1
#define machine_PipetteState_vial_held_tag       2
#define machine_PipetteState_volume_target_ul_tag 3
#define machine_PipetteState_dispense_request_number_tag 4
#define machine_SessionStatus_id_tag             1
#define machine_SessionStatus_paused_tag         2
#define machine_SessionStatus_complete_tag       3
#define machine_SessionStatus_production_tag     4
#define machine_SessionStatus_production_id_tag  5
#define machine_StreamStatus_live_tag            1
#define machine_StateReport_timestamp_unix_micros_tag 2
#define machine_StateReport_startup_counter_tag  3
#define machine_StateReport_mode_tag             4
#define machine_StateReport_status_tag           5
#define machine_StateReport_lights_on_tag        6
#define machine_StateReport_pipette_state_tag    10
#define machine_StateReport_collection_request_tag 11
#define machine_StateReport_movement_details_tag 12
#define machine_StateReport_fluid_request_tag    13
#define machine_StateReport_fluid_details_tag    14
#define machine_StateReport_paused_tag           50
#define machine_StateReport_timestamp_readable_tag 51

/* Struct field encoding specification for nanopb */
#define machine_PipetteState_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, BOOL,     spent,             1) \
X(a, STATIC,   SINGULAR, UINT32,   vial_held,         2) \
X(a, STATIC,   SINGULAR, FLOAT,    volume_target_ul,   3) \
X(a, STATIC,   SINGULAR, UINT32,   dispense_request_number,   4)
#define machine_PipetteState_CALLBACK NULL
#define machine_PipetteState_DEFAULT NULL

#define machine_CollectionRequest_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, BOOL,     completed,         1) \
X(a, STATIC,   SINGULAR, UINT64,   request_number,    2) \
X(a, STATIC,   SINGULAR, UINT64,   vial_number,       3) \
X(a, STATIC,   SINGULAR, FLOAT,    volume_ul,         4)
#define machine_CollectionRequest_CALLBACK NULL
#define machine_CollectionRequest_DEFAULT NULL

#define machine_MovementDetails_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, FLOAT,    target_x_unit,     1) \
X(a, STATIC,   SINGULAR, FLOAT,    target_y_unit,     2) \
X(a, STATIC,   SINGULAR, FLOAT,    target_z_ik,       5) \
X(a, STATIC,   SINGULAR, FLOAT,    target_ring_deg,  10) \
X(a, STATIC,   SINGULAR, FLOAT,    target_yaw_deg,   11)
#define machine_MovementDetails_CALLBACK NULL
#define machine_MovementDetails_DEFAULT NULL

#define machine_FluidRequest_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, UENUM,    fluidType,         1) \
X(a, STATIC,   SINGULAR, FLOAT,    volume_ml,         2) \
X(a, STATIC,   SINGULAR, BOOL,     complete,          3) \
X(a, STATIC,   SINGULAR, BOOL,     open_drain,        4)
#define machine_FluidRequest_CALLBACK NULL
#define machine_FluidRequest_DEFAULT NULL

#define machine_FluidDetails_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, FLOAT,    bowl_fluid_level_ml,   1)
#define machine_FluidDetails_CALLBACK NULL
#define machine_FluidDetails_DEFAULT NULL

#define machine_StateReport_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, UINT64,   timestamp_unix_micros,   2) \
X(a, STATIC,   SINGULAR, UINT64,   startup_counter,   3) \
X(a, STATIC,   SINGULAR, UENUM,    mode,              4) \
X(a, STATIC,   SINGULAR, UENUM,    status,            5) \
X(a, STATIC,   SINGULAR, BOOL,     lights_on,         6) \
X(a, STATIC,   OPTIONAL, MESSAGE,  pipette_state,    10) \
X(a, STATIC,   OPTIONAL, MESSAGE,  collection_request,  11) \
X(a, STATIC,   OPTIONAL, MESSAGE,  movement_details,  12) \
X(a, STATIC,   OPTIONAL, MESSAGE,  fluid_request,    13) \
X(a, STATIC,   OPTIONAL, MESSAGE,  fluid_details,    14) \
X(a, STATIC,   SINGULAR, BOOL,     paused,           50) \
X(a, CALLBACK, SINGULAR, STRING,   timestamp_readable,  51)
#define machine_StateReport_CALLBACK pb_default_field_callback
#define machine_StateReport_DEFAULT NULL
#define machine_StateReport_pipette_state_MSGTYPE machine_PipetteState
#define machine_StateReport_collection_request_MSGTYPE machine_CollectionRequest
#define machine_StateReport_movement_details_MSGTYPE machine_MovementDetails
#define machine_StateReport_fluid_request_MSGTYPE machine_FluidRequest
#define machine_StateReport_fluid_details_MSGTYPE machine_FluidDetails

#define machine_StateReportList_FIELDLIST(X, a) \
X(a, CALLBACK, REPEATED, MESSAGE,  StateReports,      1)
#define machine_StateReportList_CALLBACK pb_default_field_callback
#define machine_StateReportList_DEFAULT NULL
#define machine_StateReportList_StateReports_MSGTYPE machine_StateReport

#define machine_SessionStatus_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, UINT64,   id,                1) \
X(a, STATIC,   SINGULAR, BOOL,     paused,            2) \
X(a, STATIC,   SINGULAR, BOOL,     complete,          3) \
X(a, STATIC,   SINGULAR, BOOL,     production,        4) \
X(a, STATIC,   SINGULAR, UINT64,   production_id,     5)
#define machine_SessionStatus_CALLBACK NULL
#define machine_SessionStatus_DEFAULT NULL

#define machine_StreamStatus_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, BOOL,     live,              1)
#define machine_StreamStatus_CALLBACK NULL
#define machine_StreamStatus_DEFAULT NULL

extern const pb_msgdesc_t machine_PipetteState_msg;
extern const pb_msgdesc_t machine_CollectionRequest_msg;
extern const pb_msgdesc_t machine_MovementDetails_msg;
extern const pb_msgdesc_t machine_FluidRequest_msg;
extern const pb_msgdesc_t machine_FluidDetails_msg;
extern const pb_msgdesc_t machine_StateReport_msg;
extern const pb_msgdesc_t machine_StateReportList_msg;
extern const pb_msgdesc_t machine_SessionStatus_msg;
extern const pb_msgdesc_t machine_StreamStatus_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define machine_PipetteState_fields &machine_PipetteState_msg
#define machine_CollectionRequest_fields &machine_CollectionRequest_msg
#define machine_MovementDetails_fields &machine_MovementDetails_msg
#define machine_FluidRequest_fields &machine_FluidRequest_msg
#define machine_FluidDetails_fields &machine_FluidDetails_msg
#define machine_StateReport_fields &machine_StateReport_msg
#define machine_StateReportList_fields &machine_StateReportList_msg
#define machine_SessionStatus_fields &machine_SessionStatus_msg
#define machine_StreamStatus_fields &machine_StreamStatus_msg

/* Maximum encoded size of messages (where known) */
/* machine_StateReport_size depends on runtime parameters */
/* machine_StateReportList_size depends on runtime parameters */
#define machine_CollectionRequest_size           29
#define machine_FluidDetails_size                5
#define machine_FluidRequest_size                11
#define machine_MovementDetails_size             25
#define machine_PipetteState_size                19
#define machine_SessionStatus_size               28
#define machine_StreamStatus_size                2

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
