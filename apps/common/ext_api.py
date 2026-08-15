# =============================================================================
# Controlled Descent Simulator
# =============================================================================
# File        : ext_api.py
# Description : Declarative description of the external communication API —
#               the single source of truth for the ext boundary. gen_ext.py
#               reads this file and generates the struct definitions, the API
#               contract, the embind bindings and the whole web socket wire
#               layer (protocol, client marshalling, server dispatch).
#
#               To add a command: add its structs here (if new), add a Cmd
#               entry, run `python3 apps/common/gen_ext.py` (or just build:
#               CMake re-runs it when this file changes), then implement the
#               direct adapter function in apps/common/ext_comm.cpp.
# =============================================================================

# --------------------------------------------------------------------------- #
# tiny schema classes                                                          #
# --------------------------------------------------------------------------- #

class F:
    """One struct field. type defaults to ext_coord_t; js is the embind field
    name (defaults to the C++ name); doc is a trailing // comment; pre is a
    /* */ comment (possibly multi-line) emitted before the field; post is a
    verbatim comment line emitted after the field; blank_before adds an empty
    line before the field."""

    def __init__(self, name, type="ext_coord_t", js=None, doc=None, pre=None,
                 post=None, blank_before=False, count=None):
        self.name = name
        self.type = type
        self.js = js if js is not None else name
        self.doc = doc
        self.pre = pre
        self.post = post
        self.blank_before = blank_before
        # count > 0 makes the field a fixed C array `type name[count]`. Only the
        # `char` base type is supported as an array: a fixed text buffer bound to
        # JS as a std::string — the way variable text crosses the POD wire.
        self.count = count


class Struct:
    """One ext struct. file selects the generated header ('defs' ->
    ext_defs.hpp, 'comm' -> ext_comm.hpp); doc_style picks the comment style
    of the struct doc ('/*' or '//'); bind=False skips embind registration."""

    def __init__(self, name, fields, doc=None, file="defs", doc_style="/*",
                 bind=True):
        self.name = name
        self.fields = fields
        self.doc = doc
        self.file = file
        self.doc_style = doc_style
        self.bind = bind


class Raw:
    """Verbatim text emitted between structs in ext_defs.hpp."""

    def __init__(self, text):
        self.text = text


class Cmd:
    """One API command.
    wire: PascalCase stem for the wire names (reqInitRocket_t / WS_MSG_INIT_ROCKET)
    cfn / js: C++ function name / embind-exposed JS name
    req: None (no arguments) | struct name | ("scalar", type, argname)
    resp: "bool" | struct name
    doc: comment above the declaration; log: optional server-side printf tag
    """

    def __init__(self, id, wire, cfn, js, req, resp, doc, log=None):
        self.id = id
        self.wire = wire
        self.cfn = cfn
        self.js = js
        self.req = req
        self.resp = resp
        self.doc = doc
        self.log = log


# --------------------------------------------------------------------------- #
# ext_defs.hpp content (order = emission order)                                #
# --------------------------------------------------------------------------- #

DEFS = [
    Raw('/* Types */\n'
        'typedef float ext_coord_t; // type for running coordinates: position, pose, force'),

    Struct("ext_vec3_t", doc=None, fields=[
        F("x"), F("y"), F("z"),
    ]),

    Raw('\n// ----------------------------- external communication -------------------------\n'),

    Struct("ext_fullState",
           doc="struct for the communication layer: contains the state to represent the\n"
               "rocket in 3d space",
           fields=[
               F("x_dot", pre="velocities"), F("y_dot"), F("z_dot"),
               F("x", pre="Position"), F("y"), F("z"),
               F("roll_dot", pre="angular velocities", blank_before=True),
               F("pitch_dot"), F("yaw_dot"),
               F("roll", pre="angles"), F("pitch"), F("yaw"),
           ]),

    Struct("ext_setpointError",
           doc="struct to represent the error With Respect To the setpoint",
           fields=[
               F("xErr", pre="Position"), F("yErr"), F("zErr"), F("yawErr"),
           ]),

    Struct("ext_rocketParams",
           doc="struct to package the rocket model parameters",
           fields=[
               F("mass_Kg"), F("inertiaX_Kgm2"), F("inertiaY_Kgm2"),
               F("inertiaZ_Kgm2"), F("c"), F("cz"),
           ]),

    Struct("ext_quadRotorParams",
           doc="struct to package the quadRotor model parameters",
           fields=[
               F("mass_Kg"), F("inertiaX_Kgm2"), F("inertiaY_Kgm2"),
               F("inertiaZ_Kgm2"), F("c"), F("cz"),
               F("motorThrustCoefficient"), F("motorTorqueCoefficient"),
               F("distanceBtwMotorAndCoM"), F("motorMomentOfInertia"),
           ]),

    Struct("ext_trajectoryPoly4Params_t", doc_style="//",
           doc="struct of polynomial 4th order trajectory parameters",
           fields=[
               F("initialPos", type="ext_vec3_t"),
               F("initialYaw"),
               F("initialVel", type="ext_vec3_t", blank_before=True),
               F("initialYawRate",
                 post="// not enough degrees of freedom to set also initial acceleration"),
               F("finalPos", type="ext_vec3_t", blank_before=True),
               F("finalYaw"),
               F("finalVel", type="ext_vec3_t", blank_before=True),
               F("finalYawRate"),
               F("finalAcc", type="ext_vec3_t", blank_before=True),
               F("finalYawAcc"),
               F("time_s", blank_before=True,
                 doc="total duration of the maneuver towards the final state"),
           ]),

    Struct("ext_trajectoryPointParams_t", doc_style="//",
           doc="struct of the Point trajectory parameters",
           fields=[
               F("finalPos", type="ext_vec3_t"),
               F("finalYaw"),
               F("time_s", doc="total duration of the maneuver towards the finalPos"),
           ]),

    Struct("ext_trajectoryPoint",
           doc="struct of the trajectory position point",
           fields=[
               F("x"), F("y"), F("z"),
           ]),

    Struct("ext_userForce",
           doc="struct of the user input forces",
           fields=[
               F("fX"), F("fY"), F("fZ"),
           ]),

    Struct("ext_rocketActuatorLimits",
           doc="struct of the rocket's actuator limits",
           fields=[
               F("fZ_max", pre="forces"), F("fZ_min"),
               F("Tx_max",
                 pre='torques — NOTE: "Tx" acts about body Y (drives alpha/pitch) and "Ty"\n'
                     '   about body X (drives beta); names kept for backward compatibility',
                 doc="about body y axis"),
               F("Tx_min", doc="about body y axis"),
               F("Ty_max", doc="about body x axis"),
               F("Ty_min", doc="about body x axis"),
               F("Tz_max", doc="about body z axis (roll about thrust axis)"),
               F("Tz_min", doc="about body z axis (roll about thrust axis)"),
           ]),

    Struct("ext_quadRotorActuatorLimits",
           doc="struct of the quadRotor's actuator limits",
           fields=[
               F("motor_max_thrust", pre="motor thrusts"),
               F("motor_min_thrust"),
           ]),
]

# --------------------------------------------------------------------------- #
# ext_comm.hpp structs (argument/return aggregates of the API functions)       #
# --------------------------------------------------------------------------- #

COMM = [
    Struct("ext_initRocketParams", file="comm",
           doc="params for initializing rocket model",
           fields=[
               F("params", type="ext_rocketParams", js="rocketPar"),
               F("actuatorLimits", type="ext_rocketActuatorLimits", js="rocketActuatorLimits"),
           ]),

    Struct("ext_initQuadRotorParams", file="comm",
           doc="params for initializing quadRotor model",
           fields=[
               F("params", type="ext_quadRotorParams", js="quadRotorPar"),
               F("actuatorLimits", type="ext_quadRotorActuatorLimits", js="quadRotorActuatorLimits"),
           ]),

    Struct("ext_systemParams", file="comm",
           doc="struct of the argument of setSystemParams",
           fields=[
               F("timestep_seconds"),
               F("user_forces", type="ext_userForce"),
           ]),

    Struct("ext_snapshotData", file="comm",
        doc="struct of the returned data from getSnapshot",
        fields=[
            F("time_seconds", pre="elapsed simulation time"),
            F("state", type="ext_fullState"),
            F("err", type="ext_setpointError"),
            F("isError", type="bool"),
        ]),

    Struct("ext_plantSnapshotData", file="comm",
        doc="struct of the returned data from getPlantSnapshot",
        fields=[
            F("time_seconds", pre="plant-side time of the last sample"),
            F("sequence", pre="sequence number of the last sample, exact as a\n"
                              "   float up to 2^24 samples (~93 h at 50 Hz)"),
            F("state", type="ext_fullState"),
            F("isAttached", type="bool"),
            F("isReadyToStart", type="bool",
              pre="the plant is ready for a mission (staged / no staging needed)"),
            F("isError", type="bool"),
        ]),

    # ---- logger / profiler inspection (text blobs parsed on the JS side) ----

    Struct("ext_logBatch", file="comm",
        doc="struct of a batch of recent log lines. `lines` packs `count`\n"
            "newline-separated records, each 'LEVEL\\tmodule\\ttext'; the JS side\n"
            "splits on '\\n' then on '\\t'. Fixed char buffer: text on the POD wire",
        fields=[
            F("lines", type="char", count=3800),
            F("count", pre="number of records packed in `lines`"),
            F("dropped", pre="lines dropped since the last batch (UI buffer overflow)"),
        ]),

    Struct("ext_moduleList", file="comm",
        doc="struct listing registered modules, one 'index\\tname\\tvalue' record\n"
            "per newline; `value` is the log level (getLogModules) or the enabled\n"
            "flag 0/1 (getProfileModules). Fixed char buffer: text on the POD wire",
        fields=[
            F("list", type="char", count=1200),
            F("count", pre="number of modules listed in `list`"),
        ]),

    Struct("ext_profileTable", file="comm",
        doc="struct of the profiler stats table, one record per newline:\n"
            "'module\\tscope\\tcount\\tmean_us\\tmin_us\\tmax_us\\tstd_us'. Fixed char\n"
            "buffer: text on the POD wire",
        fields=[
            F("table", type="char", count=3600),
            F("count", pre="number of scope records in `table`"),
        ]),

    Struct("ext_logLevelParams", file="comm",
        doc="request: set a log module's runtime level and sampling divisor N\n"
            "(module as its index; N = 1 emits all, N>1 emits 1 in N per _SAMPLED\n"
            "call site)",
        fields=[
            F("module"), F("level"), F("sampleN"),
        ]),

    Struct("ext_profileEnableParams", file="comm",
        doc="request: enable or disable profiling for a module (module as its index)",
        fields=[
            F("module"),
            F("enabled", type="bool"),
        ]),

    Struct("ext_diagFiles", file="comm",
        doc="request: toggle the server-side diagnostics files (no-op on wasm,\n"
            "which has no real filesystem). logFile: mirror the log to a file;\n"
            "profileRaw: stream every raw profiler sample to a CSV for analysis",
        fields=[
            F("logFile", type="bool"),
            F("profileRaw", type="bool"),
        ]),

    Struct("ext_recordParams", file="comm",
        doc="request: toggle the server-side per-tick data recorder — a lossless\n"
            "wide-CSV black box of the active model's state/input/reference/error\n"
            "(no-op on wasm, which has no real filesystem)",
        fields=[
            F("enabled", type="bool"),
        ]),

    Struct("ext_recordStatus", file="comm",
        doc="response: state of the data recorder. modelName is the active\n"
            "recorder's name (empty if no model is running). active/enabled/\n"
            "droppedRows are carried as ext_coord_t (not bool) so this response\n"
            "shares no wire struct with a bool while still holding a char buffer.\n"
            "droppedRows is exact up to 2^24 rows (float mantissa)",
        fields=[
            F("modelName", type="char", count=64),
            F("active", doc="1.0 if a model recorder is registered, else 0.0"),
            F("enabled", doc="1.0 if it is currently recording, else 0.0"),
            F("droppedRows", pre="rows lost to a full ring since the run began"),
        ]),
]

# --------------------------------------------------------------------------- #
# commands                                                                     #
# --------------------------------------------------------------------------- #

COMMANDS = [
    Cmd(1, "InitRocket", "ext_initRocket_FFLQR01", "ext_rocketInit",
        req="ext_initRocketParams", resp="bool",
        doc="Initialize Rocket model: FF_LQR_01, returns true on error",
        log="init rocket"),

    Cmd(2, "InitQuadRotor", "ext_initQuadRotor_FFLQR01", "ext_quadRotorInit",
        req="ext_initQuadRotorParams", resp="bool",
        doc="Initialize QuadRotor model: FF_LQR_01, returns true on error",
        log="init quadrotor"),

    Cmd(3, "TrajGetPoint", "ext_trajectory_get_point", "ext_trajectory_get_point",
        req=("scalar", "ext_coord_t", "t"), resp="ext_trajectoryPoint",
        doc="Get a point at time instant t along the trajectory"),

    Cmd(4, "TrajAppendPoly4", "ext_trajectory_append_poly4", "ext_trajectory_append_poly4",
        req="ext_trajectoryPoly4Params_t", resp="bool",
        doc="Add a trajectory Polynomial 4th order, returns true on error"),

    Cmd(5, "TrajAppendPoint", "ext_trajectory_append_point", "ext_trajectory_append_point",
        req="ext_trajectoryPointParams_t", resp="bool",
        doc="Add a trajectory Point, returns true on error"),

    Cmd(6, "TrajRemoveLast", "ext_trajectory_remove_last_item", "ext_trajectory_remove_last_item",
        req=None, resp="bool",
        doc="Remove last trajectory item, returns true on error"),

    Cmd(7, "SetSystemParams", "ext_setSystemParams", "ext_setSystemParams",
        req="ext_systemParams", resp="bool",
        doc="Set timestep and user forces"),

    Cmd(8, "GetSnapshot", "ext_getSnapshot", "ext_getSnapshot",
        req=None, resp="ext_snapshotData",
        doc="Get simulation's time, model's state and tracking error"),

    Cmd(9, "Run", "ext_run", "ext_run",
        req=None, resp="bool",
        doc="Run simulation / plant ticking"),

    Cmd(10, "Stop", "ext_stop", "ext_stop",
        req=None, resp="bool",
        doc="Stop simulation / plant ticking"),

    Cmd(11, "GetPlantSnapshot", "ext_getPlantSnapshot", "ext_getPlantSnapshot",
        req=None, resp="ext_plantSnapshotData",
        doc="Get the plant's last sample: plant time, sequence and state"),

    Cmd(12, "BeginStaging", "ext_beginStaging", "ext_beginStaging",
        req=("scalar", "ext_coord_t", "safetyAltitude"), resp="bool",
        doc="Auto-stage the plant to (trajectory vertical range + safetyAltitude, m)",
        log="begin staging"),

    Cmd(13, "StopStaging", "ext_stopStaging", "ext_stopStaging",
        req=None, resp="bool",
        doc="Abort auto-staging (hold in place)",
        log="stop staging"),

    Cmd(14, "InitQuadRotorMPC", "ext_initQuadRotor_MPC01", "ext_quadRotorMpcInit",
        req="ext_initQuadRotorParams", resp="bool",
        doc="Initialize QuadRotor model: MPC_01 (nonlinear MPC), returns true on error",
        log="init quadrotor mpc"),

    # ---- logger / profiler inspection and control ----

    Cmd(15, "GetLogBatch", "ext_getLogBatch", "ext_getLogBatch",
        req=None, resp="ext_logBatch",
        doc="Drain a batch of recent log lines from the UI buffer"),

    Cmd(16, "GetLogModules", "ext_getLogModules", "ext_getLogModules",
        req=None, resp="ext_moduleList",
        doc="List the registered log modules with their current level"),

    Cmd(17, "SetLogLevel", "ext_setLogLevel", "ext_setLogLevel",
        req="ext_logLevelParams", resp="bool",
        doc="Set a log module's runtime level, returns true on error"),

    Cmd(18, "GetProfileModules", "ext_getProfileModules", "ext_getProfileModules",
        req=None, resp="ext_moduleList",
        doc="List the registered profiler modules with their enabled flag"),

    Cmd(19, "SetProfileEnabled", "ext_setProfileEnabled", "ext_setProfileEnabled",
        req="ext_profileEnableParams", resp="bool",
        doc="Enable or disable profiling for a module, returns true on error"),

    Cmd(20, "GetProfileTable", "ext_getProfileTable", "ext_getProfileTable",
        req=None, resp="ext_profileTable",
        doc="Get the profiler stats table from the latest published snapshot"),

    Cmd(21, "ResetProfile", "ext_resetProfile", "ext_resetProfile",
        req=None, resp="bool",
        doc="Reset all profiler statistics (clears cold-start outliers), returns true on error"),

    Cmd(22, "SetDiagFiles", "ext_setDiagFiles", "ext_setDiagFiles",
        req="ext_diagFiles", resp="bool",
        doc="Toggle server-side log-to-file and raw-profiler-CSV, returns true on error"),

    # ---- per-tick data recorder (black-box wide CSV of the active model) ----

    Cmd(23, "SetRecording", "ext_setRecording", "ext_setRecording",
        req="ext_recordParams", resp="ext_recordStatus",
        doc="Toggle the per-tick data recorder; returns the recorder status",
        log="set recording"),

    Cmd(24, "GetRecordStatus", "ext_getRecordStatus", "ext_getRecordStatus",
        req=None, resp="ext_recordStatus",
        doc="Get the data recorder status (active model, enabled flag, dropped rows)"),
]
