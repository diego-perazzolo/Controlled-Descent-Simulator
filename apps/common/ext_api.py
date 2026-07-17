# =============================================================================
# Controlled Descent Simulator
# =============================================================================
# File        : ext_api.py
# Description : Declarative description of the external communication API —
#               the single source of truth for the ext boundary. gen_ext.py
#               reads this file and generates the struct definitions, the API
#               contract, the embind bindings and the whole ws-served wire
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
                 post=None, blank_before=False):
        self.name = name
        self.type = type
        self.js = js if js is not None else name
        self.doc = doc
        self.pre = pre
        self.post = post
        self.blank_before = blank_before


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
        'typedef float ext_coord_t; // type for running coordinartes: position, pose, force'),

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

    Struct("ext_stepParams", file="comm",
           doc="struct of the arguments of step function",
           fields=[
               F("timeStep_s"),
               F("userForce", type="ext_userForce"),
           ]),

    Struct("ext_stepRet", file="comm",
           doc="struct of the return data of the step function",
           fields=[
               F("isError", type="bool"),
               F("state", type="ext_fullState"),
               F("err", type="ext_setpointError"),
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

    Cmd(3, "Step", "ext_step", "ext_step",
        req="ext_stepParams", resp="ext_stepRet",
        doc="Advance one integration step"),

    Cmd(4, "TrajGetPoint", "ext_trajectory_get_point", "ext_trajectory_get_point",
        req=("scalar", "ext_coord_t", "t"), resp="ext_trajectoryPoint",
        doc="Get a point at time instant t along the trajectory"),

    Cmd(5, "TrajAppendPoly4", "ext_trajectory_append_poly4", "ext_trajectory_append_poly4",
        req="ext_trajectoryPoly4Params_t", resp="bool",
        doc="Add a trajectory Polynomial 4th order, returns true on error"),

    Cmd(6, "TrajAppendPoint", "ext_trajectory_append_point", "ext_trajectory_append_point",
        req="ext_trajectoryPointParams_t", resp="bool",
        doc="Add a trajectory Point, returns true on error"),

    Cmd(7, "TrajRemoveLast", "ext_trajectory_remove_last_item", "ext_trajectory_remove_last_item",
        req=None, resp="bool",
        doc="Remove last trajectory item, returns true on error"),
]
