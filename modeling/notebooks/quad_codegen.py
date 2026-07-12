"""quad_codegen.py — QuadRotor C++ generator, built on the shared BaseCodegen.

BaseCodegen holds everything generic (config, symbol bindings, param access,
K_e literal, header/source scaffolding, write). RocketCodegen and QuadCodegen
override only the two vehicle-specific emitters + their feedforward interface.

Author: Diego Perazzolo, 2026.
"""
from __future__ import annotations
import os, re
from dataclasses import dataclass, field
import sympy as sp


@dataclass
class CodegenConfig:
    parent_namespace: str = "CDS::Dynamics"
    model_name: str = "QUADROTOR_FF_LQR_01"
    out_dir: str = "exported_cpp"          # base; files go to <out_dir>/<model_name>/
    state_dim: int = 13
    aug_dim: int = 17          # runtime StateVec length
    error_dim: int = 16        # LQR error length (K_e columns)  <-- NEW vs rocket
    input_dim: int = 4
    indent: str = "    "
    author: str = "Diego Perazzolo"
    notebook_name: str = "dynamics_quadRotor_FFLQR01.ipynb"
    reference_type: str = "Reference_t"
    user_forces_type: str = "std::array<double, 3>"
    core_defs_header: str = "core_defs.hpp"
    test_core_defs_header: str = "test_core_defs.hpp"
    state_enum_names: tuple = ("X","Y","Z","Qw","Qx","Qy","Qz","VX","VY","VZ",
                               "WX","WY","WZ","IntX","IntY","IntZ","IntPsi")
    param_enum_names: tuple = ("Mass","Ix","Iy","Iz","Gravity","DragLat","DragAx",
                               "KThrust","KTorque","Arm","IRotor","ThrustMax","ThrustMin")
    param_field_names: tuple = ("m","Ix","Iy","Iz","g","c","cz","kT","kQ","L","Irot","T_max","T_min")
    param_field_comments: tuple = ("vehicle mass [kg]","inertia x [kg m^2]","inertia y [kg m^2]",
        "inertia z [kg m^2]","gravity [m/s^2]","lateral drag","axial drag","thrust coeff kT",
        "torque coeff kQ","arm length [m]","rotor inertia","per-rotor thrust max [N]","per-rotor thrust min [N]")
    param_default_values: tuple = (2.4,0.025,0.025,0.045,9.81,0.20,0.30,1e-5,1.6e-7,0.275,3e-5,36.0,0.0)

    @property
    def module_name(self):
        return "dynamics_" + self.model_name.lower()   # dynamics_quadrotor_ff_lqr_01

    @property
    def export_dir(self):
        return os.path.join(self.out_dir, self.model_name)   # exported_cpp/QUADROTOR_FF_LQR_01/


class BaseCodegen:
    def __init__(self, cfg: CodegenConfig):
        self.cfg = cfg
        self._rhs = None; self._K_e = None
        self._state_syms = None; self._input_syms = None; self._phys_syms = None

    # ---- bindings ----
    def set_state_symbols(self, syms):  self._state_syms = list(syms); return self
    def set_input_symbols(self, syms):  self._input_syms = list(syms); return self
    def set_physics_symbols(self, syms):
        assert len(syms) == len(self.cfg.param_field_names)
        self._phys_syms = list(syms); return self
    def set_dynamics(self, rhs):        self._rhs = sp.Matrix(rhs); return self
    def set_lqr_gain(self, K):
        import numpy as np
        K = np.asarray(K, float)
        assert K.shape == (self.cfg.input_dim, self.cfg.error_dim), K.shape
        self._K_e = K; return self

    # ---- helpers ----
    def _param_names(self):
        return {str(s) for s in (self._phys_syms or [])}

    def _cc(self, expr):
        """ccode with physics symbols -> m_p.<field>, mapping by symbol NAME to the
        corresponding param_field_names entry (robust to symbol assumptions and to
        name != field, e.g. 'I_xx' -> field 'Ixx')."""
        e = sp.sympify(expr)
        name2field = {str(sym): fld for sym, fld in zip(self._phys_syms or [], self.cfg.param_field_names)}
        reps = {s: sp.Symbol("MPPARAM_" + name2field[str(s)]) for s in e.free_symbols if str(s) in name2field}
        code = sp.ccode(sp.N(e.xreplace(reps), 17))
        return re.sub(r'\bMPPARAM_(\w+)\b', r'm_p.\1', code)

    def _emit_cse(self, exprs, targets):
        """Common-subexpression elimination: shared terms are computed once into
        `const double` temporaries, so the generated math is fast to evaluate.
        `targets` are C++ lvalue strings (e.g. 'dxdt[0]' or 'const double F_ff')."""
        repl, reduced = sp.cse(list(exprs), symbols=sp.numbered_symbols("cse"),
                               optimizations="basic")
        lines = [f"    const double {sym} = {self._cc(sub)};" for sym, sub in repl]
        lines += [f"    {tgt} = {self._cc(red)};" for tgt, red in zip(targets, reduced)]
        return lines

    def _emit_K_e_literal(self):
        c = self.cfg
        rows = ["        {" + ", ".join(sp.ccode(sp.N(self._K_e[i, j], 17))
                for j in range(c.error_dim)) + "}" for i in range(c.input_dim)]
        return (f"const double K_e[{c.input_dim}][{c.error_dim}] = {{\n"
                + ",\n".join(rows) + "\n    };")

    # ---- vehicle-specific hooks ----
    def _emit_dynamics_body(self):        raise NotImplementedError
    def _emit_execute_control_body(self): raise NotImplementedError
    def _check_ready(self):               raise NotImplementedError

    # ---- header / source / write (generic) ----
    def _emit_header(self):
        c = self.cfg; ind = c.indent
        segs = c.parent_namespace.split("::")
        ns_open = " ".join(f"namespace {s} {{" for s in segs)
        ns_close = "} "*len(segs); ns_close = ns_close.strip()+f"  // namespace {c.parent_namespace}"
        senum = "\n".join(f"{ind}{ind}{n} = {k}," for k, n in enumerate(c.state_enum_names))
        penum = "\n".join(f"{ind}{ind}{n} = {k}," for k, n in enumerate(c.param_enum_names))
        pstruct = "\n".join(f"{ind}{ind}{ind}double {f} = {d};  // {cm}"
                            for f, cm, d in zip(c.param_field_names, c.param_field_comments, c.param_default_values))
        return f'''/* {c.module_name}.hpp -- generated by {c.notebook_name}.
 * Author: {c.author}. Do not edit by hand.
 */
#pragma once
#include <array>
#include <cstddef>
#ifdef JUST_TESTING_DYNAMICS
#include "{c.test_core_defs_header}"
#else
#include "{c.core_defs_header}"
#endif

{ns_open}

class {c.model_name} {{
public:
{ind}using StateVec = std::array<double, {c.aug_dim}>;
{ind}using InputVec = std::array<double, {c.input_dim}>;

{ind}enum class StateName : std::size_t {{
{senum}
{ind}}};
{ind}enum class ParamName : std::size_t {{
{penum}
{ind}}};

{ind}{c.model_name}();
{ind}InputVec ExecuteControl(const StateVec& s, const {c.reference_type}& r) const;
{ind}StateVec Dynamics(const StateVec& s, const InputVec& u,
{ind}                  const {c.reference_type}& ref, const {c.user_forces_type}& userF) const;
{ind}static double GetState(const StateVec& s, StateName n);
{ind}static void   SetState(StateVec& s, StateName n, double v);
{ind}double GetParam(ParamName n) const;
{ind}void   SetParam(ParamName n, double v);
{ind}static constexpr std::size_t StateToIdx(StateName n) noexcept {{ return static_cast<std::size_t>(n); }}

private:
{ind}struct PhysicsParams {{
{pstruct}
{ind}}};
{ind}PhysicsParams m_p;
}};

{ns_close}
'''

    def _emit_source(self):
        c = self.cfg; ind = c.indent; cls = c.model_name
        segs = c.parent_namespace.split("::")
        ns_open = " ".join(f"namespace {s} {{" for s in segs)
        ns_close = "} "*len(segs); ns_close = ns_close.strip()+f"  // namespace {c.parent_namespace}"
        gs = "\n".join(f"{ind}{ind}case StateName::{n}: return s[{k}];" for k, n in enumerate(c.state_enum_names))
        ss = "\n".join(f"{ind}{ind}case StateName::{n}: s[{k}] = v; return;" for k, n in enumerate(c.state_enum_names))
        gp = "\n".join(f"{ind}{ind}case ParamName::{e}: return m_p.{f};" for e, f in zip(c.param_enum_names, c.param_field_names))
        sp_ = "\n".join(f"{ind}{ind}case ParamName::{e}: m_p.{f} = v; return;" for e, f in zip(c.param_enum_names, c.param_field_names))
        return f'''/* {c.module_name}.cpp -- generated by {c.notebook_name}.
 * Author: {c.author}. Do not edit by hand.
 */
#include "{c.module_name}.hpp"
#include <cmath>
#include <algorithm>

{ns_open}

namespace {{
{ind}{self._emit_K_e_literal()}
}} // anonymous namespace

{cls}::{cls}() = default;

double {cls}::GetState(const StateVec& s, StateName n) {{
{ind}switch (n) {{
{gs}
{ind}}}
{ind}return 0.0;
}}
void {cls}::SetState(StateVec& s, StateName n, double v) {{
{ind}switch (n) {{
{ss}
{ind}}}
}}
double {cls}::GetParam(ParamName n) const {{
{ind}switch (n) {{
{gp}
{ind}}}
{ind}return 0.0;
}}
void {cls}::SetParam(ParamName n, double v) {{
{ind}switch (n) {{
{sp_}
{ind}}}
}}

{cls}::StateVec {cls}::Dynamics(const StateVec& s, const InputVec& u,
{' '*(len('StateVec')+len(cls)+len('Dynamics')+4)}const {c.reference_type}& ref, const {c.user_forces_type}& userF) const
{{
{self._emit_dynamics_body()}
}}

{cls}::InputVec {cls}::ExecuteControl(const StateVec& s, const {c.reference_type}& r) const
{{
{self._emit_execute_control_body()}
}}

{ns_close}
'''

    def write(self):
        self._check_ready()
        d = self.cfg.export_dir
        os.makedirs(d, exist_ok=True)
        h = os.path.join(d, f"{self.cfg.module_name}.hpp")
        cpath = os.path.join(d, f"{self.cfg.module_name}.cpp")
        open(h, "w").write(self._emit_header())
        open(cpath, "w").write(self._emit_source())
        return h, cpath


class QuadCodegen(BaseCodegen):
    """Quaternion 6-DOF + flatness FF + LQR, InputVec = [T1..T4]."""
    def __init__(self, cfg=None):
        super().__init__(cfg or CodegenConfig())
        self._ff = None; self._Minv = None; self._uforce_syms = []

    def set_feedforward_flat(self, F_ff, R_ref, omega_ff, tau_ff, M_inv):
        self._ff = dict(F=F_ff, R=sp.Matrix(R_ref), om=sp.Matrix(omega_ff), tau=sp.Matrix(tau_ff))
        self._Minv = sp.Matrix(M_inv); return self

    def set_user_force_symbols(self, syms):
        """External perturbation forces [Fx, Fy, Fz] (from userF[0..2]), applied in Dynamics."""
        self._uforce_syms = list(syms); return self

    def _check_ready(self):
        assert self._rhs is not None and self._K_e is not None and self._ff is not None

    def _emit_dynamics_body(self):
        SN = [str(s) for s in self._state_syms]; IN = [str(s) for s in self._input_syms]
        L = [f"    const double {n} = s[{i}];" for i, n in enumerate(SN)]
        L += [f"    const double {n} = u[{i}];" for i, n in enumerate(IN)]
        L += ["    const double refx = ref.pos[0];", "    const double refy = ref.pos[1];",
              "    const double refz = ref.pos[2];", "    const double ref_yaw = ref.yaw;"]
        if self._uforce_syms:
            L += [f"    const double {str(s)} = userF[{i}];" for i, s in enumerate(self._uforce_syms)]
        else:
            L += ["    (void)userF;"]
        L += ["", "    StateVec dxdt{};"]
        L += self._emit_cse([self._rhs[i] for i in range(self.cfg.aug_dim)],
                            [f"dxdt[{i}]" for i in range(self.cfg.aug_dim)])
        L += ["    return dxdt;"]
        return "\n".join(L)

    def _emit_execute_control_body(self):
        cc = self._cc; ff = self._ff; Mi = self._Minv
        L = ["    const double a_x=r.acc[0], a_y=r.acc[1], a_z=r.acc[2];",
             "    const double j_x=r.jerk[0], j_y=r.jerk[1], j_z=r.jerk[2];",
             "    const double s_x=r.snap[0], s_y=r.snap[1], s_z=r.snap[2];",
             "    const double psi=r.yaw, psi_dot=r.yawRate, psi_ddot=r.yawAcc;", ""]
        # feedforward with CSE: F_ff, R_ref (9), omega_ref (3), tau_ff (3)
        ff_exprs   = ([ff['F']] + [ff['R'][i,j] for i in range(3) for j in range(3)]
                      + [ff['om'][k] for k in range(3)] + [ff['tau'][k] for k in range(3)])
        ff_targets = (["const double F_ff"]
                      + [f"const double Rr{i}{j}" for i in range(3) for j in range(3)]
                      + [f"const double wref_{a}" for a in "xyz"]
                      + [f"const double tff_{a}"  for a in "xyz"])
        L += self._emit_cse(ff_exprs, ff_targets)
        L += ["",
              "    const double qw=s[3], qx=s[4], qy=s[5], qz=s[6];",
              "    const double Rc00=1-2*(qy*qy+qz*qz), Rc01=2*(qx*qy-qw*qz), Rc02=2*(qx*qz+qw*qy);",
              "    const double Rc10=2*(qx*qy+qw*qz), Rc11=1-2*(qx*qx+qz*qz), Rc12=2*(qy*qz-qw*qx);",
              "    const double Rc20=2*(qx*qz-qw*qy), Rc21=2*(qy*qz+qw*qx), Rc22=1-2*(qx*qx+qy*qy);", ""]
        for i in range(3):
            for j in range(3):
                L.append(f"    const double Re{i}{j} = Rr0{i}*Rc0{j} + Rr1{i}*Rc1{j} + Rr2{i}*Rc2{j};")
        L += ["",
              "    // dtheta = rotvec(R_ref^T R)  (exact; small-angle safe)",
              "    const double cth = 0.5*(Re00+Re11+Re22-1.0);",
              "    const double ex=Re21-Re12, ey=Re02-Re20, ez=Re10-Re01;",
              "    const double s2 = std::sqrt(ex*ex+ey*ey+ez*ez);",
              "    const double th = std::atan2(0.5*s2, cth);",
              "    const double fac = (s2>1e-9)? th/s2 : 0.5;",
              "    const double dth_x=fac*ex, dth_y=fac*ey, dth_z=fac*ez;", "",
              "    // Yaw-frame compensation: the LQR gain was synthesized at yaw=0 (body-x = world-x).",
              "    // At heading psi the horizontal position/velocity coupling is rotated by psi, so we",
              "    // express the horizontal position, velocity and integral errors in the heading frame",
              "    // (rotate by Rz(-psi)) before applying K. Without this the x/y loop slowly diverges as",
              "    // psi grows (unstable beyond ~0.5 rad).",
              "    const double cpsi = std::cos(psi), spsi = std::sin(psi);",
              "    const double drx = s[0]-r.pos[0], dry = s[1]-r.pos[1];",
              "    const double dvx = s[7]-r.vel[0], dvy = s[8]-r.vel[1];",
              "    const double iix = s[13],          iiy = s[14];", "",
              "    double err[16];",
              "    err[0]= cpsi*drx + spsi*dry;  err[1]=-spsi*drx + cpsi*dry;  err[2]=s[2]-r.pos[2];",
              "    err[3]=dth_x; err[4]=dth_y; err[5]=dth_z;",
              "    err[6]= cpsi*dvx + spsi*dvy;  err[7]=-spsi*dvx + cpsi*dvy;  err[8]=s[9]-r.vel[2];",
              "    err[9]=s[10]-wref_x; err[10]=s[11]-wref_y; err[11]=s[12]-wref_z;",
              "    err[12]= cpsi*iix + spsi*iiy; err[13]=-spsi*iix + cpsi*iiy; err[14]=s[15]; err[15]=s[16];", ""]
        for i, n in enumerate(["F","tx","ty","tz"]):
            L.append(f"    const double u_{n} = -({'+'.join(f'K_e[{i}][{j}]*err[{j}]' for j in range(16))});")
        L += ["    const double Fc  = F_ff  + u_F;",
              "    const double tcx = tff_x + u_tx;",
              "    const double tcy = tff_y + u_ty;",
              "    const double tcz = tff_z + u_tz;", ""]
        virt = ["Fc","tcx","tcy","tcz"]
        for i in range(4):
            L.append(f"    double T{i+1} = " + " + ".join(f"({cc(Mi[i,j])})*{virt[j]}" for j in range(4)) + ";")
        L += ["", "    InputVec Tcmd{};"]
        for i in range(4):
            L.append(f"    T{i+1}=std::min(std::max(T{i+1},m_p.T_min),m_p.T_max); Tcmd[{i}]=T{i+1};")
        L += ["    return Tcmd;"]
        return "\n".join(L)


class RocketCodegen(BaseCodegen):
    """Euler-angle rocket. Port the two emitters + FF setters from the original
    DescentCodegen verbatim; only the shared machinery moves to BaseCodegen."""
    pass
