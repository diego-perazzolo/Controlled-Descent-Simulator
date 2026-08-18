"""base_codegen.py — shared C++ generator base for the CDS models.

Vehicle-agnostic: config, symbol bindings, param access (symbol-name -> field),
LQR data (constant A_e/B_e/Q/R + default gain, runtime-settable via SetGain),
CSE, header/source scaffolding, write. RocketCodegen and QuadCodegen
derive from BaseCodegen and override only the two vehicle-specific emitters
(_emit_dynamics_body / _emit_execute_control_body) and their feedforward interface.

Author: Diego Perazzolo, 2026.
"""
from __future__ import annotations
import os, re
from dataclasses import dataclass, field
import sympy as sp


def _ccode(expr, prec: int = 17) -> str:
    return sp.ccode(sp.N(sp.sympify(expr), prec))


@dataclass
class CodegenConfig:
    parent_namespace: str = "CDS::Dynamics"
    model_name: str = "QUADROTOR_FF_LQR_01"
    out_dir: str = "exported_cpp"          # base; files go to <out_dir>/<model_name>/
    state_dim: int = 13
    aug_dim: int = 17          # runtime StateVec length
    error_dim: int = 16        # LQR error length (K_e columns)
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
        base = self.out_dir
        if not os.path.isabs(base):                          # anchor a relative out_dir at the
            base = os.path.join(os.path.dirname(os.path.abspath(__file__)), base)  # notebooks root
        return os.path.join(base, self.model_name)           # (not the cwd: notebooks live in model/)


class BaseCodegen:
    def __init__(self, cfg: CodegenConfig):
        self.cfg = cfg
        self._rhs = None; self._K_e = None
        self._A_e = None; self._B_e = None; self._Q_lqr = None; self._R_lqr = None
        self._state_syms = None; self._input_syms = None; self._phys_syms = None
        self._uforce_syms = []

    # ---- bindings ----
    def set_state_symbols(self, syms):  self._state_syms = list(syms); return self
    def set_input_symbols(self, syms):  self._input_syms = list(syms); return self
    def set_physics_symbols(self, syms):
        assert len(syms) == len(self.cfg.param_field_names)
        self._phys_syms = list(syms); return self
    def set_user_force_symbols(self, syms):
        """External perturbation forces (from userF[...]); stored for emitters that use them."""
        self._uforce_syms = list(syms); return self

    def set_dynamics(self, rhs):        self._rhs = sp.Matrix(rhs); return self
    def set_lqr_gain(self, K):
        import numpy as np
        K = np.asarray(K, float)
        assert K.shape == (self.cfg.input_dim, self.cfg.error_dim), K.shape
        self._K_e = K; return self

    def set_error_dynamics(self, A_e, B_e, Q, R):
        """Numeric error dynamics (A_e, B_e) and LQR weights (Q, R) at the nominal
        operating point. Emitted as constants so the runtime can re-synthesise the
        gain (CDS::control::lqr) -- e.g. after retuning Q, R -- without recomputing
        the linearisation, which stays frozen. K from set_lqr_gain is the default/
        reference gain (synthesised from these in the notebook)."""
        import numpy as np
        c = self.cfg
        self._A_e = np.asarray(A_e, float); self._B_e = np.asarray(B_e, float)
        self._Q_lqr = np.asarray(Q, float); self._R_lqr = np.asarray(R, float)
        assert self._A_e.shape == (c.error_dim, c.error_dim), self._A_e.shape
        assert self._B_e.shape == (c.error_dim, c.input_dim), self._B_e.shape
        assert self._Q_lqr.shape == (c.error_dim, c.error_dim), self._Q_lqr.shape
        assert self._R_lqr.shape == (c.input_dim, c.input_dim), self._R_lqr.shape
        return self

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

    def _replace_param_access(self, c_text, prefix="m_p."):
        """Regex form of _cc for rendered C strings: physics symbol name -> m_p.<field>."""
        for sym, fld in zip(self._phys_syms or [], self.cfg.param_field_names):
            c_text = re.sub(r"\b" + re.escape(str(sym)) + r"\b", prefix + fld, c_text)
        return c_text

    def _idx(self, k):
        return "StateToIdx(StateName::" + self.cfg.state_enum_names[k] + ")"

    def _emit_lqr_data_defs(self):
        """Out-of-line definitions of the constant LQR data: the frozen error
        dynamics A_e, B_e, the default weights Q_default, R_default, and the
        default/reference gain K_default (all static members of the model class)."""
        c = self.cfg; cls = c.model_name
        def mat(name, M, rows, cols):
            body = ",\n".join("    {" + ", ".join(sp.ccode(sp.N(M[i, j], 17))
                              for j in range(cols)) + "}" for i in range(rows))
            return f"const double {cls}::{name}[{rows}][{cols}] = {{\n{body}\n}};"
        E, I = c.error_dim, c.input_dim
        return "\n\n".join([
            mat("A_e",       self._A_e,   E, E),
            mat("B_e",       self._B_e,   E, I),
            mat("Q_default", self._Q_lqr, E, E),
            mat("R_default", self._R_lqr, I, I),
            mat("K_default", self._K_e,   I, E),
        ])

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

{ind}// LQR error dynamics (A_e, B_e) and default weights (Q, R) at the nominal
{ind}// operating point, plus the default/reference gain -- all constant, emitted
{ind}// by codegen. The runtime re-synthesises the gain from these (CDS::control::lqr,
{ind}// e.g. after retuning Q, R) and installs it with SetGain; A_e, B_e stay fixed.
{ind}static const double A_e[{c.error_dim}][{c.error_dim}];
{ind}static const double B_e[{c.error_dim}][{c.input_dim}];
{ind}static const double Q_default[{c.error_dim}][{c.error_dim}];
{ind}static const double R_default[{c.input_dim}][{c.input_dim}];
{ind}static const double K_default[{c.input_dim}][{c.error_dim}];
{ind}void SetGain(const double (&K)[{c.input_dim}][{c.error_dim}]);

private:
{ind}struct PhysicsParams {{
{pstruct}
{ind}}};
{ind}PhysicsParams m_p;
{ind}double m_K_e[{c.input_dim}][{c.error_dim}];   // active LQR gain (default K_default; SetGain overrides)
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

{self._emit_lqr_data_defs()}

{cls}::{cls}()
{{
{ind}for (std::size_t i = 0; i < {c.input_dim}; ++i)
{ind}{ind}for (std::size_t j = 0; j < {c.error_dim}; ++j) m_K_e[i][j] = K_default[i][j];
}}

void {cls}::SetGain(const double (&K)[{c.input_dim}][{c.error_dim}])
{{
{ind}for (std::size_t i = 0; i < {c.input_dim}; ++i)
{ind}{ind}for (std::size_t j = 0; j < {c.error_dim}; ++j) m_K_e[i][j] = K[i][j];
}}

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
