"""
descent_codegen.py — readable C++ code generator for the controlled-descent simulator.

Emits a single hand-written-style C++ class, in the form:

    namespace <parent_namespace> {
    class <model_name> {
    public:
        using StateVec = std::array<double, NX>;
        using InputVec = std::array<double, NU>;

        enum class StateName { ... };
        enum class ParamName { ... };

        <model_name>();

        InputVec ExecuteControl(const StateVec& s, const Reference& r) const;
        StateVec Dynamics(const StateVec& s, const InputVec& u,
                          const RefVec& ref_pos, const UserForces& userF) const;

        static double GetState(const StateVec& s, StateName n);
        static void   SetState(StateVec& s, StateName n, double v);
        double GetParam(ParamName n) const;
        void   SetParam(ParamName n, double v);

    private:
        struct PhysicsParams { ... };  // private: not exposed
        PhysicsParams m_p;
        // IDX_*, K_e, FF math stay private to the .cpp
    };
    } // namespace

The mathematical content (Dynamics body, FF expressions, K_e values) is generated
from SymPy expressions. Symbol names are aliased to short C++ locals (sa, ca, ...)
for readability, and identities like sin^2 + cos^2 = 1 are simplified before
printing via trigsimp + expand_trig.

Author: Diego Perazzolo, 2026.
"""
from __future__ import annotations

import os
import re
from dataclasses import dataclass, field
from typing import Iterable

import sympy as sp


# =============================================================================
# Helpers
# =============================================================================
def _horner(expr: sp.Expr, var: sp.Symbol, prec: int = 18) -> str:
    """Polynomial in `var` rendered in Horner form. Falls back to ccode."""
    expr = sp.expand(expr)
    poly = expr.as_poly(var)
    if poly is None:
        return sp.ccode(sp.N(expr, prec))
    coeffs = [sp.N(c, prec) for c in poly.all_coeffs()]
    if len(coeffs) == 1:
        return sp.ccode(coeffs[0])
    out = sp.ccode(coeffs[0])
    var_name = str(var)
    for c in coeffs[1:]:
        c_str = sp.ccode(c)
        if c_str.startswith("-"):
            out = f"({out})*{var_name} - {c_str[1:]}"
        else:
            out = f"({out})*{var_name} + {c_str}"
    return out


def _ccode(expr: sp.Expr, prec: int = 18) -> str:
    return sp.ccode(sp.N(expr, prec))


def _last_namespace_segment(parent_ns: str) -> str:
    """For 'CDS::Rocket' returns 'Rocket'. For 'CDS' returns 'CDS'."""
    return parent_ns.split("::")[-1]


def _derive_module_name(parent_ns: str, model_name: str) -> str:
    """'CDS::Rocket' + 'FF_LQR_01' -> 'rocket_ff_lqr_01'."""
    return f"{_last_namespace_segment(parent_ns)}_{model_name}".lower()


# =============================================================================
# Configuration
# =============================================================================
@dataclass
class CodegenConfig:
    """Configuration knobs for the C++ generator."""

    # Identity
    parent_namespace: str = "CDS::Rocket"
    model_name: str = "FF_LQR_01"
    out_dir: str = "exported_c"

    # Dimensions
    state_dim: int = 12
    aug_dim: int = 15
    input_dim: int = 4
    user_force_dim: int = 3

    # Style / metadata
    indent: str = "    "
    author: str = "Diego Perazzolo"
    notebook_name: str = "01_model_derivation.ipynb"

    # State-name enumerators (must match the order of state_syms).
    state_enum_names: tuple[str, ...] = (
        "X", "Y", "Z",
        "Alpha", "Beta", "Psi",
        "XDot", "YDot", "ZDot",
        "AlphaDot", "BetaDot", "PsiDot",
        "IntX", "IntY", "IntZ",
    )

    # Param-name enumerators (and corresponding internal struct field names).
    # Order must match `physics_param_symbols` passed to set_physics_symbols.
    param_enum_names: tuple[str, ...] = (
        "Mass", "Ix", "Iy", "Iz", "Gravity", "DragLateral", "DragAxial", "ThrustMax",
    )
    param_field_names: tuple[str, ...] = (
        "m", "Ix", "Iy", "Iz", "g", "c", "cz", "F1_max",
    )
    param_field_comments: tuple[str, ...] = (
        "vehicle mass [kg]",
        "inertia around body x [kg m^2]",
        "inertia around body y [kg m^2]",
        "inertia around body z [kg m^2]",
        "gravity [m/s^2]",
        "lateral drag coeff (body x, y) [N s/m]",
        "axial drag coeff (body z) [N s/m]",
        "thrust upper saturation [N]",
    )
    param_default_values: tuple[float, ...] = (
        10.0, 10.0/3.0, 10.0/3.0, 1.0, 9.81, 1.0, 0.02, 700.0,
    )

    # Lingua-franca types from the project header (declared elsewhere).
    reference_type: str = "Reference"
    refvec_type: str = "RefVec"
    user_forces_type: str = "UserForces"
    shared_header: str = "core_defs.hpp"

    @property
    def module_name(self) -> str:
        """Lowercase basename for header/source files (e.g. 'rocket_ff_lqr_01')."""
        return _derive_module_name(self.parent_namespace, self.model_name)


# =============================================================================
# Generator
# =============================================================================
class DescentCodegen:
    """Builds the .hpp/.cpp pair for a single model+controller class."""

    def __init__(self, config: CodegenConfig):
        self.cfg = config
        if len(config.state_enum_names) != config.aug_dim:
            raise ValueError(
                f"state_enum_names has {len(config.state_enum_names)} entries, "
                f"expected aug_dim={config.aug_dim}"
            )
        if not (
            len(config.param_enum_names)
            == len(config.param_field_names)
            == len(config.param_field_comments)
            == len(config.param_default_values)
        ):
            raise ValueError("Param enum/field/comment/default arrays must have the same length")

        self._state_syms: list[sp.Symbol] | None = None
        self._input_syms: list[sp.Symbol] | None = None
        self._physics_syms: list[sp.Symbol] | None = None
        self._userF_syms: list[sp.Symbol] | None = None

        self._dynamics_rhs: sp.Matrix | None = None
        self._ff_kin: dict | None = None
        self._ff_torque: dict | None = None
        self._K_e = None

    # ---------------- symbol bindings ----------------
    def set_state_symbols(self, syms: Iterable[sp.Symbol]) -> None:
        syms = list(syms)
        if len(syms) != self.cfg.state_dim:
            raise ValueError(f"Expected {self.cfg.state_dim} state symbols, got {len(syms)}")
        self._state_syms = syms

    def set_input_symbols(self, syms: Iterable[sp.Symbol]) -> None:
        syms = list(syms)
        if len(syms) != self.cfg.input_dim:
            raise ValueError(f"Expected {self.cfg.input_dim} input symbols, got {len(syms)}")
        self._input_syms = syms

    def set_physics_symbols(self, syms: Iterable[sp.Symbol]) -> None:
        syms = list(syms)
        if len(syms) != len(self.cfg.param_field_names):
            raise ValueError(
                f"Expected {len(self.cfg.param_field_names)} physics symbols, got {len(syms)}"
            )
        self._physics_syms = syms

    def set_user_force_symbols(self, syms: Iterable[sp.Symbol]) -> None:
        syms = list(syms)
        if len(syms) != self.cfg.user_force_dim:
            raise ValueError(
                f"Expected {self.cfg.user_force_dim} user-force symbols, got {len(syms)}"
            )
        self._userF_syms = syms

    # ---------------- math content ----------------
    def set_dynamics(self, state_rhs: sp.Matrix) -> None:
        if state_rhs.shape != (self.cfg.state_dim, 1):
            raise ValueError(f"state_rhs must be {self.cfg.state_dim}x1")
        self._dynamics_rhs = state_rhs

    def set_feedforward_kinematic(
        self,
        F1_ff: sp.Expr, alpha_ff: sp.Expr, beta_ff: sp.Expr,
        ref_acc_syms: tuple[sp.Symbol, sp.Symbol, sp.Symbol],
    ) -> None:
        self._ff_kin = {
            "F1": F1_ff, "alpha": alpha_ff, "beta": beta_ff,
            "acc_syms": ref_acc_syms,
        }

    def set_feedforward_torque(
        self,
        T1_ff: sp.Expr, T2_ff: sp.Expr,
        ref_acc_syms: tuple[sp.Symbol, sp.Symbol, sp.Symbol],
        ref_jerk_syms: tuple[sp.Symbol, sp.Symbol, sp.Symbol],
        ref_snap_syms: tuple[sp.Symbol, sp.Symbol, sp.Symbol],
    ) -> None:
        self._ff_torque = {
            "T1": T1_ff, "T2": T2_ff,
            "acc_syms": ref_acc_syms,
            "jerk_syms": ref_jerk_syms,
            "snap_syms": ref_snap_syms,
        }

    def set_lqr_gain(self, K_e) -> None:
        if K_e.shape != (self.cfg.input_dim, self.cfg.aug_dim):
            raise ValueError(f"K_e shape {K_e.shape}")
        self._K_e = K_e

    # ---------------- emission helpers ----------------
    def _replace_param_access(self, c_text: str, prefix: str = "m_p.") -> str:
        if not self._physics_syms:
            return c_text
        for sym, field_name in zip(self._physics_syms, self.cfg.param_field_names):
            c_text = re.sub(rf"\b{re.escape(str(sym))}\b", f"{prefix}{field_name}", c_text)
        return c_text

    # ---------------- Dynamics body ----------------
    def _idx(self, k: int) -> str:
        """Short form: SN(StateName::X) using the local aliases set up in each body."""
        return f"SN(StateName::{self.cfg.state_enum_names[k]})"

    def _emit_dynamics_body(self) -> str:
        assert self._dynamics_rhs is not None
        assert self._state_syms and self._input_syms

        ind = self.cfg.indent
        alpha_sym, beta_sym, psi_sym = self._state_syms[3], self._state_syms[4], self._state_syms[5]

        alpha_s, beta_s, psi_s = sp.symbols('alpha_ beta_ psi_', real=True)
        func_to_sym = {alpha_sym: alpha_s, beta_sym: beta_s, psi_sym: psi_s}

        ca, sa = sp.symbols("ca sa")
        cb, sb = sp.symbols("cb sb")
        cp_, sp_ = sp.symbols("cp_ sp_")
        trig_subs = {
            sp.cos(alpha_s): ca, sp.sin(alpha_s): sa,
            sp.cos(beta_s):  cb, sp.sin(beta_s):  sb,
            sp.cos(psi_s):   cp_, sp.sin(psi_s):  sp_,
        }

        xd_, yd_, zd_ = sp.symbols("xd yd zd")
        F1_, T1_, T2_, T3_ = sp.symbols("F1 T1 T2 T3")
        vel_subs = {
            self._state_syms[6]: xd_,
            self._state_syms[7]: yd_,
            self._state_syms[8]: zd_,
        }
        ctl_subs = {
            self._input_syms[0]: F1_,
            self._input_syms[1]: T1_,
            self._input_syms[2]: T2_,
            self._input_syms[3]: T3_,
        }

        def render_acc(expr: sp.Expr) -> str:
            e = expr.subs(func_to_sym)
            e = sp.trigsimp(e)
            e = sp.expand_trig(e)
            e = e.subs(trig_subs).subs(vel_subs).subs(ctl_subs)
            e = sp.expand(e)
            e = sp.collect(e, [F1_, xd_, yd_, zd_])
            c = _ccode(e)
            c = re.sub(r"pow\(([a-zA-Z_]\w*), 2\)", r"\1*\1", c)
            return self._replace_param_access(c)

        # First, render the three translational accelerations into strings.
        # We then inspect them to decide which trig prefetches are actually used,
        # and emit only those.
        acc_strs = {
            "XDot": render_acc(self._dynamics_rhs[6]),
            "YDot": render_acc(self._dynamics_rhs[7]),
            "ZDot": render_acc(self._dynamics_rhs[8]),
        }
        used_blob = " ".join(acc_strs.values())

        def _used(name: str) -> bool:
            # match the alias as a whole word (avoid e.g. 'sa' inside 'sample_x')
            return re.search(rf"\b{name}\b", used_blob) is not None

        L: list[str] = []
        L.append(f"{ind}using SN = StateName;")
        L.append(f"{ind}StateVec dxdt{{}};")
        L.append("")

        # Trig prefetch — emit only the ones that appear in the equations.
        prefetch_lines: list[str] = []
        if _used("sa"): prefetch_lines.append(f"{ind}const double sa  = std::sin(s[StateToIdx(SN::Alpha)]);")
        if _used("ca"): prefetch_lines.append(f"{ind}const double ca  = std::cos(s[StateToIdx(SN::Alpha)]);")
        if _used("sb"): prefetch_lines.append(f"{ind}const double sb  = std::sin(s[StateToIdx(SN::Beta)]);")
        if _used("cb"): prefetch_lines.append(f"{ind}const double cb  = std::cos(s[StateToIdx(SN::Beta)]);")
        if _used("sp_"): prefetch_lines.append(f"{ind}const double sp_ = std::sin(s[StateToIdx(SN::Psi)]);")
        if _used("cp_"): prefetch_lines.append(f"{ind}const double cp_ = std::cos(s[StateToIdx(SN::Psi)]);")
        if prefetch_lines:
            L.append(f"{ind}// Precompute attitude trig (used multiple times below).")
            L.extend(prefetch_lines)
            L.append("")

        # Velocity / input aliases — emit only those used too, for symmetry.
        # In practice xd/yd/zd/F1 always appear; T1/T2/T3 always appear in the
        # rotational block; emit unconditionally for simplicity.
        L.append(f"{ind}// State and input aliases.")
        L.append(f"{ind}const double xd = s[StateToIdx(SN::XDot)];")
        L.append(f"{ind}const double yd = s[StateToIdx(SN::YDot)];")
        L.append(f"{ind}const double zd = s[StateToIdx(SN::ZDot)];")
        L.append(f"{ind}const double F1 = u[0];")
        L.append(f"{ind}const double T1 = u[1];")
        L.append(f"{ind}const double T2 = u[2];")
        L.append(f"{ind}const double T3 = u[3];")
        L.append("")

        L.append(f"{ind}// Kinematics: dp/dt = v, dangles/dt = omega.")
        kin = [("X", "XDot"), ("Y", "YDot"), ("Z", "ZDot"),
               ("Alpha", "AlphaDot"), ("Beta", "BetaDot"), ("Psi", "PsiDot")]
        for lhs, rhs in kin:
            L.append(f"{ind}dxdt[StateToIdx(SN::{lhs})] = s[StateToIdx(SN::{rhs})];")
        L.append("")
        L.append(f"{ind}// Translational dynamics: dv/dt = (F_thrust + F_gravity + F_drag) / m.")
        for name in ("XDot", "YDot", "ZDot"):
            L.append(f"{ind}dxdt[StateToIdx(SN::{name})] = {acc_strs[name]};")
        L.append("")
        L.append(f"{ind}// User-input forces in inertial frame, divided by mass.")
        for k, name in enumerate(("XDot", "YDot", "ZDot")):
            L.append(f"{ind}dxdt[StateToIdx(SN::{name})] += userF[{k}] / m_p.m;")
        L.append("")
        L.append(f"{ind}// Rotational dynamics: I diagonal, torques are direct inputs.")
        L.append(f"{ind}dxdt[StateToIdx(SN::AlphaDot)] = T1 / m_p.Ix;")
        L.append(f"{ind}dxdt[StateToIdx(SN::BetaDot)]  = T2 / m_p.Iy;")
        L.append(f"{ind}dxdt[StateToIdx(SN::PsiDot)]   = T3 / m_p.Iz;")
        L.append("")
        L.append(f"{ind}// Augmented states: integral of position tracking error.")
        L.append(f"{ind}dxdt[StateToIdx(SN::IntX)] = ref_pos[0] - s[StateToIdx(SN::X)];")
        L.append(f"{ind}dxdt[StateToIdx(SN::IntY)] = ref_pos[1] - s[StateToIdx(SN::Y)];")
        L.append(f"{ind}dxdt[StateToIdx(SN::IntZ)] = ref_pos[2] - s[StateToIdx(SN::Z)];")
        L.append("")
        L.append(f"{ind}return dxdt;")
        return "\n".join(L)

    # ---------------- ExecuteControl body ----------------
    def _emit_execute_control_body(self) -> str:
        assert self._ff_kin is not None and self._ff_torque is not None
        ind = self.cfg.indent

        ax_sym, ay_sym, az_sym = self._ff_kin["acc_syms"]
        jx_sym, jy_sym, jz_sym = self._ff_torque["jerk_syms"]
        sx_sym, sy_sym, sz_sym = self._ff_torque["snap_syms"]

        ref_subs = {
            ax_sym: sp.Symbol("ax"), ay_sym: sp.Symbol("ay"), az_sym: sp.Symbol("az"),
            jx_sym: sp.Symbol("jx"), jy_sym: sp.Symbol("jy"), jz_sym: sp.Symbol("jz"),
            sx_sym: sp.Symbol("sx"), sy_sym: sp.Symbol("sy"), sz_sym: sp.Symbol("sz"),
        }

        def render_ff(expr: sp.Expr) -> str:
            e = expr.subs(ref_subs)
            c = _ccode(e)
            # Bare cmath functions -> std:: prefix
            for fn in ("sin", "cos", "tan", "atan2", "sqrt", "pow",
                       "exp", "log", "fabs"):
                c = re.sub(rf"\b{fn}\(", f"std::{fn}(", c)
            # pow(x, 2) -> x*x for readability (only matches simple identifier).
            c = re.sub(r"std::pow\(([a-zA-Z_]\w*), 2\)", r"\1*\1", c)
            return self._replace_param_access(c)

        # Render FF expressions first, then decide which reference derivatives
        # are actually used and only declare those.
        F1_str    = render_ff(self._ff_kin["F1"])
        alpha_str = render_ff(self._ff_kin["alpha"])
        beta_str  = render_ff(self._ff_kin["beta"])
        T1_str    = render_ff(self._ff_torque["T1"])
        T2_str    = render_ff(self._ff_torque["T2"])
        all_str = " ".join([F1_str, alpha_str, beta_str, T1_str, T2_str])

        def _used(name: str) -> bool:
            return re.search(rf"\b{name}\b", all_str) is not None

        L: list[str] = []
        L.append(f"{ind}// Pull reference derivatives into named locals for clarity.")
        # Always-on: acc is the kinematic FF input.
        L.append(f"{ind}const double ax = r.acc[0], ay = r.acc[1], az = r.acc[2];")
        # Jerk and snap only if the torque FF expressions actually need them.
        jerk_used = any(_used(n) for n in ("jx", "jy", "jz"))
        snap_used = any(_used(n) for n in ("sx", "sy", "sz"))
        if jerk_used:
            jline = []
            for n, idx in (("jx", 0), ("jy", 1), ("jz", 2)):
                if _used(n):
                    jline.append(f"{n} = r.jerk[{idx}]")
            L.append(f"{ind}const double {', '.join(jline)};")
        if snap_used:
            sline = []
            for n, idx in (("sx", 0), ("sy", 1), ("sz", 2)):
                if _used(n):
                    sline.append(f"{n} = r.snap[{idx}]")
            L.append(f"{ind}const double {', '.join(sline)};")
        L.append("")

        L.append(f"{ind}// Feedforward thrust magnitude and tilt angles from desired acceleration.")
        L.append(f"{ind}const double F1_ff    = {F1_str};")
        L.append(f"{ind}const double alpha_ff = {alpha_str};")
        L.append(f"{ind}const double beta_ff  = {beta_str};")
        L.append(f"{ind}// alpha_ff/beta_ff are exposed for diagnostic/attitude-FF use; "
                 f"they are not consumed below.")
        L.append(f"{ind}(void)alpha_ff; (void)beta_ff;")
        L.append("")
        L.append(f"{ind}// Torque feedforward: from second derivatives of FF angles "
                 f"(needs jerk and snap of the reference).")
        L.append(f"{ind}const double T1_ff    = {T1_str};")
        L.append(f"{ind}const double T2_ff    = {T2_str};")
        L.append(f"{ind}// Roll FF is zero by design (psi held at zero).")
        L.append(f"{ind}const double T3_ff    = 0.0;")
        L.append("")

        L.append(f"{ind}// LQR correction: u_lqr = -K_e * s.")
        L.append(f"{ind}InputVec u_lqr{{}};")
        L.append(f"{ind}for (size_t i = 0; i < {self.cfg.input_dim}; ++i) {{")
        L.append(f"{ind}{ind}double v = 0.0;")
        L.append(f"{ind}{ind}for (size_t j = 0; j < {self.cfg.aug_dim}; ++j) {{")
        L.append(f"{ind}{ind}{ind}v += K_e[i][j] * s[j];")
        L.append(f"{ind}{ind}}}")
        L.append(f"{ind}{ind}u_lqr[i] = -v;")
        L.append(f"{ind}}}")
        L.append("")
        L.append(f"{ind}// Total control: u = u_ff + u_lqr, then saturate F1.")
        L.append(f"{ind}InputVec u{{}};")
        L.append(f"{ind}u[0] = F1_ff + u_lqr[0];")
        L.append(f"{ind}u[1] = T1_ff + u_lqr[1];")
        L.append(f"{ind}u[2] = T2_ff + u_lqr[2];")
        L.append(f"{ind}u[3] = T3_ff + u_lqr[3];")
        L.append("")
        L.append(f"{ind}if      (u[0] > m_p.F1_max) u[0] = m_p.F1_max;")
        L.append(f"{ind}else if (u[0] < 0.0)        u[0] = 0.0;")
        L.append("")
        L.append(f"{ind}return u;")
        return "\n".join(L)

    # ---------------- K_e literal ----------------
    def _emit_K_e_literal(self) -> str:
        K = self._K_e
        ind = self.cfg.indent
        rows = []
        for i in range(self.cfg.input_dim):
            row = ", ".join(f"{v:.18e}" for v in K[i])
            rows.append(f"{ind}{ind}{{{{ {row} }}}}")
        body = ",\n".join(rows)
        return (
            f"static constexpr std::array<std::array<double, "
            f"{self.cfg.aug_dim}>, {self.cfg.input_dim}> K_e = {{{{\n"
            f"{body}\n"
            f"{ind}}}}};"
        )

    # ---------------- header ----------------
    def _emit_header(self) -> str:
        cfg = self.cfg
        ind = cfg.indent

        ns_segs = cfg.parent_namespace.split("::")
        ns_open = " ".join(f"namespace {seg} {{" for seg in ns_segs)
        ns_close = "} " * len(ns_segs)
        ns_close = ns_close.strip() + f"  // namespace {cfg.parent_namespace}"

        state_enum = "\n".join(
            f"{ind}{ind}{name} = {k},"
            for k, name in enumerate(cfg.state_enum_names)
        )
        param_enum = "\n".join(
            f"{ind}{ind}{name} = {k},"
            for k, name in enumerate(cfg.param_enum_names)
        )
        param_struct = "\n".join(
            f"{ind}{ind}{ind}double {fname} = {fdefault};  // {fcomment}"
            for fname, fcomment, fdefault in zip(
                cfg.param_field_names,
                cfg.param_field_comments,
                cfg.param_default_values,
            )
        )

        return f'''/* {cfg.module_name}.hpp -- generated by {cfg.notebook_name}.
 * Author: {cfg.author}. Do not edit by hand.
 */
#pragma once

#include <array>
#include <cstddef>

#include "{cfg.shared_header}"

{ns_open}

class {cfg.model_name} {{
public:
{ind}using StateVec = std::array<double, {cfg.aug_dim}>;
{ind}using InputVec = std::array<double, {cfg.input_dim}>;

{ind}// Public name enums for state and parameter accessors.
{ind}// Internal indices and the parameter struct layout are private to the .cpp.
{ind}enum class StateName : std::size_t {{
{state_enum}
{ind}}};

{ind}enum class ParamName : std::size_t {{
{param_enum}
{ind}}};

{ind}{cfg.model_name}();

{ind}// ----- Control -----
{ind}// u = u_ff(reference) + u_lqr(state), with F1 saturated to [0, F1_max].
{ind}InputVec ExecuteControl(const StateVec& s, const {cfg.reference_type}& r) const;

{ind}// ----- Dynamics -----
{ind}// dxdt = f(s, u, ref_pos, userF). Augmented integrators use ref_pos only.
{ind}StateVec Dynamics(const StateVec& s,
{ind}                  const InputVec& u,
{ind}                  const {cfg.refvec_type}& ref_pos,
{ind}                  const {cfg.user_forces_type}& userF) const;

{ind}// ----- Accessors -----
{ind}static double GetState(const StateVec& s, StateName n);
{ind}static void   SetState(StateVec& s, StateName n, double v);
{ind}double GetParam(ParamName n) const;
{ind}void   SetParam(ParamName n, double v);

{ind}// Convert a StateName to its array index. Use this everywhere instead of
{ind}// raw static_cast<std::size_t> to keep call sites readable.
{ind}static constexpr std::size_t StateToIdx(StateName n) noexcept {{
{ind}{ind}return static_cast<std::size_t>(n);
{ind}}}

private:
{ind}struct PhysicsParams {{
{param_struct}
{ind}}};

{ind}PhysicsParams m_p;
}};

{ns_close}
'''

    # ---------------- source ----------------
    def _emit_source(self) -> str:
        cfg = self.cfg
        ind = cfg.indent

        ns_segs = cfg.parent_namespace.split("::")
        ns_open = " ".join(f"namespace {seg} {{" for seg in ns_segs)
        ns_close = "} " * len(ns_segs)
        ns_close = ns_close.strip() + f"  // namespace {cfg.parent_namespace}"

        cls = cfg.model_name

        get_state_cases = "\n".join(
            f"{ind}{ind}case StateName::{name}: return s[{k}];"
            for k, name in enumerate(cfg.state_enum_names)
        )
        set_state_cases = "\n".join(
            f"{ind}{ind}case StateName::{name}: s[{k}] = v; return;"
            for k, name in enumerate(cfg.state_enum_names)
        )
        get_param_cases = "\n".join(
            f"{ind}{ind}case ParamName::{ename}: return m_p.{fname};"
            for ename, fname in zip(cfg.param_enum_names, cfg.param_field_names)
        )
        set_param_cases = "\n".join(
            f"{ind}{ind}case ParamName::{ename}: m_p.{fname} = v; return;"
            for ename, fname in zip(cfg.param_enum_names, cfg.param_field_names)
        )

        K_e_block = self._emit_K_e_literal()
        dyn_body = self._emit_dynamics_body()
        ctrl_body = self._emit_execute_control_body()

        # Continuation indent so multi-line method-definition parameters align
        # with the first parameter under the opening '('.
        # 'StateVec FF_LQR_01::Dynamics(' has length: len(returntype) + 1 + len(class) + 2 + len(method) + 1
        dyn_indent = " " * (len("StateVec") + 1 + len(cls) + 2 + len("Dynamics") + 1)
        ctrl_indent = " " * (len("InputVec") + 1 + len(cls) + 2 + len("ExecuteControl") + 1)

        return f'''/* {cfg.module_name}.cpp -- generated by {cfg.notebook_name}.
 * Author: {cfg.author}. Do not edit by hand.
 */
#include "{cfg.module_name}.hpp"

#include <cmath>

{ns_open}

namespace {{
{ind}// LQR augmented gain matrix. Row i = gains for input u[i].
{ind}{K_e_block}
}} // anonymous namespace

{cls}::{cls}() = default;

// =============================================================================
// Accessors
// =============================================================================
double {cls}::GetState(const StateVec& s, StateName n) {{
{ind}switch (n) {{
{get_state_cases}
{ind}}}
{ind}return 0.0;
}}

void {cls}::SetState(StateVec& s, StateName n, double v) {{
{ind}switch (n) {{
{set_state_cases}
{ind}}}
}}

double {cls}::GetParam(ParamName n) const {{
{ind}switch (n) {{
{get_param_cases}
{ind}}}
{ind}return 0.0;
}}

void {cls}::SetParam(ParamName n, double v) {{
{ind}switch (n) {{
{set_param_cases}
{ind}}}
}}

// =============================================================================
// Dynamics: dxdt = f(s, u, ref_pos, userF).
// =============================================================================
{cls}::StateVec {cls}::Dynamics(const StateVec& s,
{dyn_indent}const InputVec& u,
{dyn_indent}const {cfg.refvec_type}& ref_pos,
{dyn_indent}const {cfg.user_forces_type}& userF) const
{{
{dyn_body}
}}

// =============================================================================
// ExecuteControl: u = u_ff(r) + u_lqr(s), F1 saturated to [0, F1_max].
// =============================================================================
{cls}::InputVec {cls}::ExecuteControl(const StateVec& s,
{ctrl_indent}const {cfg.reference_type}& r) const
{{
{ctrl_body}
}}

{ns_close}
'''

    # ---------------- write ----------------
    def write(self) -> tuple[str, str]:
        os.makedirs(self.cfg.out_dir, exist_ok=True)
        h_path = os.path.join(self.cfg.out_dir, f"{self.cfg.module_name}.hpp")
        c_path = os.path.join(self.cfg.out_dir, f"{self.cfg.module_name}.cpp")

        if self._dynamics_rhs is None:
            raise RuntimeError("set_dynamics() not called")
        if self._ff_kin is None:
            raise RuntimeError("set_feedforward_kinematic() not called")
        if self._ff_torque is None:
            raise RuntimeError("set_feedforward_torque() not called")
        if self._K_e is None:
            raise RuntimeError("set_lqr_gain() not called")

        with open(h_path, "w") as f:
            f.write(self._emit_header())
        with open(c_path, "w") as f:
            f.write(self._emit_source())
        return h_path, c_path
