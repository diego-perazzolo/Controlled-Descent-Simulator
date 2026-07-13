"""rocket_codegen.py — Rocket C++ generator (derives from BaseCodegen).

Ported from the original monolithic DescentCodegen: the two vehicle-specific
emitters (Euler-angle dynamics, tilt-angle feedforward) and the tilt-angle
feedforward interface now live here, on top of the shared BaseCodegen.

Author: Diego Perazzolo, 2026.
"""
from __future__ import annotations
import re
import sympy as sp
from base_codegen import BaseCodegen, CodegenConfig, _ccode


def rocket_config(model_name="ROCKET_FF_LQR_01"):
    return CodegenConfig(
        parent_namespace="CDS::Dynamics", model_name=model_name,
        state_dim=12, aug_dim=16, error_dim=16, input_dim=4,
        state_enum_names=("X","Y","Z","Alpha","Beta","Psi","XDot","YDot","ZDot",
                          "AlphaDot","BetaDot","PsiDot","IntX","IntY","IntZ","IntPsi"),
        param_enum_names=("Mass","Ix","Iy","Iz","Gravity","DragLateral","DragAxial",
                          "ThrustMax","ThrustMin","TorqueXMax","TorqueXMin","TorqueYMax","TorqueYMin",
                          "TorqueZMax","TorqueZMin"),
        param_field_names=("m","Ix","Iy","Iz","g","c","cz","F1_max","F1_min","T1_max","T1_min","T2_max","T2_min",
                           "T3_max","T3_min"),
        # NOTE on torque naming: T1 acts about body Y (drives alpha/pitch) and T2
        # about body X (drives beta); enum/field names keep the legacy X/Y labels.
        param_field_comments=("vehicle mass [kg]","inertia around body x [kg m^2]","inertia around body y [kg m^2]",
            "inertia around body z [kg m^2]","gravity [m/s^2]","lateral drag coeff (body x, y) [N s/m]",
            "axial drag coeff (body z) [N s/m]","thrust upper saturation [N]","thrust lower saturation [N]",
            "Torque T1, about body y axis (drives alpha), upper saturation [Nm]",
            "Torque T1, about body y axis (drives alpha), lower saturation [Nm]",
            "Torque T2, about body x axis (drives beta), upper saturation [Nm]",
            "Torque T2, about body x axis (drives beta), lower saturation [Nm]",
            "Torque T3, about body z axis (drives psi), upper saturation [Nm]",
            "Torque T3, about body z axis (drives psi), lower saturation [Nm]"),
        param_default_values=(10.0, 10.0/3.0, 10.0/3.0, 1.0, 9.81, 1.0, 0.02, 500.0, 0.0, 10, -10, 10, -10, 10, -10),
        user_forces_type="Vec3")


class RocketCodegen(BaseCodegen):
    """Euler-angle 6-DOF rocket + tilt-angle flatness FF + LQR, InputVec = [F1, T1, T2, T3]."""

    def __init__(self, cfg=None):
        super().__init__(cfg or rocket_config())
        self._ff_kin = None
        self._ff_torque = None
        self._ff_att_rate = None

    # ---------------- tilt-angle feedforward interface ----------------
    def set_feedforward_kinematic(self, F1_ff, alpha_ff, beta_ff, ref_acc_syms):
        self._ff_kin = {"F1": F1_ff, "alpha": alpha_ff, "beta": beta_ff, "acc_syms": ref_acc_syms}
        return self

    def set_feedforward_torque(self, T1_ff, T2_ff, ref_acc_syms, ref_jerk_syms, ref_snap_syms):
        self._ff_torque = {"T1": T1_ff, "T2": T2_ff, "acc_syms": ref_acc_syms,
                           "jerk_syms": ref_jerk_syms, "snap_syms": ref_snap_syms}
        return self

    def set_feedforward_attitude_rate(self, alpha_ff_dot, beta_ff_dot, ref_acc_syms, ref_jerk_syms):
        self._ff_att_rate = {"alpha_dot": alpha_ff_dot, "beta_dot": beta_ff_dot,
                             "acc_syms": ref_acc_syms, "jerk_syms": ref_jerk_syms}
        return self

    def _check_ready(self):
        assert self._rhs is not None and self._K_e is not None
        assert self._ff_kin and self._ff_torque and self._ff_att_rate, \
            "rocket needs set_feedforward_kinematic / _torque / _attitude_rate"

    # ---------------- Dynamics body ----------------
    def _emit_dynamics_body(self):
        ind = self.cfg.indent
        alpha_sym, beta_sym, psi_sym = self._state_syms[3], self._state_syms[4], self._state_syms[5]
        alpha_s, beta_s, psi_s = sp.symbols('alpha_ beta_ psi_', real=True)
        func_to_sym = {alpha_sym: alpha_s, beta_sym: beta_s, psi_sym: psi_s}
        ca, sa = sp.symbols("ca sa"); cb, sb = sp.symbols("cb sb"); cp_, sp_ = sp.symbols("cp_ sp_")
        trig_subs = {sp.cos(alpha_s): ca, sp.sin(alpha_s): sa, sp.cos(beta_s): cb,
                     sp.sin(beta_s): sb, sp.cos(psi_s): cp_, sp.sin(psi_s): sp_}
        xd_, yd_, zd_ = sp.symbols("xd yd zd"); F1_, T1_, T2_, T3_ = sp.symbols("F1 T1 T2 T3")
        vel_subs = {self._state_syms[6]: xd_, self._state_syms[7]: yd_, self._state_syms[8]: zd_}
        ctl_subs = {self._input_syms[0]: F1_, self._input_syms[1]: T1_,
                    self._input_syms[2]: T2_, self._input_syms[3]: T3_}

        def render_acc(expr):
            e = expr.subs(func_to_sym); e = sp.trigsimp(e); e = sp.expand_trig(e)
            e = e.subs(trig_subs).subs(vel_subs).subs(ctl_subs); e = sp.expand(e)
            e = sp.collect(e, [F1_, xd_, yd_, zd_])
            c = _ccode(e); c = re.sub(r"pow\(([a-zA-Z_]\w*), 2\)", r"\1*\1", c)
            return self._replace_param_access(c)

        acc_strs = {"XDot": render_acc(self._rhs[6]), "YDot": render_acc(self._rhs[7]),
                    "ZDot": render_acc(self._rhs[8])}
        used_blob = " ".join(acc_strs.values())
        def _used(name): return re.search(rf"\b{name}\b", used_blob) is not None

        L = [f"{ind}using SN = StateName;", f"{ind}StateVec dxdt{{}};",
             f"{ind}// Local alias: only ref.pos is used by the dynamics (integrators).",
             f"{ind}const Vec3& ref_pos = ref.pos;", ""]
        pf = []
        if _used("sa"): pf.append(f"{ind}const double sa  = std::sin(s[StateToIdx(SN::Alpha)]);")
        if _used("ca"): pf.append(f"{ind}const double ca  = std::cos(s[StateToIdx(SN::Alpha)]);")
        if _used("sb"): pf.append(f"{ind}const double sb  = std::sin(s[StateToIdx(SN::Beta)]);")
        if _used("cb"): pf.append(f"{ind}const double cb  = std::cos(s[StateToIdx(SN::Beta)]);")
        if _used("sp_"): pf.append(f"{ind}const double sp_ = std::sin(s[StateToIdx(SN::Psi)]);")
        if _used("cp_"): pf.append(f"{ind}const double cp_ = std::cos(s[StateToIdx(SN::Psi)]);")
        if pf:
            L.append(f"{ind}// Precompute attitude trig (used multiple times below).")
            L.extend(pf); L.append("")
        L += [f"{ind}// State and input aliases.",
              f"{ind}const double xd = s[StateToIdx(SN::XDot)];",
              f"{ind}const double yd = s[StateToIdx(SN::YDot)];",
              f"{ind}const double zd = s[StateToIdx(SN::ZDot)];",
              f"{ind}const double F1 = u[0];", f"{ind}const double T1 = u[1];",
              f"{ind}const double T2 = u[2];", f"{ind}const double T3 = u[3];", ""]
        L.append(f"{ind}// Kinematics: dp/dt = v, dangles/dt = omega.")
        for lhs, rhs in [("X","XDot"),("Y","YDot"),("Z","ZDot"),("Alpha","AlphaDot"),("Beta","BetaDot"),("Psi","PsiDot")]:
            L.append(f"{ind}dxdt[StateToIdx(SN::{lhs})] = s[StateToIdx(SN::{rhs})];")
        L.append("")
        L.append(f"{ind}// Translational dynamics: dv/dt = (F_thrust + F_gravity + F_drag) / m.")
        for name in ("XDot","YDot","ZDot"):
            L.append(f"{ind}dxdt[StateToIdx(SN::{name})] = {acc_strs[name]};")
        L.append("")
        L.append(f"{ind}// User-input forces in inertial frame, divided by mass.")
        for k, name in enumerate(("XDot","YDot","ZDot")):
            L.append(f"{ind}dxdt[StateToIdx(SN::{name})] += userF[{k}] / m_p.m;")
        L.append("")
        L.append(f"{ind}// Rotational dynamics: I diagonal, torques are direct inputs.")
        L.append(f"{ind}dxdt[StateToIdx(SN::AlphaDot)] = T1 / m_p.Ix;")
        L.append(f"{ind}dxdt[StateToIdx(SN::BetaDot)]  = T2 / m_p.Iy;")
        L.append(f"{ind}dxdt[StateToIdx(SN::PsiDot)]   = T3 / m_p.Iz;")
        L.append("")
        L.append(f"{ind}// Augmented states: integral of position + heading tracking error.")
        L.append(f"{ind}dxdt[StateToIdx(SN::IntX)] = ref_pos[0] - s[StateToIdx(SN::X)];")
        L.append(f"{ind}dxdt[StateToIdx(SN::IntY)] = ref_pos[1] - s[StateToIdx(SN::Y)];")
        L.append(f"{ind}dxdt[StateToIdx(SN::IntZ)] = ref_pos[2] - s[StateToIdx(SN::Z)];")
        L.append(f"{ind}// Yaw integrator error wrapped to [-pi, pi] (ref.yaw is unbounded).")
        L.append(f"{ind}{{ const double dpsi = ref.yaw - s[StateToIdx(SN::Psi)];")
        L.append(f"{ind}  dxdt[StateToIdx(SN::IntPsi)] = std::atan2(std::sin(dpsi), std::cos(dpsi)); }}")
        L.append("")
        L.append(f"{ind}return dxdt;")
        return "\n".join(L)

    # ---------------- ExecuteControl body ----------------
    def _emit_execute_control_body(self):
        ind = self.cfg.indent
        ax_sym, ay_sym, az_sym = self._ff_kin["acc_syms"]
        jx_sym, jy_sym, jz_sym = self._ff_torque["jerk_syms"]
        sx_sym, sy_sym, sz_sym = self._ff_torque["snap_syms"]
        ref_subs = {ax_sym: sp.Symbol("ax"), ay_sym: sp.Symbol("ay"), az_sym: sp.Symbol("az"),
                    jx_sym: sp.Symbol("jx"), jy_sym: sp.Symbol("jy"), jz_sym: sp.Symbol("jz"),
                    sx_sym: sp.Symbol("sx"), sy_sym: sp.Symbol("sy"), sz_sym: sp.Symbol("sz")}

        def render_ff(expr):
            e = expr.subs(ref_subs); c = _ccode(e)
            for fn in ("sin","cos","tan","atan2","sqrt","pow","exp","log","fabs"):
                c = re.sub(rf"\b{fn}\(", f"std::{fn}(", c)
            c = re.sub(r"std::pow\(([a-zA-Z_]\w*), 2\)", r"\1*\1", c)
            return self._replace_param_access(c)

        F1_str = render_ff(self._ff_kin["F1"]); alpha_str = render_ff(self._ff_kin["alpha"])
        beta_str = render_ff(self._ff_kin["beta"]); alpha_dot_str = render_ff(self._ff_att_rate["alpha_dot"])
        beta_dot_str = render_ff(self._ff_att_rate["beta_dot"]); T1_str = render_ff(self._ff_torque["T1"])
        T2_str = render_ff(self._ff_torque["T2"])
        all_str = " ".join([F1_str, alpha_str, beta_str, alpha_dot_str, beta_dot_str, T1_str, T2_str])
        def _used(name): return re.search(rf"\b{name}\b", all_str) is not None

        L = [f"{ind}// Pull reference derivatives into named locals for clarity.",
             f"{ind}// The tilt-angle FF is only defined for thrust-positive references",
             f"{ind}// (az + g > 0): at az <= -g the FF angles flip / their rate expressions",
             f"{ind}// divide by zero (free-fall singularity). Clamp az to keep the FF",
             f"{ind}// well-posed; trajectories demanding > 1 g downward are out of envelope.",
             f"{ind}const double ax = r.acc[0], ay = r.acc[1];",
             f"{ind}const double az = std::max(r.acc[2], -m_p.g + 1e-6);"]
        if any(_used(n) for n in ("jx","jy","jz")):
            L.append(f"{ind}const double " + ", ".join(f"{n} = r.jerk[{i}]" for i, n in enumerate(("jx","jy","jz")) if _used(n)) + ";")
        if any(_used(n) for n in ("sx","sy","sz")):
            L.append(f"{ind}const double " + ", ".join(f"{n} = r.snap[{i}]" for i, n in enumerate(("sx","sy","sz")) if _used(n)) + ";")
        L.append("")
        L += [f"{ind}// Feedforward thrust magnitude and tilt angles from desired acceleration.",
              f"{ind}const double F1_ff        = {F1_str};",
              f"{ind}const double alpha_ff     = {alpha_str};",
              f"{ind}const double beta_ff      = {beta_str};",
              f"{ind}const double alpha_ff_dot = {alpha_dot_str};",
              f"{ind}const double beta_ff_dot  = {beta_dot_str};",
              f"{ind}const double T1_ff        = {T1_str};",
              f"{ind}const double T2_ff        = {T2_str};",
              f"{ind}const double T3_ff        = m_p.Iz * r.yawAcc;   // yaw-torque FF = Iz * yawAcc", ""]
        L += [f"{ind}// Build the LQR reference state s_ref; u_lqr = -K_e * (s - s_ref).",
              f"{ind}StateVec s_ref{{}};",
              f"{ind}s_ref[StateToIdx(StateName::X)]        = r.pos[0];",
              f"{ind}s_ref[StateToIdx(StateName::Y)]        = r.pos[1];",
              f"{ind}s_ref[StateToIdx(StateName::Z)]        = r.pos[2];",
              f"{ind}s_ref[StateToIdx(StateName::Alpha)]    = alpha_ff;",
              f"{ind}s_ref[StateToIdx(StateName::Beta)]     = beta_ff;",
              f"{ind}s_ref[StateToIdx(StateName::Psi)]      = r.yaw;",
              f"{ind}s_ref[StateToIdx(StateName::XDot)]     = r.vel[0];",
              f"{ind}s_ref[StateToIdx(StateName::YDot)]     = r.vel[1];",
              f"{ind}s_ref[StateToIdx(StateName::ZDot)]     = r.vel[2];",
              f"{ind}s_ref[StateToIdx(StateName::AlphaDot)] = alpha_ff_dot;",
              f"{ind}s_ref[StateToIdx(StateName::BetaDot)]  = beta_ff_dot;",
              f"{ind}s_ref[StateToIdx(StateName::PsiDot)]   = r.yawRate;", ""]
        L += [f"{ind}// Tracking error e = s - s_ref, with two corrections before applying K:",
              f"{ind}//  (1) heading wrap on the yaw error, and (2) yaw-frame compensation: the gain",
              f"{ind}//      was synthesized at yaw=0, so rotate the horizontal position/velocity/",
              f"{ind}//      integral errors into the heading frame (Rz(-psi)) before feeding K.",
              f"{ind}std::array<double, {self.cfg.aug_dim}> e{{}};",
              f"{ind}for (std::size_t j = 0; j < {self.cfg.aug_dim}; ++j) e[j] = s[j] - s_ref[j];",
              f"{ind}{{ double dp = e[StateToIdx(StateName::Psi)];",
              f"{ind}  e[StateToIdx(StateName::Psi)] = std::atan2(std::sin(dp), std::cos(dp)); }}",
              f"{ind}const double cpsi = std::cos(r.yaw), spsi = std::sin(r.yaw);",
              f"{ind}auto rot = [cpsi, spsi](double& ex, double& ey) {{",
              f"{ind}{ind}const double rx =  cpsi*ex + spsi*ey, ry = -spsi*ex + cpsi*ey;",
              f"{ind}{ind}ex = rx; ey = ry; }};",
              f"{ind}rot(e[StateToIdx(StateName::X)],    e[StateToIdx(StateName::Y)]);",
              f"{ind}rot(e[StateToIdx(StateName::XDot)], e[StateToIdx(StateName::YDot)]);",
              f"{ind}rot(e[StateToIdx(StateName::IntX)], e[StateToIdx(StateName::IntY)]);", "",
              f"{ind}// LQR correction: u_lqr = -K_e * e.",
              f"{ind}InputVec u_lqr{{}};",
              f"{ind}for (std::size_t i = 0; i < {self.cfg.input_dim}; ++i) {{",
              f"{ind}{ind}double v = 0.0;",
              f"{ind}{ind}for (std::size_t j = 0; j < {self.cfg.error_dim}; ++j) {{",
              f"{ind}{ind}{ind}v += K_e[i][j] * e[j];",
              f"{ind}{ind}}}",
              f"{ind}{ind}u_lqr[i] = -v;",
              f"{ind}}}", ""]
        L += [f"{ind}// Total control: u = u_ff + u_lqr, then saturate actuators.",
              f"{ind}InputVec u{{}};",
              f"{ind}u[0] = F1_ff + u_lqr[0];",
              f"{ind}u[1] = T1_ff + u_lqr[1];",
              f"{ind}u[2] = T2_ff + u_lqr[2];",
              f"{ind}u[3] = T3_ff + u_lqr[3];", ""]
        L += [f"{ind}if      (u[0] > m_p.F1_max) {{ u[0] = m_p.F1_max; }}",
              f"{ind}else if (u[0] < m_p.F1_min) {{ u[0] = m_p.F1_min; }}",
              f"{ind}if      (u[1] > m_p.T1_max) {{ u[1] = m_p.T1_max; }}",
              f"{ind}else if (u[1] < m_p.T1_min) {{ u[1] = m_p.T1_min; }}",
              f"{ind}if      (u[2] > m_p.T2_max) {{ u[2] = m_p.T2_max; }}",
              f"{ind}else if (u[2] < m_p.T2_min) {{ u[2] = m_p.T2_min; }}",
              f"{ind}if      (u[3] > m_p.T3_max) {{ u[3] = m_p.T3_max; }}",
              f"{ind}else if (u[3] < m_p.T3_min) {{ u[3] = m_p.T3_min; }}", "",
              f"{ind}return u;"]
        return "\n".join(L)
