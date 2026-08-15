"""quad_codegen.py — QuadRotor C++ generator (derives from BaseCodegen)."""
from __future__ import annotations
import sympy as sp
from base_codegen import BaseCodegen, CodegenConfig


def quad_config(model_name="QUADROTOR_FF_LQR_01"):
    return CodegenConfig(
        parent_namespace="CDS::Dynamics", model_name=model_name,
        state_dim=13, aug_dim=17, error_dim=16, input_dim=4,
        state_enum_names=("X","Y","Z","Qw","Qx","Qy","Qz","VX","VY","VZ","WX","WY","WZ","IntX","IntY","IntZ","IntPsi"),
        param_enum_names=("Mass","Ix","Iy","Iz","Gravity","DragLat","DragAx","KThrust","KTorque","Arm","IRotor","ThrustMax","ThrustMin"),
        param_field_names=("m","Ix","Iy","Iz","g","c","cz","kT","kQ","L","Irot","T_max","T_min"),
        param_field_comments=("vehicle mass [kg]","inertia x [kg m^2]","inertia y [kg m^2]","inertia z [kg m^2]",
            "gravity [m/s^2]","lateral drag","axial drag","thrust coeff kT","torque coeff kQ","arm length [m]",
            "rotor inertia","per-rotor thrust max [N]","per-rotor thrust min [N]"),
        param_default_values=(2.4,0.025,0.025,0.045,9.81,0.20,0.30,1e-5,1.6e-7,0.275,3e-5,36.0,0.0),
        user_forces_type="std::array<double, 3>")


class QuadCodegen(BaseCodegen):
    """Quaternion 6-DOF + flatness FF + LQR, InputVec = [T1..T4]."""
    def __init__(self, cfg=None):
        super().__init__(cfg or quad_config())
        self._ff = None; self._Minv = None

    def set_feedforward_flat(self, F_ff, R_ref, omega_ff, tau_ff, M_inv):
        self._ff = dict(F=F_ff, R=sp.Matrix(R_ref), om=sp.Matrix(omega_ff), tau=sp.Matrix(tau_ff))
        self._Minv = sp.Matrix(M_inv); return self

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
        L = ["    // Flatness FF divides by ||a + g z_w||: clamp a_z away from free-fall",
             "    // (a_z <= -g with a_x=a_y=0 would make z_B undefined). Trajectories",
             "    // demanding > 1 g downward acceleration are out of the envelope.",
             "    const double a_x=r.acc[0], a_y=r.acc[1];",
             "    const double a_z=std::max(r.acc[2], -m_p.g + 1e-6);",
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

