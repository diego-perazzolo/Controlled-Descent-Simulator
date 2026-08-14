// =============================================================================
// Controlled Descent Simulator
// =============================================================================
//
// Copyright (c) 2026 Diego Perazzolo
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//
// =============================================================================
// File        : QuadRotorMPC.cpp
// Description : Quadrotor 6-DOF (quaternion) runtime model with a nonlinear MPC.
//               The control-limited iLQR/DDP solver is folded here as private
//               machinery (anonymous namespace); the model tick samples the
//               reference over the horizon, solves warm-started, applies the
//               first command as a ZOH, and integrates the plant by the measured
//               step. Mirrors modeling/notebooks/dynamics_quadRotor_MPC01.ipynb.
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================

#include "QuadRotorMPC.hpp"

#include <array>
#include <cmath>
#include <algorithm>

#include "dynamics_quadrotor_mpc_01.hpp"
#include "rk4.hpp"

// State indexes (match CDS::Dynamics::QUADROTOR_MPC_01::StateName)
#define IDX_X   0
#define IDX_Y   1
#define IDX_Z   2
#define IDX_QW  3
#define IDX_QX  4
#define IDX_QY  5
#define IDX_QZ  6
#define IDX_VX  7
#define IDX_VY  8
#define IDX_VZ  9
#define IDX_WX  10
#define IDX_WY  11
#define IDX_WZ  12

namespace CDS {

// =============================================================================
//  Folded MPC solver -- control-limited iLQR / DDP (Tassa, Erez & Todorov 2014).
//  Private to this translation unit. Controller knobs are hard-wired here.
// =============================================================================
namespace {

using Model = Dynamics::QUADROTOR_MPC_01;

constexpr int N   = static_cast<int>(QuadRotorMPC::HORIZON);   // prediction horizon
constexpr std::size_t NX = 13;
constexpr std::size_t NU = 4;
constexpr std::size_t NR = 12;   // state-residual dimension

// ---- hard-wired controller tuning (revisit for frontend exposure later) ----
constexpr double DT_MPC    = 0.02;   // MPC prediction step [s]
constexpr int    MAX_ITERS = 12;     // iLQR iterations per tick (warm-started)

using V13   = std::array<double, NX>;
using V4    = std::array<double, NU>;
using V12   = std::array<double, NR>;
using M13   = std::array<V13, NX>;
using M13x4 = std::array<V4,  NX>;
using M4    = std::array<V4,  NU>;
using M4x13 = std::array<V13, NU>;
using M12x13= std::array<V13, NR>;

struct Weights { double wp, wq, wv, ww, wu, wterm; };
struct StageRef { std::array<double,3> p; std::array<double,4> q; std::array<double,3> v; std::array<double,3> w; };

const Weights W{6.0, 2.0, 1.0, 0.30, 0.10, 8.0};   // notebook defaults

// ---- quaternion helpers ----
inline void quatMul(const double a[4], const double b[4], double o[4])
{
    o[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
    o[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
    o[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
    o[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
}
inline void quatConj(const double a[4], double o[4]) { o[0]=a[0]; o[1]=-a[1]; o[2]=-a[2]; o[3]=-a[3]; }
inline void normalizeQuat(V13& x)
{
    const double n = std::sqrt(x[3]*x[3]+x[4]*x[4]+x[5]*x[5]+x[6]*x[6]);
    if (n > 1e-12){ const double s=1.0/n; x[3]*=s; x[4]*=s; x[5]*=s; x[6]*=s; }
}

// ---- cost: residuals, gradient, Gauss-Newton Hessian (SS4 of the notebook) ----
struct Costs { V13 lx{}; M13 lxx{}; V4 lu{}; M4 luu{}; double val{0}; };

void stateResidualAndJacobian(const V13& x, const StageRef& ref, const Weights& w, V12& r, M12x13& J)
{
    for (auto& row : J) row.fill(0.0);
    r[0]=w.wp*(x[0]-ref.p[0]); r[1]=w.wp*(x[1]-ref.p[1]); r[2]=w.wp*(x[2]-ref.p[2]);
    J[0][0]=J[1][1]=J[2][2]=w.wp;
    r[6]=w.wv*(x[7]-ref.v[0]); r[7]=w.wv*(x[8]-ref.v[1]); r[8]=w.wv*(x[9]-ref.v[2]);
    J[6][7]=J[7][8]=J[8][9]=w.wv;
    r[9]=w.ww*(x[10]-ref.w[0]); r[10]=w.ww*(x[11]-ref.w[1]); r[11]=w.ww*(x[12]-ref.w[2]);
    J[9][10]=J[10][11]=J[11][12]=w.ww;

    const double qref[4]={ref.q[0],ref.q[1],ref.q[2],ref.q[3]};
    const double qcur[4]={x[3],x[4],x[5],x[6]};
    double qrc[4]; quatConj(qref, qrc);
    double qe[4];  quatMul(qrc, qcur, qe);
    const double sgn = (qe[0]>=0.0)?1.0:-1.0;
    r[3]=w.wq*2.0*sgn*qe[1]; r[4]=w.wq*2.0*sgn*qe[2]; r[5]=w.wq*2.0*sgn*qe[3];
    const double a0=qrc[0], a1=qrc[1], a2=qrc[2], a3=qrc[3], c=2.0*sgn*w.wq;
    J[3][3]= c*a1;  J[3][4]= c*a0;  J[3][5]=-c*a3;  J[3][6]= c*a2;
    J[4][3]= c*a2;  J[4][4]= c*a3;  J[4][5]= c*a0;  J[4][6]=-c*a1;
    J[5][3]= c*a3;  J[5][4]=-c*a2;  J[5][5]= c*a1;  J[5][6]= c*a0;
}

double stateCostValue(const V13& x, const StageRef& ref, const Weights& w)
{
    V12 r; M12x13 J; stateResidualAndJacobian(x, ref, w, r, J);
    double s=0.0; for (double ri : r) s+=ri*ri; return 0.5*s;
}

Costs stageCosts(const V13& x, const V4& u, const StageRef& ref, const V4& uref, const Weights& w)
{
    Costs c;
    V12 r; M12x13 J; stateResidualAndJacobian(x, ref, w, r, J);
    for (std::size_t j=0;j<NX;++j){
        double g=0.0; for (std::size_t i=0;i<NR;++i) g+=J[i][j]*r[i]; c.lx[j]=g;
        for (std::size_t k=0;k<NX;++k){ double h=0.0; for (std::size_t i=0;i<NR;++i) h+=J[i][j]*J[i][k]; c.lxx[j][k]=h; }
    }
    double su=0.0;
    for (std::size_t a=0;a<NU;++a){ const double ru=w.wu*(u[a]-uref[a]); c.lu[a]=w.wu*ru; c.luu[a].fill(0.0); c.luu[a][a]=w.wu*w.wu; su+=ru*ru; }
    double sr=0.0; for (double ri : r) sr+=ri*ri;
    c.val=0.5*(sr+su);
    return c;
}

// ---- RK4 sensitivity: discrete Jacobians A = dF/dx, B = dF/du (pure RK4) ----
void sensitivity(const Model& model, const V13& x, const V4& u, double dt, M13& A, M13x4& B)
{
    const std::array<double,3> zero{{0,0,0}};
    auto dyn = [&](const V13& s){ return model.Dynamics(s, u, zero); };
    const V13 k1 = dyn(x);
    V13 x2, x3, x4;
    for (std::size_t i=0;i<NX;++i) x2[i]=x[i]+0.5*dt*k1[i];
    const V13 k2 = dyn(x2);
    for (std::size_t i=0;i<NX;++i) x3[i]=x[i]+0.5*dt*k2[i];
    const V13 k3 = dyn(x3);
    for (std::size_t i=0;i<NX;++i) x4[i]=x[i]+dt*k3[i];

    double fx1[NX][NX], fu1[NX][NU], fx2[NX][NX], fu2[NX][NU];
    double fx3[NX][NX], fu3[NX][NU], fx4[NX][NX], fu4[NX][NU];
    model.Jacobians(x,  u, fx1, fu1);
    model.Jacobians(x2, u, fx2, fu2);
    model.Jacobians(x3, u, fx3, fu3);
    model.Jacobians(x4, u, fx4, fu4);

    auto stepA = [&](const double fx[NX][NX], double cc, const M13& Aprev, M13& Aout){
        for (std::size_t i=0;i<NX;++i) for (std::size_t j=0;j<NX;++j){
            double s=fx[i][j]; for (std::size_t m=0;m<NX;++m) s+=fx[i][m]*(cc*Aprev[m][j]); Aout[i][j]=s; } };
    auto stepB = [&](const double fx[NX][NX], const double fu[NX][NU], double cc, const M13x4& Bprev, M13x4& Bout){
        for (std::size_t i=0;i<NX;++i) for (std::size_t a=0;a<NU;++a){
            double s=fu[i][a]; for (std::size_t m=0;m<NX;++m) s+=fx[i][m]*(cc*Bprev[m][a]); Bout[i][a]=s; } };
    M13 A1,A2,A3,A4; M13x4 B1,B2,B3,B4;
    for (std::size_t i=0;i<NX;++i){ for (std::size_t j=0;j<NX;++j) A1[i][j]=fx1[i][j];
                                    for (std::size_t a=0;a<NU;++a) B1[i][a]=fu1[i][a]; }
    stepA(fx2,0.5*dt,A1,A2); stepB(fx2,fu2,0.5*dt,B1,B2);
    stepA(fx3,0.5*dt,A2,A3); stepB(fx3,fu3,0.5*dt,B2,B3);
    stepA(fx4,dt,    A3,A4); stepB(fx4,fu4,dt,    B3,B4);
    const double s6=dt/6.0;
    for (std::size_t i=0;i<NX;++i){
        for (std::size_t j=0;j<NX;++j) A[i][j]=(i==j?1.0:0.0)+s6*(A1[i][j]+2*A2[i][j]+2*A3[i][j]+A4[i][j]);
        for (std::size_t a=0;a<NU;++a) B[i][a]=s6*(B1[i][a]+2*B2[i][a]+2*B3[i][a]+B4[i][a]);
    }
}

// ---- small SPD (Cholesky) solve for the free sub-block, up to 4x4, multi-RHS ----
bool solveSPD(const double H[NU][NU], int nf, const double R[NU][NX], int nc, double X[NU][NX])
{
    double L[NU][NU]={{0}};
    for (int i=0;i<nf;++i) for (int j=0;j<=i;++j){
        double s=H[i][j]; for (int k=0;k<j;++k) s-=L[i][k]*L[j][k];
        if (i==j){ if (s<=1e-12) return false; L[i][j]=std::sqrt(s); } else L[i][j]=s/L[j][j];
    }
    for (int c=0;c<nc;++c){
        double y[NU];
        for (int i=0;i<nf;++i){ double s=R[i][c]; for (int k=0;k<i;++k) s-=L[i][k]*y[k]; y[i]=s/L[i][i]; }
        for (int i=nf-1;i>=0;--i){ double s=y[i]; for (int k=i+1;k<nf;++k) s-=L[k][i]*X[k][c]; X[i][c]=s/L[i][i]; }
    }
    return true;
}

// ---- box-constrained QP (projected Newton) ----
void boxQP(const M4& H, const V4& g, const V4& lo, const V4& hi, V4& x, bool freeMask[NU])
{
    for (std::size_t a=0;a<NU;++a) x[a]=std::min(std::max(0.0, lo[a]), hi[a]);
    for (int it=0; it<40; ++it){
        V4 grad; for (std::size_t a=0;a<NU;++a){ double s=g[a]; for (std::size_t b=0;b<NU;++b) s+=H[a][b]*x[b]; grad[a]=s; }
        bool clamped[NU]; int nf=0; int fidx[NU];
        for (std::size_t a=0;a<NU;++a){
            clamped[a]=((x[a]<=lo[a] && grad[a]>0)||(x[a]>=hi[a] && grad[a]<0));
            freeMask[a]=!clamped[a]; if (freeMask[a]) fidx[nf++]=static_cast<int>(a);
        }
        if (nf==0) break;
        double Hf[NU][NU]; double Rf[NU][NX];
        for (int i=0;i<nf;++i){
            double gf=g[fidx[i]]; for (std::size_t b=0;b<NU;++b) if (clamped[b]) gf+=H[fidx[i]][b]*x[b];
            Rf[i][0]=-gf; for (int j=0;j<nf;++j) Hf[i][j]=H[fidx[i]][fidx[j]];
        }
        double Xf[NU][NX];
        if (!solveSPD(Hf, nf, Rf, 1, Xf)) break;
        V4 step{}; double maxstep=0.0;
        for (int i=0;i<nf;++i){ step[fidx[i]]=Xf[i][0]-x[fidx[i]]; maxstep=std::max(maxstep,std::fabs(step[fidx[i]])); }
        if (maxstep<1e-10) break;
        auto obj=[&](const V4& z){ double s=0; for (std::size_t a=0;a<NU;++a){ double Hz=0; for (std::size_t b=0;b<NU;++b) Hz+=H[a][b]*z[b]; s+=0.5*z[a]*Hz+g[a]*z[a]; } return s; };
        const double c0=obj(x); double a=1.0; V4 xn;
        for (int ls=0; ls<20; ++ls){
            for (std::size_t k=0;k<NU;++k) xn[k]=std::min(std::max(x[k]+a*step[k], lo[k]), hi[k]);
            if (obj(xn)<=c0){ x=xn; break; }
            a*=0.5; if (ls==19) x=xn;
        }
    }
}

// ---- receding-horizon solve: warm-started iLQR, writes first command, shifts warm start ----
void solveMpc(const Model& model, const V13& x0, const std::array<StageRef,N+1>& refs,
              const V4& uref, double tmin, double tmax, int maxIters,
              std::array<V4,N>& warmStart, V4& u0)
{
    const std::array<double,3> zero{{0,0,0}};
    auto Fd = [&](const V13& x, const V4& u){
        V13 xn = integrate::rk4_step<NX>(x, DT_MPC, [&](const V13& s){ return model.Dynamics(s, u, zero); });
        normalizeQuat(xn); return xn;
    };
    auto trajCost = [&](const std::array<V13,N+1>& xs, const std::array<V4,N>& us){
        double J=0.0; for (int k=0;k<N;++k) J+=stageCosts(xs[k], us[k], refs[k], uref, W).val;
        J += W.wterm*stateCostValue(xs[N], refs[N], W); return J;
    };

    std::array<V4,N>   us = warmStart;
    std::array<V13,N+1> xs; xs[0]=x0;
    for (int k=0;k<N;++k) xs[k+1]=Fd(xs[k], us[k]);
    double J = trajCost(xs, us);

    std::array<V4,N>    kff; std::array<M4x13,N> K; double mu=1e-3;

    for (int iter=0; iter<maxIters; ++iter){
        // terminal value V = wterm * (state gradient/Hessian at xs[N])
        V13 Vx; M13 Vxx;
        { Costs cN = stageCosts(xs[N], uref, refs[N], uref, W);
          for (std::size_t i=0;i<NX;++i){ Vx[i]=W.wterm*cN.lx[i];
              for (std::size_t j=0;j<NX;++j) Vxx[i][j]=W.wterm*cN.lxx[i][j]; } }

        for (int k=N-1; k>=0; --k){
            M13 A; M13x4 B; sensitivity(model, xs[k], us[k], DT_MPC, A, B);
            Costs c = stageCosts(xs[k], us[k], refs[k], uref, W);

            M4x13 BtVxx; M13 AtVxx;
            for (std::size_t a=0;a<NU;++a) for (std::size_t j=0;j<NX;++j){ double s=0; for (std::size_t i=0;i<NX;++i) s+=B[i][a]*Vxx[i][j]; BtVxx[a][j]=s; }
            for (std::size_t p=0;p<NX;++p) for (std::size_t j=0;j<NX;++j){ double s=0; for (std::size_t i=0;i<NX;++i) s+=A[i][p]*Vxx[i][j]; AtVxx[p][j]=s; }

            V13 Qx; V4 Qu; M13 Qxx; M4 Quu; M4x13 Qux;
            for (std::size_t p=0;p<NX;++p){ double s=c.lx[p]; for (std::size_t i=0;i<NX;++i) s+=A[i][p]*Vx[i]; Qx[p]=s; }
            for (std::size_t a=0;a<NU;++a){ double s=c.lu[a]; for (std::size_t i=0;i<NX;++i) s+=B[i][a]*Vx[i]; Qu[a]=s; }
            for (std::size_t p=0;p<NX;++p) for (std::size_t q=0;q<NX;++q){ double s=c.lxx[p][q]; for (std::size_t m=0;m<NX;++m) s+=AtVxx[p][m]*A[m][q]; Qxx[p][q]=s; }
            for (std::size_t a=0;a<NU;++a) for (std::size_t b=0;b<NU;++b){ double s=c.luu[a][b]; for (std::size_t m=0;m<NX;++m) s+=BtVxx[a][m]*B[m][b]; if (a==b) s+=mu; Quu[a][b]=s; }
            for (std::size_t a=0;a<NU;++a) for (std::size_t j=0;j<NX;++j){ double s=0; for (std::size_t m=0;m<NX;++m) s+=BtVxx[a][m]*A[m][j]; Qux[a][j]=s; }

            V4 lo, hi; for (std::size_t a=0;a<NU;++a){ lo[a]=tmin-us[k][a]; hi[a]=tmax-us[k][a]; }
            V4 ki; bool freeMask[NU]; boxQP(Quu, Qu, lo, hi, ki, freeMask);

            M4x13 Ki; for (auto& row : Ki) row.fill(0.0);
            int nf=0, fidx[NU]; for (std::size_t a=0;a<NU;++a) if (freeMask[a]) fidx[nf++]=static_cast<int>(a);
            if (nf>0){
                double Hf[NU][NU], Rf[NU][NX], Xf[NU][NX];
                for (int i=0;i<nf;++i){ for (int j=0;j<nf;++j) Hf[i][j]=Quu[fidx[i]][fidx[j]];
                    for (std::size_t j=0;j<NX;++j) Rf[i][j]=-Qux[fidx[i]][j]; }
                if (solveSPD(Hf, nf, Rf, static_cast<int>(NX), Xf))
                    for (int i=0;i<nf;++i) for (std::size_t j=0;j<NX;++j) Ki[fidx[i]][j]=Xf[i][j];
            }
            kff[k]=ki; K[k]=Ki;

            V4 Quuk; for (std::size_t a=0;a<NU;++a){ double s=0; for (std::size_t b=0;b<NU;++b) s+=Quu[a][b]*ki[b]; Quuk[a]=s; }
            for (std::size_t p=0;p<NX;++p){ double s=Qx[p];
                for (std::size_t a=0;a<NU;++a) s+=Ki[a][p]*Quuk[a]+Ki[a][p]*Qu[a]+Qux[a][p]*ki[a]; Vx[p]=s; }
            M4x13 QuuK; for (std::size_t a=0;a<NU;++a) for (std::size_t j=0;j<NX;++j){ double s=0; for (std::size_t b=0;b<NU;++b) s+=Quu[a][b]*Ki[b][j]; QuuK[a][j]=s; }
            M13 Vnew;
            for (std::size_t p=0;p<NX;++p) for (std::size_t q=0;q<NX;++q){ double s=Qxx[p][q];
                for (std::size_t a=0;a<NU;++a) s+=Ki[a][p]*QuuK[a][q]+Ki[a][p]*Qux[a][q]+Qux[a][p]*Ki[a][q]; Vnew[p][q]=s; }
            for (std::size_t p=0;p<NX;++p) for (std::size_t q=0;q<NX;++q) Vxx[p][q]=0.5*(Vnew[p][q]+Vnew[q][p]);
        }

        static const double alphas[]={1.0,0.5,0.25,0.125,0.0625,0.03,0.015,0.007};
        bool accepted=false;
        for (double a : alphas){
            std::array<V13,N+1> xn; std::array<V4,N> un; xn[0]=xs[0];
            for (int k=0;k<N;++k){
                V4 du;
                for (std::size_t j=0;j<NU;++j){
                    double fb=0; for (std::size_t p=0;p<NX;++p) fb+=K[k][j][p]*(xn[k][p]-xs[k][p]);
                    du[j]=a*kff[k][j]+fb; un[k][j]=std::min(std::max(us[k][j]+du[j], tmin), tmax);
                }
                xn[k+1]=Fd(xn[k], un[k]);
            }
            double Jn=trajCost(xn, un);
            if (Jn<J){ xs=xn; us=un; J=Jn; accepted=true; break; }
        }
        if (accepted) mu=std::max(mu*0.7,1e-6); else { mu*=4.0; if (mu>1e3) break; }
    }

    for (std::size_t j=0;j<NU;++j) u0[j]=std::min(std::max(us[0][j], tmin), tmax);
    for (int k=0;k<N-1;++k) warmStart[k]=us[k+1];
    warmStart[N-1]=us[N-1];
}

// ---- plant advance: one RK4 step at the measured dt with the applied command ----
V13 plantStep(const Model& model, const V13& x, const V4& u, const std::array<double,3>& uF, double dt)
{
    V13 xn = integrate::rk4_step<NX>(x, dt, [&](const V13& s){ return model.Dynamics(s, u, uF); });
    normalizeQuat(xn);
    return xn;
}

void initState(const Reference_t& ref, V13& s)
{
    s.fill(0.0);
    s[IDX_X]=ref.pos[0]; s[IDX_Y]=ref.pos[1]; s[IDX_Z]=ref.pos[2];
    const double h=0.5*ref.yaw; s[IDX_QW]=std::cos(h); s[IDX_QZ]=std::sin(h);
    s[IDX_VX]=ref.vel[0]; s[IDX_VY]=ref.vel[1]; s[IDX_VZ]=ref.vel[2];
}

} // anonymous namespace

// =============================================================================
QuadRotorMPC::QuadRotorMPC()
{
    m_modelPtr = new CDS::Dynamics::QUADROTOR_MPC_01();
    m_state.fill(0);
    m_state[IDX_QW] = 1.0;
    m_trackingErr.fill(0);
    m_userForces.fill(0);
    for (auto& u : m_warmStart) u.fill(0);
    m_trajectoryManagerPtr = nullptr;
    m_time = 0;
    m_seeded = false;
}

QuadRotorMPC::~QuadRotorMPC()
{
    if (m_modelPtr) { delete (CDS::Dynamics::QUADROTOR_MPC_01*) m_modelPtr; m_modelPtr = nullptr; }
}

bool QuadRotorMPC::SetModelParams(const std::any& params)
{
    auto dynamics = (Dynamics::QUADROTOR_MPC_01*) m_modelPtr;
    if (dynamics == nullptr || params.type() != typeid(core_quadRotorParams_t&))
    {
        // Err
        return true;
    }
    const auto& p = std::any_cast<const core_quadRotorParams_t&>(params);
    using PN = Dynamics::QUADROTOR_MPC_01::ParamName;
    dynamics->SetParam(PN::Mass,      p.m);
    dynamics->SetParam(PN::Ix,        p.Ix);
    dynamics->SetParam(PN::Iy,        p.Iy);
    dynamics->SetParam(PN::Iz,        p.Iz);
    dynamics->SetParam(PN::Gravity,   p.g);
    dynamics->SetParam(PN::DragX,     p.c);    // lateral drag on x
    dynamics->SetParam(PN::DragY,     p.c);    // lateral drag on y
    dynamics->SetParam(PN::DragZ,     p.cz);   // axial drag on z
    dynamics->SetParam(PN::KThrust,   p.kT);
    dynamics->SetParam(PN::KTorque,   p.kQ);
    dynamics->SetParam(PN::Arm,       p.L);
    dynamics->SetParam(PN::ThrustMax, p.Fm_max);
    dynamics->SetParam(PN::ThrustMin, p.Fm_min);
    return false;
}

bool QuadRotorMPC::SetTrajectoryManager(TrajectoryManager* pTrajectoryManager)
{
    Reference_t ref;
    if (pTrajectoryManager == nullptr || pTrajectoryManager->GetReference(m_time, ref))
    {
        // Error
        return true;
    }
    m_trajectoryManagerPtr = pTrajectoryManager;
    initState(ref, m_state);
    return false;
}

bool QuadRotorMPC::PerformIntegration(const core_stepParams_t& params)
{
    auto dynamics = (Dynamics::QUADROTOR_MPC_01*) m_modelPtr;
    if (dynamics == nullptr || m_trajectoryManagerPtr == nullptr) { return true; }

    // ---- sample the reference over the horizon (preview) ----
    std::array<StageRef, N + 1> refs;
    Reference_t ref0;
    if (m_trajectoryManagerPtr->GetReference(m_time, ref0)) { return true; }
    for (int k = 0; k <= N; ++k)
    {
        Reference_t r;
        if (m_trajectoryManagerPtr->GetReference(m_time + k * DT_MPC, r))
        {
            refs[k] = refs[k - 1];           // beyond the trajectory: hold the last (defensive)
            continue;
        }
        const double h = 0.5 * r.yaw;
        refs[k].p = {{r.pos[0], r.pos[1], r.pos[2]}};
        refs[k].q = {{std::cos(h), 0.0, 0.0, std::sin(h)}};   // level attitude at the commanded heading
        refs[k].v = {{r.vel[0], r.vel[1], r.vel[2]}};
        refs[k].w = {{0.0, 0.0, 0.0}};
    }

    // ---- hover command and actuator box from the model params ----
    using PN = Dynamics::QUADROTOR_MPC_01::ParamName;
    const double mg = dynamics->GetParam(PN::Mass) * dynamics->GetParam(PN::Gravity);
    const double Th = mg / 4.0;
    const V4 uref{{Th, Th, Th, Th}};
    const double tmin = dynamics->GetParam(PN::ThrustMin);
    const double tmax = dynamics->GetParam(PN::ThrustMax);
    if (!m_seeded) { for (auto& u : m_warmStart) u = uref; m_seeded = true; }

    // ---- tracking errors (position + heading) w.r.t. the current reference ----
    m_trackingErr[0] = ref0.pos[0] - m_state[IDX_X];
    m_trackingErr[1] = ref0.pos[1] - m_state[IDX_Y];
    m_trackingErr[2] = ref0.pos[2] - m_state[IDX_Z];
    const double qw=m_state[IDX_QW], qx=m_state[IDX_QX], qy=m_state[IDX_QY], qz=m_state[IDX_QZ];
    const double yaw = std::atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz));
    double eyaw = ref0.yaw - yaw; eyaw = std::atan2(std::sin(eyaw), std::cos(eyaw));
    m_trackingErr[3] = eyaw;

    // ---- solve, apply first command as ZOH, integrate plant by the measured step ----
    V4 u0;
    solveMpc(*dynamics, m_state, refs, uref, tmin, tmax, MAX_ITERS, m_warmStart, u0);

    m_userForces[0] = params.user_fX;
    m_userForces[1] = params.user_fY;
    m_userForces[2] = params.user_fZ;
    m_state = plantStep(*dynamics, m_state, u0, m_userForces, params.timestep);

    m_time += params.timestep;
    return false;
}

bool QuadRotorMPC::GetState(core_state_t& state)
{
    state.x_dot = m_state[IDX_VX]; state.y_dot = m_state[IDX_VY]; state.z_dot = m_state[IDX_VZ];
    state.x = m_state[IDX_X]; state.y = m_state[IDX_Y]; state.z = m_state[IDX_Z];
    state.roll_dot = m_state[IDX_WX]; state.pitch_dot = m_state[IDX_WY]; state.yaw_dot = m_state[IDX_WZ];

    const double qw=m_state[IDX_QW], qx=m_state[IDX_QX], qy=m_state[IDX_QY], qz=m_state[IDX_QZ];
    double sinp = 2.0*(qw*qy - qz*qx);
    if (sinp >  1.0) sinp =  1.0;
    if (sinp < -1.0) sinp = -1.0;
    state.roll  = std::atan2(2.0*(qw*qx + qy*qz), 1.0 - 2.0*(qx*qx + qy*qy));
    state.pitch = std::asin(sinp);
    state.yaw   = std::atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz));
    return false;
}

bool QuadRotorMPC::GetTrackingErrors(core_trackingErrors_t& tErrors)
{
    tErrors.x = m_trackingErr[0]; tErrors.y = m_trackingErr[1];
    tErrors.z = m_trackingErr[2]; tErrors.yaw = m_trackingErr[3];
    return false;
}

bool QuadRotorMPC::GetCurrentTimeSeconds(core_coord_t& currentTimeSeconds)
{
    currentTimeSeconds = m_time;
    return false;
}

} // namespace CDS
