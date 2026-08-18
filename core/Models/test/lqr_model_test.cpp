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
// File        : lqr_model_test.cpp
// Description : Native acid test of the runtime LQR gain synthesis in the FF-LQR
//               models (Rocket, QuadRotor). It certifies three things per model:
//               (1) the bridge -- the gain the runtime synthesises from the
//               generated error dynamics reproduces the notebook-baked K_default;
//               (2) retuning the weights (SetWeights) changes the gain; and
//               (3) restoring the default weights restores the gain exactly.
//               No trajectory or physical params are needed -- the synthesis runs
//               at construction from the model's frozen, generated data.
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================

#include "Rocket.hpp"
#include "QuadRotor.hpp"

#include <cmath>
#include <cstdio>

using namespace CDS;

namespace {

template <class Model>
bool testModel(const char* name)
{
    Model m;
    bool ok = true;

    // 1. Bridge: the runtime-synthesised gain reproduces the baked K_default.
    const double bridge = m.GetGainBridgeError();
    if (!(bridge < 1e-6)) ok = false;

    // 2. Retune: a heavier state penalty must change the gain.
    double Q[16][16], R[4][4], K0[4][16], K1[4][16], K2[4][16];
    m.GetWeights(Q, R);
    m.GetGain(K0);

    double Qp[16][16], Rp[4][4];
    for (int i = 0; i < 16; ++i) for (int j = 0; j < 16; ++j) Qp[i][j] = Q[i][j];
    for (int a = 0; a < 4; ++a)  for (int b = 0; b < 4; ++b)  Rp[a][b] = R[a][b];
    for (int i = 0; i < 16; ++i) Qp[i][i] *= 4.0;
    m.SetWeights(Qp, Rp);
    m.GetGain(K1);

    double dChange = 0.0; bool finite = true;
    for (int i = 0; i < 4; ++i) for (int j = 0; j < 16; ++j)
    {
        dChange = std::fmax(dChange, std::fabs(K1[i][j] - K0[i][j]));
        if (!std::isfinite(K1[i][j])) finite = false;
    }
    if (!(dChange > 1e-3) || !finite) ok = false;

    // 3. Round-trip: restoring the default weights restores the original gain.
    m.SetWeights(Q, R);
    m.GetGain(K2);
    double dBack = 0.0;
    for (int i = 0; i < 4; ++i) for (int j = 0; j < 16; ++j)
        dBack = std::fmax(dBack, std::fabs(K2[i][j] - K0[i][j]));
    if (!(dBack < 1e-9)) ok = false;

    // 4. Controller manifest + set-by-id (the ParamTable path used by the wire).
    char buf[2048] = {0};
    if (m.GetControllerManifest(buf, sizeof buf)) ok = false;
    int lines = 0; for (const char* p = buf; *p; ++p) if (*p == '\n') ++lines;
    if (lines != 20) ok = false;                         // 16 Q-diag + 4 R-diag
    double Kc[4][16];
    if (m.SetControllerParam(0, 5000.0)) ok = false;     // id 0 = Q.x, writable
    m.GetGain(Kc);
    double dId = 0.0;
    for (int i = 0; i < 4; ++i) for (int j = 0; j < 16; ++j)
        dId = std::fmax(dId, std::fabs(Kc[i][j] - K0[i][j]));
    if (!(dId > 1e-3)) ok = false;                       // set-by-id changed the gain
    if (!m.SetControllerParam(999, 1.0)) ok = false;     // bad id must be rejected (true=error)

    std::printf("%-10s bridge=%.2e  retune dK=%.2e  round-trip dK=%.2e  manifest=%d rows  set-by-id dK=%.2e  -> %s\n",
                name, bridge, dChange, dBack, lines, dId, ok ? "PASS" : "FAIL");
    return ok;
}

} // namespace

int main()
{
    bool ok = true;
    ok &= testModel<Rocket>("Rocket");
    ok &= testModel<QuadRotor>("QuadRotor");
    std::printf(ok ? "LQR MODEL TEST PASSED\n" : "LQR MODEL TEST FAILED\n");
    return ok ? 0 : 1;
}
