# `control/` — vehicle-agnostic controller development

Notebooks here develop the **generic control algorithms** (control-limited
iLQR/DDP, and later LQR/Riccati) independently of any specific vehicle. Each
notebook:

- derives the algorithm as the pedagogical Python reference (the "spec");
- validates it on a **synthetic, physically meaningless benchmark model** sized
  to stress the solver, not to mean anything;
- **conforms it against the hand-written C++** in `libs/control/` (the
  production artifact) on identical scenarios, comparing the *converged*
  solution — first command, final state, cost — not the bit-for-bit iterates
  (the solver has data-dependent branches: line-search, Levenberg, active-set).

The C++ solver is the product; the Python here is the derivation and the
conformance oracle. This is the opposite flow from `model/`, where the vehicle
dynamics are **generated** (SymPy → codegen → C++). We generate what is
symbolic (the model) and hand-write-then-bind what is algorithmic (the solver).

Vehicle-specific pieces — the tracking cost, the closed-loop demos — live in the
`model/` notebooks, not here.

Planned: `ilqr.ipynb` (generic iLQR + synthetic benchmark + C++ conformance).
