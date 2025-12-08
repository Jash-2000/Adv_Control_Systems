Notes on this implementation / how to improve
  1. This implementation linearizes once about the upright equilibrium x_eq, then uses that linear model discretized with dt to compute a fixed sequence of finite-horizon LQR gains (K_seq). The controller applied is a standard receding-horizon policy using the first step gain at each sample.
  2. For better performance you can:
- re-linearize at the current state each sampling instant and recompute the horizon gains (MPC with time-varying linearization);
- include feedforward terms to track non-zero reference trajectories exactly (compute nominal open-loop u_ref sequence by solving linear equality constraints);
- enlarge horizon H or tune Q/R / Qf;
- add constraints and use a QP solver for constrained MPC (not needed for this assignment if unconstrained).
3. You asked to "augment" the state for look-ahead: the finite-horizon prediction itself is the look-ahead (we build ref_seq).
    If you specifically want a stacked-state augmented formulation (z = [x(k); x(k+1); ...]) we can build the block
    prediction matrices (big-block Ad, Bd) and solve the full quadratic program analytically — I can provide that version if you prefer.
