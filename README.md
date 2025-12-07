# Adv_Control_Systems
Projects contained in this repository and the key technology used in it:
1. Acrobot Swingup -- Uses **Partial Feedback Linearization** for swing-up of the system, and then stabilizing near the fully-upright equilibrium via an **LQR (linear quadratic regulator)** balancing control.
2. Mountain Car -- Uses **Tracking controllter** that plans the min fuel path using Trajectory Optimization and implements the feedback with LQR.
3. Pendubot Swingup -- Uses **Trajectory Optimization** for swingup trajectory with minimum joint efforts.
4. Acrobot Controller -- Use a feedback controller that uses TO inside **MPC(Model Predictive Controller)** to move the acrobot from one equilibrium to another equilibrium.
5. Cart Pendulum State Estimation and Control -- Uses **Kalman Filter** to estimate the states and apply LQR controller to stabilize the self-balancing cart.
6. Segway Hybrid Control -- Used a **hybrid(cont.+ disc. time) controller** to self-balance and move a segway system.
