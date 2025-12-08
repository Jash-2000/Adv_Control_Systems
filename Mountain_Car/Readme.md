# Mountain_Car_TO_LQR

This project implements the tracking controller using a TO to get an optimal path followed by a simplified version of time-varying LQR. The code or animation is included in this implementation. As such, the code is 
heavily commented for understanding, but for details of the implememntation go through the report -- [class_report.pdf](). 

 - To run this animated demo, simply download this file, extract the contents of the folder "OptimTraj-master" in the current path.
 - To enable plotting, set **en_plot** to 1.
 - The Objective function is set to "Minimize squared control effort". To change it, define a new function as follows: **problem.func.pathObj = @(t, x, u) u.^2;**
 - The Mountain Profile is defined as y = 0.5*x^2 - 0.1*x^4 + 0.2x^3 and can be changed in "mountainCarDynamics.m".
 - The non-linear dynamics are linearized at each time step using an epsilon of 1e-6, used to compute Jacobians. This can be changed in linearizeDynamics.m
 - computeLQRTrajectory.m evaluetas the LQR feedback gain matrix "K" using the state and cost penalties of Q = diag([10, 2]) and R = 1; Change them as required.

---

The mountain profile is generated through DESMOS grapher and it looks as follows:
![Hill_Profile](https://github.com/Jash-2000/Adv_Control_Systems/blob/main/Mountain_Car/Mountain_View.png)
