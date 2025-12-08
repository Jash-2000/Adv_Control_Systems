# Mountain_Car_TO_LQR

This project implements the tracking controller using a TO to get an optimal path followed by a simplified version of time-varying LQR. The code or animation is included in this implementation. As such, the code is 
heavily commented for understanding, but for details of the implememntation go through the report -- [class_report.pdf](). 

 -- To run this animated demo, simply download this file, extract the contents of the folder "OptimTraj-master" in the current path.
 -- To enable plotting, set **en_plot** to 1.
 -- The Objective function is set to "Minimize squared control effort". To change it, define a new function as follows: **problem.func.pathObj = @(t, x, u) u.^2;**
