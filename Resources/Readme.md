# TO/MPC
Model Predictive Control (MPC), also called Receding Horizon Control, and a Preview Control approach that implements in MPC via finding an analytically-solvable offline solution. 
See Kajita03, on the Preview Control approach for a very simplified humanoid planning approach: Kajita03.pdf Download Kajita03.pdf, kajita03.m Download kajita03.m, which in turn depends on familiarity with: 
katayama85.pdf Download katayama85.pdf. Here’s a broader overview of MPC methods related to legged locomotion: Model predictive control of legged and humanoid robots models and algorithms.pdfDownload Model 
predictive control of legged and humanoid robots models and algorithms.pdf

Trajectory Optimization. Here, we seek what might be considered an “open-loop” set of states, x(k), and control inputs, u(k), that obey required dynamics (as constraints) and are also (locally) optimal, with 
respect to some cost funtion [https://www.youtube.com/watch?v=wlkRYMVUZTs](https://www.youtube.com/watch?v=wlkRYMVUZTs).
