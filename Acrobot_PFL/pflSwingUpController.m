function u = pflSwingUpController(x, params, control, E_desired)

E = computeTotalEnergy(x, params);
E_tilde = E - E_desired;

theta1 = x(1);
dtheta2 = x(4);

psi = dtheta2 * cos(theta1);

u = -control.k_energy * E_tilde * psi;
end