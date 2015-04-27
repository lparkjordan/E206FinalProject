function tau = hanoi_torque_globals( u )
%hanoi_torque Finds the required joint torque vector given positions, etc.
global massGrabbed;
global mL;
global b;
% Parameters
m = 0.25; %link masses, kg
if massGrabbed
    mL_local = mL; %load mass, kg
else
    mL_local = 0;
end
a = 0.2; %link lengths, m
Iz = 0.01; %link inertias, kgm^2
l = a/2; %Center of mass from joints
g = 9.81; % acceleration due to gravity, m/s^2

% Input
theta_r = u(1:2);
thetadot_r = u(3:4);
thetadotdot_r = u(5:6);

% Convenient variables
c1 = cos(theta_r(1));
c2 = cos(theta_r(2));
c12 = cos(theta_r(1)+theta_r(2));
s2 = sin(theta_r(2));

% System matrices
M = [m*(a^2 + 2*l^2 + 2*a*l*c2) + 2*Iz + 2*mL_local*a^2*(1 + c2), m*l*(l + a*c2)+ Iz + mL_local*a^2*(1 + c2); ...
     m*l*(l + a*c2) + Iz + mL_local*a^2*(1 + c2), m*l^2 + Iz + mL_local*a^2];
b_mat = [-2*(m*a*l + mL_local*a^2)*s2*thetadot_r(2), -(m*a*l + mL_local*a^2)*s2*thetadot_r(2); ...
         (m*a*l + mL_local*a^2)*s2*thetadot_r(1), 0];
B = [b 0; 0 b];
g_mat = [(m*(a + l) + mL_local*a)*g*c1 + (m*l + mL_local*a)*g*c12; ...
     (m*l + mL_local*a)*g*c12];
 
% Torques
tau_M = M*thetadotdot_r;
tau_b = b_mat*thetadot_r;
tau_B = B*thetadot_r;
tau_g = g_mat;
tau_sum = tau_M + tau_b + tau_B + tau_g;
tau = tau_sum;

end

