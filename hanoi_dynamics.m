function alpha = hanoi_dynamics( u )
%hanoi_dynamics Summary of this function goes here
%   Determines the angular acceleration of each arm in the hanoi robot.
global massGrabbed;
% Parameters
m = 0.25; %link masses, kg
if massGrabbed
    mL = 0.1; %load mass, kg
else
    mL = 0;
end
a = 0.2; %link lengths, m
Iz = 0.01; %link inertias, kgm^2
l = a/2; %Center of mass from joints
b = 0.02; % Joint friction coefficients, Nms
g = 9.81; % acceleration due to gravity, m/s^2

% Input
tau = u(1:2);
theta_r = u(3:4);
thetadot_r = u(5:6);

% Convenient variables
c1 = cos(theta_r(1));
c2 = cos(theta_r(2));
c12 = cos(theta_r(1)+theta_r(2));
s2 = sin(theta_r(2));

% System matrices
M = [m*(a^2 + 2*l^2 + 2*a*l*c2) + 2*Iz + 2*mL*a^2*(1 + c2), m*l*(l + a*c2)+ Iz + mL*a^2*(1 + c2); ...
     m*l*(l + a*c2) + Iz + mL*a^2*(1 + c2), m*l^2 + Iz + mL*a^2];
b_mat = [-2*(m*a*l + mL*a^2)*s2*thetadot_r(2), -(m*a*l + mL*a^2)*s2*thetadot_r(2); ...
         (m*a*l + mL*a^2)*s2*thetadot_r(1), 0];
B = [b 0; 0 b];
g_mat = [(m*(a + l) + mL*a)*g*c1 + (m*l + mL*a)*g*c12; ...
     (m*l + mL*a)*g*c12];
c = b_mat*thetadot_r+B*thetadot_r+g_mat; 
alpha = inv(M) * (tau - c);

alpha
end

