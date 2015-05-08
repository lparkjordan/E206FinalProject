function eta = calcEta( inputs )
%Hanoi_Lin Calculates the control signal for the Hanoi robot arm.

% Constants
global massGrabbed;
global mL;
global b;
global Iz;
Iz_real = 0.01;
b_real = 0.02;
% Parameters
m = 0.25; %link masses, kg
if massGrabbed
    mL_local = mL; %load mass, kg
else
    mL_local = 0;
end
a = 0.2; %link lengths, m
% Iz = 0.01; %link inertias, kgm^2
l = a/2; %Center of mass from joints
g = 9.81; % acceleration due to gravity, m/s^2

% Inputs
qdotdot_r_plus_v = inputs(1:2);
q = inputs(3:4);
qdot = inputs(5:6);

% Convenient variables
c1 = cos(q(1));
c2 = cos(q(2));
c12 = cos(q(1)+q(2));
s2 = sin(q(2));

Mhat = [m*(a^2 + 2*l^2 + 2*a*l*c2) + 2*Iz + 2*mL_local*a^2*(1 + c2), m*l*(l + a*c2)+ Iz + mL_local*a^2*(1 + c2); ...
     m*l*(l + a*c2) + Iz + mL_local*a^2*(1 + c2), m*l^2 + Iz + mL_local*a^2];

 Mreal = [m*(a^2 + 2*l^2 + 2*a*l*c2) + 2*Iz_real + 2*mL_local*a^2*(1 + c2), m*l*(l + a*c2)+ Iz_real + mL_local*a^2*(1 + c2); ...
     m*l*(l + a*c2) + Iz_real + mL_local*a^2*(1 + c2), m*l^2 + Iz_real + mL_local*a^2];
 
b_mat = [-2*(m*a*l + mL_local*a^2)*s2*qdot(2), -(m*a*l + mL_local*a^2)*s2*qdot(2); ...
         (m*a*l + mL_local*a^2)*s2*qdot(1), 0];
Bhat = [b 0; 0 b];
Breal = [b_real 0; 0 b_real];
g_mat = [(m*(a + l) + mL_local*a)*g*c1 + (m*l + mL_local*a)*g*c12; ...
     (m*l + mL_local*a)*g*c12];
c_hat = b_mat*qdot+Bhat*qdot+g_mat; 
c_real = b_mat*qdot+Breal*qdot+g_mat; 
% u = M*qdotdot_r_plus_v + c;
eta = (eye(2)-Mreal\Mhat)*qdotdot_r_plus_v+Mreal\(c_real-c_hat);
end


