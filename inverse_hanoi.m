function theta = inverse_hanoi( p )
%inverse_hanoi Calculates the joint angles given end effector position
a = 20; % arm length
theta = [0; 0];
theta(2) = -acos((p(1)^2 + p(2)^2 - 2*a^2)/(2*a^2));
theta(1) = atan(p(2)/p(1)) - theta(2)/2;

end

