function p = direct_hanoi( theta )
%direct_hanoi Calculates the position of the end eff. given joint angles
a = 20; % arm length in centimeters
p = [a*(cos(theta(1)) + cos(theta(1)+theta(2))); 
     a*(sin(theta(1)) + sin(theta(1)+theta(2)))];
end

