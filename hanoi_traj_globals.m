function p_r = hanoi_traj_globals( t )

global massGrabbed;
global tf
if (t > 1) && (t < 4)
    massGrabbed = 1;
else
    massGrabbed = 0;
end
% hanoi_traj sequences all five moves to completion
p_r = zeros(2,length(t));
used = 1;
points = [20,  10, 10, 30, 30, 20; ...
          20, -10, 0,  0, -10, 20];
if length(t) == 1 % single point
    startPt = points(:,mod(floor(t/tf),5)+1);
    endPt = points(:,mod(floor(t/tf),5)+2);
    time = t-floor(t/tf)*tf;
    p_r = straightline_hanoi(startPt, endPt, tf, time);
else
    for n = 1:5 % vector of points
        tn = t((t >= n-1) & (t <= n));
        if n == 5
            assignin('base','tn',tn);
        end
        tn = tn - min(tn);
        disp(used+length(tn)-used)
        trajn = straightline_hanoi(points(:,n), points(:,n+1), tf, tn);
        p_r(:,used:used+length(tn)-1) = trajn;
        used = used + length(tn)-1;
    end
end

end

