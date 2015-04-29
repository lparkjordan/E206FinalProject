function w = antichatter( u )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
global epsilon;
if norm(u) >= epsilon
    w = u + u/norm(u);
else
    w = u + u/epsilon;
end

end

