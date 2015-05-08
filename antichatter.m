function signS = antichatter( s )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
global epsilon;
if norm(s) >= epsilon
    signS = s./norm(s);
else
    signS = s./epsilon;
end

end

