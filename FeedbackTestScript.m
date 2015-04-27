global mL
global b
global tf
mL = 0.1;
b = 0.02;
tf = 1;

Kp = eye(2)*316;
Kd = eye(2)*14;
[tout, ~, p] = sim('FeedbackLinearizedArm',5);

figure(1)
clf
hold all
tic
t = toc;
for n = 2:length(p(:,1))
    t = t+0.01;
    plot(p(n-1:n,1),p(n-1:n,2), 'o-', 'Color', 'Blue')
    xlim([5 35])
    ylim([-15 25])
    drawnow 
    while (toc < t)
    end
end
