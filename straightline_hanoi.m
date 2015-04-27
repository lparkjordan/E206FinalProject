function p = straightline_hanoi( pinit, pf, tf, t )
%straightline_hanoi moves the end effector from pi to pf in tf seconds.
% syms a5 a4 a3
% posFinalEqn = a5*tf^5 + a4*tf^4 + a3*tf^3 == norm(pf-pinit);
% velFinalEqn = 5*a5*tf^4 + 4*a4*tf^3 + 3*a3*tf^2 == 0;
% accFinalEqn = 20*a5*tf^3 + 12*a4*tf^2 + 6*a3*tf^2 == 0;
% coeffSoln = solve([posFinalEqn, velFinalEqn, accFinalEqn], [a3 a4 a5]);
% a3 = eval(coeffSoln.a3);
% a4 = eval(coeffSoln.a4);
% a5 = eval(coeffSoln.a5);
a5 = 6;
a4 = -15;
a3 = 10;
sNorm = a5*(t/tf).^5 + a4*(t/tf).^4 + a3*(t/tf).^3;
s = sNorm * norm(pf-pinit);
unitVec = (pf-pinit)/norm(pf-pinit);
p = pinit*ones(1,length(s)) + unitVec*s;

end

