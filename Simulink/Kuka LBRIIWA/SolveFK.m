function [y1, y2, y3] = SolveFK(q1, q2, q3, q4, q5, q6, q7)
%SOLVEFK Summary of this function goes here
%   Detailed explanation goes here
KukaLink = load('KukaFK.mat');

fun = symfun(KukaLink);
res = subs(fun, [a b c d e f g], [q1, q2, q3, q4, q5, q6, q7]);

% res = KukaLink.fkine([q1 q2 q3 q4 q5 q6 q7]);
y1  = res.t(1) ;
y2  = res.t(2) ;
y3  = res.t(3) ;

end

