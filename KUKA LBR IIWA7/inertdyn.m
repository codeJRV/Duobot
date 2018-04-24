function [y] = inertdyn(u,kuk)
u = transpose(u);
Qdd = u((1):(1),(1):(6));
Q = u((1):(1),(7):(13));

lamda = inv(kuk.jacob0(Q)/(kuk.inertia(Q))*transpose(kuk.jacob0(Q)));

y = transpose(kuk.jacob0(Q))*(lamda*transpose(Qdd));


% Jbar = (inv(kuk.inertia(Q))*transpose(kuk.jacob0(Q)))*inv(kuk.jacob0(Q)*inv(kuk.inertia(Q))*transpose(kuk.jacob0(Q)));
% 
% lamda = inv(kuk.jacob0(Q)*inv(kuk.inertia(Q))*transpose(kuk.jacob0(Q)));
% 
% pofq = transpose(Jbar)*(transpose(-1*kuk.gravload(Q)));
% 
% meowofq = (transpose(Jbar)*(kuk.coriolis(Q,Qd)*transpose(Qd)))-(lamda*kuk.jacob_dot(Q,Qd));
% 
% y = transpose(kuk.jacob0(Q))*(pofq);



end