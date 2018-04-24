function [y] = cory(u,kuk)
u = transpose(u);
Q = u((1):(1),(1):(7));
Qd = u((1):(1),(8):(14));

Jbar = ((kuk.inertia(Q))\transpose(kuk.jacob0(Q)))/(kuk.jacob0(Q)/(kuk.inertia(Q))*transpose(kuk.jacob0(Q)));

lamda = inv(kuk.jacob0(Q)/(kuk.inertia(Q))*transpose(kuk.jacob0(Q)));

pofq = transpose(Jbar)*(transpose(-1*kuk.gravload(Q)));

meowofq = (transpose(Jbar)*(kuk.coriolis(Q,Qd)*transpose(Qd)))-(lamda*kuk.jacob_dot(Q,Qd));

y = transpose(kuk.jacob0(Q))*(pofq);


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