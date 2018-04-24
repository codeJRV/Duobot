Jbar = ((kuk.inertia(Q))\transpose(kuk.jacob0(Q)))/(kuk.jacob0(Q)/(kuk.inertia(Q))*transpose(kuk.jacob0(Q)));

lamda = inv(kuk.jacob0(Q)/(kuk.inertia(Q))*transpose(kuk.jacob0(Q)));

pofq = transpose(Jbar)*transpose(kuk.gravload(Q));

meowofq = (transpose(Jbar)*(kuk.coriolis(Q,Qd)*transpose(Qd)))-(lamda*kuk.jacob_dot(Q,Qd));