function [y] = jackydot(u,kuk)
u = transpose(u);
Q = u((1):(1),(1):(7));
Qd = u((1):(1),(8):(14));
y = kuk.jacob_dot(Q,Qd);
end
