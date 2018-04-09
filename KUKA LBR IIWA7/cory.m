function [y] = cory(u,kuk)
u = transpose(u);
Q = u((1):(1),(1):(7));
Qd = u((1):(1),(8):(14));
y = kuk.coriolis(Q,Qd);
end