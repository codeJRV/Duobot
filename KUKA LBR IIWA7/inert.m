function [y] = inert(u,kuk)
u = transpose(u);
Qa = transpose(u((1):(1),(1):(7)));
Q = u((1):(1),(8):(14));
y = kuk.inertia(Q)*Qa;
end