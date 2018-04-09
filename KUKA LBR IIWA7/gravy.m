function [y] = gravy(Q,kuk)
Q = transpose(Q);
y = kuk.gravload(Q);
end
