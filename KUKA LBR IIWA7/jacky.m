function [y] = jacky(Q,kuk)
% L(1) = Link([0 0.34 0 pi/2]);
% L(2) = Link([0 0 0 -pi/2]);
% L(3) = Link([0 0.4 0 -pi/2]);
% L(4) = Link([0 0 0 pi/2]);
% L(5) = Link([0 0.4 0 pi/2]);
% L(6) = Link([0 0 0 -pi/2]);
% L(7) = Link([0 0.126 0 pi/2]);
% kuk = SerialLink(L);
y = kuk.jacob0(Q);
end
