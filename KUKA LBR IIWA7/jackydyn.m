function [y] = jackydyn(Q,kuk)
%y = inv(kuk.inertia(Q))*transpose(kuk.jacob0(Q))*inv(kuk.jacob0(Q)*inv(kuk.inertia(Q))*transpose(kuk.jacob0(Q)));
y = ((kuk.inertia(Q))\transpose(kuk.jacob0(Q)))/(kuk.jacob0(Q)/(kuk.inertia(Q))*transpose(kuk.jacob0(Q)));
end
