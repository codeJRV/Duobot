function [y] = getTorq(Q)
lbr = importrobot('model.urdf');
lbr.DataFormat = 'row';
lbr.Gravity = [0 0 -9.80];

% joint1_initial_deg = 0;
% joint2_initial_deg = 90;
% joint3_initial_deg = 0;%-30
% joint4_initial_deg = 0;%90
% joint5_initial_deg = 0;
% joint6_initial_deg = 0;
% joint7_initial_deg = 0;

%homeConfig = homeConfiguration(lbr);
%randomConfig = randomConfiguration(lbr);

y = gravityTorque(lbr,Q) ;
end