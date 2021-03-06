clear all;

joint1_initial_deg = 0;
joint2_initial_deg = 0;
joint3_initial_deg = -25;%-30
joint4_initial_deg = 90;%90
joint5_initial_deg = 0;
joint6_initial_deg = 90;
joint7_initial_deg = 0;

% qpose = [joint1_initial_deg,joint2_initial_deg,joint3_initial_deg,joint4_initial_deg,joint5_initial_deg,joint6_initial_deg,joint7_initial_deg]*pi/180;
qpose = [joint1_initial_deg,joint2_initial_deg,joint3_initial_deg,joint4_initial_deg,joint5_initial_deg,joint6_initial_deg,joint7_initial_deg]*pi/180;
% qpose = [joint1_initial_deg,joint2_initial_deg,joint3_initial_deg,joint4_initial_deg,joint5_initial_deg]*pi/180;



% joint1_initial_deg = 0;
% joint2_initial_deg = 25;
% joint3_initial_deg = -30;%-30
% joint4_initial_deg = 90;%90
% joint5_initial_deg = 23;
% joint6_initial_deg = 9;
% joint7_initial_deg = 8;
% 
% joint1_initial_deg2 = 0;
% joint2_initial_deg2 = 25;
% joint3_initial_deg2 = -30;%-30
% joint4_initial_deg2 = 90;%90
% joint5_initial_deg2 = 23;
% joint6_initial_deg2 = 9;
% joint7_initial_deg2 = 8;

joint1_damping = 0.5;
joint2_damping = 0.5;
joint3_damping = 0.5;
joint4_damping = 0.5;
joint5_damping = 0.5;
joint6_damping = 0.5;
joint7_damping = 0.5;



joint1_gain = 1;
joint2_gain = 1;
joint3_gain = 1;
joint4_gain = 1;
joint5_gain = 1;
joint6_gain = 1;
joint7_gain = 1;

cor_gain = -1;

% joint1_gain = 100;
% joint2_gain = 100;
% joint3_gain = 100;
% joint4_gain = 100;
% joint5_gain = 100;
% joint6_gain = 100;
% joint7_gain = 100;

L(1) = Link([0 0.34 0 pi/2]);
L(2) = Link([0 0 0 -pi/2]);
L(3) = Link([0 0.4 0 -pi/2]);
L(4) = Link([0 0 0 pi/2]);
L(5) = Link([0 0.4 0 pi/2]);
L(6) = Link([0 0 0 -pi/2]);
L(7) = Link([0 0.126 0 pi/2]);

% L(1).I = [0.02183 0.007703 0.0283 0 -0.003887 0];
% L(2).I = [0.02076 0.02179 0.00779 0 0 -0.003626];
% L(3).I = [0.03204 0.00972 0.03042 0 0.006227 0];
% L(4).I = [0.02178 0.02075 0.007785 0 -0.003625 0];
% L(5).I = [0.01287 0.005708 0.01112 0 -0.003946 0];
% L(6).I = [0.006509 0.006259 0.004527 0 0.00031891 0];
% L(7).I = [0.01464 0.01465 0.002872 0.0005912 0 0];

% L(1).I = [0.02183 0.007703 0.0283];
% L(2).I = [0.02076 0.02179 0.00779];
% L(3).I = [0.03204 0.00972 0.03042];
% L(4).I = [0.02178 0.02075 0.007785];
% L(5).I = [0.01287 0.005708 0.01112];
% L(6).I = [0.006509 0.006259 0.004527];
% L(7).I = [0.01464 0.01465 0.002872];

L(1).I = [0.02183 0.0283 0.007703];
L(2).I = [0.02076 0.00779 0.02179];
L(3).I = [0.03204 0.03042 0.00972];
L(4).I = [0.02178 0.007785 0.02075];
L(5).I = [0.01287 0.01112 0.005708];
L(6).I = [0.006509 0.004527 0.006259];
L(7).I = [0.01464 0.002872 0.01465];

L(1).m = 3.4525;
L(2).m = 3.4821;
L(3).m = 4.05623;
L(4).m = 3.4822;
L(5).m = 2.1633;
L(6).m = 2.3466;
L(7).m = 3.129;

L(1).Jm = 0.5;
L(2).Jm = 0.5;
L(3).Jm = 0.5;
L(4).Jm = 0.5;
L(5).Jm = 0.5;
L(6).Jm = 0.5;
L(7).Jm = 0.5;

L(1).r = [0 -0.07 -0.03];
L(2).r = [0.0003 -0.042 0.059];
L(3).r = [0 0.06 -0.03];
L(4).r = [0 0.034 0.067];
L(5).r = [0.0001 -0.114 -0.021];
L(6).r = [0 -0.0004 0.0006];
L(7).r = [0 -0.025 0];

qzero = [0 0 0 0 0 0 0];
qdzero = [0 0];


kuk = SerialLink(L);
kuk.gravity = [0 0 -9.81];


%   L(1).I=([1 1 3])
% 
%   I = R.inertia(Q) is the symmetric joint inertia matrix (NxN) which relates 
%   joint torque to joint acceleration for the robot at joint configuration Q.
% 
%   C = R.coriolis(Q, QD) is the Coriolis/centripetal matrix (NxN) for
%   the robot in configuration Q and velocity QD, where N is the number of
%   joints.  The product C*QD is the vector of joint force/torque due to velocity
%   coupling.  The diagonal elements are due to centripetal effects and the 
%   off-diagonal elements are due to Coriolis effects.  This matrix is also 
%   known as the velocity coupling matrix, since it describes the disturbance forces
%   on any joint due to velocity of all other joints.
%   
%   
%   TAUG = R.gravload(Q) is the joint gravity loading (1xN) for the robot R
%   in the joint configuration Q (1xN), where N is the number of robot
%   joints.  Gravitational acceleration is a property of the robot object.
