L(1) = Link([0 0 1 0]);
% L(2) = Link([0 0 0.5 0]);
% L(3) = Link([0 0 0.5 0]);

% [Iyz Izx Ixy]

L(1).r = [-0.5, 0, 0];
L(1).m = 0.434849;
L(1).I = [1.68018e-05, 0.0367211, 0.0367332,1.47165e-09,0,0];

L(1).Jm = 0.0001;
% L(2).Jm = 0.0001;
% L(3).Jm = 0.0001;
% 
% L(2).r = [0.25+0.00455795, 8.18975e-09, 0];
% L(2).m = 0.21885;
% L(2).I = [8.44982e-06, 0.00468246, 0.00468858, 6.84398e-10,0,0];
% 
% 
% L(3).r = [0.25+0.00455795, 8.18975e-09, 0];
% L(3).m = 0.21885;
% L(3).I = [8.44982e-06, 0.00468246, 0.00468858, 6.84398e-10,0,0];


bob = SerialLink(L);
bob.gravity = [0 -9.81 0];

j1_init = 0;
% j2_init = 0;
% j3_init = 0;

qpose = [j1_init];

bob.gravload(qpose);