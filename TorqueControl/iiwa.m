%% Control LBR Manipulator Motion Through Joint Torque Commands
%% Introduction
% Given a set of desired joint configuration waypoints and a torque-controlled 
% manipulator, this example shows how to implement the computed-torque controller 
% using the |<docid:robotics_ref.bvlvr5v-1 inverseDynamics>| function. 
% The controller enables the robot to follow the given configuration waypoints 
% along a smooth trajectory. 

% Copyright 2016-2017 The MathWorks, Inc.

%% Bring Up LBR Gazebo Simulation
% Spawn an LBR robot in Gazebo Simulator. Follow steps in the <docid:robotics_examples.example-GettingStartedWithGazeboExample Getting Started With Gazebo Example>
% to launch the |Gazebo LBR Simulator| from the Ubuntu virtual machine desktop.
%
% |Gazebo LBR Simulator| brings up a barebone KUKA Light Weight Robot (LBR) manipulator
% with no default position, velocity or safety controllers.
% The only way to move the robot is through joint torques. Once the simulation 
% starts running, the LBR arm will fall onto the ground due to no joint
% torque input.
%
% <<lbr_ini_gz.png>>

%% Connect to ROS Network from MATLAB(R)
% In your MATLAB instance on the host computer, run the following commands
% to initialize ROS global node in MATLAB and connect to the ROS master
% in the virtual machine (where Gazebo is running) through its IP address 
% |ipaddress|. Replace '172.21.144.58' with the IP address of your virtual
% machine.
%
ipaddress = '192.168.56.101';

rosinit(ipaddress);

%% Create an LBR RigidBodyTree Object from URDF
lbr = importrobot('iiwa14.urdf');
lbr.DataFormat = 'row';
% Set the gravity to be the same as that in Gazebo.
lbr.Gravity = [0 0 -9.80];

% Show home configuration in MATLAB figure.
show(lbr);
view([150 12]);
axis([-0.6 0.6 -0.6 0.6 0 1.35]);
camva(9);
daspect([1 1 1]);

%%
%
% <<lbr_home.png>>

%% Pre-Compute Joint Torque Trajectory for Desired Motion
% Load joint configuration waypoints. This gives the key frames for
% the desired motion of the robot.
wpfilename = fullfile(fileparts(which('LBRTorqueControlExample')), 'data', 'lbr_waypoints.mat');
load(wpfilename);

%%
% |cdt| is the planned control stepsize. We use it to populate a set of
% time points where the trajectory needs to be evaluated and store it in vector
% |tt|.
cdt = 0.001; 
tt = 0:cdt:5; 

%%
% Generate desired motion trajectory for each joint.
% |exampleHelperJointTrajectoryGeneration| generates joint trajectories from
% given time and joint configuration waypoints. 

% The trajectories are generated using |pchip| so that the interpolated 
% joint position does not violate joint limits as long as the waypoints do not.
[qDesired, qdotDesired, qddotDesired, tt] = exampleHelperJointTrajectoryGeneration(tWaypoints, qWaypoints, tt);

%%
% Pre-compute feed-forward torques that ideally would realize the desired
% motion (assuming no disturbances or any kind of errors)
% using |<docid:robotics_ref.bvlvr5v-1 inverseDynamics>|.
% The following |for| loop takes some time to run. To accelerate, consider
% used generated code for |inverseDynamics|. See the last section for
% details on how to do it.

n = size(qDesired,1);
tauFeedForward = zeros(n,7);
for i = 1:n
    tauFeedForward(i,:) = inverseDynamics(lbr, qDesired(i,:), qdotDesired(i,:), qddotDesired(i,:));
end


%% Establish Communication Channel With Gazebo Through Customized Topics
% Gazebo provides two ROS services |/gazebo/get_joint_properties| and
% |/gazebo/apply_joint_effort| that can be used to get joint state and
% set joint torques. However, the services are too slow to close the torque
% control loop. Therefore, a customized Gazebo plug-in is used so that
% the joint state/torques in Gazebo can be read/written at a much faster 
% rate through the plain ROS topics (publisher and subscriber). The
% customized Gazebo plug-in is already brought up together with |Gazebo LBR
% Simulator|.
[jointTauPub, jtMsg] = rospublisher('/iiwa_matlab_plugin/iiwa_matlab_joint_effort');
jointStateSub = rossubscriber('/iiwa_matlab_plugin/iiwa_matlab_joint_state');


%% Reset LBR to Home Configuration in Gazebo
% Use Gazebo-provided service to reset the robot to its home configuration.
% For details on how to work with ROS service in MATLAB, see
% <docid:robotics_examples.example-ROSServicesExample Call and Provide ROS Services>.
mdlConfigClient = rossvcclient('gazebo/set_model_configuration');

% Compose the required service message. It includes the joint names 
% and corresponding joint positions to send to Gazebo. Call the service 
% using this message.
msg = rosmessage(mdlConfigClient);
msg.ModelName = 'mw_iiwa';
msg.UrdfParamName = 'robot_description';
msg.JointNames = {'mw_iiwa_joint_1', 'mw_iiwa_joint_2', 'mw_iiwa_joint_3',...
                  'mw_iiwa_joint_4', 'mw_iiwa_joint_5', 'mw_iiwa_joint_6', 'mw_iiwa_joint_7'};
msg.JointPositions = homeConfiguration(lbr);

call(mdlConfigClient, msg)

%% Computed Torque Control
% Select some PD gains.
weights = [0.3,0.8,0.6, 0.6,0.3,0.2,0.1];
Kp = 100*weights;
Kd = 2* weights;

once = 1;

%%
% Prepare for data logging.
feedForwardTorque = zeros(n, 7);
pdTorque = zeros(n, 7);
timePoints = zeros(n,1);
Q = zeros(n,7);
QDesired = zeros(n,7);

%%
% Computed torque control is implemented in the |for| loop below. As soon
% as MATLAB receives a new joint state from Gazebo, it looks up in the 
% pre-generated |tauFeedForward| and finds the feed-forward torque corresponding
% to the time stamp. It also computes a PD torque to compensate for the
% errors in joint position and velocities [1].
%
% With default settings in Gazebo, the |/iiwa_matlab_plugin/iiwa_matlab_joint_state|
% topic is updated at around 1 kHz (Gazebo sim time) with a typical 0.6 real 
% time factor. And the torque control loop below can typically run at around
% 200 Hz (Gazebo sim time).
for i = 1:n
    % Get joint state from Gazebo.
    jsMsg = receive(jointStateSub);
    data = jsMsg.Data;
    
    % Parse the received message. 
    % The Data in jsMsg is a 1-by-15 vector.
    % 1:7  - joint positions
    % 8:14 - joint velocities
    % 15   - time (Gazebo sim time) when the joint state is updated
    q = double(data(1:7))';
    qdot = double(data(8:14))';
    t = double(data(15));
    
    % Set the start time.
    if once
        tStart = t;
        once = 0;
    end
    
    % Find the corresponding index h in tauFeedForward vector for joint 
    % state time stamp t.
    h = ceil((t - tStart + 1e-8)/cdt);
    if h>n
        break
    end
    
    % Log joint positions data.
    Q(i,:) = q';
    QDesired(i,:) = qDesired(h,:);
    
    % Inquire feed-forward torque at the time when the joint state is
    % updated (Gazebo sim time).
    tau1 = tauFeedForward(h,:);
    % Log feed-forward torque.
    feedForwardTorque(i,:) = tau1;
    
    % Compute PD compensation torque based on joint position and velocity
    % errors.
    tau2 = Kp.*(qDesired(h,:) - q) + Kd.*(qdotDesired(h,:) - qdot);
    % Log PD torque.
    pdTorque(i,:) = tau2';
    
    % Combine the two torques.
    tau = tau1 + tau2;
    
    % Log the time.
    timePoints(i) = t-tStart;
    
    % Send torque to Gazebo.
    jtMsg.Data = tau;
    send(jointTauPub,jtMsg);    
end

%%
% With the joint torques sent, the LBR robot should follow the trajectory. 
% This image shows snapshots of the robot overlaid throughout the trajectory.
%
% <<lbr_overlaid.png>>


%% Inspect Results
% Plot and inspect the actual joint torques and positions versus the desired values. 
% Note that with the feed-forward torque, the PD torques should oscillate around zero.

exampleHelperLBRPlot(i-1, timePoints, feedForwardTorque, pdTorque, Q, QDesired )
%%
%
% <<lbr_joint_torques.png>>
%
% <<lbr_joint_positions.png>>

%% Code Generation for Inverse Dynamics

%%
% To speed up torque calculation in a loop, generate code for the
% |inverseDynamics| function. 
%
% Create a function called |invDyn|. Note |exampleHelperMwIiwa14|
% is a codegen-compatible function that re-creates the same
% |<docid:robotics_ref.bvan8uq-1 robotics.RigidBodyTree>|
% object as that returned by |importrobot('iiwa14.urdf')|.
% 
%      function tau = invDyn( q, qdot, qddot )
%          %#codegen
%          persistent robot 
%          if isempty(robot) 
%              robot = exampleHelperMwIiwa14; 
%          end 
%          tau = robot.inverseDynamics(q, qdot, qddot);  
%      end 
%
% Then use the following |codegen| command
%
%      codegen invDyn.m -args {zeros(1,7), zeros(1,7), zeros(1,7)}
%
% Finally, with the generated |invDyn_mex| file, you can replace the |inverseDynamics| call in the |for| loop
% 
%      tauFeedForward(i,:) = inverseDynamics(lbr, qDesired(i,:), qdotDesired(i,:), qddotDesired(i,:));
%
% with
%
%      tauFeedForward(i,:) = invDyn_mex(qDesired(i,:), qdotDesired(i,:), qddotDesired(i,:));


%% See Also
%
% * <docid:robotics_examples.example-GeneralizedInverseKinematicsExample Use Multiple Kinematic Constraints to Plan a Reaching Trajectory>

%% References
%%
% [1] B. Sicilano, L. Sciavicco, L. Villani, G. Oriolo, "Robotics:
% Modelling, Planning and Control", Springer, 2009
displayEndOfDemoMessage(mfilename)
