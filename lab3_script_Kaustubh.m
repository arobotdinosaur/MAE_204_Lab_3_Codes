%% Lab 3 (Inverse Kinematics) template
% MAE204
% Harry
%
% Computes inverse kinematics for each waypoints in the sequence, then
% outputs the joint angle sets as well as gripper state as waypoint_array.csv file
% waypoint_array.csv will be saved in Matlab's current directory

clc
clear
close all

%% Part 1: Establishing screw axes S and end-effector zero config M
% First, define the screw axes, in (mm)

S1 = [0, 0, 1, -300, 0, 0]';

% Your code should begin here

S2 = [0 1 0 -240 0 0]';
S3 = [0 1 0 -240 0 244]';
S4 = [0 1 0 -240 0 457]';
S5 = [0 0 -1 169 457 0]';
S6 = [0 1 0 -155 0 457]';

% Next, define the M matrix (the zero-position e-e transformation matrix),
% in (mm)
M = [1 0 0 457; 0 1 0 78; 0 0 1 155; 0 0 0 1];


%% Part 2: UR3e sequence planning
% You may use this space to define the waypoints for your sequence (I
% recommend using SE(3) matrices to define gripper configurations)

standby = [0, 0, 1, 323.6; -1, 0, 0, -335.6; 0, -1, 0, 237; 0, 0, 0, 1];

legs_grab = [1 0 0 400; 0 0 1 -200; 0 -1 0 50; 0 0 0 1];
legs_release = [1 0 0 450; 0 0 1 -150; 0 -1 0 55; 0 0 0 1];
body_grab = [0, 0, 1, 450; -1, 0, 0, -450; 0, -1, 0, 70; 0, 0, 0, 1];
body_release = [1 0 0 450; 0 0 1 -150; 0 -1 0 85; 0 0 0 1];
head_grab = [0, 0, 1, 450; -1, 0, 0, -300; 0, -1, 0, 130; 0, 0, 0, 1];
head_release =[1 0 0 450; 0 0 1 -150; 0 -1 0 110; 0 0 0 1];

lift_height = 150; 

legs_above = legs_grab;
legs_above(3,4) = lift_height;

legs_release_above = legs_release;
legs_release_above(3,4) = lift_height;

body_above = body_grab;
body_above(3,4) = lift_height;

body_release_above = body_release;
body_release_above(3,4) = lift_height;

head_above = head_grab;
head_above(3,4) = lift_height;

head_release_above = head_release;
head_release_above(3,4) = lift_height;

%% Part 3: Inverse kinematics for each waypoint
% Compute inverse kinematics to obtain 6 joint angles for each waypoint,
% then save them in waypoint_array
%
% waypoint_array = n x 7 array where:
% n = number of waypoints
% First 6 columns in each row = joint angles 1...6, in degrees
% Last column in each row = gripper state (0 for open, 1 for close)
% 
S_list = [S1,S2,S3,S4,S5,S6];
theta_list_0 = [-pi/6; -pi/2; pi/2; -pi/2; -pi/2; 5*pi/6];
% [standby_thetalist, success] = IKinSpace(S_list, M, standby, theta_list_0, 0.01, 0.001)

waypoints = {
standby, 0;
legs_above, 0;
legs_grab, 1;
legs_above, 1;
legs_release_above, 1;
legs_release, 0;
legs_release_above, 0;
body_above, 0;
body_grab, 1;
body_above, 1;
body_release_above, 1;
body_release, 0;
body_release_above, 0;
head_above, 0;
head_grab, 1;
head_above, 1;
head_release_above, 1;
head_release, 0;
head_release_above, 0;
standby, 0};

theta_guess = theta_list_0;   

n = size(waypoints,1);
waypoint_array = zeros(n,7);

for i = 1:n
    
    T_sd = waypoints{i,1};
    grip_state = waypoints{i,2};
    
    [theta_sol, success] = IKinSpace(S_list, M, T_sd, theta_guess, 0.01, 0.001);
    
    if success ~= 1
        error(['IK failed at waypoint ', num2str(i)]);
    end
    
   
    theta_deg = rad2deg(theta_sol)';
    

    if any(abs(theta_deg(1:5)) > 360)
        error(['Joint limit exceeded at waypoint ', num2str(i)]);
    end
    
    waypoint_array(i,1:6) = theta_deg;
    waypoint_array(i,7) = grip_state;
    
    theta_guess = theta_sol;   
end


waypoint_array



% Your code should end here

%% Some basic sanity checks (DO NOT EDIT THIS PART)
% size of waypoint_array check
if length(waypoint_array(1,:)) ~= 7
    error('waypoint_array should have 7 columns')
end

for i = 1:length(waypoint_array(:,1))
    for j = 1:5
        % Joint limit check (error if out of joint limit bounds)
        if waypoint_array(i,j) > 360 || waypoint_array(i,j) < -360
            error(['Error: joint ',num2str(j),' in waypoint number ',num2str(i),' is out of joint limit bounds']);
        end
        % Gripper state check (error if not 0 or 1)
        if waypoint_array(i,7) ~= 0 && waypoint_array(i,7) ~= 1
            error(['Error: gripper state in waypoint number ',num2str(i),' is invalid. It should be 0 or 1']);
        end
    end
end

%% Output array to waypoint_array.csv
% waypoint_array.csv will be located in Matlab's current directory
writematrix(waypoint_array,'waypoint_array.csv')