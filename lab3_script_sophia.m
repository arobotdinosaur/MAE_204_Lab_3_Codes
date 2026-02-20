%% Lab 3 (Inverse Kinematics) template
% MAE204
% Harry
%
% Computes inverse kinematics for each waypoints in the sequence, then
% outputs the joint angle sets as well as gripper state as waypoint_array.csv file
% waypoint_array.csv will be saved in Matlab's current directory

clear
close all

%% Part 1: Establishing screw axes S and end-effector zero config M
% First, define the screw axes, in (mm)

S1 = [0, 0, 1, -300, 0, 0]';
S2 = [0, 1, 0, -240, 0, 0]';
S3 = [0, 1, 0, -240, 0, 244]';
S4 = [0, 1, 0, -240, 0, 457]';
S5 = [0, 0, -1, 169, 457, 0]';
S6 = [0, 1, 0, -155, 0, 457]';

% Next, define the M matrix (the zero-position e-e transformation matrix),
% in (mm)

M = [1 0 0 457; 
    0 1 0 78; 
    0 0 1 155; 
    0 0 0 1];

%% Part 2: UR3e sequence planning
% You may use this space to define the waypoints for your sequence (I
% recommend using SE(3) matrices to define gripper configurations)

standby = [0, 0, 1, 323.6; 
          -1, 0, 0, -335.6; 
           0, -1, 0, 237; 
           0, 0, 0, 1];

legs_grab = [ 1  0  0   400;
                0  0  1  -200;
                0 -1  0   50;
                0  0  0   1];

legs_grab_up = [ 1  0  0   400;
                0  0  1  -200;
                0 -1  0   150;
                0  0  0   1];

legs_release = [ 1  0  0   450;
                   0  0  1  -150;
                   0 -1  0   55;
                   0  0  0   1];

legs_release_up = [ 1  0  0   450;
                   0  0  1  -150;
                   0 -1  0   155;
                   0  0  0   1];

body_grab = [ 0  0  1   450;
               -1  0  0  -450;
                0 -1  0   70;
                0  0  0   1];

body_grab_up = [ 0  0  1   450;
               -1  0  0  -450;
                0 -1  0   170;
                0  0  0   1];

body_release = [ 1  0  0   450;
                   0  0  1  -150;
                   0 -1  0   85;
                   0  0  0   1];

body_release_up = [ 1  0  0   450;
                   0  0  1  -150;
                   0 -1  0   185;
                   0  0  0   1];

head_grab = [ 0  0  1   450;
               -1  0  0  -300;
                0 -1  0   130;
                0  0  0   1];

head_grab_up = [ 0  0  1   450;
               -1  0  0  -300;
                0 -1  0   230;
                0  0  0   1];

head_release = [ 1  0  0   450;
                   0  0  1  -150;
                   0 -1  0   110;
                   0  0  0   1];

head_release_up = [ 1  0  0   450;
                   0  0  1  -150;
                   0 -1  0   210;
                   0  0  0   1];

%% Part 3: Inverse kinematics for each waypoint
% Compute inverse kinematics to obtain 6 joint angles for each waypoint,
% then save them in waypoint_array
%
% waypoint_array = n x 7 array where:
% n = number of waypoints
% First 6 columns in each row = joint angles 1...6, in degrees
% Last column in each row = gripper state (0 for open, 1 for close)

S_matrix = [S1 S2 S3 S4 S5 S6];

waypoints = {standby, ...
    legs_grab_up, legs_grab, legs_grab_up, ...
    legs_release_up, legs_release, legs_release_up, ...
    body_grab_up, body_grab, body_grab_up, ....
    body_release_up, body_release, body_release_up, ...
    head_grab_up, head_grab, head_grab_up, ...
    head_release_up, head_release, head_release_up, ...
    standby};

gripper_open_closed = [0;...
    0; 1; 1; ...
    1; 0; 0; ... 
    0; 1; 1; ...
    1; 0; 0; ...
    0; 1; 1; ...
    1; 0; 0; ...
    0];

n = length(waypoints);

waypoint_array = zeros(n,7);

theta0 = deg2rad([-30 -90 90 -90 -90 150])';

eomg = 1e-3;
ev   = 1e-2;

for i = 1:n
    
    T = waypoints{i};
    
    [theta, success] = IKinSpace(S_matrix, M, T, theta0, eomg, ev);

    if success == 0
        error(['Faild at waypoint ', num2str(i)]);
    end

    theta_deg = rad2deg(theta); theta_deg = wrapTo180(theta_deg); theta_deg = theta_deg';
    waypoint_array(i,1:6) = theta_deg;
    waypoint_array(i,7) = gripper_open_closed(i);
    theta0 = theta;
end

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