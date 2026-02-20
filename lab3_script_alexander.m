%% Lab 3 (Inverse Kinematics) template
% MAE204
% 
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

%putting link lengths for easier memory
%cross products seem to give the same results, going w the standard way for
%easier debugging

l1=[0,-300,88]';
w1=[0,0,1];
w1_skew=VecToso3(w1);
v1=w1_skew*-l1;
%v1=cross(l1,w1)

l2=[0,-300,152+88]';
w2=[0, 1, 0];
%v2=cross(l2,w2)
w2_skew=VecToso3(w2);
v2=w2_skew*-l2;
v2=v2';

l3=[244,-300+120,152+88]';
w3=[0, 1, 0];
%v3=cross(l3,w3);
w3_skew=VecToso3(w3);
v3=w3_skew*-l3;
v3=v3';

l4=[244+213,-300+120-93,152+88]'; %not sure if the y offsets are right,
% but it doesn't really matter because it's parallel with the axis of rotation
w4=[0, 1, 0];
%v4=cross(l4,w4);
w4_skew=VecToso3(w4);
v4=w4_skew*-l4;
v4=v4';

l5=[244+213,-300+120-93+104,152+88-85]';
w5=[0, 0, -1];
%v5=cross(l5,w5);
w5_skew=VecToso3(w5);
v5=w5_skew*-l5;
v5=v5';

l6=[244+213,-300+120-93+104,152+88-85]';
w6=[0, 1, 0];
%v6=cross(l6,w6);
w6_skew=VecToso3(w6);
v6=w6_skew*-l6;
v6=v6';

%all S exactly the same as lab 2
S1 = [0, 0, 1, -300, 0, 0]';
S2=[w2,v2]';
S3=[w3,v3]';
S4=[w4,v4]';
S5=[w5,v5]';
S6=[w6,v6]';

% Next, define the M matrix (the zero-position e-e transformation matrix),
% in (mm)
x1=457;
y1=-300+120-93+104+92+155;
z1=88+152-85;

M = [1,0,0,x1;0,1,0,y1;0,0,1,z1;0,0,0,1]


%% Part 2: UR3e sequence planning
% You may use this space to define the waypoints for your sequence (I
% recommend using SE(3) matrices to define gripper configurations)

%I am going to have all waypoints face down

%I assume that the gripper grabs after reaching the position, might need to adjust if thats not the case 
rot_standby=[0,0,1;-1,0,0;0,-1,0];
rot_legs_grab=[1,0,0;0,0,1;0,-1,0];
brow=[0,0,0,1]; %bottom row
gopen=0;
gclose=1;

standby = [0, 0, 1, 323.6; -1, 0, 0, -335.6; 0, -1, 0, 237; 0, 0, 0, 1];
sequence.a=standby;
gripper(1)=gopen;
gopen;

legs_grab = [rot_legs_grab,[400;-200;50];brow]
sequence.b=legs_grab;
gripper(2)=gclose;
gclose;

legs_grab_above=[rot_legs_grab,[400;-200;50+100];brow] %just add 100 to all the z coordinates
sequence.c=legs_grab_above;
gripper(3)=gclose;

legs_release_above=[rot_legs_grab,[450;-150;55+100];brow]
sequence.d=legs_release_above;
gripper(4)=gclose;

legs_release = [rot_legs_grab,[450;-150;55];brow]
sequence.e=legs_release;
gripper(5)=gopen;

legs_release_above;
sequence.f=legs_release_above;
gripper(6)=gopen;

body_grab_above=[rot_standby,[450;-450;70+100];brow]
sequence.g=body_grab_above;
gripper(7)=gopen;

body_grab = [rot_standby,[450;-450;70];brow]
sequence.h=body_grab;
gripper(8)=gclose;

body_grab_above;
sequence.i=body_grab_above;
gripper(9)=gclose;

body_release_above=[rot_legs_grab,[450;-150;85+100];brow]
sequence.j=body_release_above;
gripper(10)=gclose;

body_release = [rot_legs_grab,[450;-150;85];brow]
sequence.k=body_release;
gripper(11)=gopen;

body_release_above;
sequence.l=body_release_above;
gripper(12)=gopen;

head_grab_above=[rot_standby,[450;-300;130+100];brow];
sequence.m=head_grab_above;
gripper(13)=gopen;

head_grab = [rot_standby,[450;-300;130];brow]
sequence.n=head_grab;
gripper(14)=gclose;

head_grab_above;
sequence.o=head_grab_above;
gripper(15)=gclose;

head_release_above=[rot_legs_grab,[450;-150;110+100];brow]
sequence.p=head_release_above;
gripper(16)=gclose;

head_release = [rot_legs_grab,[450;-150;110];brow]
sequence.q=head_release;
gripper(17)=gopen;

head_release_above;
sequence.r=head_release_above;
gripper(18)=gopen;

standby;
sequence.s=standby;
gripper(19)=gopen;


%% Part 3: Inverse kinematics for each waypoint
% Compute inverse kinematics to obtain 6 joint angles for each waypoint,
% then save them in waypoint_array
%
% waypoint_array = n x 7 array where:
% n = number of waypoints
% First 6 columns in each row = joint angles 1...6, in degrees
% Last column in each row = gripper state (0 for open, 1 for close)

%calcangles=[-57.0723688902829;100.39552296371;-1.57066349116103;-75.262914570626;23.5619449019235;-53.9307762366931];
calcangles=[-30;-90;90;-90;-90;150];
calcangles=calcangles/57.2958;
%in my tests, the last standby seemed right but the first one seemed off.
%So, I just put in the last set of standby angles for my initial guess.
%Kind of a sketchy method but I hope it works.
%calcangles=zeros(6,1)

Slist=[S1,S2,S3,S4,S5,S6];

eomg=0.001;
ev=0.01;

fields=fieldnames(sequence);
waypoint_array=[];

for i = 1:numel(fields) %apparantly this is the best way to loop through a struct according to google...
name = fields{i};
T = sequence.(name);
calcangles=IKinSpace(Slist,M,T,calcangles,eomg,ev);

calcangles_gripper=[calcangles*57.2958;gripper(i)];%gripper state, have to concatenate as column
%separating out the gripper because I want to pass it back to the IK in
%space function, so I need calcangles to stay length 6

waypoint_array = cat(1,waypoint_array,calcangles_gripper');
end

%waypoint_array = calcangles=IKinSpace(Slist,M,standby,thetalist0,eomg,ev)
%waypoint_array = [calcangles',gopen]%possibly not a great idea, but I just figure feeding the 
% last set of calculated angles in for the initial guess might be ok

%sanity check: I have a different set of angles for the first+last standby.
%I wonder if they both have the same position when testing via forward
%kinematics?

S1_skew=VecTose3(S1);
S2_skew=VecTose3(S2);
S3_skew=VecTose3(S3);
S4_skew=VecTose3(S4);
S5_skew=VecTose3(S5);
S6_skew=VecTose3(S6);

T1 = expm(S1_skew*waypoint_array(1,1))*expm(S2_skew*waypoint_array(1,2))*expm(S3_skew*waypoint_array(1,3))*expm(S4_skew*waypoint_array(1,4))*expm(S5_skew*waypoint_array(1,5))*expm(S6_skew*waypoint_array(1,6))*M;

T2=expm(S1_skew*waypoint_array(end,1))*expm(S2_skew*waypoint_array(end,2))*expm(S3_skew*waypoint_array(end,3))*expm(S4_skew*waypoint_array(end,4))*expm(S5_skew*waypoint_array(end,5))*expm(S6_skew*waypoint_array(end,6))*M;

for i=1:numel(fields)
Ttest=expm(S1_skew*waypoint_array(i,1)/57.2958)*expm(S2_skew*waypoint_array(i,2)/57.2958)*expm(S3_skew*waypoint_array(i,3)/57.2958)*expm(S4_skew*waypoint_array(i,4)/57.2958)*expm(S5_skew*waypoint_array(i,5)/57.2958)*expm(S6_skew*waypoint_array(i,6)/57.2958)*M;
name = fields{i};
sequence.(name);
if sum(sum(abs(Ttest-sequence.(name))))<10*eomg
    disp('Looks good')
else
    disp('Houston, we have a problem')
    disp(Ttest)
    disp(sequence.(name))
end
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