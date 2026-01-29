%% =========================================================
% FORWARD KINEMATICS TEST (q playground)
% Lynxmotion 5DOF
%% =========================================================
clear; close all; clc;
addpath('tr_homogene');

%% -------------------------------
% Geometry (REAL VALUES)
% -------------------------------
L1 = 0.06;   % base -> shoulder (z)
L2 = 0.15;   % shoulder -> elbow (x)
L3 = 0.17;   % elbow -> wrist (x)
L4 = 0.03;   % wrist -> tool (x)
Lp = 0.05;   % gripper length

%% -------------------------------
% TEST JOINT VALUES (EDIT HERE)
% -------------------------------
q1 = 0;              % base
q2 = -pi/6;              % shoulder
q3 = pi/4;          % elbow
q4 = pi/6;              % wrist pitch
q5 = pi/4;              % wrist roll

q = [q1 q2 q3 q4 q5]';

%% =========================================================
% FORWARD KINEMATICS
%% =========================================================
T01 = th_rotz(q1) * th_trans(0,0,L1);
T12 = th_roty(q2) * th_trans(L2,0,0);
T23 = th_roty(q3) * th_trans(L3,0,0);
T34 = th_roty(q4) * th_trans(L4,0,0);
T45 = th_rotx(q5) * th_trans(Lp,0,0);

T02 = T01*T12;
T03 = T02*T23;
T04 = T03*T34;
T05 = T04*T45;

%% -------------------------------
% Joint positions
% -------------------------------
o0 = [0;0;0];
o1 = T01(1:3,4);
o2 = T02(1:3,4);
o3 = T03(1:3,4);
o4 = T04(1:3,4);
o5 = T05(1:3,4);

P = [o0 o1 o2 o3 o4 o5];

%% -------------------------------
% Plot
% -------------------------------
figure;
plot3(P(1,:), P(2,:), P(3,:), '-o', ...
      'LineWidth', 2, 'MarkerSize', 6);
grid on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Forward Kinematics – q test');

xlim([-0.35 0.35]);
ylim([-0.35 0.35]);
zlim([0 0.45]);
view(135,25);

%% -------------------------------
% Print end-effector
% -------------------------------
Wp = o5;
fprintf('Wp = [%.3f %.3f %.3f]^T\n', Wp);

