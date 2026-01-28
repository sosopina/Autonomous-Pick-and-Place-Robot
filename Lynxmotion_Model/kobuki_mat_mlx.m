%% =========================================================
% Lynxmotion 5DOF (your model) - 2 IK solutions + plot
% Coude haut / coude bas
% Uses YOUR transforms: th_rotz, th_roty, th_rotx, th_trans
%% =========================================================
clear; close all; clc;
addpath('tr_homogene');

%% -------------------------------
% Geometry (PUT YOUR REAL VALUES)
% -------------------------------
L1 = 0.05;   % base -> epaule (z)
L2 = 0.15;   % epaule -> coude (x)
L3 = 0.15;   % coude  -> poignet (x)
L4 = 0.04;   % poignet -> outil (x)
Lp = 0.05;   % pince length (x)

%% -------------------------------
% Target 
% -------------------------------
xd = 0.15;
yd = 0.05;
zd = 0.10;

% Desired pitch in the plane (constraint you wrote)
% theta_des = q2 + q3 + q4
theta_des = pi/2;

% Desired roll (doesn't change position)
q5_des = 0;

%% =========================================================
% STEP 1) q1 from top view
%% =========================================================
q1 = atan2(yd, xd);

%% =========================================================
% STEP 2) Express target in frame 1 (after q1, after +L1)
% Frame 1 axes: obtained by rotz(q1), then translated +L1 in z
% So: p1 = Rz(-q1)*(p0 - [0;0;L1])
%% =========================================================
Rz_m = [ cos(q1) -sin(q1) 0;
         sin(q1)  cos(q1) 0;
         0        0       1 ];

p0 = [xd; yd; zd];
p1 = Rz_m.' * (p0 - [0;0;L1]);   % coordinates in frame 1

x1 = p1(1);
y1 = p1(2); 
z1 = p1(3);

%% =========================================================
% STEP 3) Wrist center (joint 4 origin) using pitch constraint
% End effector offset along x of frame4 = (L4+Lp) * [1;0;0]
% In frame1, direction of that x after pitch theta_des:
% Ry(theta)*[1;0;0] = [cos(theta);0;-sin(theta)]
% So offset in (x,z) is: [L*cos(theta);  -L*sin(theta)]
% => wrist = end - offset  => zw = z1 + L*sin(theta)
%% =========================================================
Le = (L4 + Lp);

xw = x1 - Le*cos(theta_des);
zw = z1 + Le*sin(theta_des);

%% =========================================================
% STEP 4) Planar 2R IK in (x, z) plane for joints q2,q3
% Your planar equations are:
% xw = L2*cos(q2) + L3*cos(q2+q3)
% zw = -L2*sin(q2) - L3*sin(q2+q3)
%
% Put y = -zw to match classic planar arm:
% x = L2*cos(q2) + L3*cos(q2+q3)
% y = L2*sin(q2) + L3*sin(q2+q3)
%% =========================================================
x = xw;
y = -zw;

D = (x^2 + y^2 - L2^2 - L3^2) / (2*L2*L3);

if abs(D) > 1
    error('Target unreachable: |D|>1 (D=%.3f). Check xd,yd,zd,theta or link lengths.', D);
end

% Two elbow configurations
q3_sol = [ atan2( sqrt(1-D^2), D );
           atan2(-sqrt(1-D^2), D ) ];

q2_sol = zeros(2,1);
q4_sol = zeros(2,1);

for i = 1:2
    q3i = q3_sol(i);

    % q2 = atan2(y,x) - atan2(L3*sin(q3), L2 + L3*cos(q3))
    q2i = atan2(y, x) - atan2(L3*sin(q3i), L2 + L3*cos(q3i));

    % pitch constraint
    q4i = theta_des - q2i - q3i;

    q2_sol(i) = q2i;
    q4_sol(i) = q4i;
end

%% =========================================================
%  Display + verify with forward kinematics
%% =========================================================
fprintf('\n===== IK SOLUTIONS (q1 fixed) =====\n');
for i = 1:2
    qvec = [q1; q2_sol(i); q3_sol(i); q4_sol(i); q5_des];

    fprintf('\n--- Solution %d ---\n', i);
    fprintf('q1 = %+8.4f rad\n', qvec(1));
    fprintf('q2 = %+8.4f rad\n', qvec(2));
    fprintf('q3 = %+8.4f rad\n', qvec(3));
    fprintf('q4 = %+8.4f rad\n', qvec(4));
    fprintf('q5 = %+8.4f rad\n', qvec(5));

    T01 = th_rotz(qvec(1)) * th_trans(0,0,L1);
    T12 = th_roty(qvec(2)) * th_trans(L2,0,0);
    T23 = th_roty(qvec(3)) * th_trans(L3,0,0);
    T34 = th_roty(qvec(4)) * th_trans(L4,0,0);
    T45 = th_rotx(qvec(5)) * th_trans(Lp,0,0);

    T05 = T01*T12*T23*T34*T45;
    Wp  = T05(1:3,4);

    fprintf('FK -> Wp = [%.4f %.4f %.4f]^T\n', Wp(1), Wp(2), Wp(3));
    fprintf('Target   = [%.4f %.4f %.4f]^T\n', xd, yd, zd);
end

axis_lim = 0.35; % adjust for your scale

for i = 1:2
    qvec = [q1; q2_sol(i); q3_sol(i); q4_sol(i); q5_des];

    T01 = th_rotz(qvec(1)) * th_trans(0,0,L1);
    T02 = T01 * (th_roty(qvec(2)) * th_trans(L2,0,0));
    T03 = T02 * (th_roty(qvec(3)) * th_trans(L3,0,0));
    T04 = T03 * (th_roty(qvec(4)) * th_trans(L4,0,0));
    T05 = T04 * (th_rotx(qvec(5)) * th_trans(Lp,0,0));

    o0 = [0;0;0];
    o1 = T01(1:3,4);
    o2 = T02(1:3,4);
    o3 = T03(1:3,4);
    o4 = T04(1:3,4);
    o5 = T05(1:3,4);

    P = [o0 o1 o2 o3 o4 o5];

    figure;
    plot3(P(1,:), P(2,:), P(3,:), '-o', 'LineWidth', 2, 'MarkerSize', 6);
    hold on; grid on; axis equal;

    % Target point
    plot3(xd, yd, zd, 'ks', 'MarkerFaceColor', 'k', 'MarkerSize', 8);

    pw1 = [xw; 0; zw];                 
    pw0 = Rz_m * pw1 + [0;0;L1];       
    plot3(pw0(1), pw0(2), pw0(3), 'kd', 'MarkerFaceColor', 'y', 'MarkerSize', 7);

    xlim([-axis_lim axis_lim]); ylim([-axis_lim axis_lim]); zlim([0 0.45]);
    view(135,25);

    if i == 1
        title('Solution 1 (Coude haut)');
    else
        title('Solution 2 (Coude bas)');
    end

    legend('Robot','Target','Wrist center','Location','best');
end

