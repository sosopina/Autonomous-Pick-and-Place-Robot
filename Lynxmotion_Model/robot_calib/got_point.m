%% =========================================================
% IK – Lynxmotion 5DOF
% Cible = pointe de la pince
% Contrainte : pince vers le bas (theta_des = +pi/2)
%% =========================================================
clear; close all; clc;
addpath('tr_homogene');

%% -------------------------------
% Geometry (REAL VALUES)
% -------------------------------
L1 = 0.04;
L2 = 0.15;
L3 = 0.18;
L4 = 0.035;
Lp = 0.05;
Le = L4 + Lp;

%% -------------------------------
% Target point (PINCE TIP)
% -------------------------------
xd = 0.15;
yd = 0.15;
zd = 0.10;

theta_des = +pi/2;   % pince vers le bas
q5_des = 0;

%% =========================================================
% STEP 1 — q1 (vue du dessus)
%% =========================================================
q1 = atan2(yd, xd);

%% =========================================================
% STEP 2 — cible exprimée dans le repère 1
%% =========================================================
Rz = [ cos(q1) -sin(q1) 0;
       sin(q1)  cos(q1) 0;
       0        0       1 ];

p0 = [xd; yd; zd];
p1 = Rz.' * (p0 - [0;0;L1]);

x1 = p1(1);
z1 = p1(3);

%% =========================================================
% STEP 3 — centre du poignet (retrait pince)
%% =========================================================
% Direction outil selon theta_des
% Ry(theta)*[1;0;0] = [cos(theta); 0; -sin(theta)]

xw = x1 - Le*cos(theta_des);
zw = z1 + Le*sin(theta_des);

%% =========================================================
% STEP 4 — IK planaire 2R (q2, q3)
%% =========================================================
x = xw;
y = -zw;

D = (x^2 + y^2 - L2^2 - L3^2) / (2*L2*L3);

if abs(D) > 1
    error('Target unreachable (|D| > 1)');
end

% Deux solutions : coude haut / coude bas
q3_sol = [ atan2( sqrt(1-D^2), D );
           atan2(-sqrt(1-D^2), D ) ];

q2_sol = zeros(2,1);
q4_sol = zeros(2,1);

for i = 1:2
    q3i = q3_sol(i);

    q2i = atan2(y,x) - atan2(L3*sin(q3i), L2 + L3*cos(q3i));

    % contrainte orientation
    q4i = theta_des - q2i - q3i;

    q2_sol(i) = q2i;
    q4_sol(i) = q4i;
end

%% =========================================================
% DISPLAY + VERIFICATION MGD
%% =========================================================
fprintf('\n===== IK SOLUTIONS (PINCE = CIBLE) =====\n');

axis_lim = 0.35;

for i = 1:2
    q = [q1; q2_sol(i); q3_sol(i); q4_sol(i); q5_des];

    fprintf('\n--- Solution %d ---\n', i);
    fprintf('q1 = %+8.4f rad\n', q(1));
    fprintf('q2 = %+8.4f rad\n', q(2));
    fprintf('q3 = %+8.4f rad\n', q(3));
    fprintf('q4 = %+8.4f rad\n', q(4));

    %% MGD
    T01 = th_rotz(q(1)) * th_trans(0,0,L1);
    T12 = th_roty(q(2)) * th_trans(L2,0,0);
    T23 = th_roty(q(3)) * th_trans(L3,0,0);
    T34 = th_roty(q(4)) * th_trans(L4,0,0);
    T45 = th_rotx(q(5)) * th_trans(Lp,0,0);

    T02 = T01*T12;
    T03 = T02*T23;
    T04 = T03*T34;
    T05 = T04*T45;

    Wp = T05(1:3,4);

    fprintf('FK -> Wp = [%.4f %.4f %.4f]^T\n', Wp);
    fprintf('Target   = [%.4f %.4f %.4f]^T\n', xd, yd, zd);

    %% Plot
    o0 = [0;0;0];
    o1 = T01(1:3,4);
    o2 = T02(1:3,4);
    o3 = T03(1:3,4);
    o4 = T04(1:3,4);
    o5 = T05(1:3,4);

    P = [o0 o1 o2 o3 o4 o5];

    figure;
    plot3(P(1,:),P(2,:),P(3,:),'-o','LineWidth',2);
    hold on; grid on; axis equal;
    plot3(xd,yd,zd,'ks','MarkerFaceColor','k');
    plot3(xw*cos(q1), xw*sin(q1), zw+L1, ...
          'kd','MarkerFaceColor','y'); % centre poignet

    title('IK solution');

    xlabel X; ylabel Y; zlabel Z;
    xlim([-axis_lim axis_lim]);
    ylim([-axis_lim axis_lim]);
    zlim([0 0.45]);
    view(135,25);
end

