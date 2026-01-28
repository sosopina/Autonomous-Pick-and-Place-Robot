%% =========================================================
% MODELE GEOMETRIQUE DIRECT & INVERSE
% Bras Lynxmotion 5 DOF sur base Kobuki
% (méthode analytique – inspirée du cours)
% =========================================================
clear; close all; clc;
addpath('tr_homogene');
addpath('equations_de_paul');
%% -------------------------------
% Paramètres géométriques
% (à adapter selon mesures réelles)
% -------------------------------
% L1 = 0.10; % base -> épaule (m)
% L2 = 0.12; % épaule -> coude
% L3 = 0.12; % coude -> poignet
% L4 = 0.08; % poignet -> pince
% Lp = 0.05; % longueur pince
%% -------------------------------
% Variables symboliques
% -------------------------------
syms q1 q2 q3 q4 q5 real
syms xd yd zd thetad real
syms L1 L2 L3 L4 Lp h real
  
Pi = sym(pi); 
q = [q1 q2 q3 q4 q5]';

%% =========================================================
% MODELE GEOMETRIQUE DIRECT (MGD)
% =========================================================  

T01 = th_rotz(q1) * th_trans(0,0,L1);
T12 = th_roty(q2) * th_trans(L2,0,0);
T23 = th_roty(q3) * th_trans(L3,0,0);
T34 = th_roty(q4) * th_trans(L4,0,0);
T45 = th_rotx(q5) * th_trans(Lp,0,0);
 
T05 = simplify(T01 * T12 * T23 * T34 * T45);
 
% Position cartésienne de la pince

Wp = simplify(T05(1:3,4));
  
disp('--- MGD : Position cartésienne de la pince Wp = [x;y;z] ---');
disp(Wp);  

%% =========================================================
% VERIFICATION NUMERIQUE DU MGD
% =========================================================
q_test = [0; 0*-Pi/2; 0*Pi/2; 0;0];  

Wp_num = subs(Wp, ...
q, q_test);
Wp_num = simplify(Wp_num)  

disp('--- Vérification numérique du MGD ---');
disp(Wp_num);  

%% =========================================================
% MODELE GEOMETRIQUE INVERSE (MGI)
% =========================================================

r5P = sym([0;0;0])
Wp_des = [xd; yd; zd];
r1T5 = T12 * T23 * T34 * T45 ;
r1P = r1T5(1:3,:) * [r5P;1]
r1P = simplify(r1P)

%% --- Etape 1 : calcul de q1 (vue du dessus)
q1_sol = atan2(yd, xd);  
disp('MGI : q1 = atan2(yd, xd)');

%% --- Passage dans le repère 1
r1T0 = th_invT(T01);  
r1P_des = simplify(r1T0(1:3,:)*[Wp_des;1])
syms q23 real;
r1P = subs(r1P,[q2+q3+q4,q2+q3],[thetad,q23])  
r1ERR = r1P - r1P_des

equation=identif_equ_paul([q2,q23],r1ERR([1,3]),"q2_q23")

disp('--- Point désiré exprimé dans le repère 1 ---');

disp(P1);

y1 = P1(2);
z1 = P1(3);

%% --- Hypothèse d'orientation

% La pince est orientée vers le sol
theta = -pi/2;

disp('Contrainte orientation : q2 + q3 + q4 = -pi/2');

%% --- Résolution plane

disp('MGI : résolution plane (méthode analytique – équations trigonométriques)');

%% =========================================================
% FIN DU SCRIPT
% =========================================================