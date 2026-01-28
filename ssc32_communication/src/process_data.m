clear
close all
clc

% Parametres de Kobuki
r = 0.07 / 2; % rayon des roues en m
L = 0.23; % largeur essieu en m ( dist entre milieu des roues )
Lx=0.25+0.17; % distance mesure camera RGB suivant l'axe Xe du robot
reduction = 6545.0 / 132.0; % rapport de reduction
tickByTour = 52 * reduction; % nb pas codeur / tour de roue
encToRad =  2 * pi/ tickByTour ; % angles roues  gauche, droite= encToRad *  left_encoder , encToRad *  right_encoder


% 1 -temps relevé par robot kobuki en secondes ( environ 1 échantillon toutes les 0.02s )
% 2 -temps en millisecondes modulo 65536
% 3 -control.left_encoder : position angulaire de la roue gauche, en 'pas codeur'
% 4- control.right_encoder: position angulaire de la roue droite, en 'pas codeur'
% 5- control.left_pwm: tension du moteur gauche, entre -128 ( tension =-Ualim  ) et +127 ( tension =127/128. U Alim)
% 6- control.right_pwm : tension du moteur droit
% 7- sensorData.battery : tension de la battery( unité non vérifiée )
% 8- sensorData.over_current : indicateur de sur intensité ( unité non vérifiée )
% 9- currentData.current[0] : courant moteur 0 ( unité et moteur non vérifiés )
% 10-currentData.current[1]: courant moteur 1 ( unité et moteur non vérifiés )
% 11-control.vx : (vitesse lineaire de référence pour e_g kobuki: entrée de commande, en m/s )
% 12-control.wz: (vitesse angulaire de référence pour e_g kobuki: entrée de commande, en rad/s )
% 13-control.measure ( mesure camera  en m: depend de control.type_measure, voir initControl dans control.cpp)
% 14- pas encore implementé, angle moyen cible en rad, en mode camera de profondeur )

% wD=vx/r + L/(2R) . wz                   , vx=r/2 .(wD+wG)
% 
% wG=vx/r -LY/(2R) . wz                   , wz=r/L .(wD-wG)

%Chargement de données
%data = load('data_kobuki_kp_kd_ki.txt');t0=2;t1=25;
%data = load('data_kobuki_kd.txt');t0=0;t1=inf;
data = load('data_kobuki.txt');t0=0;t1=inf;


t=data(:,1);t=t-t(1);
k=find((t>t0)&(t<t1));data=data(k,:);
t=data(:,1);t=t-t(1);

e_g=data(:,3); % position angulaire de la roue gauche, en 'pas codeur'
e_d=data(:,4); % position angulaire de la roue droite, en 'pas codeur'
pwm_g=data(:,5); %tension du moteur gauche
pwm_d=data(:,6); %tension du moteur droit
vx=data(:,11); %vitesse lineaire  en m/s
wz =data(:,12); %vitesse angulaire en rad/s
measure =data(:,13); %mesure
current_d=data(:,9);
current_l=data(:,10);

figure;
subplot(2,1,1);plot(t,measure);grid on; hold on; title('mesure camera en  m = f(t en s)');
subplot(2,1,2);plot(t,wz);grid on; hold on; title('vitesse angulaire en rad/s = f(t en s)');
