S=load('test_delta_3.mat');

data_values=S.data_values; 
angleDroite=data_values('angleDroite');
speed=data_values('speed');
omegaD=data_values('Omega Droite');

U=data_values('U');
log_time = S.log_time; 
%
% figure;
% hold on;
% plot(log_time,angleDroite,'DisplayName','Angle Droite');
% plot(log_time,speed,'DisplayName','Speed');
% plot(log_time,omegaD,'DisplayName','OmegaD');
% grid on;
% legend('Location', 'best');
% hold off;

simu=iddata((angleDroite(:)-angleDroite(1))*2*pi/360,U(:),0.005); % (:) transforme en vecteur colonne > 1 colonne remplie de mesures, et pas l'inverse
% on retire la valeur initiale de l'angle pour avoir une CI = 0

sys_est=tfest(simu,2,0); % 2 pôles, 0 zéros a/s(s+b)

figure
compare(simu, sys_est)  % compare entre le modèle iddata et celui estimé

a_est = sys_est.Numerator  % Numérateur constant
b_est = sys_est.Denominator % Coefficient de s au dénominateur


% a = {3.02e3;}
% b = {18,9;}
% Ir = {7}
% corr = {99,82;98.71}

eta=30;  % rapport de réduction
R=2.14; % Ohm
Iw=1.54e-5;% kg.m²
k=0.338; % V.s

Ir_1 =  1/(eta^2)*(k*eta/(R*a_est)-Iw);
Ir_2 = 1/(eta^2)*((k*eta)^2/(R*b_est(2))-Iw);

% recalcule la valeur de K pour vérification
K_0 = b_est(2)/(a_est*eta);
K_1 = a_est*R*(Iw+eta^2*Ir_1)/eta;
K_2 = sqrt(b_est*R*(Iw+eta^2*Ir_2))/eta;

% --- Affichage formaté en notation scientifique (3 chiffres significatifs)
fprintf('Ir_1 = %.3e\n', Ir_1);
fprintf('Ir_2 = %.3e\n', Ir_2);