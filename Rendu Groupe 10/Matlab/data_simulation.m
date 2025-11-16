%% Simulation du filtre, recherche du TAU optimal

S=load('ID_param_step2_11.mat');

data_values=S.data_values; 
angleDroite=data_values('angleDroite');
log_time = S.log_time; 


% Paramètres de la boucle de contrôle
CONTROL_LOOP_PERIOD = 3e-3;   % [s]
TAU = 0;                 % constante de filtrage (s)

figure;
hold on;
plot(log_time,data_values('speed'),'DisplayName','Speed');

for i=2:10  % itération sur différentes valeurs de TAU

    TAU=50e-3+TAU;
    % Coefficients du filtre
    a1 =-(1 - 2*TAU / CONTROL_LOOP_PERIOD);
    b1 = (1 + 2*TAU / CONTROL_LOOP_PERIOD);
    
    N=numel(angleDroite);
    
    % Initialisation
    OmegaDtest = zeros(1, N);   % vecteur de vitesses 
    OmegaDtest_prev = 0;
    
    % Boucle de calcul (implémentation du filtre dérivateur de Tustin)
    for k = 2:N
        OmegaDtest(k) = (2/CONTROL_LOOP_PERIOD* (angleDroite(k) - angleDroite(k-1)) + a1 * OmegaDtest_prev)/b1;
        OmegaDtest_prev = OmegaDtest(k);
    end

    plot(log_time, OmegaDtest, 'DisplayName', sprintf('TAU : %.2f',TAU));
end

title(sprintf('Omega droite (filtre de Tustin)'));
xlabel('Temps (s)');     
ylabel('Vitesse (°/s)');
legend('Location', 'best');
grid on;


%% Vérification de la vitesse rpm
S=load('test_delta_1.mat');

data_values=S.data_values; 
speed = data_values('speed');
VD=data_values('Omega Droite');
log_time = S.log_time;

figure;
hold on;
OmegaDtest_rpm= VD*60/360;
speed=speed*60/360;
plot(log_time, OmegaDtest_rpm,'DisplayName','Filtrage rpm');
plot(log_time,speed,'DisplayName', 'Encodeur rpm');
title('Test puissance max à 11.2V');
xlabel('Temps (s)');     
ylabel('Vitesse (rpm)');
grid;
hold off;
legend('Location', 'best');

%% Différence de vitesse des moteurs
S=load('diff_vitesse2.mat');

data_values=S.data_values; 
VD=data_values('Omega Droite');
VG=data_values('Omega Gauche');
log_time = S.log_time;

corr=1.06;
figure
hold on;
VD_corr= VD*corr; %vitesse corrigée
plot(log_time,VD_corr,'DisplayName','VD corrigée');
plot(log_time, VD, 'DisplayName', 'VD');
plot(log_time,VG,'DisplayName','VG');
title('Test vitesse  corrigée');
xlabel('Temps (s)');     
ylabel('Vitesse (°/s)');
hold off;
grid;
legend('Location', 'best');

%% Itérations pour l'identification de paramètres

% Constantes utiles

eta = 30;  % rapport de réduction
R = 2.14;  % Ohm
Iw = 1.54e-5; % kg.m²
k = 0.338;    % V.s

Vbatt=10.9; % à modifier 

Ir_1_all = [];  % vecteur pour stocker les Ir_1 (moments d'inertie issus de la 1ère équation)
Ir_2_all = [];  % vecteur pour stocker les Ir_2
nu_all=[];         % coefficients de frottement
file_names = {}; % pour stocker les noms de fichiers correspondants
a_all = [];      % valeurs de a
b_all = [];      % valeurs de b

% --- Boucle sur les fichiers ID_param_step**
for i = 1:3
    for j = 1:2

        % Chargement des données
        S = load(file_name);

        data_values = S.data_values; 
        angleDroite = data_values('angleDroite');

        U_plus = data_values('U_plus');
        U_moins = data_values('U_moins');
        U=min((U_plus+U_moins)/Vbatt,12);

        log_time = S.log_time; 

        % Simulation des données
        simu = iddata((angleDroite(:)-angleDroite(1))*2*pi/360, U(:), 0.003); 

        % Estimation du système
        sys_est = tfest(simu,2,0); % 2 pôles, 0 zéros a/s(s+b)

        % Extraction des coefficients
        a_est = sys_est.Numerator;  % Numérateur constant
        b_est = sys_est.Denominator; % Coefficient de s au dénominateur

        % Stockage des résultats de a et b
        a_all = [a_all; a_est];
        b_all = [b_all; b_est];

        % Calculs des inerties
        Ir_1 = 1/(eta*eta)*(k*eta/(R*a_est)-Iw);
        Ir_2 = 1/(eta*eta)*((k*eta)^2/(R*b_est(2))-Iw);
        nu=b_est(2)*(Iw+eta^2*Ir_1)*R/((k*eta)^2);

        % Stockage des résultats
        Ir_1_all = [Ir_1_all; Ir_1];
        Ir_2_all = [Ir_2_all; Ir_2];
        nu_all = [nu_all; nu];
        file_names{end+1} = file_name;

    end
end

% --- Moyennes finales ---
Ir_1_mean = mean(Ir_1_all);  
Ir_2_mean = mean(Ir_2_all);  
nu_mean = mean(nu_all);
a_mean = mean(a_all);
b_mean = mean(b_all);

fprintf('\n=== Moyennes des résultats ===\n');
fprintf('Moyenne Ir_1 = %.3e\n', Ir_1_mean);
fprintf('Moyenne Ir_2 = %.3e\n', Ir_2_mean);
fprintf('Moyenne nu = %.3e\n', nu_mean);
fprintf('Moyenne a = %.3e\n', a_mean);
fprintf('Moyenne b = %.3e\n', b_mean);

% Créer un modèle de simulation avec les valeurs moyennes obtenues
sys_sim = tf(a_mean, [1, b_est(2),0.0]);
figure;
compare(simu, sys_sim);  % Compare entre les données réelles (du dernier fichier de données ici) et le modèle estimé
title('Comparaison avec le modèle moyen obtenu');

% Valeurs obtenues
%
% === Moyennes des résultats, sans frottements ===
% Moyenne Ir_1 = 8.671e-05
% Moyenne Ir_2 = 2.365e-03
% Moyenne a = 6.334e+01
% Moyenne b = 2.354e+01
% 
% === Moyennes des résultats, avec frottements ===
% Moyenne Ir_1 = 8.755e-06
% Moyenne Ir_2 = 2.370e-03
% Moyenne nu = 3.702e-03
% Moyenne a = 6.036e+02
% Moyenne b = 2.266e+01
