% Initialisation
eta = 30;  % rapport de réduction
R = 2.14;  % Ohm
Iw = 1.54e-5; % kg.m²
k = 0.338;    % V.s

Ir_1_all = [];  % vecteur pour stocker les Ir_1
Ir_2_all = [];  % vecteur pour stocker les Ir_2
file_names = {}; % pour stocker les noms de fichiers correspondants
a_all = [];      % vecteur pour stocker les valeurs de a
b_all = [];      % vecteur pour stocker les valeurs de b

% --- Boucle sur les fichiers ID_param_step**
for i = 1:3
    for j = 1:2

        file_name = sprintf('ID_param_step%d%d.mat', i, j);
        fprintf('\n=== Traitement du fichier : %s ===\n', file_name);

        % Chargement des données
        S = load(file_name);

        data_values = S.data_values; 
        angleDroite = data_values('angleDroite');
        speed = data_values('speed');
        omegaD = data_values('Omega Droite');

        U = data_values('U');
        log_time = S.log_time; 

        % Simulation des données
        simu = iddata((angleDroite(:)-angleDroite(1))*2*pi/360, U(:), 0.005); 

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

        % Stockage des résultats
        Ir_1_all = [Ir_1_all; Ir_1];
        Ir_2_all = [Ir_2_all; Ir_2];
        file_names{end+1} = file_name;

        % Affichage formaté en notation scientifique (3 chiffres significatifs)
        fprintf('Ir_1 = %.3e\n', Ir_1);
        fprintf('Ir_2 = %.3e\n', Ir_2);

    end
end

%% --- Moyennes finales ---
Ir_1_mean = mean(Ir_1_all);  
Ir_2_mean = mean(Ir_2_all);  
a_mean = mean(a_all);
b_mean = mean(b_all);

fprintf('\n=== Moyennes des résultats ===\n');
fprintf('Moyenne Ir_1 = %.3e\n', Ir_1_mean);
fprintf('Moyenne Ir_2 = %.3e\n', Ir_2_mean);
fprintf('Moyenne a = %.3e\n', a_mean);
fprintf('Moyenne b = %.3e\n', b_mean);

%% Import d'un autre fichier si besoin
S = load('data_sim_monté.mat');

data_values = S.data_values; 
angleDroite = data_values('angleDroite');
speed = data_values('speed');
omegaD = data_values('Omega Droite');
U = data_values('U');
log_time = S.log_time; 

% Simulation des données
simu = iddata((angleDroite(:)-angleDroite(1))*2*pi/360, U(:), 0.005); 

%% Cherche le fit

% Paramètres aléatoires pour a' et b'
a_prime = a_mean;   % k*eta/(R*(Iw+eta^2*Ir_1))
b_prime = b_mean(2);   % (k*eta)^2/(R*(Iw+eta^2*Ir_1))
% Créer un modèle de simulation avec ces valeurs a' et b'
sys_sim = tf([a_prime], [1, b_prime,0.0]);

% Comparaison avec les données d'entrée (si nécessaire)
figure;
compare(simu, sys_sim);  % Compare entre les données réelles et le modèle estimé
title('Comparaison avec les valeurs de Ir obtenues');

%% Valeurs
% === Traitement du fichier : ID_param_step11.mat ===
% Ir_1 = 6.619e-05
% Ir_2 = 1.827e-03
% 
% === Traitement du fichier : ID_param_step12.mat ===
% Ir_1 = 1.255e-04
% Ir_2 = 3.414e-03
% 
% === Traitement du fichier : ID_param_step21.mat ===
% Ir_1 = 8.987e-05
% Ir_2 = 2.467e-03
% 
% === Traitement du fichier : ID_param_step22.mat ===
% Ir_1 = 8.424e-05
% Ir_2 = 2.308e-03
% 
% === Traitement du fichier : ID_param_step31.mat ===
% Ir_1 = 6.998e-05
% Ir_2 = 1.890e-03
% 
% === Traitement du fichier : ID_param_step32.mat ===
% Ir_1 = 8.449e-05
% Ir_2 = 2.283e-03
% 
% === Moyennes des résultats ===
% Moyenne Ir_1 = 8.671e-05
% Moyenne Ir_2 = 2.365e-03
% Moyenne a = 6.334e+01
% Moyenne b = 2.354e+01
