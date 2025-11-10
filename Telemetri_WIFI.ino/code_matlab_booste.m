% WiFi settings
IP = '192.168.4.1';
PORT_NUMBER = 80;

% USB settings
port_name = 'COM5';
baudrate = 230400;

% Recording time in seconds
Tmax = 8;

% Communication method
method = 'WiFi';   % 'WiFi' pour la télémétrie de MecatroUtils, sinon 'USB'

% ===== PARAMETRES A ENVOYER AU ROBOT =====
% ATTENTION : Maintenant on envoie 6 valeurs au lieu de 5
% Ordre : [K_moins, Kp, Ki, K_lambda, Vbatt, u_barre]

Kp = 12.0;           % Gain proportionnel vitesse
Ki = 6.0;            % Gain intégral vitesse
K_moins = 24.0;      % Gain proportionnel direction
K_lambda = 7.0;      % Gain intégral direction
Vbatt = 11.2;        % Tension batterie mesurée
u_barre = 0.3;       % NOUVEAU : Vitesse cible en m/s

% Tableau des gains à envoyer (6 valeurs maintenant)
K = [K_moins, Kp, Ki, K_lambda, Vbatt, u_barre];

fprintf('=== PARAMETRES A ENVOYER ===\n');
fprintf('K_moins = %.2f\n', K_moins);
fprintf('Kp = %.2f\n', Kp);
fprintf('Ki = %.2f\n', Ki);
fprintf('K_lambda = %.2f\n', K_lambda);
fprintf('Vbatt = %.2f V\n', Vbatt);
fprintf('u_barre = %.2f m/s\n', u_barre);
fprintf('============================\n\n');

% Acquisition
if strcmp(method,'WiFi')
    [log_time, data_values, line_idx] = get_data_WiFi(IP, PORT_NUMBER, Tmax, K);
elseif strcmp(method,'USB')
    [log_time, data_values, line_idx] = get_data_USB(port_name, Tmax, baudrate, K);
else
    error('Use a valid connection method')
end

% Affiche les noms de variables loguées par le firmware
disp('Variables recues du robot:')
disp(data_values.keys)

% Tracé des données
figure;
hold on;
legend_labels = data_values.keys;
for i = 1:numel(legend_labels)
    d = legend_labels{i};
    v = data_values(d);
    plot(log_time, v, 'DisplayName', d, 'LineWidth', 1.5);
end
hold off;
legend('Location', 'best');
grid on;
xlabel('Temps (s)');
ylabel('Valeurs');
title('Telemetrie Robot Suiveur de Ligne');

% Tracé séparé pour mieux visualiser
figure;

% Lambda et position SensorBar
subplot(3,1,1);
hold on;
plot(log_time, data_values('lambda'), 'b-', 'LineWidth', 1.5, 'DisplayName', 'lambda');
plot(log_time, data_values('SensorBar'), 'r--', 'LineWidth', 1.5, 'DisplayName', 'SensorBar');
hold off;
legend('Location', 'best');
grid on;
ylabel('Position (m)');
title('Ecart lateral');

% Vitesses angulaires
subplot(3,1,2);
hold on;
plot(log_time, data_values('Omega Droite'), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Omega Droite');
plot(log_time, data_values('Omega Gauche'), 'r-', 'LineWidth', 1.5, 'DisplayName', 'Omega Gauche');
hold off;
legend('Location', 'best');
grid on;
ylabel('Vitesse angulaire (rad/s)');
title('Vitesses des roues');

% Tensions de commande
subplot(3,1,3);
hold on;
plot(log_time, data_values('U_plus'), 'g-', 'LineWidth', 1.5, 'DisplayName', 'U_{plus}');
plot(log_time, data_values('U_moins'), 'm-', 'LineWidth', 1.5, 'DisplayName', 'U_{moins}');
plot(log_time, data_values('u')*10, 'k--', 'LineWidth', 1.5, 'DisplayName', 'u (vitesse x10)');
hold off;
legend('Location', 'best');
grid on;
xlabel('Temps (s)');
ylabel('Tension (V)');
title('Commandes moteurs');

fprintf('\n=== ACQUISITION TERMINEE ===\n');