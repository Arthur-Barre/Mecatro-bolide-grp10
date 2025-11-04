S=load('test_delta_1.mat');

data_values=S.data_values; 
angleDroite=data_values('angleDroite');
%disp(angleDroite);

log_time = S.log_time; 

%% Filtre

% Paramètres de la boucle de contrôle
CONTROL_LOOP_PERIOD = 3e-3;   % [s]
TAU = 0;                 % constante de filtrage (s)

figure;
hold on;
plot(log_time,data_values('speed'),'DisplayName','Speed');

for i=2:10

    TAU=50e-3+TAU;
    % Coefficients du filtre dérivateur (méthode de Tustin)
    a1 =-(1 - 2*TAU / CONTROL_LOOP_PERIOD);
    b1 = (1 + 2*TAU / CONTROL_LOOP_PERIOD);
    
    N=numel(angleDroite);
    
    % Initialisation
    OmegaDtest = zeros(1, N);   % vecteur de vitesses V_Milan
    OmegaDtest_prev = 0;
    
    % Boucle de calcul (implémentation du filtre dérivateur de Tustin)
    for k = 2:N
        OmegaDtest(k) = (2/CONTROL_LOOP_PERIOD* (angleDroite(k) - angleDroite(k-1)) + a1 * OmegaDtest_prev)/b1;
        OmegaDtest_prev = OmegaDtest(k);
    end

    % Prepare for plotting the filtered angular velocity
    plot(log_time, OmegaDtest, 'DisplayName', sprintf('TAU : %.2f',TAU));
    %plot(log_time,data_values('speed')*1e-4,'DisplayName','Speed')
    %title(sprintf('Omega droite (filtre de Tustin) — \\tau = %.1f ms', TAU))   
end

legend('Location', 'best');
grid on;

%% Avec le bon Tau = 0.080 s
TAU=0.10;
    
% Coefficients du filtre dérivateur (méthode de Tustin)
a1 =-(1 - 2*TAU / CONTROL_LOOP_PERIOD);
b1 = (1 + 2*TAU / CONTROL_LOOP_PERIOD);
    
N=numel(angleDroite);
    
% Initialisation
OmegaDtest = zeros(1, N);   % vecteur de vitesses V_Milan
OmegaDtest_prev = 0;
   
    
% Boucle de calcul (implémentation du filtre dérivateur de Tustin)
for k = 2:N
    OmegaDtest(k) = (2/CONTROL_LOOP_PERIOD* (angleDroite(k) - angleDroite(k-1)) + a1 * OmegaDtest_prev)/b1;
    OmegaDtest_prev = OmegaDtest(k); 
end
    
% Moyenne glissante 
windowSize = 50;  % à ajuster selon ton niveau de lissage souhaité
OmegaDtest_smooth = movmean(OmegaDtest, windowSize);

figure;  
% Prepare for plotting the filtered angular velocity
plot(log_time, OmegaDtest, 'DisplayName', sprintf('TAU : %.2f',TAU));
plot(log_time, OmegaDtest_smooth, 'DisplayName','Vitesse lissée');
title(sprintf('Omega droite lissée (filtre de Tustin) — \\tau = %.2f ms et WindowSize = %.2f ms', TAU,windowSize));  
legend('Location', 'best');
grid on;

%% Angle
figure;
plot(log_time,angleDroite,'DisplayName','Angle Droite');
grid on;
legend('Location', 'best');
%% Vitesse
figure;
OmegaDtest_rpm= OmegaDtest*60/360;
plot(log_time, OmegaDtest_rpm,'DisplayName','Omega rpm');
title('Test puissance max à 11.2V');
grid;
legend('Location', 'best');

%%
%figure;
%hold on;
%plot(log_time, OmegaDtest2, 'DisplayName', 'Filtered OmegaDtest2');
%grid on;
% 
% % Extract specific data for plotting
% omegaDroite = data_values('Omega Droite');
% speed = data_values('speed');


%Display logged variable names
%disp(data_values.keys)

% % Plot logged variables
% figure; 
% hold on;
% legend_labels = data_values.keys;
% for i = 1:numel(legend_labels)
%     %if i==1 || i==4
%     if legend_labels{i}=="Omega Droite" || legend_labels{i}=="speed"
%         d = legend_labels{i};
%         v = data_values(d);
%         plot(log_time, v, 'DisplayName', d);
%     end
% end
% 
% hold off;
% legend('Location', 'best');
% grid on;