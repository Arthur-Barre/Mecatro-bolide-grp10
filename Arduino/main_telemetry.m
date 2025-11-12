
%WiFi settingsWifFI
IP = '192.168.4.1';
PORT_NUMBER = 80;

%USB settings
port_name = "COM8";
baudrate = 230400;

%Recording time (s)
Tmax = 10;

%Communication method
method = 'WiFi';

%Gains
K_moins= 200; %450  200
Kp= -30;
Ki= -8;
K_lambda = 11;  %11
K_psi= 0.002;  % 0.005 0.002?
Vbatt=11.2;
coef=1.038;
u_barre=0.4; % problème de friction
K = [K_moins Kp Ki K_lambda Vbatt coef u_barre K_psi];  

if strcmp(method,'WiFi')
    [log_time, data_values, line_idx] = get_data_WiFi(IP, PORT_NUMBER, Tmax, K);
elseif strcmp(method,'USB')
    [log_time, data_values, line_idx] = get_data_USB(port_name,Tmax,baudrate, K);
else
    print('Use a valid connection method')
end

%Display logged variable names
disp(data_values.keys)  

%Plot logged variables
% figure; 
% hold on;
% legend_labels = data_values.keys;
% for i = 1:numel(legend_labels)
%     %if legend_labels{i}=="SensorBar" 
%     if legend_labels{i}=="u" || legend_labels{i}=="dlambda" || legend_labels{i}=="Omega Droite" || legend_labels{i}=="Omega Gauche"
%         d = legend_labels{i};
%         v = data_values(d);
%         plot(log_time, v, 'DisplayName', d);
%     end
% end
% hold off;
% legend('Location', 'best');
% grid on;

% figure
% hold on;
% plot(log_time, data_values("Omega Droite"), 'DisplayName', "Droite");
% plot(log_time, data_values("Omega Gauche"), 'DisplayName', "Gauche");
% hold off;
% legend('Location', 'best');
% grid on;
