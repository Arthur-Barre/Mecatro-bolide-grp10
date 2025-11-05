
%WiFi settingsWifFI
IP = '192.168.4.1';
PORT_NUMBER = 80;

%USB settings
port_name = "COM8";
baudrate = 230400;

%Recording time (s)
Tmax = 20;

%Communication method
method = 'WiFi';

%Gains
K = [1 1 1];  % dans l'ordre K_moins, Kp, Ki

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
figure; 
hold on;
legend_labels = data_values.keys;
for i = 1:numel(legend_labels)
    %if legend_labels{i}=="SensorBar" 
    if legend_labels{i}=="u" || legend_labels{i}=="dlambda" || legend_labels{i}=="Omega Droite" || legend_labels{i}=="Omega Gauche"
        d = legend_labels{i};
        v = data_values(d);
        plot(log_time, v, 'DisplayName', d);
    end
end
hold off;
legend('Location', 'best');
grid on;

figure
hold on;
plot(log_time, data_values("U_plus"), 'DisplayName', "U_plus");
plot(log_time, data_values("U_moins"), 'DisplayName', "U_moins");
hold off;
legend('Location', 'best');
grid on;
