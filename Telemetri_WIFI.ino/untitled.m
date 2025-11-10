% ============================ CONFIG ============================
% Renseigne l'IP de la carte et les ports utilisés par MecatroUtils
ARDUINO_IP   = "192.168.4.1";   % par défaut si la carte crée son propre AP
PORT_TELEM   = 5005;            % port où la carte envoie les logs
PORT_CMD     = 5006;            % port où la carte reçoit les gains
LOCAL_PORT   = 5007;            % port local pour écouter la télémétrie

% Gains à transmettre dans l'ordre attendu par recieveGains
% [K_moins, Kp, Ki, K_lambda, Vbatt]
G = [0.8, 1.5, 0.4, 0.2, 11.2];

% Noms dans le même ordre que côté Arduino
varNames = ["angleDroite","angleGauche","Omega Droite","Omega Gauche", ...
            "SensorBar","lambda","U","u","U_plus","U_moins"];

% ========================= CONNEXION UDP ========================
% Connecte toi d'abord au WiFi Group10 depuis l'ordinateur
% Puis lance ce script
u = udpport("datagram","IPV4","LocalPort",LOCAL_PORT);
cleanupObj = onCleanup(@() clear("u"));

% ===================== ENVOI DES GAINS AU ROBOT =================
% Format texte simple, très robuste pour la plupart des TP
% Exemple de trame: GAINS,0.8,1.5,0.4,0.2,11.2\n
gainMsg = compose("GAINS,%.6f,%.6f,%.6f,%.6f,%.6f\n", G);
write(u, uint8(gainMsg), ARDUINO_IP, PORT_CMD);

fprintf("Gains envoyés à %s sur le port %d\n", ARDUINO_IP, PORT_CMD);

% ===================== RÉCEPTION TÉLÉMÉTRIE =====================
% Affichage en direct des courbes Omega Droite et Omega Gauche
figure("Name","Mecatro télémétrie");
ax1 = subplot(2,1,1); grid on; hold on; title("Vitesses roues");
hOD = animatedline(ax1); hOG = animatedline(ax1);
legend(ax1, "Omega Droite", "Omega Gauche","Location","best");
xlabel(ax1,"échantillon"); ylabel(ax1,"rad s");

ax2 = subplot(2,1,2); grid on; hold on; title("U_plus et U_moins");
hUp = animatedline(ax2); hUm = animatedline(ax2);
legend(ax2, "U_plus", "U_moins","Location","best");
xlabel(ax2,"échantillon"); ylabel(ax2,"V");

% Petit tampon pour afficher les dernières lignes reçues
bufN = 2000; k = 0;

% Optionnel: stockage complet dans un tableau
dataLog = zeros(bufN, numel(varNames), "double");

fprintf("Écoute de la télémétrie sur le port local %d, envoi depuis %s:%d\n", ...
        LOCAL_PORT, ARDUINO_IP, PORT_TELEM);

while isvalid(u)
    % lit un datagramme, puis tente de parser une ligne CSV
    if u.NumDatagramsAvailable > 0
        d = read(u, 1, "uint8");
        line = string(char(d.Data.'));
        % Chaque message peut contenir une ou plusieurs lignes
        lines = splitlines(strtrim(line));
        for li = 1:numel(lines)
            s = strtrim(lines(li));
            if s == "" || startsWith(s,"#")
                continue
            end
            % On s'attend à 10 valeurs, même ordre que varNames
            vals = split(s, ",");
            if numel(vals) ~= numel(varNames)
                % selon certains TP, la première trame peut être l'entête
                % ou contenir les noms, on ignore si dimension différente
                continue
            end
            v = double(str2double(vals));
            if any(isnan(v))
                continue
            end
            k = k + 1;
            idx = 1 + mod(k - 1, bufN);
            dataLog(idx, :) = v;

            % mise à jour des courbes
            addpoints(hOD, k, v(3));     % Omega Droite
            addpoints(hOG, k, v(4));     % Omega Gauche
            addpoints(hUp, k, v(9));     % U_plus
            addpoints(hUm, k, v(10));    % U_moins

            if mod(k, 10) == 0
                drawnow limitrate
            end
        end
    else
        pause(0.005)
    end
end
