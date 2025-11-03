% Définition des constantes

rho = 0.04; % rayon des roues
k = 0.0854; % constante de couple
R = 3.87; % résistance des moteurs
d = 0.05; % distance entre le centre des roues et le centre de masse
l = 0.16; % distance entre les roues
dCapt = 0.01;
%L = 0.005;
mb = 2; % masse du châssis du robot
mw = 0.5; % masse d'une unité
M = 2*mw + mb; % masse du robot
Ix = 8.48e-3; % moments d'inertie du robot
Iz = 1.233e-2;
Ixw  = 4.00e-4;    % roue - inertie de rotation autour de l'axe (spin)
Izw  = 4.00e-4;    % roue - inertie (même ordre que Ixw)
Iyw  = 2.375e-4;   % roue - inertie transversale (axe perpendiculaire au spin)
Ipsi = Ix + Iz + Ixw + Izw + mw*(l^2)/2 + Iyw*(l^2)/(2*(rho^2));
A = M + 2*Iyw*(rho^2);
B = Ipsi + mb*(d^2);