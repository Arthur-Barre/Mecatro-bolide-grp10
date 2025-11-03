function f = courbe(s, xA, yA, psi)

    dCapt = 0.01;
    iBody = [cos(psi);sin(psi)];
    f = dot(iBody,[xA-2*cos(s)+dCapt*cos(psi);yA-2*sin(s)+dCapt*sin(psi)]);
end

