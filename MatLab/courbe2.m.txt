function g = courbe2(s, xA, yA, psi, z)

    dCapt = 0.01;
    jBody = [-sin(psi);cos(psi)];
    g = dot(jBody,[xA-2*cos(s)+dCapt*cos(psi);yA-2*sin(s)+dCapt*sin(psi)])+z;
end