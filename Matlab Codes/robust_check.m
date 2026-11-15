% ROBUST_CHECK  Closed-loop poles with fixed Gc and a perturbed plant.
P = guardian_params();
s = tf('s');
Gc = P.Kc*(s^2 + P.c1*s + P.c0)/(s^2 + P.a*s + P.b_);   % nominal, never retuned

cases = {'nominal',   1,   1,   1,   1;      % name, Jt, Kt, Ke, Lm multipliers
         'Jt +30%',   1.3, 1,   1,   1;
         'Jt -30%',   0.7, 1,   1,   1;
         'Kt +20%',   1,   1.2, 1,   1;
         'Kt -20%',   1,   0.8, 1,   1;
         'Ke +20%',   1,   1,   1.2, 1;
         'Ke -20%',   1,   1,   0.8, 1;
         'Lm +50%',   1,   1,   1,   1.5;
         'all worst', 1.3, 0.8, 1.2, 1.5};

fprintf('%-10s  max Re(pole)  slowest-pole  dominant pair\n', 'case');
for k = 1:size(cases,1)
    Jt = P.Jt*cases{k,2};  Kt = P.Kt*cases{k,3};
    Ke = P.Ke*cases{k,4};  Lm = P.Lm*cases{k,5};
    J  = Jt + P.N^2*P.Jm;
    Gp = P.N*Kt / ( s*((Lm*s + P.Rm)*(J*s + P.b) + P.N^2*Kt*Ke) );
    pc = pole(feedback(Gc*Gp, 1));
    [~, i] = max(real(pc));
    fprintf('%-10s  %9.2f     %s\n', cases{k,1}, max(real(pc)), mat2str(round(pc(i),2)));
end

% resonance loop gain of the nominal design
w0 = sqrt(P.c0);
L  = Gc*(P.Kp/(s*(s^2 + P.c1*s + P.c0)));
fprintf('|L(j w0)| at w0 = %.1f rad/s : %.1f dB\n', w0, 20*log10(abs(evalfr(L, 1j*w0))));