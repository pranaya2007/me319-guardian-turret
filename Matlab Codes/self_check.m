%SELF_CHECK  Independent checks of the main numbers. Run from the project folder.
P = guardian_params();  s = tf('s');
w = [0.1 1 10 55.9 200];
fr = @(G) squeeze(freqresp(G, w));

% 1. Plant coefficients (by hand: 320, 3350, 1001500, divided by 320)
chk('1  plant coefficients', abs(P.J-3200)<1e-9 && abs(P.Kp-3.125)<1e-9 && ...
    abs(P.c1-10.469)<1e-3 && abs(P.c0-3129.69)<0.05);

% 2. Plant from a state-space model, independent of the algebra
A  = [0 1 0; 0 -P.b/P.J P.N*P.Kt/P.J; 0 -P.Ke*P.N/P.Lm -P.Rm/P.Lm];
G1 = tf(ss(A, [0;0;1/P.Lm], [1 0 0], 0));
G2 = P.Kp/(s*(s^2 + P.c1*s + P.c0));
chk('2  Gp: state space equals transfer function', max(abs(fr(G1)-fr(G2))./abs(fr(G2))) < 1e-5);

% 3. Steady speed from turret_plant.m with 10 V applied for 20 s
x = [0;0;0];
for k = 1:20000
    [d1,d2,d3] = turret_plant(0, 10, x(1), x(2), x(3));
    x = x + 1e-3*[d1; d2; d3];
end
w_expect = 10/(P.Ke*P.N + P.Rm*P.b/(P.N*P.Kt));
chk('3  turret_plant steady speed = V/(Ke N + Rm b/(N Kt))', abs(x(2)-w_expect) < 1e-5);

% 4. Closed-loop poles, margins, Routh
p = roots([1 P.a P.b_ P.K]);
[Gm, Pm, ~, Wp] = margin(P.K/(s*(s+5)*(s+20)));
chk('4a closed-loop poles -20.62, -2.19 +/- 2.21j', all(abs(sort(real(p))' - [-20.62 -2.19 -2.19]) < 0.01));
chk('4b gain margin = 20 log10(a b / K) = 21.9 dB', abs(20*log10(Gm) - 20*log10(P.a*P.b_/P.K)) < 1e-6);
chk('4c phase margin 64.2 deg at 1.87 rad/s', abs(Pm-64.2) < 0.1 && abs(Wp-1.87) < 0.01);

% 5. The closed loop acts like a 0.5 s delay at low frequency
wd = [0.2 0.5 1 2];  jw = 1j*wd;
T  = P.K./(jw.^3 + P.a*jw.^2 + P.b_*jw + P.K);
d  = -angle(T)./wd;
chk('5  phase delay between 0.50 and 0.56 s up to 2 rad/s', all(d > 0.499 & d < 0.56));
fprintf('     delays [s]: %s\n', mat2str(d, 4));

% 6. Compensator: state space equals Gc, and Gc*Gp equals L
Gc  = P.Kc*(s^2 + P.c1*s + P.c0)/(s^2 + P.a*s + P.b_);
Gss = P.Kc*(1 + tf(ss(P.Ac, P.Bc, P.Cc, 0)));
Lt  = P.K/(s*(s+5)*(s+20));
chk('6a Gc state space = Gc(s)', max(abs(fr(Gc)-fr(Gss))) < 1e-6*max(abs(fr(Gc))));
chk('6b Gc*Gp = 200/(s(s+5)(s+20))', max(abs(fr(Gc*G2)-fr(Lt))./abs(fr(Lt))) < 1e-6);

% 7. The exact zero-order-hold discretisation used in the controller
dt = 0.05;  E = expm([P.Ac P.Bc; zeros(1,3)]*dt);
sd = c2d(ss(P.Ac, P.Bc, eye(2), zeros(2,1)), dt, 'zoh');
chk('7  expm discretisation = c2d (zoh)', max(max(abs([E(1:2,1:2) E(1:2,3)] - [sd.A sd.B]))) < 1e-10);

% 8. Repeatability: Scenario A must not depend on what ran before it
o1 = simulate_engagement('A');  o2 = simulate_engagement('B');  o3 = simulate_engagement('A'); %#ok<NASGU>
chk('8  Scenario A identical after running B', isequal(o1.pulses, o3.pulses) && isequal(o1.V, o3.V));

% 9. The single missed pulse (case 17)
o = simulate_engagement(1017);
fprintf('9  case 17 pulses [time, miss]:\n');  disp(o.pulses(1:3,:));
chk('9  case 17: second pulse misses by 2.81 m', abs(o.pulses(2,2) - 2.81) < 0.01);

function chk(name, cond)
if cond, r = 'PASS'; else, r = 'FAIL'; end
fprintf('%-56s %s\n', name, r);
end