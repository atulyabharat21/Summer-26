
Ra = 1.0321;
La = 505.5e-6;
N = 50.9;
Kt = 0.754;
Kb = 0.9173;
Jm = 4.3e-6; %motor shaft
Bf = 9.43e-5; %motor shaft
Jm_output = Jm * N^2;
Bf_output = Bf * N^2;
Kt_nominal = 0.754;

A_plant = [
   -Ra/La           0             -Kb/La;
    0               0                  1;
    Kt/Jm_output    0   -Bf_output/Jm_output
];

B_plant = [
    1/La     0  ;
    0        0  ;
    0      -1/Jm_output
];

C_plant = [1 0 0; 0 1 0];
D_plant = zeros(2,2);

A = [
   -Ra/La           0             -Kb/La               0;
    0               0                  1                0;
    Kt/Jm_output    0   -Bf_output/Jm_output   -1/Jm_output;
    0               0                  0                0
];
B = [1/La; 0; 0; 0];
C = [1 0 0 0; 0 1 0 0];
D = zeros(2,1);

assert(rank(obsv(A,C)) == 4, 'System not observable');

Ts = 200e-6;

sysC = ss(A, B, C, D);
sysD = c2d(sysC, Ts, 'zoh');
Ad = sysD.A;
Bd = sysD.B;
Cd = sysD.C;
Dd = sysD.D;

cont_poles = [-1500, -2000, -2500, -3000];
z_poles    = exp(cont_poles * Ts);
L          = place(Ad', Cd', z_poles)';

eig_check = eig(Ad - L*Cd);
disp('Observer closed-loop eigenvalues:'); disp(eig_check);

Aobs = Ad - L*Cd;
Bobs = [Bd, L];
Cobs = eye(4);
Dobs = zeros(4,3);

assert(all(abs(eig(Aobs)) < 1), 'Observer unstable');

wc_lpf          = 50;
[lpf_num, lpf_den] = bilinear(wc_lpf, [1 wc_lpf], 1/Ts);
disp('LPF numerator:');   disp(lpf_num);
disp('LPF denominator:'); disp(lpf_den);

Kp_current = La * 500;
Ki_current = Ra * 500;
disp('Current PID gains:');
fprintf('  Kp = %.4f\n', Kp_current);
fprintf('  Ki = %.4f\n', Ki_current);

open_system('ObserverSimulation.slx')