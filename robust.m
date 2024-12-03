load statespace_data A B C D

% SISO: only first input, only first column of B
BSISO = B(:,1);
Bd = B(:,3);
DSISO = 0;
CSISO = C(1,:);


sys = -ss(A, BSISO, CSISO, DSISO);

bode(sys)

[b, a] = ss2tf(A, BSISO, CSISO, DSISO);
[bd, ad] = ss2tf(A, Bd, CSISO, DSISO);

Gsiso = -tf(b, a);
Gdisturbed = tf(bd, ad);

step(sys)
stepinfo(sys)

%% Figures
figure
pzmap(Gsiso)
p = pole(Gsiso);
z = zero(Gsiso);

[Gm, Pm, Wcg, Wcp] = margin(Gsiso);
Kp = 0.91;
Ki = 0.28;
Kd = 0;
K = pid(Kp,Ki,Kd);

CL = feedback(K, Gsiso);

% pidTuner(Gsiso)

figure
bode(CL)
step(CL)
stepinfo(CL)

%% H infinity

[K, CL, gamma] = hinfsyn(sys, 1, 1);

