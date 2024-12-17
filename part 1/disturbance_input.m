load statespace_data A B C D

% disturbance: only third input, only third column of B
Bd = B(:,3);
DSISO = 0;
CSISO = C(1,:);

sysd = -ss(A, Bd, CSISO, DSISO);

% defining system TF
[bd, ad] = ss2tf(A, Bd, CSISO, DSISO);

Gdisturbed = tf(bd, ad);

% uncontrolled response
figure
bode(sysd)
figure
step(sysd)
stepinfo(sysd)

%% pz maps
figure
pzmap(Gdisturbed)
p = pole(Gdisturbed);
z = zero(Gdisturbed);


%% response to original controller
[Gm, Pm, Wcg, Wcp] = margin(Gdisturbed);
Kp = 0.91;
Ki = 0.28;
Kd = 0;
K = pid(Kp,Ki,Kd);

% defining CL with input disturbance between G and Ko
CL0 = Gdisturbed/(1+Gdisturbed*K);

figure
bode(CL0)
step(CL0)
stepinfo(CL0)

%% tuning controller to fit disturbance
Kpn = 1.5;
Kin = 0.5;
Kdn = 0;
Knew = pid(Kpn, Kin, Kdn);

% pidTuner(Gdisturbed)

CLn = Gdisturbed/(1+Gdisturbed*Knew);

figure
bode(CLn)
step(CLn)
stepinfo(CLn)
