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

%% Figures
figure
bode(Gsiso)

figure
pzmap(Gsiso)
p = pole(Gsiso);
z = zero(Gsiso);

[Gm, Pm, Wcg, Wcp] = margin(Gsiso);

% K = pid(Kp,Ki,Kd);

% pidTuner(Gsiso,'PID')

CL = series(K,G);
figure
bode(CL)

%% H infinity

[K, CL, gamma] = hinfsyn(sys, 1, 1);

%% Fixed Structure SISO controller
s = tf('s');
Kp = realp('Kp',1);
Ki = realp('Ki',1);
Kd = realp('Kd',1);
Tf = realp('Tf',1);

Wp_simple = 0.95*(s+0.02*2*pi)/(0.0001*2*pi+s);
C_struct = Kp+Ki/s+(Kd*s)/(Tf*s+1);

% SISO hinfstruct

Wp_siso = Wp_simple;
Wp_siso.u = 'y_act';
Wp_siso.y = 'z1';

G_siso = -1*Gsiso;
G_siso.u = 'u';
G_siso.y = 'y_plant';

C_siso = C_struct;
C_siso.u = 'e';
C_siso.y = 'u';

Sum1 = sumblk('e = w_ref - y_act');
Sum2 = sumblk('y_act = y_plant + y_dist');
Siso_Con = connect(G_siso, Wp_siso, C_siso, Sum1, Sum2, {'w_ref', 'y_dist'} ,{'y_plant', 'z1'});

opt = hinfstructOptions ('Display', 'final', 'RandomStart', 5);
N_siso = hinfstruct(Siso_Con, opt);

% Extract controller gains :
Kp_opt = N_siso.Blocks.Kp.Value;

Ki_opt = N_siso.Blocks.Ki.Value;
Kd_opt = N_siso.Blocks.Kd.Value;
Tf_opt = N_siso.Blocks.Tf.Value;

Kfb_opt = Kp_opt + Ki_opt / s +( Kd_opt * s ) /( Tf_opt * s +1);
