load statespace_data.mat FWT

Gsiso = FWT(1, 1);

% Fixed Structure SISO controller
s = tf('s');
Kp = realp('Kp',1);
Ki = realp('Ki',1);
% Kd = realp('Kd',1); %uncomment for PID
% Tf = realp('Tf',1);
Kd = 0;
Tf = 0;

Wp_simple = 0.95*(s+0.02*2*pi)/(0.016*pi+s);
C_struct = Kp+Ki/s+(Kd*s)/(Tf*s+1);


% SISO hinfstruct
Wp_siso = Wp_simple;
Wp_siso.u = 'e'; % On output or on error???
Wp_siso.y = 'z1';

G_siso = -Gsiso;
G_siso.u = 'u';
G_siso.y = 'y';

C_siso = C_struct;
C_siso.u = 'e';
C_siso.y = 'u';

Sum1 = sumblk('e = r - y');
Siso_Con = connect(G_siso, Wp_siso, C_siso, Sum1, 'r', 'z1');

opt = hinfstructOptions ('Display', 'final', 'RandomStart', 5);
[N_siso, GAM] = hinfstruct(Siso_Con, opt);

% Extract controller gains :
Kp_opt = N_siso.Blocks.Kp.Value;

Ki_opt = N_siso.Blocks.Ki.Value;
% Kd_opt = N_siso.Blocks.Kd.Value;
% Tf_opt = N_siso.Blocks.Tf.Value;
Kd_opt = 0;
Tf_opt = 0;

Kfb_opt = Kp_opt + Ki_opt / s +( Kd_opt * s ) /( Tf_opt * s +1);

Kfb_opt.u = 'e';
Kfb_opt.y = 'u';

S = 1/ (1 + series(G_siso, Kfb_opt));
bodemag(S, 1/Wp_siso)
CLsys = connect(G_siso, Kfb_opt, Sum1, 'r', 'y');

figure
step(CLsys)
stepinfo(CLsys)
