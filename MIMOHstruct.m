load statespace_data.mat FWT

Gmimo = FWT(1:2, 1:2);

% Fixed Structure SISO controller
s = tf('s');
Kp = realp('Kp',1);
Ki = realp('Ki',1);
Kd = realp('Kd',1);
Tf = realp('Tf',1);

wp = [(s/3+0.3*2*pi)/(s+pi*6e-5) 0;
    0 0.05];
wp.u = 'y';
wp.y = 'z1';

wu = [0.005 0;
      0 (0.005*s^2 + 0.0007*s+0.00005)/(s^2 + 0.0014*s + 10e-6)];
wu.u = 'u';
wu.y = 'z2';

C_struct = Kp+Ki/s+(Kd*s)/(Tf*s+1);

% SISO hinfstruct
Wp_mimo = wp;
Wp_mimo.u = 'y';
Wp_mimo.y = 'z1';

Wu_mimo = wu;
Wu_mimo.u = 'u';
Wu_mimo.y = 'z2';


G_mimo = -Gmimo;
G_mimo.u = 'u';
G_mimo.y = 'y';

C_mimo = C_struct;
C_mimo.u = 'e';
C_mimo.y = 'u';

Sum1 = sumblk('e = r - y');
Mimo_Con = connect(G_mimo, Wp_mimo, Wu_mimo, C_mimo, Sum1, 'r', {'z1', 'z2'});

opt = hinfstructOptions ('Display', 'final', 'RandomStart', 5);
N_mimo = hinfstruct(Mimo_Con, opt);

% Extract controller gains :
Kp_opt = N_mimo.Blocks.Kp.Value;
Ki_opt = N_mimo.Blocks.Ki.Value;
Kd_opt = N_mimo.Blocks.Kd.Value;
Tf_opt = N_mimo.Blocks.Tf.Value;

Kfb_opt = Kp_opt + Ki_opt / s +( Kd_opt * s ) /( Tf_opt * s +1);

Kfb_opt.u = 'x';
Kfb_opt.y = 'u';

CLsys = connect(G_mimo, Kfb_opt, Sum1, 'r', 'y');

step(CLsys)
