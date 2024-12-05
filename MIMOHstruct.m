load statespace_data.mat FWT

Gmimo = FWT(1:2, 1:2);

% Fixed Structure SISO controller
s = tf('s');
Kp1 = realp('Kp1',1);
Ki1 = realp('Ki1',1);
Kd1 = realp('Kd1',1);
Tf1 = realp('Tf1',1);

Kp2 = realp('Kp2',1);
Ki2 = realp('Ki2',1);
Kd2 = realp('Kd2',1);
Tf2 = realp('Tf2',1);

wp = [(s/3+0.3*2*pi)/(s+pi*6e-5) 0;
    0 0.05];

wu = [0.005 0;
      0 (0.005*s^2 + 0.0007*s+0.00005)/(s^2 + 0.0014*s + 10e-6)];

C1 = Kp1+Ki1/s+(Kd1*s)/(Tf1*s+1);
C2 = Kp2+Ki2/s+(Kd2*s)/(Tf2*s+1);

% SISO hinfstruct
Wp_mimo = wp;
Wp_mimo.u = 'e';
Wp_mimo.y = 'z1';

Wu_mimo = wu;
Wu_mimo.u = 'u';
Wu_mimo.y = 'z2';


G_mimo = -Gmimo;
G_mimo.u = 'u';
G_mimo.y = 'y';

C_mimo = [0 C1; C2 0];
C_mimo.u = 'e';
C_mimo.y = 'u';

Sum1 = sumblk('e = r - y',2);
Mimo_Con = connect(G_mimo, Wp_mimo, Wu_mimo, C_mimo, Sum1, 'r', {'z1', 'z2', 'y'});

opt = hinfstructOptions ('Display', 'final', 'RandomStart', 5);
N_mimo = hinfstruct(Mimo_Con, opt);
 
% Extract controller gains :
Kp1_opt = N_mimo.Blocks.Kp1.Value;
Ki1_opt = N_mimo.Blocks.Ki1.Value;
Kd1_opt = N_mimo.Blocks.Kd1.Value;
Tf1_opt = N_mimo.Blocks.Tf1.Value;

Kp2_opt = N_mimo.Blocks.Kp2.Value;
Ki2_opt = N_mimo.Blocks.Ki2.Value;
Kd2_opt = N_mimo.Blocks.Kd2.Value;
Tf2_opt = N_mimo.Blocks.Tf2.Value;

Kfb1_opt = Kp1_opt + Ki1_opt / s +( Kd1_opt * s ) /( Tf1_opt * s +1);
Kfb2_opt = Kp2_opt + Ki2_opt / s +( Kd2_opt * s ) /( Tf2_opt * s +1);

Kfb_opt = [0 Kfb1_opt; Kfb2_opt 0];

Kfb_opt.u = 'e';
Kfb_opt.y = 'u';

CLsys = connect(G_mimo, Kfb_opt, Sum1, 'r', 'y');

step(CLsys)
