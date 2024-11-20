load statespace_data A B C D

% MIMO
Bm = B(:,1:2);
Dm = D(:,1:2);

% defining state-space and TF
sys = -ss(A, Bm, C, Dm);

[b1,a1] = ss2tf(sys(1,1))
tf1 = tf(b1, a1)



% computing RGA
w1 = 0;
w2 = 0.3*2*pi;

%RGA = G.*inv(G)'


