load statespace_data A B C D

% MIMO
Bm = B(:,1:2);
Dm = D(:,1:2);

% defining state-space and TF

[b,a] = ss2tf(A, Bm, C(1,:), Dm(1,:),1);
g11 = tf(b,a);
[b, a] = ss2tf(A, Bm, C(1,:), Dm(1,:),2);
g12 = tf(b,a);
[b, a] = ss2tf(A, Bm, C(2,:), Dm(2,:),1);
g21 = tf(b,a);
[b, a] = ss2tf(A, Bm, C(2,:), Dm(2,:),2);
g22 = tf(b,a);

G = [g11 g12;
    g21 g22];


RGA = G.*pinv(G).'


