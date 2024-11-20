load statespace_data A B C D

w0 = 0;
w1 = 0.2*3*pi;

% MIMO
Bm = B(:,1:2);
Dm = D(:,1:2);

% defining state-space and TF
s = tf('s');

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

Gw0 = freqresp(G, w0)
Gw1 = freqresp(G, w1);

RGAw0 = Gw0.*pinv(Gw0).'
RGAw1 = Gw1.*pinv(Gw1).'


