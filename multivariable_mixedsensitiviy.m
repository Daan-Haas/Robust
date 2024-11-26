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

Gw0 = freqresp(G, w0);
Gw1 = freqresp(G, w1);

RGAw0 = Gw0.*pinv(Gw0).';
RGAw1 = Gw1.*pinv(Gw1).';

% MIMO poles and zeros 
p = [pole(g11), pole(g12), pole(g21), pole(g22)]
z = union(union(union(zero(g11), zero(g12)), zero(g21)), zero(g22))

% pzmap(G)
% title('Pole-zero map of MIMO system')

% Computing wp11

f_co = 0.3; % cut-off = bandwidth

wp = [tf((s/3+0.3)/ (s+3e-5)) 0;
    0 0.05];

wu = [0.005 0;
       0 tf((0.005*s^2 + 0.0007*s+0.00005)/(s^2 + 0.0014*s + 0.000001))];

% K
Kp = 1.5;
Ki = 0.8;
Kd = 0.9;
K = pid(Kp,Ki,Kd);

S = 1/(1+G.*K);
norm(S, inf)

N = [wp*S;
    wu*K*S];




