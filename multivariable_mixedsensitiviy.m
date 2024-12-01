load statespace_data A B C D FWT

w0 = 0;
w1 = 0.2*3*pi;

% MIMO
Bm = B(:,1:2);
Dm = D(:,1:2);

% defining state-space and TF
s = tf('s');

% [b,a] = ss2tf(A, Bm, C(1,:), Dm(1,:),1);
% g11 = tf(b,a);
% [b, a] = ss2tf(A, Bm, C(1,:), Dm(1,:),2);
% g12 = tf(b,a);
% [b, a] = ss2tf(A, Bm, C(2,:), Dm(2,:),1);
% g21 = tf(b,a);
% [b, a] = ss2tf(A, Bm, C(2,:), Dm(2,:),2);
% g22 = tf(b,a);
% 
% G = [g11 g12;
%     g21 g22];

G = FWT(1:2, 1:2);

Gw0 = freqresp(G, w0);
Gw1 = freqresp(G, w1);

RGAw0 = Gw0.*pinv(Gw0).';
RGAw1 = Gw1.*pinv(Gw1).';

% MIMO poles and zeros 
p = [pole(g11), pole(g12), pole(g21), pole(g22)];
z = union(union(union(zero(g11), zero(g12)), zero(g21)), zero(g22));


% Computing wp11

f_co = 0.3; % cut-off = bandwidth

wp = [(s/3+0.3*2*pi)/(s+pi*6e-5) 0;
    0 0.05];

wu = [0.005 0;
       0 (0.005*s^2 + 0.0007*s+0.00005)/(s^2 + 0.0014*s + 10e-6)];

P = [wp series(G, wp);
    zeros(2,2) wu;
    eye(2) G];

P = balreal(minreal(P));

nmeas = 2; %number of outputs of plant = number of inputs controller
ncont = 2; 
[K,CL,gamma] = hinfsyn(P,nmeas,ncont);

S = feedback(eye(2),series(G,K), 1:2, 1:2, 1);
KS = feedback(K,series(G,K), 1:2, 1:2, 1);

wp11 = wp(1,1);
wp22 = wp(2,2);

figure(1)
bodemag(S, [inv(wp11) inv(wp11); inv(wp22) inv(wp22)])
legend('sensitivity', 'weights')

norm(S, inf)

N = [wp*S;
    wu*K*S];



hinfnorm(N)

systemnames = 'G wp wu' ; % Define systems
inputvar = '[w(2); u(2)]' ; % Input generalized plant
input_to_G= '[u]';
input_to_wu= '[u]';
input_to_wp= '[w+G]';
outputvar= '[wp; wu; G+w]'; % Output generalized plant
sysoutname='P2';
sysic;
[K2,CL2,GAM2,INFO2] = hinfsyn(P2,2,2); % Hinf design




