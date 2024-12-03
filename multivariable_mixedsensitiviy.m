load statespace_data A B C D FWT

w0 = 0;
w1 = 0.2*3*pi;

% MIMO
Bm = B(:,1:2);
Dm = D(:,1:2);

% defining state-space and TF
s = tf('s');

% Define plant and in and outputs
G = FWT(1:2, 1:2);
G.u = 'u';
G.y = 'v';

% Calculate RGA
Gw0 = freqresp(G, w0);
Gw1 = freqresp(G, w1);

RGAw0 = Gw0.*pinv(Gw0).';
RGAw1 = Gw1.*pinv(Gw1).';

% MIMO poles and zeros 
p = pole(G);
z = zero(G);

% Computing wp11

f_co = 0.3; % cut-off = bandwidth

wp = [(s/3+0.3*2*pi)/(s+pi*6e-5) 0;
    0 0.05];
wp.InputName = 'v';
wp.OutputName = 'z1';

% wu given in question
wu = [0.005 0;
       0 (0.005*s^2 + 0.0007*s+0.00005)/(s^2 + 0.0014*s + 10e-6)];
wu.InputName = 'u';
wu.OutputName = 'z2';

% Build system
sumblock = sumblk('v = w + x');

P = connect(G, wu, wp, sumblock, {'u', 'w'}, {'z1', 'z2'});


% H infinity synthesis
nmeas = 2; %number of outputs of plant = number of inputs controller
ncont = 2; 
[K,CL,gamma] = hinfsyn(P,nmeas,ncont);

K.InputName = 'v';
K.OutputName = 'u';

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

N = connect(P, K, 'w', {'z1', 'z2'});


hinfnorm(N);

systemnames = 'G wp wu' ; % Define systems
inputvar = '[w(2); u(2)]' ; % Input generalized plant
input_to_G= '[u]';
input_to_wu= '[u]';
input_to_wp= '[w+G]';
outputvar= '[wp; wu; G+w]'; % Output generalized plant
sysoutname='P2';
sysic;
[K2,CL2,GAM2,INFO2] = hinfsyn(P2,2,2); % Hinf design
