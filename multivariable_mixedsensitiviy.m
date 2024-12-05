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
G.y = 'y';

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
wp.u = 'v';
wp.y = 'z1';

% wu given in question
wu = [0.005 0;
       0 (0.005*s^2 + 0.0007*s+0.00005)/(s^2 + 0.0014*s + 10^(-6))];
wu.u = 'u';
wu.y = 'z2';

% Build system
sumblock = sumblk("v = w + y", 2);

P = connect(G, wu, wp, sumblock, {'w', 'u'}, {'z1', 'z2', 'v'});


% H infinity synthesis
nmeas = 2; %number of outputs of plant = number of inputs controller
ncont = 2; 
[K,CL,gamma] = hinfsyn(P,nmeas,ncont);

K.u = 'v';
K.y = 'u';

CLstep = feedback(-K*G, eye(2));

[y, tout] = step(CLstep);
figure
subplot(2,1,1)
plot(tout, y(:,1,1))
subplot(2,1,2)
plot(tout, y(:,2,1))
warning off
systemnames ='FWT';     % The full wind turbine model should be available in your workspace
inputvar ='[V; Beta; Tau]';    % Input generalized plant
input_to_FWT= '[Beta; Tau; V]';

outputvar= '[FWT; Beta; Tau; FWT]';    % Output generalized plant also includes the inputs

sysoutname='Gsim';

sysic;

warning on

%CL_sisocontroller=minreal(lft(Gsim(1:end-1,1:end-1),sisocontroller)); % SISO controller

CL_mimocontroller = minreal(lft(Gsim, K)); % MIMO controller

figure

step(CL_mimocontroller); % simple code for a step on the wind

















% S = feedback(eye(2),series(G,K), 1:2, 1:2, 1);
% KS = feedback(K,series(G,K), 1:2, 1:2, 1);
% 
% wp11 = wp(1,1);
% wp22 = wp(2,2);
% 
% figure(1)
% bodemag(S, [inv(wp11) inv(wp11); inv(wp22) inv(wp22)])
% legend('sensitivity', 'weights')
% 
% norm(S, inf);
% 
% N = connect(P, K, 'w', {'z1', 'z2'});


% hinfnorm(N);
% 
% systemnames = 'G wp wu' ; % Define systems
% inputvar = '[w(2); u(2)]' ; % Input generalized plant
% input_to_G= '[u]';
% input_to_wu= '[u]';
% input_to_wp= '[w+G]';
% outputvar= '[wp; wu; G+w]'; % Output generalized plant
% sysoutname='P2';
% sysic;
% [K2,CL2,GAM2,INFO2] = hinfsyn(P2,2,2); % Hinf design
