load statespace_data A B C D

% SISO: only first input, only first column of B
Bs = B(:,1);
Ds = [0;0];

G = -1*tf(ss2tf(A, Bs, C, Ds));
bode(G)

ltiview(G)