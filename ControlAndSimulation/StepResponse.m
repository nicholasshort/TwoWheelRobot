m = 0.5;
M = 1;
l = 1;
g = 9.81;
I = 1/12*m*l^2;

A = [0, 1; m*g*l*(M+m)/(I*(M+m)+m*M*l^2), 0];
B = [0; -m*l/(I*(M+m)+m*M*l^2)];
C = [1 0];
D = 0;

[a, b] = ss2tf(A, B, C, D);

G = tf(a,b);
C = tf([-11.1111, -66.6666, -88.8888], [1, 0]);

CL = C*G/(1+C*G);


t = 0:0.01:10;
r =10*ones(size(t));
figure(2);clf;subplot(311)
[y,x]=lsim(CL,r,t);
plot(x,y)



