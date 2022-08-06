% Proportional gain
kp = linspace(-50,0,5); %-50
% Integral gain
ki = linspace(-100,0,10); %-1000
% Derivative gain
kd = linspace(-100,0,10); %-1000


% System Parameters
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
greatest = 0;


for k1 = 1:length(ki)
    for k2 = 1:length(kp)
        for k3 = 1:length(kd)
            %PID Controller
            Controller = tf([kd(k3), kd(k2), ki(k1)], [1, 0]);
            
            %Open Loop TF
            Gol = Controller*G;
            
            E = 1/(1+Gol);
            
            % Poles
            p = pole(E);
            
            % Plot Root Locus
            hold on
            plot(real(p),imag(p),'*');

            %Check for possible PID parameters
            if((real(p) < 0))
                if(max(real(p)) < greatest)
                    greatest = max(real(p));
                end
                
                disp("PID Values");
                disp([kd(k2), ki(k1), kd(k3)]);
                disp("Poles");
                disp(real(p));
            end
        end
    end
end
hold off
disp(greatest);
