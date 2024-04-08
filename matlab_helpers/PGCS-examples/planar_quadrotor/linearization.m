function [hA, B, ha] = linearization(x0)
% linearization of the draged double integrator, at point x0
% sigma: time scaling
% output: linearization matrix A, and the rank of controllability matrix Co

g=9.81;
m=0.486;
J=0.00383;
l=0.25;

hA = [  0    0     -x0(4)*sin(x0(3))-x0(5)*cos(x0(3))    cos(x0(3))       -sin(x0(3))       0; 
        0    0     x0(4)*cos(x0(3))-x0(5)*sin(x0(3))     sin(x0(3))        cos(x0(3))       0;
        0    0                    0                         0                 0             1;
        0    0                -g*cos(x0(3))                 0                x0(6)        x0(5);
        0    0                g*sin(x0(3))                 -x0(6)             0          -x0(4);
        0    0                    0                         0                 0             0 ];
      
% B = [ 0, 0; 
%         0, 0; 
%         0, 0; 
%         0, 0; 
%         1/m, 1/m; 
%         l/J, -l/J];

B = 10.*[ 0, 0; 
        0, 0; 
        0, 0; 
        0, 0; 
        1/sqrt(2), 1/sqrt(2); 
        1/sqrt(2), -1/sqrt(2)];

ha =  ( [x0(4)*cos(x0(3)) - x0(5)*sin(x0(3)); 
            x0(4)*sin(x0(3)) + x0(5)*cos(x0(3));
            x0(6);
            x0(5)*x0(6) - g*sin(x0(3));
            -x0(4)*x0(6) - g*cos(x0(3));
            0] ) - hA*x0;

% Co = ctrb(hA, B);
% rkCo = rank(Co);


% %% symbolized version
% g=9.81;
% syms x1 x2 x3 x4 x5 x6 real
% 
% pinvBBT = [   0         0         0         0         0         0;
%                          0         0         0         0         0         0;
%                          0         0         0         0         0         0;
%                          0         0         0         0         0         0;
%                          0         0         0         0        0.01    0;
%                          0         0         0         0         0        0.01];
%    
% grad_f_T = [ 0    0        -x4*sin(x3)-x5*cos(x3)    cos(x3)       -sin(x3)           0; 
%                                 0    0          x4*cos(x3)-x5*sin(x3)     sin(x3)        cos(x3)          0;
%                                 0    0                                  0                        0                   0                 1;
%                                 0    0                          -g*cos(x3)               0                  x6             x5;
%                                 0    0                           g*sin(x3)                -x6                0             -x4;
%                                 0    0                                  0                        0                    0               0 ];
%       
% Ak = sym('A', [6 6], 'real');
% Sigk = sym('S', [6 6], 'real');
% Tr = trace(pinvBBT*(grad_f_T - Ak)*Sigk*(grad_f_T' - Ak'));
% grad_Tr = gradient(Tr, [x1, x2, x3, x4, x5, x6]);
% simplify(grad_Tr)

end

