function [hAk,hak,nTr] = linearAa_pquad(Sk,zk,A)
% assume V = 0
% hAk  = A;
% hak  = a;
% Qk   = Q;
% rk   = r;

[nx,~,nt] = size(Sk);
hAk = zeros(nx,nx,nt);
hak = zeros(nx,nt);
nTr = zeros(nx,nt);
g=9.81;

for i = 1:nt

    hAk(:,:,i) = [0 0     -zk(4,i)*sin(zk(3,i))-zk(5,i)*cos(zk(3,i))    cos(zk(3,i)) -sin(zk(3,i))      0; 
                        0 0     zk(4,i)*cos(zk(3,i))-zk(5,i)*sin(zk(3,i))     sin(zk(3,i)) cos(zk(3,i))       0;
                        0 0         0                                           0               0                 1;
                        0 0     -g*cos(zk(3,i))                            0       zk(6,i)             zk(5,i);
                        0 0     g*sin(zk(3,i))                              -zk(6,i)    0             -zk(4,i);
                        0 0         0                                           0               0               0 ];
                
    hak(:,i) = ( [zk(4,i)*cos(zk(3,i)) - zk(5,i)*sin(zk(3,i)); 
                                    zk(4,i)*sin(zk(3,i)) + zk(5,i)*cos(zk(3,i));
                                    zk(6,i);
                                    zk(5,i)*zk(6,i) - g*sin(zk(3,i));
                                    -zk(4,i)*zk(6,i) - g*cos(zk(3,i));
                                    0] ) - hAk(:,:,i)*zk(:,i);
            
% with time scaling:            
    nTr(:,i) = [0;
                      0;
                       -(981*cos(zk(3,i))*(50*Sk(3,4,i)*zk(6,i) + 50*Sk(3,6,i)*zk(4,i) + 50*Sk(4,3,i)*zk(6,i) + 50*Sk(6,3,i)*zk(4,i) - 981*Sk(3,3,i)*sin(zk(3,i)) + 50*A(5,1,i)*Sk(1,3,i) + 50*A(5,2,i)*Sk(2,3,i) + 50*A(5,1,i)*Sk(3,1,i) + 50*A(5,2,i)*Sk(3,2,i) + 100*A(5,3,i)*Sk(3,3,i) + 50*A(5,4,i)*Sk(3,4,i) + 50*A(5,5,i)*Sk(3,5,i) + 50*A(5,6,i)*Sk(3,6,i) + 50*A(5,4,i)*Sk(4,3,i) + 50*A(5,5,i)*Sk(5,3,i) + 50*A(5,6,i)*Sk(6,3,i)))/500000;
                        Sk(4,6,i)*(A(5,4,i)/100 + zk(6,i)/100) + Sk(6,6,i)*(A(5,6,i)/100 + zk(4,i)/100) + (Sk(6,3,i)*(A(5,3,i) - (981*sin(zk(3,i)))/100))/100 + (Sk(6,4,i)*(A(5,4,i) + zk(6,i)))/100 + (Sk(6,6,i)*(A(5,6,i) + zk(4,i)))/100 + (A(5,1,i)*Sk(1,6,i))/100 + (A(5,2,i)*Sk(2,6,i))/100 + (A(5,5,i)*Sk(5,6,i))/100 + (A(5,1,i)*Sk(6,1,i))/100 + (A(5,2,i)*Sk(6,2,i))/100 + (A(5,5,i)*Sk(6,5,i))/100 + Sk(3,6,i)*(A(5,3,i)/100 - (981*sin(zk(3,i)))/10000);
                      0;
                        Sk(4,4,i)*(A(5,4,i)/100 + zk(6,i)/100) + Sk(6,4,i)*(A(5,6,i)/100 + zk(4,i)/100) + (Sk(4,3,i)*(A(5,3,i) - (981*sin(zk(3,i)))/100))/100 + (Sk(4,4,i)*(A(5,4,i) + zk(6,i)))/100 + (Sk(4,6,i)*(A(5,6,i) + zk(4,i)))/100 + (A(5,1,i)*Sk(1,4,i))/100 + (A(5,2,i)*Sk(2,4,i))/100 + (A(5,1,i)*Sk(4,1,i))/100 + (A(5,2,i)*Sk(4,2,i))/100 + (A(5,5,i)*Sk(4,5,i))/100 + (A(5,5,i)*Sk(5,4,i))/100 + Sk(3,4,i)*(A(5,3,i)/100 - (981*sin(zk(3,i)))/10000)
                      ];
    
end
end