function error = linearization_error(hAt, hat, Bt, zt, nt, sig)
% The linearization error on the nominal points zt

error = zeros(nt,1);
dt = sig/(nt-1);
g = 9.81;
for i = 1:nt
%     % controllability
%     Co = ctrb(hAt(:,:,i), Bt(:,:,i));
%     if (rank(Co)<6)
%         disp('Uncontrollable')
%     end
    linearized_dyn = hAt(:,:,i)*zt(:,i)*dt + hat(:,i)*dt;
    
    nonlinear_dyn = [zt(4,i)*cos(zt(3,i)) - zt(5,i)*sin(zt(3,i));
                                      zt(4,i)*sin(zt(3,i)) + zt(5,i)*cos(zt(3,i));
                                      zt(6,i);
                                      zt(5,i)*zt(6,i) - g*sin(zt(3,i));
                                      -zt(4,i)*zt(6,i) - g*cos(zt(3,i));
                                      0]*dt;
    
    error(i) = norm(linearized_dyn - nonlinear_dyn, 'fro');

end

% % plot linearization error with the dotphi
% figure
% grid minor
% hold on
% plot(error, 'LineWidth', 2.0)
% xlabel('time')
% ylabel('Linearization error')
