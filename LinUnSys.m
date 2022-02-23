%% INTELLIGENT AND ADAPTIVE CONTROL SYSTEMS
% Assignment 2 - Winter Semester 2021/2022
% Kavelidis Frantzis Dimitrios - AEM 9351 - kavelids@ece.auth.gr - ECE AUTH

% Ideal System equations
function xxdot = LinUnSys(t,xx)
    global  B Kx Am Bm D ka kq
    xxdot = zeros(size(xx));
    r = RefSig(t);
    % xx description
    x = xx(1:3);
    xm = xx(4:6);
    
    % Controller
    u = -Kx*x;
%     Kxideal = (1/D*Kx+[0 ka kq]);
%     u = -Kxideal*x;
    % System description
    xxdot(1:3) = Am*x + B*D*(u + 1/D*Kx*x + [0 ka kq]*x) + Bm*r;
    xxdot(4:6) = Am*xm + Bm*r;
end