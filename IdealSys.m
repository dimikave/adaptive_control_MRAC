%% INTELLIGENT AND ADAPTIVE CONTROL SYSTEMS
% Assignment 2 - Winter Semester 2021/2022
% Kavelidis Frantzis Dimitrios - AEM 9351 - kavelids@ece.auth.gr - ECE AUTH

% Ideal System equations
function xxdot = IdealSys(t,xx)
    global  B Kx Am Bm
    xxdot = zeros(size(xx));
    r = RefSig(t);
    
    % xx description
    x = xx(1:3);
    xm = xx(4:6);
    
    % Controller
    u = -Kx*x;
    
    % System description
    xxdot(1:3) = Am*x + B*(u + Kx*x) + Bm*r;
    xxdot(4:6) = Am*xm + Bm*r;
end