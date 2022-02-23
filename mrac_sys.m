%% INTELLIGENT AND ADAPTIVE CONTROL SYSTEMS
% Assignment 2 - Winter Semester 2021/2022
% Kavelidis Frantzis Dimitrios - AEM 9351 - kavelids@ece.auth.gr - ECE AUTH

% Ideal System equations
function xxdot = mrac_sys(t,xx)
    global  B Kx Am Bm D ka kq Gammax Pm
    xxdot = zeros(size(xx));
    r = RefSig(t);
    
    % xx description
    x = xx(1:3);
    xm = xx(4:6);
    Kxest = xx(7:9);
    
    % Error
    e = x - xm;
    % Controller
    u = -Kxest'*x;

    % System description
    xxdot(1:3) = Am*x + B*D*(u + 1/D*Kx*x + [0 ka kq]*x) + Bm*r;
    xxdot(4:6) = Am*xm + Bm*r;
    xxdot(7:9) = Gammax*x*e'*Pm*B;
end