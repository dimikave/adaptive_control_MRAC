%% INTELLIGENT AND ADAPTIVE CONTROL SYSTEMS
% Assignment 2 - Winter Semester 2021/2022
% Kavelidis Frantzis Dimitrios - AEM 9351 - kavelids@ece.auth.gr - ECE AUTH

% Reference Signal
function r = RefSig(t)
    if(t>=0)&(t<1)
        r = 0;
 	elseif (t>=1)&(t<10)
        r = 0.5;
    elseif (t>=10)&(t<22)
        r = 0;
    elseif (t>=22)&(t<32)
        r = -0.5;
    elseif (t>=32)&(t<45)
        r = 0;
    elseif (t>=45)&(t<55)
        r = 1;
    elseif (t>=55)&(t<65)
        r = 0;
    elseif (t>=65)&(t<75)
        r = -1;
    elseif (t>=75)&(t<85)
        r = 0;
    elseif (t>=85)&(t<95)
        r = 0.5;
    elseif t>=95
        r = 0;
    else
        r = 0;
    end
end