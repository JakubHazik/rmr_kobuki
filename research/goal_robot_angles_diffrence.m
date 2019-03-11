clear all;

%% Angle examples
R = [ 30, 120, -150,  -60,  120,  130, 130, -170]; % robot angle
G = [-60,  30,  120, -150, -150, -110, -30, -160]; % goal vector angle
ANS = [-90, -90, -90, -90,   90,  120,-160,  10]; % expected result

%% Algorith of computation No 1
for i = 1:length(R)
    
    if sign(R(i)) == sign(G(i))
        % body su v rovnakej polrovine, nad alebo pod X osou
        RESULT(i) = G(i) - R(i);
    else
        % body su v rozdielnych polrovinach        
        if (R(i) > 0)
            RESULT(i) = -1 * (abs(G(i)) + abs(R(i)));
        else
            RESULT(i) = (abs(G(i)) + abs(R(i)));
        end
            
        if RESULT(i) > 180
            RESULT(i) = -360 + abs(G(i)) + abs(R(i));
        end  
        if RESULT(i) < -180
            RESULT(i) = 360 - abs(G(i)) - abs(R(i));
        end       
    end
    
end

%% Algorith of computation No 2
% https://stackoverflow.com/questions/5188561/signed-angle-between-two-3d-vectors-with-same-origin-within-the-same-plane
for i = 1: length(R)
    Rxy = angle2vector(R(i));
    Gxy = angle2vector(G(i));
    
    RESULT(i) = acos(dot(Gxy, Rxy)) * 180 /pi;
    A = cross([Rxy 1],[Gxy 1]);
    RESULT(i) = RESULT(i)*sign(dot([0 0 1], A));
    
end


%% check result
for i = 1:length(R)
    if ANS(i) ~= RESULT(i)
        disp("!!!!!!!!!!!!!!!!!!nevychadza to!!!!!!!!!!!!!!!")   
        break;  
    end
end

function [vector] = angle2vector(angle)
    angle = angle * pi /180;
    vector(1) = cos(angle);
    vector(2) = sin(angle);
end
