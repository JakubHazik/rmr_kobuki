%   Toto je skript na vytvorenie funkcie, ktora popisuje zavislost
%   polomeru trajektorie robota vzhladom na pozadovany uhol otocenia (
%   rozdiel pozadovaneho a aktualneho uhla 

%% priprava dat
angle =  [180 150 120 90 60  45  30  15   7     0  ]';   % uhol natocenia
radius = [0   0   0   20 60  110 200 500  1500  20000]';  % polomer kruznice 

angle = angle.*(pi/180); % deg to rad

plot(angle, radius, '*');
hold on;

%% aproximacia
% https://www.mathworks.com/examples/curvefitting/mw/curvefit-ex72685292-fit-exponential-models-using-the-fit-function
f = fit(angle,radius,'exp2');
a=f.a;
b=f.b;
c=f.c;
d=f.d;

angle_fit = 0:180;
angle_fit = angle_fit.*(pi/180);

result = a*exp(b*angle_fit) + c*exp(d*angle_fit);
plot(angle_fit, result) 


%% jednoduche prenasobenie
% angle = 0:180;
% angle = angle.*(pi/180);
% radius = 100./(angle).^2;
% plot(angle, radius);
