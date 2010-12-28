q1=0;
q2=0;
q3=0;
q4=0;
q5=0;
q6=0;
q7=0;
% eept = eval(T07)

disc = 0.05;

wkspt = [];
for q3=-20*pi/180:disc:20*pi/180
    eept = eval(T07);
    wkspt = [wkspt eept(1:3,4)];
end
wkspt