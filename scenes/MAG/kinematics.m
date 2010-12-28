clear all
syms q1 q2 q3 q4 q5 q6 q7;

T01a = trans(0,0,1.905);
T01b = trans(0,q1,0);
T01 = simple(T01a*T01b);

T12a = trans(0,0,0);
T12b = trans(q2,0,0);
T12 = simple(T12a*T12b);

T23a = trans(0,0,0);
T23b = roty(q3);
T23 = simple(T23a*T23b);

T34a = trans(0,0,0);
T34b = rotx(q4);
T34 = simple(T34a*T34b);

T45a = trans(1.245,0,0);
T45b = rotz(q5);
T45 = simple(T45a*T45b);

T56a = trans(.575,0,0);
T56b = roty(q6);
T56 = simple(T56a*T56b);

T67a = trans(1,0,0);
T67b = rotx(q7);
T67 = simple(T67a*T67b);

T01 = T01
T02 = simple(T01*T12)
T03 = simple(T02*T23)
T04 = simple(T03*T34)
T05 = simple(T04*T45)
T06 = simple(T05*T56)
T07 = simple(T06*T67)

q = [q1 q2 q3 q4 q5 q6 q7];
p7 = T07(1:3,4);


Jv = simple(jacobian(p7,q))
Jw = [ [0;0;0] [0;0;0] T03(1:3,2) T04(1:3,1) T05(1:3,3) T06(1:3,2) T07(1:3,1) ]

J = simple([ Jv ; Jw ])

% t1 = .3;
% t2 = .4;
% t3 = deg2rad(45);
% t4 = deg2rad(40);
% t5 = deg2rad(35);
% t6 = deg2rad(-90);
% t7 = deg2rad(90);

%subs(T07)

trig = {sin(q1), 's1';
        cos(q1), 'c2';
        sin(q2), 's2';
        cos(q2), 'c2';
        sin(q3), 's3';
        cos(q3), 'c3';
        sin(q4), 's4';
        cos(q4), 'c4';
        sin(q5), 's5';
        cos(q5), 'c5';
        sin(q6), 's6';
        cos(q6), 'c6';
        sin(q7), 's7';
        cos(q7), 'c7'};
 JJ = transpose(J)*J
 JJ = subs(JJ, trig(:,1), trig(:,2))
    
        
