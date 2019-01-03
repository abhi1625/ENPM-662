Rotx = @(a)[1 0 0 0;0 cos(a) -sin(a) 0;0 sin(a) cos(a) 0;0 0 0 1];
Rotz = @(a)[cos(a) -sin(a) 0 0;sin(a) cos(a) 0 0;0 0 1 0;0 0 0 1];
Tranx = @(a)[eye(3,3),[a;0;0];0 0 0 1];
Tranz = @(a)[eye(3,3),[0;0;a];0 0 0 1];

Trans = @(x)Rotz(x(1))*Tranz(x(2))*Tranx(x(3))*Rotx(x(4));
syms t1 t2 d3 t4 t5 t6 l1 l2 l3 l4 l5 l6
Twb = [eye(3,3),[0;0;0.6];0 0 0 1];
% For Arm 1
Tb1 = [eye(3,3),[0.09;-0.3508;-0.005];0 0 0 1]*Rotx(pi/2);
A1 = Twb*Tb1;
%For Arm 2
Tb2 = [eye(3,3),[0.09;0.3508;-0.005];0 0 0 1]*Rotx(pi/2);
A2 = Twb*Tb2;

%Lengths of the parameters
l1 = 0.0392; %length of the Shoulder
l2 = 0.2; %length of the Arm
l3 = 0.0792; %length of the Elbow
l4 = 0.1620; %length of the Wrist
l5 = 0.02; %Distance to the CoM of the end effector
l6 = 0.0688;
%dh = [0,l1,0,-pi/2;pi/2,0,0,pi/2;-pi/2,(l2+l3),0,0;0,l4,0,pi/2;(pi/2),0,l5,pi/2];
% For Arm 1
T01_1 = simplify(Trans([t1,l1,0,-pi/2]));
T12_1 = simplify(Trans([t2+pi/2,0,0,pi/2]));
T23_1 = simplify(Trans([pi/2,(l2+l3+d3),0,0]));
T34_1 = simplify(Trans([t4,l4,0,-pi/2]));
T45_1 = simplify(Trans([t5,0,0,pi/2]));
T5n_1 = simplify(Trans([t6+pi,l5+l6,0,0]));

T0n_1 =simplify(T01_1*T12_1*T23_1*T34_1*T45_1*T5n_1);
p0n_1 = T0n_1(1:3,4);

%For Arm 2
T01_2 = simplify(Trans([t1,-l1,0,-pi/2]));
T12_2 = simplify(Trans([t2+pi/2,0,0,pi/2]));
T23_2 = simplify(Trans([pi/2,(l2+l3+d3),0,0]));
T34_2 = simplify(Trans([t4,l4,0,-pi/2]));
T45_2 = simplify(Trans([t5,0,0,pi/2]));
T5n_2 = simplify(Trans([t6+pi,l5+l6,0,0]));

T0n_2 =simplify(T01_2*T12_2*T23_2*T34_2*T45_2*T5n_2);
p0n_2 = T0n_2(1:3,4);


% T01 = vpa(simplify(subs(T01,{t1,t2,d3,t4,t5},{0,0,0,0,0})),4);
% T12 = vpa(simplify(subs(T12,{t1,t2,d3,t4,t5},{0,0,0,0,0})),4);
% T23 = vpa(simplify(subs(T23,{t1,t2,d3,t4,t5},{0,0,0,0,0})),4); 
% T34 = vpa(simplify(subs(T34,{t1,t2,d3,t4,t5},{0,0,0,0,0})),4);
% T4n = vpa(simplify(subs(T4n,{t1,t2,d3,t4,t5},{0,0,0,0,0})),4);
%T0n = vpa(simplify(subs(T0n,{t1,t2,d3,t4,t5},{0,0,0,0,0})),4);
%For Arm 1
Twn1 = vpa(simplify(A1*T0n_1),4);
pwn1 = Twn1(1:3,4);
%For Arm 2
Twn2 = vpa(simplify(A2*T0n_2),4);
pwn2 = Twn2(1:3,4);
%% Testcases
%%
%Case1: q = (0,-pi/6,0.1,pi/2,pi/3,pi/3)
Twn_1 = vpa(simplify(subs(Twn1,{t1,t2,d3,t4,t5,t6},{pi/3,0,0.1,0,pi/6,0})),4)
























Twn_2 = vpa(simplify(subs(Twn2,{t1,t2,d3,t4,t5,t6},{-pi/2,0,0,0,0,0})),4);

%pwn_1 = vpa(simplify(subs(pwn,{t1,t2,d3,t4,t5},{0,-pi/6,0.1,pi/2,0})),4);

% %Case2 : q = (0,pi/3,0,0,0)
% Twn_2 = vpa(simplify(subs(Twn,{t1,t2,d3,t4,t5},{0,pi/3,0,0,0})),4);
% 
% %Case2 : q = (0,0,0.2,0,0)
% Twn_3 = vpa(simplify(subs(Twn,{t1,t2,d3,t4,t5},{0,0,0.2,0,0})),4);
% 
% %Case4 : q = (0,0,0,pi/3,0)
% Twn_4 = vpa(simplify(subs(Twn,{t1,t2,d3,t4,t5},{0,0,0,pi/3,0})),4);
% 
% %Case5 : q = (0,0,0,0,pi/3)
% Twn_5 = vpa(simplify(subs(Twn,{t1,t2,d3,t4,t5},{0,0,0,0,pi/3})),4);