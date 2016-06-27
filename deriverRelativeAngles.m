%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%% Derive double pendulum dynamics %%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Matthew Sheen

%Derive double pendulum equations of motion, write to Thdotdot1 and
%Thdotdot2 matlab function files. THIS VERSION DERIVES IT WITH THE ELBOW
%ANGLE RELATIVE TO THE UPPER ARM ANGLE.
% Controllers designed based on: http://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=407375&tag=1

%Parameters symbolically.
syms m1 m2 I1 I2 g l1 l2 d1 d2 th1 th2 thdot1 thdot2 thdotdot1 thdotdot2 er1 er2 eth1 eth2 T1 T2 real

%Unit vectors, cartesian
i = [1 0 0]';
j = [0 1 0]';
k = [0 0 1]';

%Rotating reference frames
er1 = [cos(th1), sin(th1), 0]'; %Just changed it so 0 is horizontal to the right.
er2 = [cos(th2+th1), sin(th2+th1), 0]';

eth1 = [-sin(th1),cos(th1),0]';
eth2 = [-sin(th2+th1),cos(th2+th1),0]';

%Vectors to significant points
ra_c1 = d1*er1; %A is fixed point, B is the elbow, c1 and c2 are COMs
rb_c2 = d2*er2;
ra_b = l1*er1;
ra_c2 = ra_b + rb_c2;

%Velocities
Vc1 = d1*thdot1*eth1;
VB = l1*thdot1*eth1;
Vc2 = VB + d2*(thdot2+thdot1)*eth2;

%Accelerations
Ac1 = d1*thdotdot1*eth1 - d1*thdot1^2*er1;
AB = l1*thdotdot1*eth1 - l1*thdot1^2*er1;
Ac2 = d2*(thdotdot2+thdotdot1)*eth2 - d2*(thdot1 + thdot2)^2*er2  + AB;

%AMB for just link 2:
M_B = cross(rb_c2,-m2*g*j)+T2*k;
Hdot2 = I2*(thdotdot2+thdotdot1)*k + cross(rb_c2, m2*Ac2);

eqn2forthdotdot2 = solve(dot(Hdot2 - M_B,k),thdotdot2);

%AMB for whole thing:
M_A = cross(ra_c2,-m2*g*j) + cross(ra_c1,-m1*g*j) + T1*k; %Gravity for both, plus a control torque.
Hdot1 = I2*(thdotdot2+thdotdot1)*k + cross(ra_c2, m2*Ac2) + I1*thdotdot1*k + cross(ra_c1, m1*Ac1);

eqn1forthdotdot1 = solve(dot(Hdot1 - M_A,k),thdotdot1);

%One equation for thdotdot1, one for thdotdot2.
eqn2 = simplify(solve(subs(eqn2forthdotdot2, thdotdot1, eqn1forthdotdot1)-thdotdot2,thdotdot2));
eqn1 = simplify(solve(subs(eqn1forthdotdot1, thdotdot2, eqn2forthdotdot2)-thdotdot1,thdotdot1));

%Create matlab functions for thdotdot1 and 2:
matlabFunction(eqn1, 'file', 'Thdotdot1');
matlabFunction(eqn2, 'file', 'Thdotdot2');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%% Partial Feedback Linearization   %%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Based on: http://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=407375

%%% Basic equation coefficients %%%

%We want the form M11q1'' + M12q2'' + h1 + phi1 = 0  -- h1 should contain
%the thdot terms, phi1 should contain everything leftover
pflin1 = simplify(subs(dot(Hdot1 - M_A,k),{'T1','T2'},{0,'T'})); %AMB for the whole thing doesn't contain a torque term.
M11 = diff(pflin1,'thdotdot1');
M12 = diff(pflin1,'thdotdot2');
h1 = simplify(diff(pflin1,'thdot1')*thdot1/2 + diff(pflin1,'thdot2')*thdot2/2); %The squared terms pop out a 2 coefficient which we need to divide by.
phi1 = simplify(pflin1 - M11*thdotdot1 - M12*thdotdot2 - h1);


%And M21q1'' + M22q2'' + h2 + phi2 = T
pflin2 = subs(dot(Hdot2 - M_B,k),{'T1','T2'},{0,'T'}); %AMB for just the second link contains T
pflin2 = simplify(solve(pflin2,'T'));
M21 = diff(pflin2,'thdotdot1');
M22 = diff(pflin2,'thdotdot2');
h2 = simplify(diff(pflin2,'thdot1')*thdot1/2 + diff(pflin2,'thdot2')*thdot2/2);%The squared terms pop out a 2 coefficient which we need to divide by.
phi2 = simplify(pflin2 - M21*thdotdot1 - M22*thdotdot2 - h2);

%Link 1 linearization
syms th1d kd kp real

d1bar = M21-M22*M11/M12;

h1bar = h2-M22*h1/M12;

phi1bar = phi2-M22*phi1/M12;

v1 = kd*-thdot1 + kp*(th1d - th1);

Tc = d1bar*v1 + h1bar + phi1bar;

matlabFunction(Tc, 'file', 'ControlTorque1');


%Link 2 linearization
syms th2d real

d2bar = M22 - M21*M12/M11;

h2bar = h2 - M21*h1/M11;

phi2bar = phi2 - M21*phi1/M11;

v2 = kd*-thdot2 + kp*(th2d - th2);

Tc = d2bar*v2 + h2bar + phi2bar;

matlabFunction(Tc, 'file', 'ControlTorque2');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%% Energy eqns  %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

syms k1 k2 real

PE = g*dot(ra_c2,j)*m2 + g*dot(ra_c1,j)*m1;
KE = 1/2*I1*thdot1^2 + 1/2*I2*(thdot2+thdot1)^2 + 1/2*m1*dot(Vc1,Vc1) + 1/2*m2*dot(Vc2,Vc2);

Etot = PE + KE;

matlabFunction(Etot, 'file', 'TotEnergy');

