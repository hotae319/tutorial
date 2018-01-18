
function [sys,x0,str,ts] = sim_s_4link_after_rel(t,x,u,flag,dgain,pgain,desire,ini)


format long;

switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts]=mdlInitializeSizes(ini);

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1,
    sys=mdlDerivatives(t,x,u,dgain,pgain,desire,ini);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3,
    sys=mdlOutputs(t,x,u,dgain,pgain,desire,ini);

  %%%%%%%%%%%%%%%%%%%
  % Unhandled flags %
  %%%%%%%%%%%%%%%%%%%
  case { 2, 4, 9 },
    sys = [];

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    error(['Unhandled flag = ',num2str(flag)]);

end
% end csfunc

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts]=mdlInitializeSizes(ini)

sizes = simsizes;
sizes.NumContStates  = 8;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 14;
sizes.NumInputs      = 1;
sizes.DirFeedthrough = 0; %input x 0 ; input o 1 about output
sizes.NumSampleTimes = 1;

sys = simsizes(sizes);


% x0=[0 pi/2 0 0.01 0.00 0 0 0 0 0];
x0=ini;
% x0=[6*pi/12; 6*pi/12-7.22;6*pi/12-pi/3;pi/3-5*pi/12; 35.3;36.5; 100;-105]; %test for 2017.01.16 inverse kinematics

str = [];
ts  = [0  0];

% end mdlInitializeSizes
%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
function sys=mdlDerivatives(t,x,u,dgain,pgain,desire,ini)
%%four link calculation
Lc0=0.06; Lc1=0.07; Lc2=0.04; Lc3=0.04;
L0=0.12; L1=0.14; L2=0.08; L3=0.08;
m0=0.0425; m1=0.0111; m2=0.01955; m3=0.0119;
I0=5.1*10^(-5); I1=1.81*10^(-5); I2=1.19*10^(-5); I3=0.63*10^(-5);


g=9.8; 

%PID gain
c=dgain;k=pgain;
%%MODEL
q0=x(1);qr1=x(2);qr2=x(3);qr3=x(4);dq0=x(5);dqr1=x(6);dqr2=x(7);dqr3=x(8);
T=u;
D =[ I0 + L0^2*m0 + L0^2*m1 + L2^2*m0 + L2^2*m1 + L3^2*m0 + L2^2*m2 + L3^2*m1 + L3^2*m2 + L3^2*m3 + Lc0^2*m0 + Lc1^2*m1 + Lc2^2*m2 + Lc3^2*m3 - 2*L0*Lc0*m0 - 2*L2*Lc2*m2 - 2*L3*Lc3*m3 + 2*L0*L3*m0*cos(qr2 + qr3) + 2*L0*L3*m1*cos(qr2 + qr3) - 2*L3*Lc0*m0*cos(qr2 + qr3) + 2*L0*L2*m0*cos(qr2) + 2*L0*L2*m1*cos(qr2) + 2*L2*L3*m0*cos(qr3) + 2*L2*L3*m1*cos(qr3) + 2*L2*L3*m2*cos(qr3) + 2*L0*Lc1*m1*cos(qr1) - 2*L2*Lc0*m0*cos(qr2) - 2*L3*Lc2*m2*cos(qr3) + 2*L2*Lc1*m1*cos(qr1 - qr2) + 2*L3*Lc1*m1*cos(qr2 - qr1 + qr3), -Lc1*m1*(Lc1 + L3*cos(qr2 - qr1 + qr3) + L0*cos(qr1) + L2*cos(qr1 - qr2)), 2*L2*Lc2*m2 - L2^2*m1 - L3^2*m0 - L2^2*m2 - L3^2*m1 - L3^2*m2 - L3^2*m3 - Lc2^2*m2 - Lc3^2*m3 - L2^2*m0 + 2*L3*Lc3*m3 - L0*L3*m0*cos(qr2 + qr3) - L0*L3*m1*cos(qr2 + qr3) + L3*Lc0*m0*cos(qr2 + qr3) - L0*L2*m0*cos(qr2) - L0*L2*m1*cos(qr2) - 2*L2*L3*m0*cos(qr3) - 2*L2*L3*m1*cos(qr3) - 2*L2*L3*m2*cos(qr3) + L2*Lc0*m0*cos(qr2) + 2*L3*Lc2*m2*cos(qr3) - L2*Lc1*m1*cos(qr1 - qr2) - L3*Lc1*m1*cos(qr2 - qr1 + qr3), 2*L3*Lc3*m3 - L3^2*m1 - L3^2*m2 - L3^2*m3 - Lc3^2*m3 - L3^2*m0 - L0*L3*m0*cos(qr2 + qr3) - L0*L3*m1*cos(qr2 + qr3) + L3*Lc0*m0*cos(qr2 + qr3) - L2*L3*m0*cos(qr3) - L2*L3*m1*cos(qr3) - L2*L3*m2*cos(qr3) + L3*Lc2*m2*cos(qr3) - L3*Lc1*m1*cos(qr2 - qr1 + qr3);
                                                                                                                                                                                                                                                                                                                                                                                                                                                  -Lc1*m1*(Lc1 + L3*cos(qr2 - qr1 + qr3) + L0*cos(qr1) + L2*cos(qr1 - qr2)),                                                             m1*Lc1^2 + I1,                                                                                                                                                                                                                                                                                                                                                                   Lc1*m1*(L3*cos(qr2 - qr1 + qr3) + L2*cos(qr1 - qr2)),                                                                                                                                                                                                                                  L3*Lc1*m1*cos(qr2 - qr1 + qr3);
                                                                                                     2*L2*Lc2*m2 - L2^2*m1 - L3^2*m0 - L2^2*m2 - L3^2*m1 - L3^2*m2 - L3^2*m3 - Lc2^2*m2 - Lc3^2*m3 - L2^2*m0 + 2*L3*Lc3*m3 - L0*L3*m0*cos(qr2 + qr3) - L0*L3*m1*cos(qr2 + qr3) + L3*Lc0*m0*cos(qr2 + qr3) - L0*L2*m0*cos(qr2) - L0*L2*m1*cos(qr2) - 2*L2*L3*m0*cos(qr3) - 2*L2*L3*m1*cos(qr3) - 2*L2*L3*m2*cos(qr3) + L2*Lc0*m0*cos(qr2) + 2*L3*Lc2*m2*cos(qr3) - L2*Lc1*m1*cos(qr1 - qr2) - L3*Lc1*m1*cos(qr2 - qr1 + qr3),                      Lc1*m1*(L3*cos(qr2 - qr1 + qr3) + L2*cos(qr1 - qr2)),                                                                                                                                                                                                    I2 + L2^2*m0 + L2^2*m1 + L3^2*m0 + L2^2*m2 + L3^2*m1 + L3^2*m2 + L3^2*m3 + Lc2^2*m2 + Lc3^2*m3 - 2*L2*Lc2*m2 - 2*L3*Lc3*m3 + 2*L2*L3*m0*cos(qr3) + 2*L2*L3*m1*cos(qr3) + 2*L2*L3*m2*cos(qr3) - 2*L3*Lc2*m2*cos(qr3),                                                                                                                 L3^2*m0 + L3^2*m1 + L3^2*m2 + L3^2*m3 + Lc3^2*m3 - 2*L3*Lc3*m3 + L2*L3*m0*cos(qr3) + L2*L3*m1*cos(qr3) + L2*L3*m2*cos(qr3) - L3*Lc2*m2*cos(qr3);
                                                                                                                                                                                                                                                            2*L3*Lc3*m3 - L3^2*m1 - L3^2*m2 - L3^2*m3 - Lc3^2*m3 - L3^2*m0 - L0*L3*m0*cos(qr2 + qr3) - L0*L3*m1*cos(qr2 + qr3) + L3*Lc0*m0*cos(qr2 + qr3) - L2*L3*m0*cos(qr3) - L2*L3*m1*cos(qr3) - L2*L3*m2*cos(qr3) + L3*Lc2*m2*cos(qr3) - L3*Lc1*m1*cos(qr2 - qr1 + qr3),                                            L3*Lc1*m1*cos(qr2 - qr1 + qr3),                                                                                                                                                                                                                                                                        L3^2*m0 + L3^2*m1 + L3^2*m2 + L3^2*m3 + Lc3^2*m3 - 2*L3*Lc3*m3 + L2*L3*m0*cos(qr3) + L2*L3*m1*cos(qr3) + L2*L3*m2*cos(qr3) - L3*Lc2*m2*cos(qr3),                                                                                                                                                                                             I3 + L3^2*m0 + L3^2*m1 + L3^2*m2 + L3^2*m3 + Lc3^2*m3 - 2*L3*Lc3*m3];
 
 
C =[ - dqr2*(L0*L3*m0*sin(qr2 + qr3) + L0*L3*m1*sin(qr2 + qr3) - L3*Lc0*m0*sin(qr2 + qr3) + L0*L2*m0*sin(qr2) + L0*L2*m1*sin(qr2) - L2*Lc0*m0*sin(qr2) - L2*Lc1*m1*sin(qr1 - qr2) + L3*Lc1*m1*sin(qr2 - qr1 + qr3)) - L3*dqr3*(Lc1*m1*sin(qr2 - qr1 + qr3) + L0*m0*sin(qr2 + qr3) + L0*m1*sin(qr2 + qr3) - Lc0*m0*sin(qr2 + qr3) + L2*m0*sin(qr3) + L2*m1*sin(qr3) + L2*m2*sin(qr3) - Lc2*m2*sin(qr3)) - Lc1*dqr1*m1*(L0*sin(qr1) - L3*sin(qr2 - qr1 + qr3) + L2*sin(qr1 - qr2)), -Lc1*m1*(dq0 - dqr1)*(L0*sin(qr1) - L3*sin(qr2 - qr1 + qr3) + L2*sin(qr1 - qr2)), L3*Lc1*dqr2*m1*sin(qr2 - qr1 + qr3) - L3*Lc1*dq0*m1*sin(qr2 - qr1 + qr3) + L3*Lc1*dqr3*m1*sin(qr2 - qr1 + qr3) - L0*L3*dq0*m0*sin(qr2 + qr3) - L0*L3*dq0*m1*sin(qr2 + qr3) + L0*L3*dqr2*m0*sin(qr2 + qr3) + L0*L3*dqr2*m1*sin(qr2 + qr3) + L0*L3*dqr3*m0*sin(qr2 + qr3) + L0*L3*dqr3*m1*sin(qr2 + qr3) + L3*Lc0*dq0*m0*sin(qr2 + qr3) - L3*Lc0*dqr2*m0*sin(qr2 + qr3) - L3*Lc0*dqr3*m0*sin(qr2 + qr3) - L0*L2*dq0*m0*sin(qr2) - L0*L2*dq0*m1*sin(qr2) + L0*L2*dqr2*m0*sin(qr2) + L0*L2*dqr2*m1*sin(qr2) + L2*L3*dqr3*m0*sin(qr3) + L2*L3*dqr3*m1*sin(qr3) + L2*L3*dqr3*m2*sin(qr3) + L2*Lc0*dq0*m0*sin(qr2) - L2*Lc0*dqr2*m0*sin(qr2) - L3*Lc2*dqr3*m2*sin(qr3) + L2*Lc1*dq0*m1*sin(qr1 - qr2) - L2*Lc1*dqr2*m1*sin(qr1 - qr2), L3*(dqr2 - dq0 + dqr3)*(Lc1*m1*sin(qr2 - qr1 + qr3) + L0*m0*sin(qr2 + qr3) + L0*m1*sin(qr2 + qr3) - Lc0*m0*sin(qr2 + qr3) + L2*m0*sin(qr3) + L2*m1*sin(qr3) + L2*m2*sin(qr3) - Lc2*m2*sin(qr3));
                                                                                                                                                                                                                                                                                                       Lc1*m1*(L2*dq0*sin(qr1 - qr2) - L2*dqr2*sin(qr1 - qr2) - L3*dq0*sin(qr2 - qr1 + qr3) + L3*dqr2*sin(qr2 - qr1 + qr3) + L3*dqr3*sin(qr2 - qr1 + qr3) + L0*dq0*sin(qr1)),                                                                                0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           -Lc1*m1*(L2*dq0*sin(qr1 - qr2) - L2*dqr2*sin(qr1 - qr2) - L3*dq0*sin(qr2 - qr1 + qr3) + L3*dqr2*sin(qr2 - qr1 + qr3) + L3*dqr3*sin(qr2 - qr1 + qr3)),                                                                                                                                             -L3*Lc1*m1*sin(qr2 - qr1 + qr3)*(dqr2 - dq0 + dqr3);
                                                            L3*Lc1*dq0*m1*sin(qr2 - qr1 + qr3) - L3*Lc1*dqr1*m1*sin(qr2 - qr1 + qr3) + L0*L3*dq0*m0*sin(qr2 + qr3) + L0*L3*dq0*m1*sin(qr2 + qr3) - L3*Lc0*dq0*m0*sin(qr2 + qr3) + L0*L2*dq0*m0*sin(qr2) + L0*L2*dq0*m1*sin(qr2) + L2*L3*dqr3*m0*sin(qr3) + L2*L3*dqr3*m1*sin(qr3) + L2*L3*dqr3*m2*sin(qr3) - L2*Lc0*dq0*m0*sin(qr2) - L3*Lc2*dqr3*m2*sin(qr3) - L2*Lc1*dq0*m1*sin(qr1 - qr2) + L2*Lc1*dqr1*m1*sin(qr1 - qr2),               -Lc1*m1*(L3*sin(qr2 - qr1 + qr3) - L2*sin(qr1 - qr2))*(dq0 - dqr1),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             -L3*dqr3*sin(qr3)*(L2*m0 + L2*m1 + L2*m2 - Lc2*m2),                                                                                                                               -L3*sin(qr3)*(dqr2 - dq0 + dqr3)*(L2*m0 + L2*m1 + L2*m2 - Lc2*m2);
                                                                                                                                     L3*(Lc1*dq0*m1*sin(qr2 - qr1 + qr3) - Lc1*dqr1*m1*sin(qr2 - qr1 + qr3) + L0*dq0*m0*sin(qr2 + qr3) + L0*dq0*m1*sin(qr2 + qr3) - Lc0*dq0*m0*sin(qr2 + qr3) + L2*dq0*m0*sin(qr3) + L2*dq0*m1*sin(qr3) + L2*dq0*m2*sin(qr3) - L2*dqr2*m0*sin(qr3) - L2*dqr2*m1*sin(qr3) - L2*dqr2*m2*sin(qr3) - Lc2*dq0*m2*sin(qr3) + Lc2*dqr2*m2*sin(qr3)),                                     -L3*Lc1*m1*sin(qr2 - qr1 + qr3)*(dq0 - dqr1),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     -L3*sin(qr3)*(dq0 - dqr2)*(L2*m0 + L2*m1 + L2*m2 - Lc2*m2),                                                                                                                                                                                               0];
 
 
poten =[ g*m2*(L3*cos(qr2 - q0 + qr3) + cos(q0 - qr2)*(L2 - Lc2)) + g*m1*(L3*cos(qr2 - q0 + qr3) + L0*cos(q0) + L2*cos(q0 - qr2) + Lc1*cos(q0 - qr1)) + g*m0*(L3*cos(qr2 - q0 + qr3) + cos(q0)*(L0 - Lc0) + L2*cos(q0 - qr2)) + g*m3*cos(qr2 - q0 + qr3)*(L3 - Lc3);
                                                                                                                                                                                                                                    -Lc1*g*m1*cos(q0 - qr1);
                                                     - g*m0*(L3*cos(qr2 - q0 + qr3) + L2*cos(q0 - qr2)) - g*m1*(L3*cos(qr2 - q0 + qr3) + L2*cos(q0 - qr2)) - g*m2*(L3*cos(qr2 - q0 + qr3) + cos(q0 - qr2)*(L2 - Lc2)) - g*m3*cos(qr2 - q0 + qr3)*(L3 - Lc3);
                                                                                                                                                                                            -g*cos(qr2 - q0 + qr3)*(L3*m0 + L3*m1 + L3*m2 + L3*m3 - Lc3*m3)];
 
% %control after impact
q0d=desire(1);qr1d=desire(2);qr2d=desire(3);qr3d=desire(4);
% q0d=17*pi/18; q2d=pi/4; q3d=14*pi/18; q1d=4*pi/3; 
% a=D^-(1)*C*[dq0;dq1;dq2;dq3]+D^(-1)*poten;
% b=[a(1,:);a(3,:);a(4,:)]+[-c(1)*dq0-k(1)*(q0-q0d);-c(2)*dq2-k(2)*(q2-q2d);-c(3)*dq3-k(3)*(q3-q3d)];
% c=D^(-1)*[1 -1 0;0 1 0;-1 0 1;0 0 -1]; d=[c(1,:);c(3,:);c(4,:)];
% % T=[0;0;0;0];
% T=d^(-1)*b;
% T0=T(1);
% T1=T(2); 
% T2=T(3);

% TE=[T0-T1;T1;T2-T0;T3-T2];

u=D*[-dgain(1)*dq0-pgain(1)*(q0-q0d);-dgain(2)*dqr1-pgain(2)*(qr1-qr1d);-dgain(3)*dqr2-pgain(3)*(qr2-qr2d);-dgain(4)*dqr3-pgain(4)*(qr3-qr3d)]+C*[dq0;dqr1;dqr2;dqr3]+poten;
   
sys=[dq0;dqr1;dqr2;dqr3;-D^(-1)*(C*[dq0;dqr1;dqr2;dqr3]+poten)+D^(-1)*u]; 
% sys = zeros(8,1);



% end mdlDerivatives
%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u,dgain,pgain,desire,ini)

%%four link calculation
Lc0=0.06; Lc1=0.07; Lc2=0.04; Lc3=0.04;
L0=0.12; L1=0.14; L2=0.08; L3=0.08;
m0=0.0425; m1=0.0111; m2=0.01955; m3=0.0119;
I0=5.1*10^(-5); I1=1.81*10^(-5); I2=1.19*10^(-5); I3=0.63*10^(-5);


g=9.8; 

%PID gain
c=dgain;k=pgain;
%%MODEL
q0=x(1);qr1=x(2);qr2=x(3);qr3=x(4);dq0=x(5);dqr1=x(6);dqr2=x(7);dqr3=x(8);
T=u;
D =[ I0 + L0^2*m0 + L0^2*m1 + L2^2*m0 + L2^2*m1 + L3^2*m0 + L2^2*m2 + L3^2*m1 + L3^2*m2 + L3^2*m3 + Lc0^2*m0 + Lc1^2*m1 + Lc2^2*m2 + Lc3^2*m3 - 2*L0*Lc0*m0 - 2*L2*Lc2*m2 - 2*L3*Lc3*m3 + 2*L0*L3*m0*cos(qr2 + qr3) + 2*L0*L3*m1*cos(qr2 + qr3) - 2*L3*Lc0*m0*cos(qr2 + qr3) + 2*L0*L2*m0*cos(qr2) + 2*L0*L2*m1*cos(qr2) + 2*L2*L3*m0*cos(qr3) + 2*L2*L3*m1*cos(qr3) + 2*L2*L3*m2*cos(qr3) + 2*L0*Lc1*m1*cos(qr1) - 2*L2*Lc0*m0*cos(qr2) - 2*L3*Lc2*m2*cos(qr3) + 2*L2*Lc1*m1*cos(qr1 - qr2) + 2*L3*Lc1*m1*cos(qr2 - qr1 + qr3), -Lc1*m1*(Lc1 + L3*cos(qr2 - qr1 + qr3) + L0*cos(qr1) + L2*cos(qr1 - qr2)), 2*L2*Lc2*m2 - L2^2*m1 - L3^2*m0 - L2^2*m2 - L3^2*m1 - L3^2*m2 - L3^2*m3 - Lc2^2*m2 - Lc3^2*m3 - L2^2*m0 + 2*L3*Lc3*m3 - L0*L3*m0*cos(qr2 + qr3) - L0*L3*m1*cos(qr2 + qr3) + L3*Lc0*m0*cos(qr2 + qr3) - L0*L2*m0*cos(qr2) - L0*L2*m1*cos(qr2) - 2*L2*L3*m0*cos(qr3) - 2*L2*L3*m1*cos(qr3) - 2*L2*L3*m2*cos(qr3) + L2*Lc0*m0*cos(qr2) + 2*L3*Lc2*m2*cos(qr3) - L2*Lc1*m1*cos(qr1 - qr2) - L3*Lc1*m1*cos(qr2 - qr1 + qr3), 2*L3*Lc3*m3 - L3^2*m1 - L3^2*m2 - L3^2*m3 - Lc3^2*m3 - L3^2*m0 - L0*L3*m0*cos(qr2 + qr3) - L0*L3*m1*cos(qr2 + qr3) + L3*Lc0*m0*cos(qr2 + qr3) - L2*L3*m0*cos(qr3) - L2*L3*m1*cos(qr3) - L2*L3*m2*cos(qr3) + L3*Lc2*m2*cos(qr3) - L3*Lc1*m1*cos(qr2 - qr1 + qr3);
                                                                                                                                                                                                                                                                                                                                                                                                                                                  -Lc1*m1*(Lc1 + L3*cos(qr2 - qr1 + qr3) + L0*cos(qr1) + L2*cos(qr1 - qr2)),                                                             m1*Lc1^2 + I1,                                                                                                                                                                                                                                                                                                                                                                   Lc1*m1*(L3*cos(qr2 - qr1 + qr3) + L2*cos(qr1 - qr2)),                                                                                                                                                                                                                                  L3*Lc1*m1*cos(qr2 - qr1 + qr3);
                                                                                                     2*L2*Lc2*m2 - L2^2*m1 - L3^2*m0 - L2^2*m2 - L3^2*m1 - L3^2*m2 - L3^2*m3 - Lc2^2*m2 - Lc3^2*m3 - L2^2*m0 + 2*L3*Lc3*m3 - L0*L3*m0*cos(qr2 + qr3) - L0*L3*m1*cos(qr2 + qr3) + L3*Lc0*m0*cos(qr2 + qr3) - L0*L2*m0*cos(qr2) - L0*L2*m1*cos(qr2) - 2*L2*L3*m0*cos(qr3) - 2*L2*L3*m1*cos(qr3) - 2*L2*L3*m2*cos(qr3) + L2*Lc0*m0*cos(qr2) + 2*L3*Lc2*m2*cos(qr3) - L2*Lc1*m1*cos(qr1 - qr2) - L3*Lc1*m1*cos(qr2 - qr1 + qr3),                      Lc1*m1*(L3*cos(qr2 - qr1 + qr3) + L2*cos(qr1 - qr2)),                                                                                                                                                                                                    I2 + L2^2*m0 + L2^2*m1 + L3^2*m0 + L2^2*m2 + L3^2*m1 + L3^2*m2 + L3^2*m3 + Lc2^2*m2 + Lc3^2*m3 - 2*L2*Lc2*m2 - 2*L3*Lc3*m3 + 2*L2*L3*m0*cos(qr3) + 2*L2*L3*m1*cos(qr3) + 2*L2*L3*m2*cos(qr3) - 2*L3*Lc2*m2*cos(qr3),                                                                                                                 L3^2*m0 + L3^2*m1 + L3^2*m2 + L3^2*m3 + Lc3^2*m3 - 2*L3*Lc3*m3 + L2*L3*m0*cos(qr3) + L2*L3*m1*cos(qr3) + L2*L3*m2*cos(qr3) - L3*Lc2*m2*cos(qr3);
                                                                                                                                                                                                                                                            2*L3*Lc3*m3 - L3^2*m1 - L3^2*m2 - L3^2*m3 - Lc3^2*m3 - L3^2*m0 - L0*L3*m0*cos(qr2 + qr3) - L0*L3*m1*cos(qr2 + qr3) + L3*Lc0*m0*cos(qr2 + qr3) - L2*L3*m0*cos(qr3) - L2*L3*m1*cos(qr3) - L2*L3*m2*cos(qr3) + L3*Lc2*m2*cos(qr3) - L3*Lc1*m1*cos(qr2 - qr1 + qr3),                                            L3*Lc1*m1*cos(qr2 - qr1 + qr3),                                                                                                                                                                                                                                                                        L3^2*m0 + L3^2*m1 + L3^2*m2 + L3^2*m3 + Lc3^2*m3 - 2*L3*Lc3*m3 + L2*L3*m0*cos(qr3) + L2*L3*m1*cos(qr3) + L2*L3*m2*cos(qr3) - L3*Lc2*m2*cos(qr3),                                                                                                                                                                                             I3 + L3^2*m0 + L3^2*m1 + L3^2*m2 + L3^2*m3 + Lc3^2*m3 - 2*L3*Lc3*m3];
 
 
C =[ - dqr2*(L0*L3*m0*sin(qr2 + qr3) + L0*L3*m1*sin(qr2 + qr3) - L3*Lc0*m0*sin(qr2 + qr3) + L0*L2*m0*sin(qr2) + L0*L2*m1*sin(qr2) - L2*Lc0*m0*sin(qr2) - L2*Lc1*m1*sin(qr1 - qr2) + L3*Lc1*m1*sin(qr2 - qr1 + qr3)) - L3*dqr3*(Lc1*m1*sin(qr2 - qr1 + qr3) + L0*m0*sin(qr2 + qr3) + L0*m1*sin(qr2 + qr3) - Lc0*m0*sin(qr2 + qr3) + L2*m0*sin(qr3) + L2*m1*sin(qr3) + L2*m2*sin(qr3) - Lc2*m2*sin(qr3)) - Lc1*dqr1*m1*(L0*sin(qr1) - L3*sin(qr2 - qr1 + qr3) + L2*sin(qr1 - qr2)), -Lc1*m1*(dq0 - dqr1)*(L0*sin(qr1) - L3*sin(qr2 - qr1 + qr3) + L2*sin(qr1 - qr2)), L3*Lc1*dqr2*m1*sin(qr2 - qr1 + qr3) - L3*Lc1*dq0*m1*sin(qr2 - qr1 + qr3) + L3*Lc1*dqr3*m1*sin(qr2 - qr1 + qr3) - L0*L3*dq0*m0*sin(qr2 + qr3) - L0*L3*dq0*m1*sin(qr2 + qr3) + L0*L3*dqr2*m0*sin(qr2 + qr3) + L0*L3*dqr2*m1*sin(qr2 + qr3) + L0*L3*dqr3*m0*sin(qr2 + qr3) + L0*L3*dqr3*m1*sin(qr2 + qr3) + L3*Lc0*dq0*m0*sin(qr2 + qr3) - L3*Lc0*dqr2*m0*sin(qr2 + qr3) - L3*Lc0*dqr3*m0*sin(qr2 + qr3) - L0*L2*dq0*m0*sin(qr2) - L0*L2*dq0*m1*sin(qr2) + L0*L2*dqr2*m0*sin(qr2) + L0*L2*dqr2*m1*sin(qr2) + L2*L3*dqr3*m0*sin(qr3) + L2*L3*dqr3*m1*sin(qr3) + L2*L3*dqr3*m2*sin(qr3) + L2*Lc0*dq0*m0*sin(qr2) - L2*Lc0*dqr2*m0*sin(qr2) - L3*Lc2*dqr3*m2*sin(qr3) + L2*Lc1*dq0*m1*sin(qr1 - qr2) - L2*Lc1*dqr2*m1*sin(qr1 - qr2), L3*(dqr2 - dq0 + dqr3)*(Lc1*m1*sin(qr2 - qr1 + qr3) + L0*m0*sin(qr2 + qr3) + L0*m1*sin(qr2 + qr3) - Lc0*m0*sin(qr2 + qr3) + L2*m0*sin(qr3) + L2*m1*sin(qr3) + L2*m2*sin(qr3) - Lc2*m2*sin(qr3));
                                                                                                                                                                                                                                                                                                       Lc1*m1*(L2*dq0*sin(qr1 - qr2) - L2*dqr2*sin(qr1 - qr2) - L3*dq0*sin(qr2 - qr1 + qr3) + L3*dqr2*sin(qr2 - qr1 + qr3) + L3*dqr3*sin(qr2 - qr1 + qr3) + L0*dq0*sin(qr1)),                                                                                0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           -Lc1*m1*(L2*dq0*sin(qr1 - qr2) - L2*dqr2*sin(qr1 - qr2) - L3*dq0*sin(qr2 - qr1 + qr3) + L3*dqr2*sin(qr2 - qr1 + qr3) + L3*dqr3*sin(qr2 - qr1 + qr3)),                                                                                                                                             -L3*Lc1*m1*sin(qr2 - qr1 + qr3)*(dqr2 - dq0 + dqr3);
                                                            L3*Lc1*dq0*m1*sin(qr2 - qr1 + qr3) - L3*Lc1*dqr1*m1*sin(qr2 - qr1 + qr3) + L0*L3*dq0*m0*sin(qr2 + qr3) + L0*L3*dq0*m1*sin(qr2 + qr3) - L3*Lc0*dq0*m0*sin(qr2 + qr3) + L0*L2*dq0*m0*sin(qr2) + L0*L2*dq0*m1*sin(qr2) + L2*L3*dqr3*m0*sin(qr3) + L2*L3*dqr3*m1*sin(qr3) + L2*L3*dqr3*m2*sin(qr3) - L2*Lc0*dq0*m0*sin(qr2) - L3*Lc2*dqr3*m2*sin(qr3) - L2*Lc1*dq0*m1*sin(qr1 - qr2) + L2*Lc1*dqr1*m1*sin(qr1 - qr2),               -Lc1*m1*(L3*sin(qr2 - qr1 + qr3) - L2*sin(qr1 - qr2))*(dq0 - dqr1),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             -L3*dqr3*sin(qr3)*(L2*m0 + L2*m1 + L2*m2 - Lc2*m2),                                                                                                                               -L3*sin(qr3)*(dqr2 - dq0 + dqr3)*(L2*m0 + L2*m1 + L2*m2 - Lc2*m2);
                                                                                                                                     L3*(Lc1*dq0*m1*sin(qr2 - qr1 + qr3) - Lc1*dqr1*m1*sin(qr2 - qr1 + qr3) + L0*dq0*m0*sin(qr2 + qr3) + L0*dq0*m1*sin(qr2 + qr3) - Lc0*dq0*m0*sin(qr2 + qr3) + L2*dq0*m0*sin(qr3) + L2*dq0*m1*sin(qr3) + L2*dq0*m2*sin(qr3) - L2*dqr2*m0*sin(qr3) - L2*dqr2*m1*sin(qr3) - L2*dqr2*m2*sin(qr3) - Lc2*dq0*m2*sin(qr3) + Lc2*dqr2*m2*sin(qr3)),                                     -L3*Lc1*m1*sin(qr2 - qr1 + qr3)*(dq0 - dqr1),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     -L3*sin(qr3)*(dq0 - dqr2)*(L2*m0 + L2*m1 + L2*m2 - Lc2*m2),                                                                                                                                                                                               0];
 
 
poten =[ g*m2*(L3*cos(qr2 - q0 + qr3) + cos(q0 - qr2)*(L2 - Lc2)) + g*m1*(L3*cos(qr2 - q0 + qr3) + L0*cos(q0) + L2*cos(q0 - qr2) + Lc1*cos(q0 - qr1)) + g*m0*(L3*cos(qr2 - q0 + qr3) + cos(q0)*(L0 - Lc0) + L2*cos(q0 - qr2)) + g*m3*cos(qr2 - q0 + qr3)*(L3 - Lc3);
                                                                                                                                                                                                                                    -Lc1*g*m1*cos(q0 - qr1);
                                                     - g*m0*(L3*cos(qr2 - q0 + qr3) + L2*cos(q0 - qr2)) - g*m1*(L3*cos(qr2 - q0 + qr3) + L2*cos(q0 - qr2)) - g*m2*(L3*cos(qr2 - q0 + qr3) + cos(q0 - qr2)*(L2 - Lc2)) - g*m3*cos(qr2 - q0 + qr3)*(L3 - Lc3);
                                                                                                                                                                                            -g*cos(qr2 - q0 + qr3)*(L3*m0 + L3*m1 + L3*m2 + L3*m3 - Lc3*m3)];
 
% %control after impact
q0d=desire(1);qr1d=desire(2);qr2d=desire(3);qr3d=desire(4);
u=D*[-dgain(1)*dq0-pgain(1)*(q0-q0d);-dgain(2)*dqr1-pgain(2)*(qr1-qr1d);-dgain(3)*dqr2-pgain(3)*(qr2-qr2d);-dgain(4)*dqr3-pgain(4)*(qr3-qr3d)]+C*[dq0;dqr1;dqr2;dqr3]+poten;
  
%Find COM 
p0=[L3*cos(q0-qr2-qr3)+L2*cos(q0-qr2)+(L0-Lc0)*cos(q0);L3*sin(q0-qr2-qr3)+L2*sin(q0-qr2)+(L0-Lc0)*sin(q0)]; 
p1=[L3*cos(q0-qr2-qr3)+L2*cos(q0-qr2)+L0*cos(q0)+Lc1*cos(q0-qr1);L3*sin(q0-qr2-qr3)+L2*sin(q0-qr2)+L0*sin(q0)+Lc1*sin(q0-qr1)];
p2=[L3*cos(q0-qr2-qr3)+(L2-Lc2)*cos(q0-qr2);L3*sin(q0-qr2-qr3)+(L2-Lc2)*sin(q0-qr2)];
p3=[(L3-Lc3)*cos(q0-qr2-qr3);(L3-Lc3)*sin(q0-qr2-qr3)];
fcm=(m0*p0+m1*p1+m2*p2+m3*p3)/(m0+m1+m2+m3);

sys=[x(1);x(2);x(3);x(4);x(5);x(6);x(7);x(8);u;fcm(1,1);fcm(2,1)];


% end mdlOutputs
