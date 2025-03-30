function [out_Fyw, out_Fxw]=Tire_Model(alpha,kappa,Fzt,roll)

% tire longitudinal stiffness
Tire.CS0=3.840078061e3; %C_km
Tire.Eta=1.256085510479; %n
Tire.Fzx0=2.385857011e2; %F_ZC_km
% tire lateral stiffness
Tire.CA1=-549866853851; %C_1
Tire.CA2=-211626207379; %C_2
Tire.CAm=1.604281826e4; %C_am
Tire.CAFzm=2500; %F_ZC_am
% inclination angle lateral force stiffness
Tire.GAMMA1=0.743808159855; %C_r1
Tire.GAMMA2=-8.772919630e-5; %C_r2
% aligning moment pneumatic trail
Tire.tz2=-8.488540386e-5; %t_z2
Tire.tz1=-1.446513558e-8; %t_z1
% aligning moment constants
Tire.m1=0.6; %m_1
Tire.m0=pi/4; %m_0
% sliding force eccentricity
Tire.Epsx=0.01; %e_x
% overturning moment arm
Tire.Tx3=-0.05374567698; %t_x3
Tire.Tx2=3.106407447e-5; %t_x2
Tire.Tx1=2.397975474e-8; %t_x1
% road to test friction rate
Tire.MUNOM=0.85; %Sliding Friction of Simulation
Tire.MUNTEST=0.85; %Estimated Sliding Friction of Testing Surface
% longitudinal peak coefficient of friction
Tire.Mux0=1.075904559373; %u_px0
Tire.Mux1=0.026628558245; %n_1x
Tire.Mux2=-0.02196648837; %n_2x
% Lateral peak coefficient of friction
Tire.Muy0=1.192842481344; %u_py0
Tire.Muy1=-0.04960551335; %n_1y
Tire.Muy2=-0.14093687309; %n_2y
% minimum load for valid peak friction values
Tire.Fz0=4.607008849e2; %F_z0
% longitudinal sliding coefficient of friction
Tire.KMUx3=0.095162164771; %d_3
Tire.KMUx2=2.186713890e-4; %d_2
Tire.KMUx1=-1.999969731e-9; %d_1%%%%%%%%%
Tire.Lsx=1.05; %e_sx
% lateral sliding coefficient of friction
Tire.KMUy3=0.327952438694; %d_3
Tire.KMUy2=1.798822501e-4; %d_2
Tire.KMUy1=-9.131892344e-8; %d_1
Tire.Lsy=1.10; %e_sy
% plysteer
Tire.PLYSTEER=3.714581147e-4; %b

Fyw=[0 0 0 0]; %[N]
Fxw=[0 0 0 0]; %[N]

if roll<-1.5 
    roll=-1.5
elseif roll>1.5
    roll=1.5
end

Fzt_lbs=(Fzt/0.453592/9.8+1)./cos(roll);%N to lbs
%tire forces
[Fyw(1),Fxw(1),Mz,Mx]=Salaani_Model(Tire,alpha(1), kappa(1), 0, Fzt_lbs(1));
[Fyw(2),Fxw(2),Mz,Mx]=Salaani_Model(Tire,alpha(2), kappa(2), 0, Fzt_lbs(2));
[Fyw(3),Fxw(3),Mz,Mx]=Salaani_Model(Tire,alpha(3), kappa(3), 0, Fzt_lbs(3));
[Fyw(4),Fxw(4),Mz,Mx]=Salaani_Model(Tire,alpha(4), kappa(4), 0, Fzt_lbs(4));
Fyw=Fyw*0.453592*9.8;%lbs to N
Fxw=Fxw*0.453592*9.8;%lbs to N

Fyt=Fzt*tan(roll);
Fyw=Fyw+Fyt;

% Fxw%=Fxw-Fzt*0.1
% Fzt*0.1
% Fxw=
% % Fxw
% Fxw-Fzt*0.2;
% Fxw=Fxw-100;%-Fzt*1;

out_Fyw=Fyw; out_Fxw=Fxw;

return