function [out1, out2]=Vehicle_Parameters()
% structure for vehicle parameters

par.T=0.001; %[sec]
par.g=9.8; %[m/s^2]

%body
par.mc=1210; %[kg]
par.ixc=711.0424; %[kgm^2]
par.iyc=2.6070e+03; %[kgm^2]
par.izc=2.6744e+03; %[kgm^2]
par.af=1.320; %[m]
par.ar=1.320; %[m]
par.b=1.586/2; %[m]
par.h0=0.762; %[m]

%wheel
par.Rw=0.7018/2; %[m]
par.iyw=2.0; %[kgm^2]

Veh.Tyw=[0 0 0 0]; %[Nm]

Veh.delta=[0 0]; %[rad]
Veh.delta_center=0;%[rad]
Veh.alpha=[0 0 0 0]; %[rad]
Veh.kappa=[0 0 0 0]; %[rate]
% Veh.last_kappa=[0 0 0 0]; %[rate]
% Veh.velocity.target=-5; %[m/s]
% Veh.velocity.ini=10; %[m/s]

%states
Veh.rot_matrix=eye(3);
Veh.vxc=0; %[m/s]
Veh.vyc=0; %[m/s]
Veh.vzc=0; %[m/s]

Veh.vrxc=0; %[rad/s]
Veh.vryc=0; %[rad/s]
Veh.vrzc=0; %[rad/s]

Veh.vxo=8.8; %[m/s]
Veh.vyo=0; %[m/s]
Veh.vzo=0; %[m/s]
Veh.xo=0; %[m]
Veh.yo=0; %[m]
Veh.zo=0; %[m]

Veh.lw=[0 0 0 0]; %[m]

Veh.roll=0;
Veh.pitch=0;
Veh.yaw=0;

Veh.delta=[0 0]; %[rad]29+12+93+tire16

%Suspension a
Veh.hr=[0 0 0 0]; %[m]
Veh.hw=[0 0 0 0]; %[m]
Veh.Fzc=[0 0 0 0]; %[N]
Veh.vhw=[0 0 0 0]; %[m/s]
Veh.last_ls=[0 0 0 0]; %[m]
%Suspension b
Veh.Fzt=[0 0 0 0]; %[N]
Veh.Fzw=[0 0 0 0]; %[N]
Veh.ahw=[0 0 0 0]; %[m/s^2]
Veh.hc=[0 0 0 0]; %[m]
Veh.ls=[0 0 0 0]; %[m]
Veh.vls=[0 0 0 0]; %[m/s]

%Tire 
Veh.Fyw=[0 0 0 0]; %[N]
Veh.Fxw=[0 0 0 0]; %[N]

%Aero
Veh.Fxca=0;

Veh.Fxc=[0 0 0 0]; %[N]
Veh.Fyc=[0 0 0 0]; %[N]

Veh.Mxc=[0 0 0 0]; %[Nm]
Veh.Myc=[0 0 0 0]; %[Nm]
Veh.Mzc=[0 0 0 0]; %[Nm]

Veh.aryw=[0 0 0 0]; %[rad/s^2]
Veh.vryw=[0 0 0 0]; %[rad/s]
Veh.ryw=[0 0 0 0]; %[rad]

Veh.vxw=[0 0 0 0]; %[m/s]
Veh.vyw=[0 0 0 0]; %[m/s]23

out1=Veh;
out2=par;

return