clear all;
close all;

%% Vehicle Parameters
[Veh, par]=Vehicle_Parameters;
Simulation_Time=6;
lock=0;
r_path=[0,0,0,0];

for i=1:Simulation_Time/par.T;
if i<=2000
else
Veh.hr=[0.1,0,0,0];
end

%% Guidance Control
ref_delta=0;
%% Automatic Steering Model
[Veh.delta_center]=Automatic_Steering_Model(par,[1/0.00777],[1, 0.1515/0.00777, 1/0.00777],0.0284,ref_delta);%(par,[1/0.01248],[1, 0.1799/0.01248, 1/0.01248],0,ref_delta);
% Veh.delta_center=ref_delta;
%% Torque Control
ref_Tq=0;
%% Automatic Torque Control Model
[Tq]=Automatic_Torque_Control_Model(par,[1],[0.05117, 0.5545, 1],0.07,ref_Tq);
Veh.Tyw=[Tq,Tq,Tq,Tq]/4;
%% Ackerman Turning Geometry
[Veh.delta]=Ackerman_Turning_Geometry(par,Veh.delta_center);
% Veh.delta=[Veh.delta_center,Veh.delta_center];
%% Suspension Model
[Veh.hw,Veh.Fzc,Veh.vhw,Veh.Fzt,Veh.Fzw,Veh.ahw,Veh.hc,Veh.ls,Veh.vls]=Suspension_Model(par, Veh.hr, Veh.zo, Veh.rot_matrix);
%% Tire Model
[Veh.Fyw,Veh.Fxw]=Tire_Model(Veh.alpha,Veh.kappa,Veh.Fzt,0);%(Veh.alpha,Veh.kappa,Veh.Fzt,0);%(Veh.alpha,Veh.kappa,Veh.Fzt,Veh.roll);
%% Aerodynamic Model
Veh.Fxca=Aerodynamic_Model(Veh.vxc);
%% Resultant Force & Resultant Moment
[Fxcg,Fycg,Fzcg,Mxcg,Mycg,Mzcg]=Resultant_Force(par,Veh.delta,Veh.Fxw,Veh.Fyw,Veh.ls,Veh.lw,Veh.Fzc,Veh.Fxca,Veh.rot_matrix);
%% Euler's Equation of Motion
[Veh.axo,Veh.ayo,Veh.azo,Veh.arxc,Veh.aryc,Veh.arzc]=Euler_Equation(par,Fxcg,Fycg,Fzcg,Mxcg,Mycg,Mzcg,Veh.vrxc,Veh.vryc,Veh.vrzc,Veh.rot_matrix);
%% Velocity
[Veh.vxo,Veh.vyo,Veh.vzo,Veh.vxc,Veh.vyc,Veh.vzc,Veh.vrxc,Veh.vryc,Veh.vrzc]=Velocity(par,Veh.axo,Veh.ayo,Veh.azo,Veh.vxo,Veh.vyo,Veh.vzo,Veh.arxc,Veh.aryc,Veh.arzc,Veh.vrxc,Veh.vryc,Veh.vrzc,Veh.rot_matrix);
%% Tire_Slip
[Veh.alpha,Veh.kappa,Veh.vxw,Veh.vyw,Veh.ryw]=Tire_Slip(par,Veh.delta,Veh.vxc,Veh.vyc,Veh.vrzc,Veh.Tyw,Veh.Fzt,Veh.Fxw,lock);
%% Attitude
[Veh.rot_matrix,Veh.roll,Veh.pitch,Veh.yaw]=Attitude(par,Veh.vrxc,Veh.vryc,Veh.vrzc,Veh.rot_matrix);
%% Position
Veh.xo=Veh.xo+Veh.vxo*par.T;
Veh.yo=Veh.yo+Veh.vyo*par.T;
Veh.zo=Veh.zo+Veh.vzo*par.T;
%% Data Aqu
% data.gravity(i,1:3)=gravity;
data.delta_center(i)=Veh.delta_center;
data.rot_matrix(i,1:9)=[Veh.rot_matrix(1,:),Veh.rot_matrix(2,:),Veh.rot_matrix(3,:)];
data.r(i)=(Veh.hr(1)+Veh.hr(2)+Veh.hr(3)+Veh.hr(4))/4;
data.xo(i)=Veh.xo;
data.yo(i)=Veh.yo;
data.zo(i)=Veh.zo;
data.ayo(i)=Veh.ayo;
data.vrzc(i)=Veh.vrzc;
data.i(i)=i;
data.delta(i,1:2)=Veh.delta;
data.ls(i,1:4)=Veh.ls;
data.hc(i,1:4)=Veh.hc;
data.kappa(i,1:4)=Veh.kappa;
data.alpha(i,1:4)=Veh.alpha;
data.ryw(i,1:4)=Veh.ryw;
data.vxc(i)=Veh.vxc;
data.roll(i)=Veh.roll;
data.pitch(i)=Veh.pitch;
data.yaw(i)=Veh.yaw;
data.Fxw(i,1:4)=Veh.Fxw;
data.ref_delta(i)=ref_delta;
Fxcg; 
data.axc(i)= Fycg/par.mc;
end

data_sim_3d(:,1)=data.i*par.T;
data_sim_3d(:,2:10)=data.rot_matrix;
data_sim_3d(:,11:13)=[data.xo',data.yo',data.zo'+0.762+0.12];
data_sim_3d(:,14:16)=[data.delta(:,1),data.ls(:,1),data.ryw(:,1)];
data_sim_3d(:,17:19)=[data.delta(:,2),data.ls(:,2),data.ryw(:,2)];
data_sim_3d(:,20:21)=[data.ls(:,3),data.ryw(:,3)];
data_sim_3d(:,22:23)=[data.ls(:,4),data.ryw(:,4)];
data_sim_3d(:,24:26)=[data.xo',data.yo',data.r'+1];

Veh.mw=[50 50 50 50]; %[Kg]
Veh.kw=[220000, 220000, 220000, 220000]; %[N/m]
Veh.ks=[20000, 20000, 20000, 20000]; %[N/m]
Veh.ds=[3000, 3000, 3000, 3000]; %[N/m]

s=tf('s');
P=((Veh.kw(1)*Veh.ds(1))/(par.mc/4*Veh.mw(1))*(s+Veh.ks(1)/Veh.ds(1))) / (s^4 + (Veh.ds(1)/Veh.mw(1) + Veh.ds(1)/(par.mc/4))*s^3 + (Veh.ks(1)/Veh.mw(1) + Veh.ks(1)/(par.mc/4) + Veh.kw(1)/Veh.mw(1))*s^2 +(Veh.kw(1)*Veh.ds(1)/(par.mc/4*Veh.mw(1)))*s + Veh.kw(1)*Veh.ks(1)/(par.mc/4*Veh.mw(1))) 
[y,t]=step(P,5);

hold on
plot(0:0.001:4,data.hc(1000:5000,1)+0.1639,'k','linewidth',1)
plot(0:0.001:4,data.hc(1000:5000,2)+0.1639,'r--','linewidth',1)
plot(0:0.001:4,data.hc(1000:5000,3)+0.1639,'k-.','linewidth',1)
plot(0:0.001:4,data.hc(1000:5000,4)+0.1639,'r','linewidth',1)
set(gcf,'position',[0 0 450 300])
grid on

xlabel('Time[sec]')
ylabel('h_c_i [m]')

xlim([0.5 4])
legend(' h_c_1',' h_c_2', ' h_c_3',' h_c_4')

figure
hold on
plot(0:0.001:4,data.roll(1000:5000),'k','linewidth',1)
plot(0:0.001:4,data.pitch(1000:5000),'r--','linewidth',1)
plot(0:0.001:4,data.yaw(1000:5000),'k-.','linewidth',1)
set(gcf,'position',[0 0 450 300])
grid on
xlabel('Time[sec]')
ylabel('Angle[rad]')
xlim([0.5 4])
legend(' roll',' pitch', ' yaw')
