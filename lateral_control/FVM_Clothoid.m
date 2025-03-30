clear all;
close all;

for sim_count=1:4

%% Vehicle Parameters
[Veh, par]=Vehicle_Parameters;
Simulation_Time=25;
lock=0;
load pathu2
r_path=pathu2;

for i=1:Simulation_Time/par.T;
%% Lateral Deviation (ycg)
ahead_dist=0;
sp=[Veh.xo+ahead_dist*cos(Veh.yaw), Veh.yo+ahead_dist*sin(Veh.yaw)];
[yr1,sn1]=min(((r_path(:,2)-sp(1)).*(r_path(:,2)-sp(1))) + ((r_path(:,3)-sp(2)).*(r_path(:,3)-sp(2))));
vec_unit1=[cos(r_path(sn1,4)) -sin(r_path(sn1,4));sin(r_path(sn1,4)) cos(r_path(sn1,4))]*[0;1];
vec_r_path_s1=[sp(1)-r_path(sn1,2), sp(2)-r_path(sn1,3)];
ycg=vec_r_path_s1*vec_unit1;

%% lookahead deviation (ya)
ahead_dist=par.af+5;
sp=[Veh.xo+ahead_dist*cos(Veh.yaw), Veh.yo+ahead_dist*sin(Veh.yaw)];
[yr2,sn2]=min(((r_path(:,2)-sp(1)).*(r_path(:,2)-sp(1))) + ((r_path(:,3)-sp(2)).*(r_path(:,3)-sp(2))));
vec_unit2=[cos(r_path(sn2,4)) -sin(r_path(sn2,4));sin(r_path(sn2,4)) cos(r_path(sn2,4))]*[0;1];
vec_r_path_s2=[sp(1)-r_path(sn2,2), sp(2)-r_path(sn2,3)];
ya=vec_r_path_s2*vec_unit2;

%% Guidance Control
switch sim_count
    case 1
        kp = 0.05;
    case 2
        kp = 0.1;
    case 3
        kp = 0.2;
    case 4
        kp = 0.3;
end

ref_delta=-kp*ya;
last_delta=Veh.delta_center;
%% Automatic Steering Model
[Veh.delta_center]=Automatic_Steering_Model(par,[1/0.00777],[1, 0.1515/0.00777, 1/0.00777],0.0284,ref_delta);%(par,[1/0.01248],[1, 0.1799/0.01248, 1/0.01248],0,ref_delta);
%% Torque Control
vx=Acc_Filterling(par,[1],[1],1,Veh.vxc);
acc=Veh.vxc-vx;
k21 = 2.875606630400000e+02;
k22 = 3.449612515435763;
k23 =-0.394030880000000;
K2=[k21 k22 k23];
ref_Tq=-K2*[Veh.vxc-5;0;0];
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

switch sim_count
    case 1
     data_ycg1(i)=ycg;
     data.xo1(i)=Veh.xo;
     data.yo1(i)=Veh.yo;
    case 2
     data_ycg2(i)=ycg;
     data.xo2(i)=Veh.xo;
     data.yo2(i)=Veh.yo;
    case 3
     data_ycg3(i)=ycg;
     data.xo3(i)=Veh.xo;
     data.yo3(i)=Veh.yo;
    case 4
     data_ycg4(i)=ycg;
     data.xo4(i)=Veh.xo;
     data.yo4(i)=Veh.yo;
end


end

end %simcount

figure
plot(data.i*par.T,data_ycg1,'b-.','linewidth',1)
hold on
plot(data.i*par.T,data_ycg2,'b--','linewidth',1)
plot(data.i*par.T,data_ycg3,'r','linewidth',1)

xlabel('Time[sec]')
ylabel('y_c_g[m]')
xlim([0 25])
set(gcf,'position',[0 0 450 300])
legend('  kp1=0.05','  kp1=0.1', '  kp1=0.2')
grid on


figure
plot(r_path(:,2),r_path(:,3),'k','linewidth',1.5);
hold on
plot(data.xo1,data.yo1,'b-.','linewidth',1);
plot(data.xo2,data.yo2,'b--','linewidth',1);
plot(data.xo3,data.yo3,'r','linewidth',1);

xlabel('Xo[m]')
ylabel('Yo[m]')

grid on
set(gcf,'position',[0 0 450 300])
legend('  Path','  kp1=0.05','  kp1=0.1', '  kp1=0.2')
axis equal

xlim([-10 40])
ylim([-2 33])
set(gcf,'position',[0 0 450 300])
