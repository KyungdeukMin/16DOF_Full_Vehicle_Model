clear all;
close all;

Gateway = load('1 Sinusoid Steering input\\30km 60A 3w steering\\Gateway_0504_1706.txt');

i1 = 2150:2950;

index1               = Gateway(i1,1);       time = (index1-2150) * 0.01;
Steering_Angle      = Gateway(i1,6);
Reference_input     = Gateway(i1,11);
Yaw_Rate            = Gateway(i1,12);
Gway_LAT_ACCEL      = Gateway(i1,16);
Gway_LONG_ACCEL     = Gateway(i1,17);
Gway_Vehicle_Speed  = Gateway(i1,18);


%% Vehicle Parameters
[Veh, par]=Vehicle_Parameters;
Simulation_Time=8;
lock=0;
r_path=[0,0,0,0];

for i=1:Simulation_Time/par.T;

%% Guidance Control
ref_delta=0;
temp_delta= Reference_input(ceil(i/10));%0.057140040000000*sin(3*i*par.T+1.537);%2*pi*0.2*i
ref_delta= temp_delta*10*abs(temp_delta*10)*3.039e-09 + temp_delta*10*9.341e-05;
%% Automatic Steering Model
[Veh.delta_center]=Automatic_Steering_Model(par,[1/0.00777],[1, 0.1515/0.00777, 1/0.00777],0.0284,ref_delta);%(par,[1/0.01248],[1, 0.1799/0.01248, 1/0.01248],0,ref_delta);
% Veh.delta_center=ref_delta;
%% Torque Control
ref_Tq=90;
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
data.ayc(i)= Fycg/par.mc;
end


data_sim_3d(:,1)=data.i*par.T;
data_sim_3d(:,2:10)=data.rot_matrix;
data_sim_3d(:,11:13)=[data.xo',data.yo',data.zo'+0.762+0.12];
data_sim_3d(:,14:16)=[data.delta(:,1),data.ls(:,1),data.ryw(:,1)];
data_sim_3d(:,17:19)=[data.delta(:,2),data.ls(:,2),data.ryw(:,2)];
data_sim_3d(:,20:21)=[data.ls(:,3),data.ryw(:,3)];
data_sim_3d(:,22:23)=[data.ls(:,4),data.ryw(:,4)];
data_sim_3d(:,24:26)=[data.xo',data.yo',data.r'+1];

figure

plot(data.i*par.T,data.vrzc(:),'k','linewidth',1)
hold on
plot(time(:),Yaw_Rate(:)/180*pi+0.010,'r--','linewidth',1)
ylabel('Yawrate[rad/s]')
xlabel('Time[sec]')
legend('FVM Simulation','Experiment')
ylim([-0.2 0.3])
set(gcf,'position',[0 0 450 300])
grid on

figure

plot(data.i*par.T,data.ayc(:),'k','linewidth',1)
hold on
plot(time(:),Gway_LAT_ACCEL(:)+0.16,'r--','linewidth',1)
ylabel('Lateral Acceleration[m/s^2]')
xlabel('Time[sec]')
legend('FVM Simulation','Experiment')
ylim([-2 3])
set(gcf,'position',[0 0 450 300])
grid on

