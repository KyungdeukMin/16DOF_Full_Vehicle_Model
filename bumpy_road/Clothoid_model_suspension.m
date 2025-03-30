%% initialization

clear all;
close all;

par.T=0.01; %[sec]
par.g=9.8; %[m/s^2]

%body
par.mc=1410; %[kg]
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
Veh.alpha=[0 0 0 0]; %[rad]
Veh.kappa=[0 0 0 0]; %[rate]
Veh.velocity.target=-5; %[m/s]
Veh.velocity.ini=10; %[m/s]

%states
Veh.rot_matrix=eye(3);
Veh.vxc=0; %[m/s]
Veh.vyc=0; %[m/s]
Veh.vzc=0; %[m/s]

Veh.vrxc=0; %[rad/s]
Veh.vryc=0; %[rad/s]
Veh.vrzc=0; %[rad/s]

Veh.vxo=0; %[m/s]
Veh.vyo=0; %[m/s]
Veh.vzo=0; %[m/s]
Veh.xo=0; %[m]
Veh.yo=0; %[m]
Veh.zo=0; %[m]

Veh.lw=[0 0 0 0]; %[m]

Veh.roll=0;
Veh.pitch=0;
Veh.yaw=0;

Veh.delta=[0 0]; %[rad]

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
Veh.vyw=[0 0 0 0]; %[m/s]



for i=1:1500;
if i>=500
  Veh.delta=[-0.3 -0.3] ;
% Veh.hr(2)=0.1;
% Veh.hr(4)=0.1;
end    
    
%% (1)Force[Vehicle_Coordinate]


data.r(i)=(Veh.hr(1)+Veh.hr(2)+Veh.hr(3)+Veh.hr(4))/4;
%% Suspension Model
[Veh.hw Veh.Fzc Veh.vhw Veh.Fzt Veh.Fzw Veh.ahw Veh.hc Veh.ls Veh.vls]=Suspension_Model(par, Veh.hr, Veh.zo, Veh.rot_matrix);

%% Tire Model
[Veh.Fyw,Veh.Fxw]=Tire_Model(Veh.alpha,Veh.kappa,Veh.Fzt);
% Veh.Fxw
%% Aerodynamic Model
Veh.Fxca=Aerodynamic_Model(Veh.vxc);

%% Generalized Force

Veh.Fxc(1)=cos(Veh.delta(1))*Veh.Fxw(1) - sin(Veh.delta(1))*Veh.Fyw(1);
Veh.Fxc(2)=cos(Veh.delta(2))*Veh.Fxw(2) - sin(Veh.delta(2))*Veh.Fyw(2);
Veh.Fxc(3)=Veh.Fxw(3);
Veh.Fxc(4)=Veh.Fxw(4);

Veh.Fyc(1)=cos(Veh.delta(1))*Veh.Fyw(1) + sin(Veh.delta(1))*Veh.Fxw(1);
Veh.Fyc(2)=cos(Veh.delta(2))*Veh.Fyw(2) + sin(Veh.delta(2))*Veh.Fxw(2);
Veh.Fyc(3)=Veh.Fyw(3);
Veh.Fyc(4)=Veh.Fyw(4);

Veh.Mxc(1)=Veh.Fyc(1)*(par.h0-Veh.ls(1)-Veh.lw(1)) + Veh.Fzc(1)*par.b;
Veh.Mxc(2)=Veh.Fyc(2)*(par.h0-Veh.ls(2)-Veh.lw(2)) - Veh.Fzc(2)*par.b;
Veh.Mxc(3)=Veh.Fyc(3)*(par.h0-Veh.ls(3)-Veh.lw(3)) + Veh.Fzc(3)*par.b;
Veh.Mxc(4)=Veh.Fyc(4)*(par.h0-Veh.ls(4)-Veh.lw(4)) - Veh.Fzc(4)*par.b;

Veh.Myc(1)=-Veh.Fxc(1)*(par.h0-Veh.ls(1)-Veh.lw(1)) - Veh.Fzc(1)*par.af;
Veh.Myc(2)=-Veh.Fxc(2)*(par.h0-Veh.ls(2)-Veh.lw(2)) - Veh.Fzc(2)*par.af;
Veh.Myc(3)=-Veh.Fxc(3)*(par.h0-Veh.ls(3)-Veh.lw(3)) + Veh.Fzc(3)*par.ar;
Veh.Myc(4)=-Veh.Fxc(4)*(par.h0-Veh.ls(4)-Veh.lw(4)) + Veh.Fzc(4)*par.ar;

Veh.Mzc(1)=-Veh.Fxc(1)*par.b + Veh.Fyc(1)*par.af;
Veh.Mzc(2)= Veh.Fxc(2)*par.b + Veh.Fyc(2)*par.af;
Veh.Mzc(3)=-Veh.Fxc(3)*par.b - Veh.Fyc(3)*par.ar;
Veh.Mzc(4)= Veh.Fxc(4)*par.b - Veh.Fyc(4)*par.ar;

%%ac=fc/mc
gravity=par.mc*par.g*Veh.rot_matrix^-1*[0;0;-1];

Fxcg=sum(Veh.Fxc) + gravity(1)- Veh.Fxca;
Fycg=sum(Veh.Fyc) + gravity(2);
Fzcg=sum(Veh.Fzc) + gravity(3);
Mxcg=sum(Veh.Mxc);
Mycg=sum(Veh.Myc);
Mzcg=sum(Veh.Mzc);

data.gravity(i,1:3)=gravity;

% data_Veh.Fyc(1)(i)=Veh.Fyc(1);

data_Fxc(i)=Fxcg;
data_Fyc(i)=Fycg;
data_Fzc(i)=Fzcg;
data_Txc(i)=Mxcg;
data_Tyc(i)=Mycg;
data_Tzc(i)=Mzcg;

%% (2)Acceleration[Vehicle_Coordinate, Absolue_Coordinate]
force_abs=Veh.rot_matrix*[Fxcg; Fycg; Fzcg];
axo= force_abs(1)/par.mc;
ayo= force_abs(2)/par.mc;
azo= force_abs(3)/par.mc;%mb

% Euler's eq. of motion of a rigid body
arxc= (Mxcg+(par.iyc-par.izc)*Veh.vryc*Veh.vrzc)/par.ixc;
aryc= (Mycg+(par.izc-par.ixc)*Veh.vrzc*Veh.vrxc)/par.iyc;
arzc= (Mzcg+(par.ixc-par.iyc)*Veh.vrxc*Veh.vryc)/par.izc;

%% (3)Velocity[Absolute_Coordinate, Vehicle_Coordinate]
% Absolute_Coordinate
Veh.vxo=Veh.vxo+axo*par.T;
Veh.vyo=Veh.vyo+ayo*par.T;
Veh.vzo=Veh.vzo+azo*par.T;

Veh.vrxc=Veh.vrxc+arxc*par.T;
Veh.vryc=Veh.vryc+aryc*par.T;
Veh.vrzc=Veh.vrzc+arzc*par.T;

vel_vec=(Veh.rot_matrix^-1*[Veh.vxo;Veh.vyo;Veh.vzo])';

Veh.vxc=vel_vec(1);
Veh.vyc=vel_vec(2);
Veh.vzc=vel_vec(3);

data.vxc(i)=Veh.vxc;
data.vyc(i)=Veh.vyc;
data.vzc(i)=Veh.vzc;

%velocity contorl
% Veh.Tyw(1)=(Veh.velocity.target-Veh.vxc)*100;
% Veh.Tyw(2)=(Veh.velocity.target-Veh.vxc)*100;
Veh.Tyw(1)=300;
Veh.Tyw(2)=300;

%% (4)Tire_Slip_Angle[Vehicle_Coordinate]
Veh.vxw(1)=Veh.vxc - par.b*Veh.vrzc;
Veh.vyw(1)=Veh.vyc + par.af*Veh.vrzc;
Veh.vxw(2)=Veh.vxc + par.b*Veh.vrzc;
Veh.vyw(2)=Veh.vyc + par.af*Veh.vrzc;
Veh.vxw(3)=Veh.vxc - par.b*Veh.vrzc;
Veh.vyw(3)=Veh.vyc - par.ar*Veh.vrzc;
Veh.vxw(4)=Veh.vxc + par.b*Veh.vrzc;
Veh.vyw(4)=Veh.vyc - par.ar*Veh.vrzc;

Veh.alpha=tire_slip_ang (Veh.vxw,Veh.vyw,Veh.delta);

Veh.aryw=(Veh.Tyw-Veh.Fxw*par.Rw)/par.iyw;
Veh.vryw=Veh.vryw+Veh.aryw*par.T;
Veh.ryw=Veh.ryw+Veh.vryw*par.T;
% Veh.vryw
if Veh.vxw==0
    Veh.kappa=-sign(Veh.vryw);
else
    Veh.kappa=(Veh.vxw-par.Rw*Veh.vryw)./Veh.vxw;
end
for wheel_num=1:4
    if Veh.kappa(wheel_num)>0.5
    Veh.kappa(wheel_num)=0.5;
    elseif Veh.kappa(wheel_num)<-0.5
    Veh.kappa(wheel_num)=-0.5;
    end
end

% 
% Veh.kappa=



% if i>1
% data_w_lf(i)=data_w_lf(i-1)+vxw.lf/par.Rw*par.T;
% data_w_rf(i)=data_w_rf(i-1)+vxw.rf/par.Rw*par.T;
% data_w_lr(i)=data_w_lr(i-1)+vxw.lr/par.Rw*par.T;
% data_w_rr(i)=data_w_rr(i-1)+vxw.rr/par.Rw*par.T;
% else
% data_w_lf(i)=vxw.lf/par.Rw*par.T;
% data_w_rf(i)=vxw.rf/par.Rw*par.T;
% data_w_lr(i)=vxw.lr/par.Rw*par.T;
% data_w_rr(i)=vxw.rr/par.Rw*par.T;
% end

%% (5)Attitude
rot_instant=[1,-Veh.vrzc*par.T,Veh.vryc*par.T;Veh.vrzc*par.T,1,-Veh.vrxc*par.T;-Veh.vryc*par.T,Veh.vrxc*par.T,1];

Veh.rot_matrix=Veh.rot_matrix*rot_instant;
data.rot_matrix(i,1:9)=[Veh.rot_matrix(1,:),Veh.rot_matrix(2,:),Veh.rot_matrix(3,:)];

Veh.pitch=atan2(-Veh.rot_matrix(3,1),(Veh.rot_matrix(1,1)^2+Veh.rot_matrix(2,1)^2)^(1/2));
Veh.yaw=atan2(Veh.rot_matrix(2,1)/cos(Veh.pitch), Veh.rot_matrix(1,1)/cos(Veh.pitch));
Veh.roll=atan2(Veh.rot_matrix(3,2)/cos(Veh.pitch),Veh.rot_matrix(3,3)/cos(Veh.pitch));




%% Position[Absolute_Coordinate]
Veh.xo=Veh.xo+Veh.vxo*par.T;
Veh.yo=Veh.yo+Veh.vyo*par.T;
Veh.zo=Veh.zo+Veh.vzo*par.T;

data.xo(i)=Veh.xo;
data.yo(i)=Veh.yo;
data.zo(i)=Veh.zo;

data.i(i)=i;

data.delta(i,1:2)=Veh.delta;
% data.delta(i,2)=Veh.delta(2);

%% Data Aqu
data.ls(i,1:4)=Veh.ls;
data.hc(i,1:4)=Veh.hc;
data.kappa(i,1:4)=Veh.kappa;
data.ryw(i,1:4)=Veh.ryw;
data.vxc(i)=Veh.vxc;

data.roll(i)=Veh.roll;
data.pitch(i)=Veh.pitch;
data.yaw(i)=Veh.yaw;
end


data_sim_3d(:,1)=data.i*par.T;
data_sim_3d(:,2:10)=data.rot_matrix;
data_sim_3d(:,11:13)=[data.xo',data.yo',data.zo'+0.762+0.12];
data_sim_3d(:,14:16)=[data.delta(:,1),data.ls(:,1),data.ryw(:,1)];
data_sim_3d(:,17:19)=[data.delta(:,2),data.ls(:,2),data.ryw(:,2)];
data_sim_3d(:,20:21)=[data.ls(:,3),data.ryw(:,3)];
data_sim_3d(:,22:23)=[data.ls(:,4),data.ryw(:,4)];
data_sim_3d(:,24:26)=[data.xo',data.yo',data.r'*0];


plot(data.kappa(:,1))
figure
plot(data.vxc(:))
figure
plot(data.pitch(:))
asdf
% s=tf('s');
% P=((Veh.kw(1)*Veh.ds(1))/(par.mc/4*Veh.mw(1))*(s+Veh.ks(1)/Veh.ds(1))) / (s^4 + (Veh.ds(1)/Veh.mw(1) + Veh.ds(1)/(par.mc/4))*s^3 + (Veh.ks(1)/Veh.mw(1) + Veh.ks(1)/(par.mc/4) + Veh.kw(1)/Veh.mw(1))*s^2 +(Veh.kw(1)*Veh.ds(1)/(par.mc/4*Veh.mw(1)))*s + Veh.kw(1)*Veh.ks(1)/(par.mc/4*Veh.mw(1)))
% 
% [y,t]=step(P,2);


figure(1)
% plot(t+1,y/10,'r','linewidth',2)
% figure(2)
hold on
% h_sus=data_hw_lf+data_ls_lf;
plot(0:0.001:4,data.hc(4000:8000,1)+0.1884,'r-','linewidth',2)
plot(0:0.001:4,data.hc(4000:8000,2)+0.1884,'--','linewidth',2)
plot(0:0.001:4,data.hc(4000:8000,3)+0.1884,'b-.','linewidth',2)
plot(0:0.001:4,data.hc(4000:8000,4)+0.1884,'m-')


set(gcf,'outerposition',[0 0 560 450])
grid on

xlabel('Time[sec]')
ylabel('h_c_i [m]')

xlim([0.5 4])

legend(' h_c_1 FVM',' h_c_2 FVM', ' h_c_3 FVM',' h_c_4 FVM')
