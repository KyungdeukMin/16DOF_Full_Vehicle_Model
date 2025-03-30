%% initialization

clear all;
close all;


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
Tire.Muy0=1.192842481344; %n_py0
Tire.Muy1=-0.04960551335; %n_1y
Tire.Muy2=-0.14093687309; %n_2y
% minimum load for valid peak friction values
Tire.Fz0=4.607008849e2; %F_z0
% longitudinal sliding coefficient of friction
Tire.KMUx3=0.095162164771; %d_3
Tire.KMUx2=2.186713890e-4; %d_2
Tire.KMUx1=-1999969731e-9; %d_1
Tire.Lsx=1.05; %e_sx
% lateral sliding coefficient of friction
Tire.KMUy3=0.327952438694; %d_3
Tire.KMUy2=1.798822501e-4; %d_2
Tire.KMUy1=-9.131892344e-8; %d_1
Tire.Lsy=1.10; %e_sy
% plysteer
Tire.PLYSTEER=3.714581147e-4; %b

% Salaani_Model(Tire,ALPHA, S, GAMMA, FZ)

for j=0

mc=1410; %[kg]
mw_f=50; %[Kg]
mw_r=50; %[Kg]
mb=mc-mw_f*2-mw_r*2;
g=9.8; %[m/s^2]
rot_matrix=eye(3);
% rot_matrix=angle2rm(0, 0.2, 0) %(roll, pitch, yaw)
wb=2.640;
af=2.640/2; %[m]
ar=2.640-af; %[m]
b=1.586/2; %[m]

T=0.001;
delta_lf=0;
delta_rf=0;
Tw_lf=0; %[Nm]
Tw_rf=0; %[Nm]
Tw_lr=0; %[Nm]
Tw_rr=0; %[Nm]

Rw=0.7018/2; %[m]
% Ad=1;
% cs_lf=10000;
% cs_rf=10000;
cs_lr=10000;
cs_rr=10000;
alpha_lf=0;
alpha_rf=0;
alpha_lr=0;
alpha_rr=0;

kw_f=220000; %[N/m]
kw_r=220000; %[N/m]
ks_lf=20000; %[N/m]
ks_rf=20000; %[N/m]
ks_lr=20000; %[N/m]
ks_rr=20000; %[N/m]
ds_lf=3000; %[Ns/m]
ds_rf=3000; %[Ns/m]
ds_lr=3000; %[Ns/m]
ds_rr=3000; %[Ns/m]


hc=0.762; %[m]

ixc=711.0424; %[kgm^2]
iyc=2.6070e+03; %[kgm^2]
izc=2.6744e+03; %[kgm^2]

% if j==0
%   target_vel=10; %[m/s] 
% elseif j==1
%   target_vel=20; %[m/s]   
% elseif j==2
%   target_vel=20; %[m/s]   
% end
target_vel=10; %[m/s]
vxc=target_vel; %[m/s]
vyc=0; %[m/s]
vzc=0; %[m/s]
vrxc=0; %[rad/s]
vryc=0; %[rad/s]
vrzc=0; %[rad/s]

vxo=target_vel;
vyo=0;
vzo=0;

xo=0; %[m]
yo=0; %[m]
zo=0; %[m]

lw_lf=0; %[m]
lw_rf=0; %[m]
lw_lr=0; %[m]
lw_rr=0; %[m]

hwo_lf=0; %[m]
hwo_rf=0; %[m]
hwo_lr=0; %[m]
hwo_rr=0; %[m]

Fzc_lf=0; %[N]
Fzc_rf=0; %[N]
Fzc_lr=0; %[N]
Fzc_rr=0; %[N]

vhwo_lf=0; %[m/s]
vhwo_rf=0; %[m/s]
vhwo_lr=0; %[m/s]
vhwo_rr=0; %[m/s]

r_lf=0; %[m]
r_rf=0; %[m]
r_lr=0; %[m]
r_rr=0; %[m]

last_ls_lf=-zo - rot_matrix(3,:)*[af;b;0];
last_ls_rf=-zo - rot_matrix(3,:)*[af;-b;0];
last_ls_lr=-zo - rot_matrix(3,:)*[-af;b;0];
last_ls_rr=-zo - rot_matrix(3,:)*[-af;-b;0];

roll=0;
pitch=0;
yaw=0;
tau=0.2;

load path30
r_path=path30;

delta=0;
target_delta=0;

k1=1;
k2=1;
tau=0.2;



for i=1:9000;
    
%% steering
ahead_dist=tau*vxc*(1+j*2);
sp=[xo+ahead_dist*cos(yaw), yo+ahead_dist*sin(yaw)];

[yr1,sn1]=min(((r_path(:,2)-sp(1)).*(r_path(:,2)-sp(1))) + ((r_path(:,3)-sp(2)).*(r_path(:,3)-sp(2))));


% output yr1
vec_unit1=[cos(r_path(sn1,4)) -sin(r_path(sn1,4));sin(r_path(sn1,4)) cos(r_path(sn1,4))]*[0;1];

vec_r_path_s1=[sp(1)-r_path(sn1,2) sp(2)-r_path(sn1,3)];
yr1=vec_r_path_s1*vec_unit1;
 
data_yr1(i)=yr1;  

delta=0.995*delta+0.004988*target_delta;
delta_lf=delta;
delta_rf=delta;

delta_lf=0;
delta_rf=0;
if i>=5000
sec=i/1000;
delta_lf=0.1*sin(2*pi*sec);
delta_rf=0.1*sin(2*pi*sec);
end

kin_r=2.640/tan(delta_lf);
kin_yawrate=vxc/kin_r;
data_kin_yawrate(i)=kin_yawrate;
kin_lateral_accel=vxc^2 /kin_r;
data_kin_lateral_accel(i)=kin_lateral_accel;

target_yaw=r_path(sn1,4)-yr1*k2;
yaw_err=target_yaw-yaw;
target_yawrate=yaw_err*k1;
target_delta=target_yawrate*wb/vxc;

% target_delta=1;

data_delta(i)=delta;


    
%% (1)Force[Vehicle_Coordinate]
%Vertical tire forces & location update
if i>=5000
% r_lf=0.1;
% r_rf=0.1;
% r_lr=0.1;
% r_rr=0.1;

%delta_lf=0.2;
%delta_rf=0.2;    
end

data_r(i)=(r_lf+r_rf+r_lr+r_rr)/4;

lw_lf=r_lf-hwo_lf;
lw_rf=r_rf-hwo_rf;
lw_lr=r_lr-hwo_lr;
lw_rr=r_rr-hwo_rr;

data_lw_lf(i)=lw_lf;
data_lw_rf(i)=lw_rf;
data_lw_lr(i)=lw_lr;
data_lw_rr(i)=lw_rr;

if lw_lf<0
    lw_lf=0;
end
if lw_rf<0
    lw_rf=0;
end
if lw_lr<0
    lw_lr=0;
end
if lw_rr<0
    lw_rr=0;
end

Fzw_lf=lw_lf*kw_f;
Fzw_rf=lw_rf*kw_f;
Fzw_lr=lw_lr*kw_r;
Fzw_rr=lw_rr*kw_r;

Fw_lf=Fzw_lf - Fzc_lf;
Fw_rf=Fzw_rf - Fzc_rf;
Fw_lr=Fzw_lr - Fzc_lr;
Fw_rr=Fzw_rr - Fzc_rr;

Fzw_lf=Fzw_lf/0.453592/9.8;%N to lbs
Fzw_rf=Fzw_rf/0.453592/9.8;%N to lbs
Fzw_lr=Fzw_lr/0.453592/9.8;%N to lbs
Fzw_rr=Fzw_rr/0.453592/9.8;%N to lbs

if Fzw_lf<1
    Fzw_lf=1; 
end
if Fzw_rf<1
    Fzw_rf=1;
end
if Fzw_lr<1
    Fzw_lr=1;
end
if Fzw_rr<1
    Fzw_rr=1;
end

ahwo_lf= Fw_lf/mw_f;
ahwo_rf= Fw_rf/mw_f;
ahwo_lr= Fw_lr/mw_r;
ahwo_rr= Fw_rr/mw_r;

vhwo_lf=vhwo_lf+ahwo_lf*T;
vhwo_rf=vhwo_rf+ahwo_rf*T;
vhwo_lr=vhwo_lr+ahwo_lr*T;
vhwo_rr=vhwo_rr+ahwo_rr*T;

hwo_lf=hwo_lf+vhwo_lf*T;
hwo_rf=hwo_rf+vhwo_rf*T;
hwo_lr=hwo_lr+vhwo_lr*T;
hwo_rr=hwo_rr+vhwo_rr*T;

data_hwo_lf(i)=hwo_lf;
data_hwo_rf(i)=hwo_rf;
data_hwo_lr(i)=hwo_lr;
data_hwo_rr(i)=hwo_rr;

hb_lf=zo + rot_matrix(3,:)*[af;b;0];
hb_rf=zo + rot_matrix(3,:)*[af;-b;0];
hb_lr=zo + rot_matrix(3,:)*[-ar;b;0];
hb_rr=zo + rot_matrix(3,:)*[-ar;-b;0];

data_hb_lf(i)=hb_lf;
data_hb_rf(i)=hb_rf;
data_hb_lr(i)=hb_lr;
data_hb_rr(i)=hb_rr;

%Vertical shock absorber forces
ls_lf=hwo_lf-hb_lf;
ls_rf=hwo_rf-hb_rf;
ls_lr=hwo_lr-hb_lr;
ls_rr=hwo_rr-hb_rr;

data_ls_lf(i)=ls_lf;
data_ls_rf(i)=ls_rf;
data_ls_lr(i)=ls_lr;
data_ls_rr(i)=ls_rr;

vls_lf=(ls_lf-last_ls_lf)/T;
vls_rf=(ls_rf-last_ls_rf)/T;
vls_lr=(ls_lr-last_ls_lr)/T;
vls_rr=(ls_rr-last_ls_rr)/T;

last_ls_lf=ls_lf;
last_ls_rf=ls_rf;
last_ls_lr=ls_lr;
last_ls_rr=ls_rr;

%tire forces
[Fyw_lf,Fxw_lf,Mz,Mx]=Salaani_Model(Tire,alpha_lf, 0, 0, Fzw_lf);
[Fyw_rf,Fxw_rf,Mz,Mx]=Salaani_Model(Tire,alpha_rf, 0, 0, Fzw_rf);
[Fyw_lr,Fxw_lr,Mz,Mx]=Salaani_Model(Tire,alpha_lr, 0, 0, Fzw_lr);
[Fyw_rr,Fxw_rr,Mz,Mx]=Salaani_Model(Tire,alpha_rr, 0, 0, Fzw_rr);
Fyw_lf=Fyw_lf*0.453592*9.8;%lbs to N
Fyw_rf=Fyw_rf*0.453592*9.8;%lbs to N
Fyw_lr=Fyw_lr*0.453592*9.8;%lbs to N
Fyw_rr=Fyw_rr*0.453592*9.8;%lbs to N
Fxw_lf=Fxw_lf*0.453592*9.8;%lbs to N
Fxw_rf=Fxw_rf*0.453592*9.8;%lbs to N
Fxw_lr=Fxw_lr*0.453592*9.8;%lbs to N
Fxw_rr=Fxw_rr*0.453592*9.8;%lbs to N


Fxc_lf=cos(delta_lf)*Tw_lf/Rw - sin(delta_lf)*Fyw_lf;
Fxc_rf=cos(delta_rf)*Tw_rf/Rw - sin(delta_rf)*Fyw_rf;
Fxc_lr=Tw_lr/Rw;
Fxc_rr=Tw_rr/Rw;

Fyc_lf=cos(delta_lf)*Fyw_lf + sin(delta_lf)*Tw_lf/Rw;
Fyc_rf=cos(delta_rf)*Fyw_rf + sin(delta_rf)*Tw_rf/Rw;
Fyc_lr=Fyw_lr;
Fyc_rr=Fyw_rr;

Fzc_lf=ls_lf*ks_lf + vls_lf*ds_lf;
Fzc_rf=ls_rf*ks_rf + vls_rf*ds_rf;
Fzc_lr=ls_lr*ks_lr + vls_lr*ds_lr;
Fzc_rr=ls_rr*ks_rr + vls_rr*ds_rr;

Txc_lf=Fyc_lf*(hc-ls_lf-lw_lf) + Fzc_lf*b;
Txc_rf=Fyc_rf*(hc-ls_rf-lw_rf) - Fzc_rf*b;
Txc_lr=Fyc_lr*(hc-ls_lr-lw_lr) + Fzc_lr*b;
Txc_rr=Fyc_rr*(hc-ls_rr-lw_rr) - Fzc_rr*b;

Tyc_lf=-Fxc_lf*(hc-ls_lf-lw_lf) - Fzc_lf*af;
Tyc_rf=-Fxc_rf*(hc-ls_rf-lw_rf) - Fzc_rf*af;
Tyc_lr=-Fxc_lr*(hc-ls_lr-lw_lr) + Fzc_lr*ar;
Tyc_rr=-Fxc_rr*(hc-ls_rr-lw_rr) + Fzc_rr*ar;

Tzc_lf=-Fxc_lf*b + Fyc_lf*af;
Tzc_rf= Fxc_rf*b + Fyc_rf*af;
Tzc_lr=-Fxc_lr*b - Fyc_lr*ar;
Tzc_rr= Fxc_rr*b - Fyc_rr*ar;

%%ac=fc/mc
weight=mc*g*rot_matrix^-1*[0;0;-1];

Fxc=Fxc_lf+Fxc_rf+Fxc_lr+Fxc_rr + weight(1);
Fyc=Fyc_lf+Fyc_rf+Fyc_lr+Fyc_rr + weight(2);
Fzc=Fzc_lf+Fzc_rf+Fzc_lr+Fzc_rr + weight(3);
Txc=Txc_lf+Txc_rf+Txc_lr+Txc_rr ;
Tyc=Tyc_lf+Tyc_rf+Tyc_lr+Tyc_rr ;
Tzc=Tzc_lf+Tzc_rf+Tzc_lr+Tzc_rr ;

data_weight(i,1:3)=weight;

data_Fyc_lf(i)=Fyc_lf;

data_Fxc(i)=Fxc;
data_Fyc(i)=Fyc;
data_Fzc(i)=Fzc;
data_Txc(i)=Txc;
data_Tyc(i)=Tyc;
data_Tzc(i)=Tzc;

%% (2)Acceleration[Vehicle_Coordinate, Absolue_Coordinate]
force_abs=rot_matrix*[Fxc; Fyc; Fzc];
axo= force_abs(1)/mc;
ayo= force_abs(2)/mc;
azo= force_abs(3)/mb;

ayc=Fyc/mc;

data_ayc(i)=ayc;
% arxc= Txc/ixc;
% aryc= Tyc/iyc;
% arzc= Tzc/izc;
% Euler's eq. of motion of a rigid body
arxc= (Txc+(iyc-izc)*vryc*vrzc)/ixc;
aryc= (Tyc+(izc-ixc)*vrzc*vrxc)/iyc;
arzc= (Tzc+(ixc-iyc)*vrxc*vryc)/izc;

%Rotate to Absolute_Coordinate
% temp_accel = rot_matrix*[axc; ayc; azc];
% axo=temp_accel(1);
% ayo=temp_accel(2);
% azo=temp_accel(3);

%% (3)Velocity[Absolute_Coordinate, Vehicle_Coordinate]
% Absolute_Coordinate
vxo=vxo+axo*T;
vyo=vyo+ayo*T;
vzo=vzo+azo*T;

vrxc=vrxc+arxc*T;
vryc=vryc+aryc*T;
vrzc=vrzc+arzc*T;

% rot_instant=angle2rm(vrxc*T,vryc*T,vrzc*T);
% vel_vec=(rot_instant^-1*[vxc;vyc;vzc])';
vel_vec=(rot_matrix^-1*[vxo;vyo;vzo])';


vxc=vel_vec(1);
vyc=vel_vec(2);
vzc=vel_vec(3);

% data_vxc(i)=vxc;
% data_vyc(i)=vyc;
% data_vzc(i)=vzc;

data_vrxc(i)=vrxc;
data_vryc(i)=vryc;
data_vrzc(i)=vrzc;

% %velocity contorl
Tw_lf=(target_vel-vxc)*100;
Tw_rf=(target_vel-vxc)*100;

%% (4)Tire_Slip_Angle[Vehicle_Coordinate]
vxw_lf=vxc - b*vrzc;
vyw_lf=vyc + af*vrzc;

vxw_rf=vxc + b*vrzc;
vyw_rf=vyc + af*vrzc;

vxw_lr=vxc - b*vrzc;
vyw_lr=vyc - ar*vrzc;

vxw_rr=vxc + b*vrzc;
vyw_rr=vyc - ar*vrzc;

[alpha_lf,alpha_rf,alpha_lr,alpha_rr]=tire_slip_ang (vxw_lf,vyw_lf,vxw_rf,vyw_rf,vxw_lr,vyw_lr,vxw_rr,vyw_rr,delta_lf,delta_rf);

if i>1
data_w_lf(i)=data_w_lf(i-1)+vxw_lf/Rw*T;
data_w_rf(i)=data_w_rf(i-1)+vxw_rf/Rw*T;
data_w_lr(i)=data_w_lr(i-1)+vxw_lr/Rw*T;
data_w_rr(i)=data_w_rr(i-1)+vxw_rr/Rw*T;
else
data_w_lf(i)=vxw_lf/Rw*T;
data_w_rf(i)=vxw_rf/Rw*T;
data_w_lr(i)=vxw_lr/Rw*T;
data_w_rr(i)=vxw_rr/Rw*T;
end



% alpha_lf=0.01;
% alpha_rf=0.01;
% alpha_lr=0.01;
% alpha_rr=0.01;
%% (5)Attitude
% rot_instant=angle2rm(vrxc*T,vryc*T,vrzc*T);
rot_instant=[1,-vrzc*T,vryc*T;vrzc*T,1,-vrxc*T;-vryc*T,vrxc*T,1];

rot_matrix=rot_matrix*rot_instant;
data_rot_matrix(i,1:9)=[rot_matrix(1,:),rot_matrix(2,:),rot_matrix(3,:)];

pitch=atan2(-rot_matrix(3,1),(rot_matrix(1,1)^2+rot_matrix(2,1)^2)^(1/2));
yaw=atan2(rot_matrix(2,1)/cos(pitch), rot_matrix(1,1)/cos(pitch));
roll=atan2(rot_matrix(3,2)/cos(pitch),rot_matrix(3,3)/cos(pitch));

data_roll(i)=roll;
data_pitch(i)=pitch;
data_yaw(i)=yaw;

%% Velocity[Absolute_Coordinate]
% temp_vel=(rot_matrix*[vxc,vyc,vzc]')';
% vxo=temp_vel(1);
% vyo=temp_vel(2);
% vzo=temp_vel(3);

%% Position[Absolute_Coordinate]
xo=xo+vxo*T;
yo=yo+vyo*T;
zo=zo+vzo*T;

data_xo(i)=xo;
data_yo(i)=yo;
data_zo(i)=zo;

data_i(i)=i;

data_delta_lf(i)=delta_lf;
data_delta_rf(i)=delta_rf;
end

switch j
    case 0
        hold on
%         plot(data_xo,data_yo,':r')
    case 1
%         plot(data_xo,data_yo,'r')
    case 2
%         plot(data_xo,data_yo,'--r')
% plot(r_path(:,2),r_path(:,3),'b')
end
% legend('k3=1','k3=3','k3=5')
% xlabel('x[m]')
% ylabel('y[m]')
%  axis equal
% xlim([10,80])
% ylim([-5,20])

end


data_sim_3d(:,1)=data_i*T;
data_sim_3d(:,2:10)=data_rot_matrix;
data_sim_3d(:,11:13)=[data_xo',data_yo',data_zo'+0.762+0.12];
data_sim_3d(:,14:16)=[data_delta_lf',data_ls_lf',data_w_lf'];
data_sim_3d(:,17:19)=[data_delta_rf',data_ls_rf',data_w_rf'];
data_sim_3d(:,20:21)=[data_ls_lr',data_w_lr'];
data_sim_3d(:,22:23)=[data_ls_rr',data_w_rr'];
data_sim_3d(:,24:26)=[data_xo',data_yo',data_r'*0];

s=tf('s');
P=((kw_f*ds_lf)/(mb/4*mw_f)*(s+ks_lf/ds_lf)) / (s^4 + (ds_lf/mw_f + ds_lf/(mb/4))*s^3 + (ks_lf/mw_f + ks_lf/(mb/4) + kw_f/mw_f)*s^2 +(kw_f*ds_lf/(mb/4*mw_f))*s + kw_f*ks_lf/(mb/4*mw_f))

[y,t]=step(P,2);
figure(1)
% plot(t+1,y/10,'r','linewidth',2)

subplot(2,1,1),plot(0:0.001:4,data_kin_yawrate(4000:8000),'r','linewidth',2)
% plot(t+1,y/10,'r','linewidth',2)
% figure(2)
hold on
% title('Yawrate')

subplot(2,1,1),plot(0:0.001:4,data_vrzc(4000:8000),'b--','linewidth',2)

legend(' KM',' FVM')
xlim([0.5 4])
ylim([-1 1])
% set(gcf,'outerposition',[0 0 560 450])
grid on

xlabel('Time[sec]')
ylabel('Yawrate [rad/s]')


% data_kin_lateral_accel
subplot(2,1,2),plot(0:0.001:4,data_kin_lateral_accel(4000:8000),'r','linewidth',2)
hold on
subplot(2,1,2),plot(0:0.001:4,data_ayc(4000:8000),'b--','linewidth',2)

legend(' KM',' FVM')
xlim([0.5 4])
ylim([-16 16])
% set(gcf,'outerposition',[0 0 560 450])
grid on

xlabel('Time[sec]')
ylabel('Lateral Acceleration [m/s^2]')

set(gcf,'outerposition',[0 0 560 450])



% legend(' Lateral Accelerration')

% plot(x(1:2:101),Data_Fy1(1:2:101)*0.453592*9.8,'r','linewidth',2)
% hold on
% plot(x(1:2:101),Data_Fy2(1:2:101)*0.453592*9.8,'g-','linewidth',2)
% plot(x(1:2:101),Data_Fy3(1:2:101)*0.453592*9.8,'--','linewidth',2)
% plot(x(1:2:101),Data_Fy4(1:2:101)*0.453592*9.8,'c-.','linewidth',2)
% plot(x(1:2:101),Data_Fy5(1:2:101)*0.453592*9.8,'m.-')
% set(gcf,'outerposition',[0 0 560 450])
% grid on
% 
% xlabel('Slip Angle[rad]')
% ylabel('Lateral Force[N]')
% 
% legend(' 1044.16 kg','  835.51 kg','  626.41 kg', '  417.75 kg','  209.01 kg')


%  figure(3)
% plot(data_ls_lf)
% figure(4)
% plot(data_hwo_lf)
% 
% figure(5)
% plot(data_lw_lf)
% 
% figure(6)
% plot(data_hwo_lf+data_ls_lf)
% 
% figure(7)
% plot(data_xo,data_yo)


% s=tf('s');
% P=((kw_f*ds_lf)/(mc/4*mw_f)*(s+ks_lf/ds_lf)) / (s^4 + (ds_lf/mw_f + ds_lf/(mc/4))*s^3 + (ks_lf/mw_f + ks_lf/(mc/4) + kw_f/mw_f)*s^2 +(kw_f*ds_lf/(mc/4*mw_f))*s + kw_f*ks_lf/(mc/4*mw_f))
% 
% step(P)
% figure
% plot(data_zo)
%  figure
% plot(data_hs_lf)
% figure
% plot(data_hwo_lf)
% 
% figure
% plot(data_vxc)
% % 
% % figure
% % plot(data_xo,data_yo)
% figure('Name','yr1')
% plot(data_yr1)
% 
% figure('Name','delta')
% plot(data_delta)
% 
% figure('Name','xoyo')
% plot(data_xo,data_yo,'r')
% hold on
% plot(r_path(:,2),r_path(:,3))
% axis equal