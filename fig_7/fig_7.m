
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

alpha=-0.5;
Data_Fy=0;
Fz=2302;
for j=1:5

for i=1:101
[Fy,Fx,Mz,Mx]=Salaani_Model(Tire,alpha, 0, 0, Fz);
Data_Fy(i)=Fy;
alpha=alpha+0.01;
end

    switch j
        case 1
            Fz=1842;
            Data_Fy1=Data_Fy;
        case 2
            Fz=1381;
            Data_Fy2=Data_Fy;
        case 3
            Fz=921;
            Data_Fy3=Data_Fy;
        case 4
            Fz=460.8;
            Data_Fy4=Data_Fy;     
        case 5
            Data_Fy5=Data_Fy;  
    end
alpha=-0.5;
end
format long
Fz=2302*0.453592
Fz=1842*0.453592
Fz=1381*0.453592
Fz=921*0.453592
Fz=460.8*0.453592
x=-0.5:0.01:0.5;
% plot(x,Data_Fy1)
% hold on
% plot(x,Data_Fy2)
% plot(x,Data_Fy3)
% plot(x,Data_Fy4)
% plot(x,Data_Fy5)


plot(x(1:2:101),Data_Fy1(1:2:101)*0.453592*9.8,'k','linewidth',2)
hold on
plot(x(1:2:101),Data_Fy2(1:2:101)*0.453592*9.8,'r--','linewidth',2)
plot(x(1:2:101),Data_Fy3(1:2:101)*0.453592*9.8,'k.-','linewidth',1)
plot(x(1:2:101),Data_Fy4(1:2:101)*0.453592*9.8,'r','linewidth',2)
plot(x(1:2:101),Data_Fy5(1:2:101)*0.453592*9.8,'k-.','linewidth',2)
% set(gcf,'outerposition',[0 0 560 450])
grid on
xlabel('Slip Angle[rad]')
ylabel('Lateral Force[N]')
% xlabel('Slip Angle[rad]','fontsize',14,'fontweight','n')
% ylabel('Lateral Force[N]','fontsize',14,'fontweight','n')
% set(gca,'fontsize',14,'fontweight','n','xcolor','k','ycolor','k')

legend(' 1044.16 kg','  835.51 kg','  626.41 kg', '  417.75 kg','  209.01 kg')

set(gcf,'position',[0 0 450 300])
