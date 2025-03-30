hold on;
% plot(time,Reference_input,'r'); plot(time,Steering_Angle,'b');


% 
% plot(time,Gway_Vehicle_Speed/3.6,'--r')
% plot(data.i*par.T,data.vxc,'b')


% plot(data.i*par.T,data.delta_center(:))

%% yaw rate
% plot(time,Yaw_Rate/180*pi+0.015 ,'--r')
% plot(data.i*par.T,data.vrzc(:),'b')
% legend('Vehicle Test','3D Model')
% ylabel('Yawrate[rad/s]')
% ylim([-0.25 0.3])
% xlim([0 8])

%% lateral accel
plot(time,Gway_LAT_ACCEL+0.2 ,'--r')
plot(data.i*par.T,data.axc(:),'b')
legend('Vehicle Test','3D Model')
ylabel('Lateral Acceleration[m/s^2]')

ylim([-3 4])
xlim([0 8])


set(gcf,'outerposition',[0 0 560 450])
% grid on
xlabel('Time[sec]')



