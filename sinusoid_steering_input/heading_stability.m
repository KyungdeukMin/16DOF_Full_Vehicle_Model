s=tf('s');
% % k1=1;
% % k2=0.3;
% % tau=0.2;
% % % lcp=10;
% % % v=10;
% % To=k1*k2/(tau*s^3+s^2+k1*s+k1*k2);
% % % Ti=k1/(tau*s^2+s+k1);
% % % step(Ti)
% % % To=(k1*k2*v+k1*k2*lcp*s)/(v*tau*s^3+(v+k1*lcp)*s^2+(v*k1+k1*k2*lcp)*s+k1*k2*v)
% % ec=To/(v/s^2);
% % step(To)
% % % bode(To)
% ec=To/(v/s^2);

% k1=1;
% k2=0.3;
% tau=0.25;
% To=k1*k2/(tau*s^3+s^2+k1*s+k1*k2);
% ec=To/(v/s^2);
% % step(To)
% step(ec)
% 
% hold on
% 
% k1=2;
% To=k1*k2/(tau*s^3+s^2+k1*s+k1*k2);
% ec=To/(v/s^2);
% % step(To)
% step(ec)
% k1=3;
% To=k1*k2/(tau*s^3+s^2+k1*s+k1*k2);
% ec=To/(v/s^2);
% % step(To)
% step(ec)
% k1=4;
% To=k1*k2/(tau*s^3+s^2+k1*s+k1*k2);
% ec=To/(v/s^2);
% % step(To)
% step(ec)
% k1=0.5;
% To=k1*k2/(tau*s^3+s^2+k1*s+k1*k2);
% ec=To/(v/s^2);
% % step(To)
% step(ec)

% tau=0.25;
% 
% k1=0.5;
% Ti=k1/(tau*s^2+s+k1);
% step(Ti)
% hold on
% 
% k1=1;
% Ti=k1/(tau*s^2+s+k1);
% step(Ti)
% k1=2;
% Ti=k1/(tau*s^2+s+k1);
% step(Ti)
% k1=3;
% Ti=k1/(tau*s^2+s+k1);
% step(Ti)
% k1=4;
% Ti=k1/(tau*s^2+s+k1);
% step(Ti)
% k1=5;
% Ti=k1/(tau*s^2+s+k1);
% step(Ti)

% v=10;
% tau=0.25;
% k1=1;
% Ti=k1/(tau*s^2+s+k1);
% vec=Ti*s*10;
% step(vec,'b-.')
% hold on
% 
% % k1=2;
% % Ti=k1/(tau*s^2+s+k1);
% % vec=Ti*s*10;
% % step(vec,'r')
% % 
% % k1=0.5;
% % Ti=k1/(tau*s^2+s+k1);
% % vec=Ti*s*10;
% % step(vec,'r')
% 
% legend('k_1 = 1','k_1 = 2')
% ylabel('Lateral Acceleration[m/s^2]')
% ylabel('Lateral Jerk[m/s^3]')

% v=10;
% tau=0.25;
% k1=1;
% Ti=k1/(tau*s^2+s+k1);
% vec=Ti*s^2*10;
% step(vec)
% hold on
% 
% k1=2;
% Ti=k1/(tau*s^2+s+k1);
% vec=Ti*s^2*10;
% step(vec)

lwb=2.64;
v=10;
tau=0.25;
k1=1;
Ti=k1/(tau*s^2+s+k1);
vec=Ti*s;
[y1,t1]=step(vec,5);


k1=2;
Ti=k1/(tau*s^2+s+k1);
vec=Ti*s;
[y2,t2]=step(vec,5);

y1=atan2(lwb,v./y1);
y2=atan2(lwb,v./y2);
for i=1:size(y2)
    if y2(i)>pi/2
        y2(i)=y2(i)-pi;
    end
end

plot(t1,y1,'b-.')
hold on
plot(t2,y2,'r')
plot([0 5],[0 0],'k')

legend('k_1 = 1','k_1 = 2')
ylabel('Lateral Acceleration[m/s^2]')
ylabel('Lateral Jerk[m/s^3]')
ylabel('Steering Angle[rad]')
xlabel('Time[seconds]')

