function [out]=Automatic_Torque_Control_Model(par,num,den,td,delta)

persistent yk
persistent uk
persistent a
persistent b
persistent td_fifo

if isempty(yk)
%     s=tf('s');
ps=tf(num,den);
% ps=ps*exp(-0.1*s);
pz=c2d(ps,par.T,'tustin')
[n,d]=tfdata(pz);
order=size(d{1},2);
% b(1:order)=fliplr(n{1})
% a(1:order)=fliplr(d{1})
b(1:order)=(n{1});
a(1:order)=(d{1});
a(1)=0;

yk=zeros(order,1);
uk=zeros(order,1);
td_fifo=zeros(round(td/par.T),1);
end

% circshift(yk,1);

td_fifo(1)=delta;
uk(1)=td_fifo(end);
temp=-a*yk + b*uk;
uk=circshift(uk,1);
yk(1)=temp;
yk=circshift(yk,1);
out=yk(1);
td_fifo=circshift(td_fifo,1);
% circshift(uk,1);

return