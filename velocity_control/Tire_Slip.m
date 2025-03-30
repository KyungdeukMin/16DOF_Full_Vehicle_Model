function [alpha,kappa,vxw,vyw,ang]=Tire_Slip(par,delta,vxc,vyc,vrzc,Tyw,Fzt,Fxw,lock)
% persistent aryw
persistent vryw
persistent ryw
if isempty(vryw)
% aryw=[0 0 0 0]; %[rad/s^2]
vryw=[0 0 0 0]; %[rad/s]
ryw=[0 0 0 0]; %[rad]
end


vxw(1)=vxc - par.b*vrzc;
vyw(1)=vyc + par.af*vrzc;
vxw(2)=vxc + par.b*vrzc;
vyw(2)=vyc + par.af*vrzc;
vxw(3)=vxc - par.b*vrzc;
vyw(3)=vyc - par.ar*vrzc;
vxw(4)=vxc + par.b*vrzc;
vyw(4)=vyc - par.ar*vrzc;

% Veh.alpha=tire_slip_ang (Veh.vxw,Veh.vyw,Veh.delta);
      
R_frame=[cos(delta(1)) sin(delta(1));
         -sin(delta(1)) cos(delta(1))];  
vw.lf=(R_frame*[vxw(1); vyw(1)]);
    alpha_lf=-atan2(vw.lf(2,1),vw.lf(1,1));
     if (alpha_lf>pi/2) 
         alpha_lf=pi-alpha_lf;
     end
     if (alpha_lf<-pi/2) 
         alpha_lf=-pi-alpha_lf;
     end 

R_frame=[cos(delta(2)) sin(delta(2));
         -sin(delta(2)) cos(delta(2))];

vw.rf=(R_frame*[vxw(2); vyw(2)]);
    alpha_rf=-atan2(vw.rf(2,1),vw.rf(1,1));
     if (alpha_rf>pi/2) 
         alpha_rf=pi-alpha_rf;
     end
     if (alpha_rf<-pi/2) 
         alpha_rf=-pi-alpha_rf;
     end 

R_frame=[1 0;
         0 1];

vw.lr=(R_frame*[vxw(3); vyw(3)]);
    alpha_lr=-atan2(vw.lr(2,1),vw.lr(1,1));
     if (alpha_lr>pi/2) 
         alpha_lr=pi-alpha_lr;
     end
     if (alpha_lr<-pi/2) 
         alpha_lr=-pi-alpha_lr;
     end 

vw.rr=(R_frame*[vxw(4); vyw(4)]);
    alpha_rr=-atan2(vw.rr(2,1),vw.rr(1,1));
     if (alpha_rr>pi/2) 
         alpha_rr=pi-alpha_rr;
     end
     if (alpha_rr<-pi/2) 
         alpha_rr=-pi-alpha_rr;
     end 
if alpha_lf>0.5
    alpha_lf=0.5;
elseif alpha_lf<-0.5
    alpha_lf=-0.5;
end
if alpha_rf>0.5
    alpha_rf=0.5;
elseif alpha_rf<-0.5
    alpha_rf=-0.5;
end
if alpha_lr>0.5
    alpha_lr=0.5;
elseif alpha_lr<-0.5
    alpha_lr=-0.5;
end
if alpha_rr>0.5
    alpha_rr=0.5;
elseif alpha_rr<-0.5
    alpha_rr=-0.5;
end

alpha=[alpha_lf,alpha_rf,alpha_lr,alpha_rr];

for wheel_num=1:4
    
    if vryw(wheel_num)>0
    aryw(wheel_num)=((Tyw(wheel_num)-Fzt(wheel_num)*0.015*par.Rw)-Fxw(wheel_num)*par.Rw)/par.iyw;
    elseif vryw(wheel_num)<0
    aryw(wheel_num)=((Tyw(wheel_num)+Fzt(wheel_num)*0.015*par.Rw)-Fxw(wheel_num)*par.Rw)/par.iyw;   
    else
    aryw(wheel_num)=(Tyw(wheel_num)-Fxw(wheel_num)*par.Rw)/par.iyw;   
    end
    vryw(wheel_num)=vryw(wheel_num)+aryw(wheel_num)*par.T;
    if lock==1 
       vryw(wheel_num)=0; 
    end
    ryw(wheel_num)=ryw(wheel_num)+vryw(wheel_num)*par.T;
    
    if vxw(wheel_num)==0
    kappa(wheel_num)=-sign(vryw(wheel_num));
    else
    kappa(wheel_num)=(vxw(wheel_num)-par.Rw*vryw(wheel_num))/abs(vxw(wheel_num));
    end
    
    if kappa(wheel_num)>0.5
    kappa(wheel_num)=0.5;
    elseif kappa(wheel_num)<-0.5
    kappa(wheel_num)=-0.5;
    end
end
ang=ryw;
return