function out=tire_slip_ang (vxw,vyw,delta)
      
R_frame=[cos(delta(1)) sin(delta(1));
         -sin(delta(1)) cos(delta(1))];
     
vw.lf=(R_frame*[vxw(1); vyw(1)]);
% if (vw.lf(1,1) == 0) 
% else
    alpha_lf=-atan2(vw.lf(2,1),vw.lf(1,1));
     if (alpha_lf>pi/2) 
         alpha_lf=pi-alpha_lf;
     end
     if (alpha_lf<-pi/2) 
         alpha_lf=-pi-alpha_lf;
     end 
% end

R_frame=[cos(delta(2)) sin(delta(2));
         -sin(delta(2)) cos(delta(2))];

vw.rf=(R_frame*[vxw(2); vyw(2)]);
% if (vw.rf(1,1) == 0) 
% else
    alpha_rf=-atan2(vw.rf(2,1),vw.rf(1,1));
     if (alpha_rf>pi/2) 
         alpha_rf=pi-alpha_rf;
     end
     if (alpha_rf<-pi/2) 
         alpha_rf=-pi-alpha_rf;
     end 
% end

R_frame=[1 0;
         0 1];

vw.lr=(R_frame*[vxw(3); vyw(3)]);
% if (vw.lr(1,1) == 0) 
% else
    alpha_lr=-atan2(vw.lr(2,1),vw.lr(1,1));
     if (alpha_lr>pi/2) 
         alpha_lr=pi-alpha_lr;
     end
     if (alpha_lr<-pi/2) 
         alpha_lr=-pi-alpha_lr;
     end 
% end

vw.rr=(R_frame*[vxw(4); vyw(4)]);
% if (vw.rr(1,1) == 0) 
% else
    alpha_rr=-atan2(vw.rr(2,1),vw.rr(1,1));
     if (alpha_rr>pi/2) 
         alpha_rr=pi-alpha_rr;
     end
     if (alpha_rr<-pi/2) 
         alpha_rr=-pi-alpha_rr;
     end 
% end
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

out=[alpha_lf,alpha_rf,alpha_lr,alpha_rr];
return