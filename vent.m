%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Fonction qui d�finit le vent en fonction du temps                  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [z]= vent(tps,choix_traj)

if choix_traj==1 % sans vent
  v1= 0; v2=0; v3=0;
elseif choix_traj==2 % vent constant
  v1= 3; v2=2; v3=-0.5;
elseif  choix_traj==3 % vent periodique
  v1= 2 +.5*cos(tps/2); v2 =.5*sin(tps/2); v3=0.5*sin(tps/2);
elseif choix_traj==4 % vent en rafale
  v1= cos(tps/2); v2 =sin(tps/2); v3=sin(tps/2);
else % vent general
  if tps<20
    v1=0; v2=0; v3=0;
  elseif tps <35
    v1=8; v2=0; v3=0;
  elseif tps<50
    v1= 6 +3*cos(tps-35); v2 = 3*sin(tps-30); v3= 3*sin(tps-30);
  elseif tps<65
    v1= 3*cos(tps); v2 = 3*sin(tps); v3=3*sin(tps);
  else
    v1=0; v2=0; v3=0;
  end;
end;

z=[v1;v2;v3];

