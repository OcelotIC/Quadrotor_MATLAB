%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Fonction qui définit le vent en fonction du temps                  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [z]= vr(tps,choix_traj)

if choix_traj==1 % vitesse nulle
    v_ref= zeros(3,1);
    omega3_ref= 0;
elseif choix_traj==2 % vitesse constante selon i_0
    v_ref= [2;0;0];
    omega3_ref= 0;
elseif  choix_traj==3 % vitesse periodique
    if tps<3
        v_ref= zeros(3,1);
        omega3_ref= 0;
    elseif tps<5
          v_ref= [2;0;0];
          omega3_ref= 0;
    elseif tps<7
          v_ref= -[1;0;0];
          omega3_ref= 0;
    elseif tps<8
         v_ref= [0;0;0];
         omega3_ref= 0;
    elseif tps<12
          v_ref= pi*[-sin(2*pi*(tps-8)/4-pi/2);cos(2*pi*(tps-8)/4-pi/2);0];
          omega3_ref=2*pi/4;
          %omega3_ref= 0;
    else
        v_ref= zeros(3,1);
        omega3_ref= 0;
    end; 
else % vitesse nulle
    v_ref= zeros(3,1);
    omega3_ref= 0;
end;

z=[v_ref;omega3_ref];

