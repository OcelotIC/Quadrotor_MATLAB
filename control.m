function [cont]=control(inp)

% parametres;
    rho= 1.2;
    masse= 1.3; 
    inertie= [0.02 0 0; 0 0.02 0; 0 0 0.04];
    Np= 4;
    d= 0.275;
    w0= 644;
    kappa_L= masse*9.81/Np/w0^2;
    kappa_D= 1.1e-7;
    wm_max= 1000;
    k_moteur= 15;
    c0 = 0.1;
    cijk=[0.02;0.02;0.2];
   

% Calcul de la matrice d'allocation (N_p moteurs)
    theta=zeros(Np,1); % On redéfinit les directions des moteurs dans le repère North-East
    for i=1:Np
        theta(i)= pi/Np+(i-1)*(2*pi/Np);
    end
    Alloc= zeros(4,Np);
    for i=1:Np
    Alloc(:,i)= [kappa_L; -d*kappa_L*sin(theta(i)); d*kappa_L*cos(theta(i)); (-1)^i*kappa_D];
    end  

% Recuperation donnes/mesures
    p_mes= [inp(1);inp(2);inp(3)];
    v_mes= [inp(4);inp(5);inp(6)];
    q_mes= [inp(7);inp(8);inp(9); inp(10)];
    qs_mes= q_mes(1); qv_mes= [q_mes(2);q_mes(3);q_mes(4)];
    R_mes= eye(3)+2*qs_mes*skew(qv_mes)+2*skew(qv_mes)^2;
    omega_mes= [inp(11);inp(12);inp(13)];
    wm_mes= [inp(14);inp(15);inp(16);inp(17)];
    v_ref=[inp(18);inp(19);inp(20)];
    omega3_ref= inp(21);
    
% Calcul de poussée/couples désirés (voir Articles Automatica 2015/Automatica 2017, Eq. (26))
    k_v= 3;
    varpi= 7;
    k_gamma= 2*varpi/3;
    k_omega= diag([3*varpi/2;3*varpi/2;5]);
    e3=[0;0;1];
   
    vt= v_mes-v_ref;
    xi= -k_v*vt/(1+0.2*norm(vt));
    T_ref= masse*(9.81*e3-xi)-c0*norm(v_mes)*v_mes;
    Poussee_d= norm(T_ref);
    k_ref= T_ref/Poussee_d;
    omega_ref= k_gamma*cross(e3,R_mes'*k_ref)+omega3_ref*e3;
    Couple_d= -k_omega*inertie*(omega_mes-omega_ref);

% Passage de poussee/couples désirées aux vitesses moteur de consigne
    wm_square_d= Alloc'*inv(Alloc*Alloc')*[Poussee_d;Couple_d];
    wm_d=zeros(Np,1);
    for i=1:Np
        wm_d(i)= sqrt(max(wm_square_d(i),0));
        wm_d(i)= min(wm_d(i),wm_max);
    end

cont= [wm_d;omega_ref];

function [sk]= skew(z)
sk= [0 -z(3) z(2); z(3) 0 -z(1); -z(2) z(1) 0];

