function [dyn]=dynamique(u)

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

% Défintion des variables
    p= [u(1);u(2);u(3)]; % position du CM du drone
    v= [u(4);u(5);u(6)]; % v= dp/dt
    qs=u(7); qv=[u(8); u(9); u(10)]; q=[qs;qv]; % quaternion associé à  l'orientation du drone
    omega= [u(11);u(12);u(13)]; % vitesse angulaire exprimée en repère corps
    wm= [u(14);u(15); u(16); u(17)]; % vitesses moteur
    vvent= [u(18);u(19);u(20)]; % vitesse vent
    wm_d= [u(21);u(22);u(23);u(24)]; % vitesses moteur désirées

    Rot= eye(3)+2*qs*skew(qv)+2*skew(qv)^2; % matrice de rotation (non utilisée directement)
    e3= [0;0;1];
    Rot_e3= Rot*e3;

% calcul des forces et moments
    va= v-vvent; % vitesse relative air
    nva=norm(va);
    vak= va'*Rot_e3; 
    vah= sqrt(nva^2-vak^2);

    Mp= Alloc(2:4,:)*wm.^2; % Moments sur les hélices (en repère drone)
    Fg= masse*9.81*e3; % Forces de gravité
    Fa= -nva*Rot*diag(cijk)*Rot'*va-c0*nva*va;
    T= Alloc(1,:)*wm.^2; % Intensité de poussée
    F_total= Fg-T*Rot_e3+Fa;

% dynamique
    dv= F_total/masse;
    dq= [-1/2*qv'*omega-qs*(norm(q)^2-1);1/2*(qs*omega+cross(qv,omega))-qv*(norm(q)^2-1)];
    domega= inv(inertie)* (-cross(omega,inertie*omega)+Mp);
    dwm= -k_moteur*(wm-wm_d);
    dyn= [v;dv;dq;domega;dwm];

function [sk]= skew(z)
    sk= [0 -z(3) z(2); z(3) 0 -z(1); -z(2) z(1) 0];
