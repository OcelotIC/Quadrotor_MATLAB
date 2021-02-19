function [mes]=imu(etat)

p= [etat(1);etat(2);etat(3)];
v= [etat(4);etat(5);etat(6)];
q= [etat(7);etat(8);etat(9);etat(10)]; qv= [etat(8);etat(9);etat(10)];
omega= [etat(11);etat(12);etat(13)];
wm= [etat(14);etat(15);etat(16);etat(17)];
vdot= [etat(18);etat(19);etat(20)];

R= eye(3)+2*q(1)*skew(qv)+2*skew(qv)^2;
acc=R'*(vdot-9.81*[0;0;1]);
gyro= omega;
magneto= 4.7e-5*R'*[cos(65*pi/180);0;sin(65*pi/180)];
mes= [acc;gyro;magneto];

function [sk]= skew(z)
sk= [0 -z(3) z(2); z(3) 0 -z(1); -z(2) z(1) 0];