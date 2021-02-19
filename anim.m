function [szOut]=anim
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% anim.m: Fonction qui fait l'animation du drone
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% chargement des variables
load etat;
etat=ans';
save etat.data etat -ascii;
load vvent;
vvent= ans';
load control;
control= ans';

% on dessine le premier
pose= [etat(1,2);etat(1,3);etat(1,4);etat(1,8);etat(1,9);etat(1,10);etat(1,11)];
vent= [vvent(1,2);vvent(1,3);vvent(1,4)];
draw(pose,vent,1);
refresh;

% on fait l'animation
cl0=clock;
for i=1:1:size(etat,1)
  while  etime(clock,cl0) < etat(i,1),
     etime(clock,cl0);
  end;
  pose= [etat(i,2);etat(i,3);etat(i,4);etat(i,8);etat(i,9);etat(i,10);etat(i,11)];
  vent= [vvent(i,2);vvent(i,3);vvent(i,4)];
  draw(pose,vent,i);
end;
disp('Taper sur une touche pour fermer la fenetre');
pause;
close;

function [szOut] = draw(pose,vent,index)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% draw.m: Fonction qui fait les tracés
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

global hc;

% Definition de la geometrie des corps et reperes
N= 32;
for i=1:1:N+1
  theta= 2*(i-1)*pi/N;
  co(i)= cos(theta);
  so(i)= sin(theta);
  pco(i)= 0.7*co(i);
  pso(i)= 0.7*so(i);
end;

diamhelice= 0.2;
longbras= 0.4;

Helice= diamhelice*[co(1) co(2) nan co(2) co(3) nan co(3) co(4) nan co(4) co(5) nan...
      co(5) co(6) nan co(6) co(7) nan co(7) co(8) nan co(8) co(9) nan...
      co(9) co(10) nan co(10) co(11) nan co(11) co(12) nan co(12) co(13) nan...
      co(13) co(14) nan co(14) co(15) nan co(15) co(16) nan co(16) co(17) nan...
      co(17) co(18) nan co(18) co(19) nan co(19) co(20) nan co(20) co(21) nan...
      co(21) co(22) nan co(22) co(23) nan co(23) co(24) nan co(24) co(25) nan...
      co(25) co(26) nan co(26) co(27) nan co(27) co(28) nan co(28) co(29) nan...
      co(29) co(30) nan co(30) co(31) nan co(31) co(32) nan co(32) co(33);
      so(1) so(2) nan so(2) so(3) nan so(3) so(4) nan so(4) so(5) nan...
      so(5) so(6) nan so(6) so(7) nan so(7) so(8) nan so(8) so(9) nan...
      so(9) so(10) nan so(10) so(11) nan so(11) so(12) nan so(12) so(13) nan...
      so(13) so(14) nan so(14) so(15) nan so(15) so(16) nan so(16) so(17) nan...
      so(17) so(18) nan so(18) so(19) nan so(19) so(20) nan so(20) so(21) nan...
      so(21) so(22) nan so(22) so(23) nan so(23) so(24) nan so(24) so(25) nan...
      so(25) so(26) nan so(26) so(27) nan so(27) so(28) nan so(28) so(29) nan...
      so(29) so(30) nan so(30) so(31) nan so(31) so(32) nan so(32) so(33);
      0 0 nan 0 0 nan 0 0 nan 0 0 nan 0 0 nan 0 0 nan...
      0 0 nan 0 0 nan 0 0 nan 0 0 nan 0 0 nan 0 0 nan...
      0 0 nan 0 0 nan 0 0 nan 0 0 nan 0 0 nan 0 0 nan...
      0 0 nan 0 0 nan 0 0 nan 0 0 nan 0 0 nan 0 0 nan...
      0 0 nan 0 0 nan 0 0 nan 0 0 nan 0 0 nan 0 0 nan...
      0 0 nan 0 0];
  
Croixav= [0 longbras; 0 0; 0 0];
Croixdr= [0 0; 0 longbras; 0 0];
Croixn=  [0 -longbras nan 0 0 nan 0 0;0 0 nan 0 -longbras nan 0 0; 0 0 nan 0 0 nan 0 0];

Solrep1= [1 0; 0 0; 0 0]; 
Solrep2= [0 0; 0 1; 0 0];
Solrep3= [0 0; 0 0; 0 1];

position= [pose(1);pose(2);pose(3)];
qs= pose(4); qv=[pose(5);pose(6);pose(7)];
orientation0= eye(3)+2*qs*skew(qv)+2*skew(qv)^2;
orientation= [1 0 0; 0 -1 0; 0 0 -1]*orientation0*[1 0 0; 0 -1 0; 0 0 -1]; % Passage du repere NED au repere Matlab

for i=1:95
   Helice1(:,i)=  Helice(:,i)+ [longbras;0;0];
   Helice2(:,i)=  Helice(:,i)+ [0;longbras;0];
   Helice3(:,i)=  Helice(:,i)- [longbras;0;0];
   Helice4(:,i)=  Helice(:,i)- [0;longbras;0];
end;   
for i=1:95
  Helice1(:,i)= position+ orientation*Helice1(:,i);
  Helice2(:,i)= position+ orientation*Helice2(:,i);
  Helice3(:,i)= position+ orientation*Helice3(:,i);
  Helice4(:,i)= position+ orientation*Helice4(:,i);
end;
for i=1:2
  Croixav(:,i)= position+ orientation*Croixav(:,i);
  Croixdr(:,i)= position+ orientation*Croixdr(:,i);   
end;
for i=1:8
  Croixn(:,i)= position+ orientation*Croixn(:,i);
end;
for i=1:2 % artifice de visualisation pour que le repere au sol ne soit pas efface par les segments
  Solrep1(:,i)= 0.000001*max(position(1),position(2))+ Solrep1(:,i);
  Solrep2(:,i)= 0.000001*max(position(1),position(2))+ Solrep2(:,i);
  Solrep3(:,i)= 0.000001*max(position(1),position(2))+ Solrep3(:,i);
end;
vevent= [0 vent(1) nan 0 vent(2)/5 nan 0 -vent(2)/5;...
 -4 -4+vent(2) nan -4 -4-vent(1)/5 nan -4 -4+vent(1)/5;...
         0 0 nan 0 0 nan 0 0]; 

% On dessine les objets
if index==1
% Initialisation des propriétés de la figure
  figure(1);
  clf;
  set(gcf, 'Name', 'Animation Window');
  set(gcf,'Position',[ 10 20 1000 900]);
  set(1,'BackingStore','on');
  hold on;
  set(gca, 'UserData', hc,'NextPlot', 'add','Visible', 'on','DataAspectRatio', [1 1 1], ...
	  'Color', 'w','SortMethod', 'childorder','Xlim',[-1 6],'Ylim',[-1 6],'Zlim',[0 3] );
 
  hc(1)=plot3(Helice1(1,:),Helice1(2,:),Helice1(3,:),'k-','Linewidth',3);
  hc(2)=plot3(Helice2(1,:),Helice2(2,:),Helice2(3,:),'k-','Linewidth',3);
  hc(3)=plot3(Helice3(1,:),Helice3(2,:),Helice3(3,:),'k-','Linewidth',3);
  hc(4)=plot3(Helice4(1,:),Helice4(2,:),Helice4(3,:),'k-','Linewidth',3);
  hc(5)=plot3(Croixav(1,:),Croixav(2,:),Croixav(3,:),'r-','Linewidth',3);
  hc(6)=plot3(Croixdr(1,:),Croixdr(2,:),Croixdr(3,:),'k-','Linewidth',3);
  hc(7)=plot3(Croixn(1,:),Croixn(2,:),Croixn(3,:),'k-','Linewidth',3);
  hc(8)=plot3(Solrep1(1,:),Solrep1(2,:),Solrep1(3,:),'r-','Linewidth',2);
  hc(9)=plot3(Solrep2(1,:),Solrep2(2,:),Solrep2(3,:),'b-','Linewidth',3);
  hc(10)=plot3(Solrep3(1,:),Solrep3(2,:),Solrep3(3,:),'k-','Linewidth',3);
  hc(11)= plot3(vevent(1,:),vevent(2,:),vevent(3,:),'g-','Linewidth',3);

  drawnow;
  view(3);
  view(-37,30);
  grid minor;
  zoom(1);
  xlabel('x');
  ylabel('y');
  zlabel('z');
  box on;
 
else
  set(hc(1),'XData',Helice1(1,:),'YData',Helice1(2,:),'ZData',Helice1(3,:));
  set(hc(2),'XData',Helice2(1,:),'YData',Helice2(2,:),'ZData',Helice2(3,:));
  set(hc(3),'XData',Helice3(1,:),'YData',Helice3(2,:),'ZData',Helice3(3,:));
  set(hc(4),'XData',Helice4(1,:),'YData',Helice4(2,:),'ZData',Helice4(3,:));
  set(hc(5),'XData',Croixav(1,:),'YData',Croixav(2,:),'ZData',Croixav(3,:));
  set(hc(6),'XData',Croixdr(1,:),'YData',Croixdr(2,:),'ZData',Croixdr(3,:));
  set(hc(7),'XData',Croixn(1,:),'YData',Croixn(2,:),'ZData',Croixn(3,:));
  set(hc(8),'XData',Solrep1(1,:),'YData',Solrep1(2,:),'ZData',Solrep1(3,:));
  set(hc(9),'XData',Solrep2(1,:),'YData',Solrep2(2,:),'ZData',Solrep2(3,:));
  set(hc(10),'XData',Solrep3(1,:),'YData',Solrep3(2,:),'ZData',Solrep3(3,:));
  set(hc(11),'XData',vevent(1,:),'YData',vevent(2,:),'ZData',vevent(3,:));
  drawnow;
end;

function [sk]= skew(z)
sk= [0 -z(3) z(2); z(3) 0 -z(1); -z(2) z(1) 0];
