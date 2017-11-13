clear all; close all; 
ang=pi/2; w=0; x=0; y=0; I=20; m=1; vx=0; vy=10; d=2;
Kd=0.2; Ks=0.3; 
t=0; dt=0.05; g=9.8; cm=0.02; factor=2; angp=4.5; co=2; cont=co;
W=[3,3]; mx=x; my=y; 
while t<20
    %W=[0,0.5*(1+sin(2*pi*t/10))];
    W=W+[randn()*0.2,randn()*0.2]; 
    W(abs(W)>3)=W(abs(W)>3)/abs(W(abs(W)>3))*3;
    W=W*0;
    u=[cos(ang), sin(ang)];  %Dirección actual del avion, orientación del eje del avion
    upost=[cos(ang-angp*pi/180), sin(ang-angp*pi/180)];  %Dirección actual del avion, orientación del eje del avion
    v=[vx,vy]-W;               %Vector de velocidad del avion
    uv=v/norm(v);            %Vector en la dirección de la velocidad del movimiento
    up=[-uv(2), uv(1)];      %Vector perpendicular a la velocidad del movimiento
    if v(1)*u(2)-v(2)*u(1)<0  %El vector de velocidad está por encima del eje del avion
      up=-up;
    end  
    theta=acos(dot(u,v)/norm(v)); %Angulo de ataque
    Fs=Ks*norm(v)^2*Cs(theta)*up; %Fuerza de sustentacion
    Fa=Kd*norm(v)^2*Ca(theta)*-uv;%Fuerza de arrastre
    
    %Fuerzas de la colita de atrás
    Vgiro=[v(1)+d/2*sin(ang)*w, v(2)-d/2*cos(ang)*w];
    if(1<2) %norm(Vgiro)==0)
      thetaP=acos(dot(Vgiro,upost)/norm(Vgiro));
      Uvg=Vgiro/norm(Vgiro); 
      Upg=[-Uvg(2), Uvg(1)];
      if Vgiro(1)*upost(2)-Vgiro(2)*upost(1)<0  %El vector de velocidad está por encima del eje del avion
        Upg=-Upg;
      end  
    
      Fsp=Ks/factor*norm(Vgiro)^2*Cs(thetaP)*Upg; %Fuerza de sustentacion
      Fap=Kd/factor*norm(Vgiro)^2*Ca(thetaP)*-Uvg;%Fuerza de arrastre
      Fp=Fsp+Fap;
    else
      Fp=[0,0]; Fap=[0,0]; Fsp=[0,0];
    end
    Ko=5;
    P=[0,-m*g];
    F=Fs+Fa+P;
    Torque=-cm*m*g+(-d/2*u(1)*Fp(2)+d/2*u(2)*Fp(1));
    dw=Torque/I;
    ra=[x,y]+[cos(ang), sin(ang)]; 
    rb=[x,y]-[cos(ang), sin(ang)]; 
    cont=cont-1;
    if cont==0
      cont=co;
      plot([ra(1), rb(1)],[ra(2), rb(2)], '-k'); hold on;
      plot([0 W(1)]+x, [0 W(2)]+y, '-r');
      %plot([0 v(1)/5]+x, [0 v(2)/5]+y, '-b');
      title(sprintf('t=%f',t));
      axis equal;
      pause(0.1)
    end
    %plot(ra(1), ra(2), 'sr'); 
    %plot([0, vx]+x, [0, vy]+y, '-b'); 
    
    %plot([0, Fs(1)]*Ko+x, [0, Fs(2)]*Ko+y, '-r'); 
    %plot([0, Fa(1)]*Ko+x, [0, Fa(2)]*Ko+y, '-m'); 
    
    %plot([0, Fsp(1)]*Ko+x-d/2*u(1), [0, Fsp(2)]*Ko+y-d/2*u(2), '-r'); 
    %plot([0, Fap(1)]*Ko+x-d/2*u(1), [0, Fap(2)]*Ko+y-d/2*u(2), '-m'); 
    %axis([x-5,x+5,y-5,y+5]); 
    %hold off;
    ax=1/m*(F(1));
    ay=1/m*(F(2));
    x=x+vx*dt;
    y=y+vy*dt;
    mx=[mx x]; my=[my y]; 
    ang=ang+w*dt;
    vx=vx+ax*dt;
    vy=vy+ay*dt;
    w=w+dw*dt;
    t=t+dt;
end

plot(mx, my, '-c');