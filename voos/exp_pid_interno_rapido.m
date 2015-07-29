clc
%clear all
close all
%load voo_20150728_181722.mat

tei=1001
tef=3500

a=a0(tei:tef)
r=r0(tei:tef)
g=g0(tei:tef)



figure(1);
plot(a/636/(2*pi)*360,'b','Linewidth',1);
hold on;
plot(r/636/(2*pi)*360,'k','Linewidth',1);

M1=zpk([],[.9],[.1],1);
yd1=lsim(M1,r);

plot(yd1/636/(2*pi)*360,'r','Linewidth',1);
hold off;
axis([0 length(yd1) -50 50])
xlabel('Amostras','Interpreter','Latex');
ylabel('\^Angulo pitch [graus]','Interpreter','Latex');
h=legend('\^Angulo medido','Refer{\^e}ncia','Modelo de refer\^encia');
set(h,'Interpreter','latex');
set(h,'Location','SouthWest');
aq=get(h,'position');
set(h,'Position',[aq(1) aq(2) 0.3 0.1]);
saveas(h,'exp1ang.fig')
print('exp1ang','-depsc')

J1=sqrt((yd1-a)'*(yd1-a)/length(yd1)/636/(2*pi)*360/636/(2*pi)*360)

C1=tf([4.864 -4.813],[1 -1],1);


r2=lsim(C1,(r-a));

r2=r2-mean(r2);
g=g-mean(g);

C2=tf([2.456 -2.361],[1 0],1);
u2=lsim(C2,(r2-g));


figure(2);
plot(g,'b','Linewidth',1);
hold on;
plot(r2,'k','Linewidth',1);

M2=zpk([],[.7],[.3],1);
yd2=lsim(M2,r2);

plot(yd2,'r','Linewidth',1);
hold off;
axis([0 length(yd2) min(yd2)*2 max(yd2)*2])
xlabel('Amostras','Interpreter','Latex');
ylabel('Velocidade \^angular pitch','Interpreter','Latex');
h=legend('Velocidade angular medida','Refer\^encia','Modelo de refer\^encia','Location','SouthWest');
set(h,'Interpreter','latex');
set(h,'Location','SouthWest');
aq=get(h,'position');
set(h,'Position',[aq(1) aq(2) 0.4 0.1]);
saveas(h,'exp1giro.fig')
print('exp1giro','-depsc')

J2=sqrt((yd2-g)'*(yd2-g)/length(yd2))




C1=tf([1],[1],1);
C2=tf([1 -1],[1 0],1);
C3=tf([1 0],[1 -1],1);


%Cb=[C1;C2;C3];
Cb=[C1;C3];
%Cb=[C1];

yv=lsim((1-M1),a);
yvv=lsim(Cb,yv);
 
uv=lsim(M1,r2);
uI=lsim(C3,yv);
gI=0.0;
uv2=uv+uI*gI;
 
gg=inv(yvv'*yvv)*yvv'*uv2;
 
Cv1=gg'*Cb
Cv1=zpk(minreal(Cv1))

figure(3)
plot(uv2,'b')
hold on
plot(lsim(Cv1,yv),'r')
hold off





Cb=[C1;C2];


yv=lsim((1-M2),g);
yvv=lsim(Cb,yv);

uv=lsim(M2,u2);
uI=lsim(C3,yv);
gI=0.0;
uv2=uv+uI*gI;


gg=inv(yvv'*yvv)*yvv'*uv2;
 
Cv2=gg'*Cb
Cv2=zpk(minreal(Cv2))

figure(4)
plot(uv2,'b')
hold on
plot(lsim(Cv2,yv),'r')
%plot(uI*gI,'g')
hold off
