pkg load control
s=tf('s');
Gs=1.05/(0.002*s^2 + 0.01*s +1);    %Plant model

figure(1)
bode(Gs)
[gamma_gs, phi_gs, W_gamma_gs, W_phi_gs]=margin(Gs)   %Margins

Yoss=1.05;          %steady state step reponse of Gs
desired_ess=0.01;   %desired steady state error
kp=1/desired_ess -1
K=kp/Yoss

ph_m=70;wmax=90;      %phase margin required, gain crossover freq.
ph_max_dg=pi/180 *ph_m;

figure(2)
KGs=K*Gs
bode(KGs)
[gamma_kgs, phi_kgs, W_gamma_kgs, W_phi_kgs]=margin(KGs)

alpha=(1-sin(ph_max_dg))/(1+sin(ph_max_dg));
T=1/(wmax*sqrt(alpha));
#Ds=(T*s+1)/(alpha*T*s+1)%Lead compensator
Ds=-(6.3e-6*(s+1/(10000*6.3e-6)))/(0.066e-6 *(s+1/(10000*0.066e-6)))
KDGs=KGs*Ds;

g_r=31;%gain reduction amount
alpha_g=10^(g_r/20);
Zcg=wmax/10;
Tg=1/Zcg;
#Ds2=(Tg*s+1)/(alpha_g*Tg*s+1)%Lag compensator
Ds2=-(0.11e-6*(s+1/(1.01e6 *0.11e-6)))/(3.94e-6 *(s+1/(1e6*3.94e-6)))
LLGs=KDGs*Ds2;    %Loop transfer function model
LLs=K*Ds*Ds2;     %Lead-Lag compensator model

figure(3)
bode(Ds)
[gamma_ds, phi_ds, W_gamma_ds, W_phi_ds]=margin(Ds)
figure(4)
[gamma_dgs, phi_dgs, W_gamma_dgs, W_phi_dgs]=margin(KDGs)
bode(KDGs);
figure(6)
[gamma_llgs, phi_llgs, W_gamma_llgs, W_phi_llgs]=margin(LLGs)
bode(LLGs);
Ts=feedback(LLGs,1,-1); %feedback system model

% Comparison of Step responses.
figure(7)
t=0:0.01:1;
subplot(2,1,1);
step(Gs,t);
subplot(2,1,2);
step(Ts,t);

% Bode plots
W=1:0.1:1000;
[Mg, Pg,W]=bode(KGs,W);
[Md, Pd,W]=bode(Ds,W);
[Md2, Pd2,W]=bode(Ds2,W);
[Mdg, Pdg,W]=bode(LLGs,W);

figure(8)
subplot(2,1,1);
semilogx(W,20*log10(Mg),";KGs;", W,20*log10(Md),";Lead;", W,20*log10(Md2),...
 ";Lag;", W,20*log10(Mdg),";KGDDs2;");
title ("Magnitude Gs,Ds,Ds2,LLGs");
h=legend("show");
subplot(2,1,2);
semilogx(W,Pg,";KGs;", W,Pd,";Lead;", W,Pd2,";Lag;", W,Pdg,";KGDDs2;");
title ("Phase Gs,Ds,Ds2,LLGs");

%digital model for LLs
LLZ=c2d(LLs,0.002)
figure(9)
Gsz=c2d(Gs,0.002)
Tsz=feedback(LLZ*Gsz,1,-1)
#step(Tsz)
tt=0:0.002:1;
[yy,tt]=step(Tsz,tt);
data=[tt,yy];
save '-ascii' 'step_Tsz_octave.txt' 'data'

figure(10)
#Digital model of PID controller
lls=(4.54+0.05*s/(0.0007*s+1)+98.7/s) #PID with low pass fillter at D controller
ld=c2d(lls,0.002)
Gsz=c2d(Gs,0.002)
Tszp=feedback(ld*Gsz,1,-1)

tt=0:0.002:1;
[yy,tt]=step(Tszp,tt);
dataP=[tt,yy];
save '-ascii' 'step_PID_Tsz_octave.txt' 'dataP'
plot(tt,yy)
