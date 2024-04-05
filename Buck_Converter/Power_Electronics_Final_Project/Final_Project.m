%Final Project for Power Eletronics Laboratory
%% Buck Data
clc
clear
s=tf('s');

%Buck Parameters
Vd=36*sqrt(2); 
R=22;
L=1e-3;
C=22e-6;
fs = 12e3;

%Sensor Parameters
Hs=0.1; %Static Gain= R2/R1
R1=20e3;
R2=Hs*R1;

%Pwm modulator Parameters (First Order Pade Aprox.):
Td=0.5/fs;

%% Controler Project
%Plant
Gp=Vd/(L*C*s*s+ (L/R)*s +1);
% zpk(Gp)

%Pwm delay
Tds=(1-s*Td/2)/(1+s*Td/2);
% Tds=exp(-Td*s);
% zpk(Tds)


%Controller (I)
Ki=38;
Gc=Ki/s;


%Open Loop Transfer Function (Direct Branch*Sensor)
TF_PLANT=Gp*Tds;
FTMA=Gp*Tds*Hs/s;
% rlocus(FTMA)

%Closed Loop Transfer Function
FTMF=feedback(Gc*Gp*Tds,Hs);
Step_Data=stepinfo(FTMF);
% step(FTMF)
% pzmap(FTMF);


%Print Settling Time and Maximum Overshoot
Ts_ms=(Step_Data.SettlingTime)*1e3;
Mp=Step_Data.Overshoot;
Msg=['Settling Time: ',num2str(Ts_ms),' ms. || Mp%: ', num2str(Mp)];
disp(Msg);


%step from 25v to 45v
Config = RespConfig('InputOffset',2.5,'Amplitude',2.0);
step(FTMF,0.05,Config);
grid on


%Send Controller and Delay Data from MATLAB to Simulink
Gc_Num=cell2mat(Gc.Numerator);
Gc_Den=cell2mat(Gc.Denominator);

Tds_Num=cell2mat(Tds.Numerator);
Tds_Den=cell2mat(Tds.Denominator);

%% Parameters for analog Controller
R1_c=10e3;
R3_c=10e3;
R4_c=3.9e3;
C2_c=1e-6;
Ki_c=R4_c/(R3_c*R1_c*C2_c);
