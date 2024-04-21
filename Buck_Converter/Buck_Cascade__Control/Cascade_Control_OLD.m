%Cascade Control of Buck Converter
clc
clear

Vg=250;    %Input voltage(V)
L=1e-3;    %Inductance (L)
C=20e-6;   %Capacitance (F)
R=10;      %Load Resistance (Ohm)
fs=10e3;   %switching Frequency  (Hz)

s=tf('s');

%% 1.Single Loop Controller
%Plants Transfer Function
G1=Vg/(L*C*s*s+ (L/R)*s +1);

%Plant Analysis
% zpk(G1)
% pzmap(G1)

%Controler
ts=3e-3;
z=1.5;
wn=(6.772*z-1.967)/ts; %Ts formula for Z>0.6 aprox.

C1_sl= (wn*wn/( s*(s+2*z*wn)))/G1;
% zpk(C1_sl)

%FTMA analysis
ftma=G1*C1_sl;
% rlocus(ftma)

%FTMF analysis
ftmf=feedback(ftma,1);
% zpk(ftmf)
% pzmap(ftmf);
step(ftmf)

%Test settling time Response (5%)
Step_Data=stepinfo(ftmf,'SettlingTimeThreshold',0.05);
Set_time_ms=Step_Data.SettlingTime*1000;
% disp(Set_time_ms)

C1_sl_Num=cell2mat(C1_sl.Numerator);
C1_sl_Den=cell2mat(C1_sl.Denominator);

%% 2.Cascade Controller
%Current Control ========================
%Plant Analysis
G1_dl= Vg*( C*s+(1/R) )/(L*C*s*s+ (L/R)*s + 1); % Transfer Function
% G1_dl_gain=2.5e5;
% zpk(G1_dl)
% pzmap(G1_dl)

%Current Controller (C1_dl)
z=1e3;
% Kp=0.5;
Kp=0.6;
C1_dl=(s+z)*Kp/(s);

%FTMA analysis (C1_dl)
ftma_dl=G1_dl*C1_dl;
% zpk(ftma_dl)
% rlocus(ftma_dl)


%FTMF analysis
ftmf_dl=feedback(ftma_dl,1);
% pzmap(ftmf_dl);
step(ftmf_dl)
% zpk(ftmf_dl)
% stepinfo(ftmf_dl)


%Test settling time Response (5%)
Step_Data_dl_C1=stepinfo(ftmf_dl,'SettlingTimeThreshold',0.05);
Set_time_us_dl_C1=Step_Data_dl_C1.SettlingTime*1e6;
disp(Set_time_us_dl_C1)

%Voltage Control =======================================================
%Plant Analysis
G2_dl= R/(R*C*s + 1); %Voltage Control Transfer Function
% zpk(G2_dl)
% pzmap(G2_dl)

%Voltage Controller
% C2_dl=1/s;
Ki=85;
C2_dl=Ki/s;

%FTMA analysis
ftma_dl_C2=G2_dl*C2_dl;
% rlocus(ftma_dl_C2)


%FTMF analysis
ftmf_dl_C2=feedback(ftma_dl_C2,1);
% pzmap(ftmf_dl_C2)
% zpk(ftmf_dl_C2)
% step(ftmf_dl_C2)
% stepinfo(ftmf_2)

%Test settling time Response (5%)
Step_Data_C2=stepinfo(ftmf_dl_C2,'SettlingTimeThreshold',0.05);
Set_time_ms_C2=Step_Data_C2.SettlingTime*1e3;
disp(Set_time_ms_C2)


% %From MATLAB to Simulink
C1_dl_Num=cell2mat(C1_dl.Numerator);
C1_dl_Den=cell2mat(C1_dl.Denominator);

C2_dl_Num=cell2mat(C2_dl.Numerator);
C2_dl_Den=cell2mat(C2_dl.Denominator);



%% 3.Cascade Controller 2
%Voltage Control ========================
%Voltage Controller
G2_dl= R/(R*C*s + 1); %Voltage Control Transfer Function
z=5e3;
Kp_dlh=0.122;
C2_dlh=(s+z)*Kp_dlh/(s);
zpk(C2_dlh)

%FTMA analysis
ftma_dlh=G2_dl*C2_dlh;
% rlocus(ftma_dlh)


%FTMF analysis
ftmf_dlh=feedback(ftma_dlh,1);
% pzmap(ftmf_dlh)
% zpk(ftmf_dlh)
% step(ftmf_dlh)
% stepinfo(ftmf_dlh)

%Test settling time Response (5%)
Step_Data_dlh=stepinfo(ftmf_dlh,'SettlingTimeThreshold',0.05);
Set_time_ms_dlh=Step_Data_dlh.SettlingTime*1e3;
disp(Set_time_ms_dlh)

C2_dlh_Num=cell2mat(C2_dlh.Numerator);
C2_dlh_Den=cell2mat(C2_dlh.Denominator);


% % disp(Disp_message);
