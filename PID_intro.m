%% Introduction to PID-controller design
%  This introduction gives you a short overview how the different parts of
%  a PID-controller work.

%%%%%%%%%%%%%%%%%%%%%%%% NOTE %%%%%%%%%%%%%%%%%%%%%%%%%%
% Do not forget to add ypur system's transfer function before running the
% code!
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all
clc
clear all

%% mass, spring and damper values and transfer function
M = 20;                     % mass in kg
K = 400;                    % spring stiffness coeficient in N/m
C = 200;                    % damping coefficient in Ns/m    
s = tf('s');                % LaPlace parameter
t = 0:0.01:2;
%% plot the step response of the different controllers (P,PD,PI and PID)
KP = 0;
KI = 0;
KD = 0;
figure(3)
set(gcf,'Position',[100 100 600 600])
grid on
daten = line(0,0,'LineWidth',3);
title('Step response of spring-damper-mass system (START: no controller active) ','FontSize',12)
xlabel('Time (sec)','FontSize',12)
ylabel('Amplitude')
% xlim([-50 50])
ylim([-0.5 1.5])
% Slider erzeugen
KP_Slider = uicontrol('Style', 'slider',...
'Position', [100 75 120 20],'min',0,'max',200000,...
'value',KP,'String','Kp');
text(0.2,-0.33,'K_{p}=','FontSize',14)
KP_wert=text(0.4,-0.33,'test','FontSize',14);

KI_Slider = uicontrol('Style', 'slider',...
'Position', [250 75 120 20],'min',0,'max',200000,...
'value',KI,'String','Ki');
text(0.8,-0.33,'K_{i}=','FontSize',14)
KI_wert=text(1.0,-0.33,num2str(KI),'FontSize',14);

KD_Slider = uicontrol('Style', 'slider',...
'Position', [400 75 120 20],'min',0,'max',20000,...
'value',KD,'String','Kd');
text(1.5,-0.33,'K_{d}=','FontSize',14)
KD_wert=text(1.7,-0.33,num2str(KD),'FontSize',14);

M_Slider = uicontrol('Style', 'slider',...
'Position', [100 125 120 20],'min',1,'max',400,...
'value',M,'String','M');
text(0.2,-0.1,'M=','FontSize',14)
M_wert=text(0.4,-.1,num2str(M),'FontSize',14);

K_Slider = uicontrol('Style', 'slider',...
'Position', [250 125 120 20],'min',0,'max',25000,...
'value',K,'String','K');
text(0.8,-0.1,'K=','FontSize',14)
K_wert=text(1.0,-0.1,num2str(K),'FontSize',14);

C_Slider = uicontrol('Style', 'slider',...
'Position', [400 125 120 20],'min',0,'max',4500,...
'value',C,'String','C');
text(1.5,-0.1,'C=','FontSize',14)
C_wert=text(1.7,-0.1,num2str(C),'FontSize',14);

while(1)
KP = get(KP_Slider,'value');
KI = get(KI_Slider,'value');
KD = get(KD_Slider,'value');
M = get(M_Slider,'value');
K = get(K_Slider,'value');
C = get(C_Slider,'value');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Implement the transfer function of the equation of motion here:
% Use variable s for the first derivative of x and s^2 for second
% derivative of x
T = 1/(K+C*s+M*(s^2)); 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Ctrl = pid(KP,KI,KD);
S = feedback(Ctrl*T,1);
set(daten,'xdata',t,'ydata',step(S,t));
set(KP_wert,'string',num2str(KP))
set(KI_wert,'string',num2str(KI))
set(KD_wert,'string',num2str(KD))
set(M_wert,'string',num2str(M))
set(K_wert,'string',num2str(K))
set(C_wert,'string',num2str(C))
drawnow
end