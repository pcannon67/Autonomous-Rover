clear;
clc;

data = importdata('TFData.txt');

figure(1)
plot(-1*data);
grid on;

Tr = 0.25;
K = 1.8;

zeta = 2.8;
%wn = (2.16*zeta + 0.6)/2.16
wn = 2.5*pi/0.16;

G = tf((wn^2)*K,[1 2*zeta*wn wn^2]);

PO = 10;
Tr = 0.2;

Zeta = sqrt((log(PO/100)^2)/((log(PO/100)^2)+(pi^2)))
Wn = (2.16*Zeta + 0.6)/2.16

Ki = ((Wn^2 *(10*Zeta*Wn))/((Wn^2) * K))
Kp = 2*Zeta*Wn*(10*Zeta*Wn)

figure(2)
step(G*100)
grid on;
