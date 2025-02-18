function [Cbn]=FCbn(posture)
% ��������������� rad
format long
% st=posture(1);
% r=posture(2);
% fai=posture(3);
% 
% t11=cos(r)*cos(fai)-sin(r)*sin(st)*sin(fai);
% t12=cos(r)*sin(fai)+sin(r)*sin(st)*cos(fai);
% t13=-sin(r)*cos(st);
% t21=-cos(st)*sin(fai);
% t22=cos(st)*cos(fai);
% t23=sin(st);
% t31=sin(r)*cos(fai)+cos(r)*sin(st)*sin(fai);
% t32=sin(r)*sin(fai)-cos(r)*sin(st)*cos(fai);
% t33=cos(r)*cos(st);
% cnb=[t11,t12,t13;t21,t22,t23;t31,t32,t33];
% Cbn=cnb';

pitch=posture(1);
roll=posture(2);
head=posture(3);
Cbn(1, 1) = cos(roll) * cos(head) - sin(roll) * sin(pitch) * sin(head);
Cbn(1, 2) = -cos(pitch) * sin(head);
Cbn(1, 3) = sin(roll) * cos(head) + cos(roll) * sin(pitch) * sin(head);     
Cbn(2, 1) = cos(roll) * sin(head) + sin(roll) * sin(pitch) * cos(head);
Cbn(2, 2) = cos(pitch) * cos(head);
Cbn(2, 3) = sin(roll) * sin(head) - cos(roll) * sin(pitch) * cos(head);     
Cbn(3, 1) = -sin(roll) * cos(pitch); 
Cbn(3, 2) = sin(pitch);
Cbn(3, 3) = cos(roll) * cos(pitch);
% Cnb=Cbn';
