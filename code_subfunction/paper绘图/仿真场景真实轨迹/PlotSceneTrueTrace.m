%% 在视景仿真场景上绘制真实轨迹

trueTraceResult = importdata('trueTraceResult.mat');
position = trueTraceResult{1}.data ;

Sence_Simu = imread('Sence Simu.png') ;

A=[ 9 2.5  ];
Xmax = size(Sence_Simu,1) ;
Ymax = size(Sence_Simu,2) ;

figure
image(Sence_Simu)
hold on
plot(position(1,:)*A(1)+20,Xmax-position(2,:)*A(2)-10,'r','linewidth',2)

disp('OK')