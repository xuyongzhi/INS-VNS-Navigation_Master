%%%%%%% paper 绘图
function PaperPlot_9_14_paper2()

clc
clear all
close all

VOResult = importdata('VOResult.mat');
trueTraceResult = importdata('trueTraceResult.mat');
SINS_Result = importdata('SINS_Result.mat');
newdQTb_Result = importdata('INS_VNS_Result_new_dQTb.mat');
dQTb_Result = importdata('INS_VNS_Result_dQTb.mat');


[ VO_position,VO_attitude,VO_position_error,VO_attitude_error ] = resolve_result(VOResult) ;
[ true_position,true_attitude,~,~ ] = resolve_result(trueTraceResult) ;
[ SINS_position,SINS_attitude,SINS_position_error,SINS_attitude_error ] = resolve_result(SINS_Result) ;
[ newdQTb_position,newdQTb_attitude,newdQTb_position_error,newdQTb_attitude_error ] = resolve_result(newdQTb_Result) ;
[ dQTb_position,dQTb_attitude,dQTb_position_error,dQTb_attitude_error ] = resolve_result(dQTb_Result) ;

VOfre = VOResult{1}.frequency ;
INSfre = SINS_Result{1}.frequency ;

time_VO = ((1:length(VO_position))-1)/VOfre ;
time_INS = ((1:length(SINS_position))-1)/INSfre ;


%%% 将INS和真实数据的频率降低并对齐到VOfre，这样对比数据虽然会带来一点不真实，但是更能反映不同频率数据的对比精度
true_position = reduceDataFre( true_position,INSfre,VOfre) ;
true_attitude = reduceDataFre( true_attitude,INSfre,VOfre) ;
SINS_position = reduceDataFre( SINS_position,INSfre,VOfre) ;
SINS_attitude = reduceDataFre( SINS_attitude,INSfre,VOfre) ;
SINS_position_error = reduceDataFre( SINS_position_error,INSfre,VOfre) ;
SINS_attitude_error = reduceDataFre( SINS_attitude_error,INSfre,VOfre) ;

Nins = length(SINS_position) ;

path = [pwd,'\paperFigure'];
if ~isdir(path)
   mkdir(path); 
else
%     delete([path,'\*']);
end

lineWidth = 2 ;

labelFontSize = 13;
legFontsize=11;
axesFontsize = 11;
figurePosition = [300 300 450 412] ;


%% 真实轨迹
figure('name','真实轨迹')
set(gcf,'position',figurePosition) ;
set(cla,'fontsize',axesFontsize)
plot(true_position(1,:),true_position(2,:),'b','lineWidth',lineWidth);
xlabel('x（m）','fontsize',labelFontSize)
ylabel('y（m）','fontsize',labelFontSize)
hold on
plot(true_position(1,1),true_position(2,1),'o');
saveas(gcf,[path,'\真实轨迹.emf'])

figure('name','真实姿态')
set(gcf,'position',figurePosition) ;
set(cla,'fontsize',axesFontsize)
subplot(3,1,1)
plot(time_VO,true_attitude(1,:)*180/pi,'b','lineWidth',lineWidth);
ylabel('俯仰（°）','fontsize',labelFontSize)
subplot(3,1,2)
plot(time_VO,true_attitude(2,:)*180/pi,'b','lineWidth',lineWidth);
ylabel('横滚（°）','fontsize',labelFontSize)
subplot(3,1,3)
plot(time_VO,true_attitude(3,:)*180/pi,'b','lineWidth',lineWidth);
ylabel('航向（°）','fontsize',labelFontSize)

xlabel('时间(s)','fontsize',labelFontSize)
% saveas(gcf,[path,'\真实姿态.emf'])
% saveas(gcf,[path,'\真实姿态.fig'])

%% SINS+真实： xy轨迹  1
figure('name','SINS_trace')
set(gcf,'position',figurePosition) ;
set(cla,'fontsize',axesFontsize)
plot(true_position(1,:),true_position(2,:),'k','lineWidth',lineWidth);
hold on
plot(SINS_position(1,:),SINS_position(2,:),'g--','lineWidth',lineWidth);
% plot(SINS_position(1,1),SINS_position(2,1),'o')
lh=legend('真实','惯导');
set(lh,'fontsize',legFontsize);
% title('惯导轨迹','fontsize',fontsize)
xlabel('x（m）','fontsize',labelFontSize)
ylabel('y（m）','fontsize',labelFontSize)

% saveas(gcf,[path,'\SINS_trace.emf'])

%% SINS xyz 误差 合并 1
figure('name','SINS_positionError')
set(gcf,'position',figurePosition) ;
set(cla,'fontsize',axesFontsize)
plot(time_VO(1:Nins),SINS_position_error','lineWidth',lineWidth)
lh=legend('x','y','z');
set(lh,'fontsize',legFontsize);
% title('惯导位置误差','fontsize',fontsize)
xlabel('时间（s）','fontsize',labelFontSize)
ylabel('位置误差（m）','fontsize',labelFontSize)

% saveas(gcf,[path,'\SINS_positionError.emf'])

%% 真实+视觉+dQTb+newdQTb： xy轨迹 1
figure('name','导航轨迹')
set(gcf,'position',figurePosition) ;
set(cla,'fontsize',axesFontsize)
plot(true_position(1,:),true_position(2,:),'k','lineWidth',lineWidth);
hold on
plot(VO_position(1,:),VO_position(2,:),'b--','lineWidth',lineWidth);
hold on
plot(dQTb_position(1,:),dQTb_position(2,:),'r-.','lineWidth',lineWidth);
% hold on
% plot(newdQTb_position(1,:),newdQTb_position(2,:),'r--','lineWidth',lineWidth);
% plot(SINS_position(1,1),SINS_position(2,1),'o')

lh=legend('真实','视觉','组合');

set(lh,'fontsize',legFontsize);
% title('导航轨迹','fontsize',fontsize)
xlabel('x（m）','fontsize',labelFontSize)
ylabel('y（m）','fontsize',labelFontSize)

% saveas(gcf,[path,'\导航轨迹.emf'])
% %% 视觉+dQTb+newdQTb： x y z 位置误差 3
% 
%   %% x
% figure('name','x方向位置误差')
% set(gcf,'position',figurePosition) ;
% plot(time_VO,VO_position_error(1,:),'g','lineWidth',lineWidth);
% hold on
% plot(time_VO,dQTb_position_error(1,:),'b-.','lineWidth',lineWidth);
% % hold on
% % plot(time_VO,newdQTb_position_error(1,:),'r--','lineWidth',lineWidth);
% lh=legend('视觉','组合');
% set(lh,'fontsize',legFontsize);
% % title('x方向位置误差','fontsize',fontsize)
% xlabel('时间（s）','fontsize',labelFontSize)
% ylabel('x误差（m）','fontsize',labelFontSize)
% 
% % saveas(gcf,[path,'\x位置误差.emf'])
% %% y
% figure('name','y方向位置误差')
% set(gcf,'position',figurePosition) ;
% plot(time_VO,VO_position_error(2,:),'g','lineWidth',lineWidth);
% hold on
% plot(time_VO,dQTb_position_error(2,:),'b-.','lineWidth',lineWidth);
% % hold on
% % plot(time_VO,newdQTb_position_error(2,:),'r--','lineWidth',lineWidth);
% lh=legend('视觉','组合');
% set(lh,'fontsize',legFontsize);
% % title('y方向位置误差','fontsize',fontsize)
% xlabel('时间（s）','fontsize',labelFontSize)
% ylabel('y误差（m）','fontsize',labelFontSize)
% 
% % saveas(gcf,[path,'\y位置误差.emf'])
% %% z
% figure('name','z方向位置误差')
% set(gcf,'position',figurePosition) ;
% plot(time_VO,VO_position_error(3,:),'g','lineWidth',lineWidth);
% hold on
% plot(time_VO,dQTb_position_error(3,:),'b-.','lineWidth',lineWidth);
% % hold on
% % plot(time_VO,newdQTb_position_error(3,:),'r--','lineWidth',lineWidth);
% lh=legend('视觉','组合');
% set(lh,'fontsize',legFontsize);
% % title('z方向位置误差','fontsize',fontsize)
% xlabel('时间（s）','fontsize',labelFontSize)
% ylabel('z误差（m）','fontsize',labelFontSize)
% 
% % saveas(gcf,[path,'\z位置误差.emf'])

%% 位置误差 subplot
%% x
figure('name','x误差')
set(gcf,'position',figurePosition) ;
subplot(3,1,1)
plot(time_VO,SINS_position_error(1,:),'g','lineWidth',lineWidth);
ylabel('惯导','fontsize',labelFontSize)
subplot(3,1,2)
plot(time_VO,VO_position_error(1,:),'b','lineWidth',lineWidth);
ylabel('视觉','fontsize',labelFontSize)
subplot(3,1,3)
plot(time_VO,dQTb_position_error(1,:),'r','lineWidth',lineWidth);
ylabel('组合','fontsize',labelFontSize)
xlabel('时间（s）','fontsize',labelFontSize)

% saveas(gcf,[path,'\x位置误差.emf'])
%% y
figure('name','y误差')
set(gcf,'position',figurePosition) ;
subplot(3,1,1)
plot(time_VO,SINS_position_error(2,:),'g','lineWidth',lineWidth);
ylabel('惯导','fontsize',labelFontSize)
subplot(3,1,2)
plot(time_VO,VO_position_error(2,:),'b','lineWidth',lineWidth);
ylabel('视觉','fontsize',labelFontSize)
subplot(3,1,3)
plot(time_VO,dQTb_position_error(2,:),'r','lineWidth',lineWidth);
ylabel('组合','fontsize',labelFontSize)
xlabel('时间（s）','fontsize',labelFontSize)

% saveas(gcf,[path,'\y位置误差.emf'])
%% z
figure('name','z误差')
set(gcf,'position',figurePosition) ;
subplot(3,1,1)
plot(time_VO,SINS_position_error(3,:),'g','lineWidth',lineWidth);
ylabel('惯导','fontsize',labelFontSize)
subplot(3,1,2)
plot(time_VO,VO_position_error(3,:),'b','lineWidth',lineWidth);
ylabel('视觉','fontsize',labelFontSize)
subplot(3,1,3)
plot(time_VO,dQTb_position_error(3,:),'r','lineWidth',lineWidth);
ylabel('组合','fontsize',labelFontSize)
xlabel('时间（s）','fontsize',labelFontSize)

% saveas(gcf,[path,'\z位置误差.emf'])

%% 姿态误差
%% 俯仰误差
figure('name','俯仰误差')
set(gcf,'position',figurePosition) ;
subplot(3,1,1)
plot(time_VO(1:Nins),SINS_attitude_error(1,:)*180/pi,'g','lineWidth',lineWidth);
ylabel('惯导','fontsize',labelFontSize)
subplot(3,1,2)
plot(time_VO,VO_attitude_error(1,:)*180/pi,'b','lineWidth',lineWidth);
ylabel('视觉','fontsize',labelFontSize)
subplot(3,1,3)
plot(time_VO,dQTb_attitude_error(1,:)*180/pi,'r','lineWidth',lineWidth);
ylabel('组合','fontsize',labelFontSize)
xlabel('时间（s）','fontsize',labelFontSize)

%% 横滚误差
figure('name','横滚误差')
set(gcf,'position',figurePosition) ;
subplot(3,1,1)
plot(time_VO(1:Nins),SINS_attitude_error(2,:)*180/pi,'g','lineWidth',lineWidth);
ylabel('惯导','fontsize',labelFontSize)
subplot(3,1,2)
plot(time_VO,VO_attitude_error(2,:)*180/pi,'b','lineWidth',lineWidth);
ylabel('视觉','fontsize',labelFontSize)
subplot(3,1,3)
plot(time_VO,dQTb_attitude_error(2,:)*180/pi,'r','lineWidth',lineWidth);
ylabel('组合','fontsize',labelFontSize)
xlabel('时间（s）','fontsize',labelFontSize)

%% 航向误差
figure('name','航向误差')
set(gcf,'position',figurePosition) ;
subplot(3,1,1)
plot(time_VO(1:Nins),SINS_attitude_error(3,:)*180/pi,'g','lineWidth',lineWidth);
ylabel('惯导','fontsize',labelFontSize)
subplot(3,1,2)
plot(time_VO,VO_attitude_error(3,:)*180/pi,'b','lineWidth',lineWidth);
ylabel('视觉','fontsize',labelFontSize)
subplot(3,1,3)
plot(time_VO,dQTb_attitude_error(3,:)*180/pi,'r','lineWidth',lineWidth);
ylabel('组合','fontsize',labelFontSize)
xlabel('时间（s）','fontsize',labelFontSize)

% %% 视觉+SINS+dQTb+newdQTb：俯仰 横滚 航向 3
% %%% 俯仰
% figure('name','俯仰')
% set(gcf,'position',figurePosition) ;
% plot(time_VO,true_attitude(1,:)*180/pi,'k','lineWidth',lineWidth);
% hold on
% plot(time_VO(1:Nins),SINS_attitude(1,:)*180/pi,'c-.','lineWidth',lineWidth);
% hold on
% plot(time_VO,VO_attitude(1,:)*180/pi,'g','lineWidth',lineWidth);
% hold on
% plot(time_VO,dQTb_attitude(1,:)*180/pi,'b-.','lineWidth',lineWidth);
% hold on
% plot(time_VO,newdQTb_attitude(1,:)*180/pi,'r--','lineWidth',lineWidth);
% lh=legend('真实','惯导','视觉轨迹','传统组合','新组合');
% set(lh,'fontsize',legFontsize);
% % title('俯仰','fontsize',fontsize)
% xlabel('时间（s）','fontsize',labelFontSize)
% ylabel('俯仰（°）','fontsize',labelFontSize)
% 
% saveas(gcf,[path,'\俯仰.emf'])
%  %%% 横滚
% figure('name','横滚')
% set(gcf,'position',figurePosition) ;
% plot(time_VO,true_attitude(2,:)*180/pi,'k','lineWidth',lineWidth);
% hold on
% plot(time_VO(1:Nins),SINS_attitude(2,:)*180/pi,'c-.','lineWidth',lineWidth);
% hold on
% plot(time_VO,VO_attitude(2,:)*180/pi,'g','lineWidth',lineWidth);
% hold on
% plot(time_VO,dQTb_attitude(2,:)*180/pi,'b-.','lineWidth',lineWidth);
% hold on
% plot(time_VO,newdQTb_attitude(2,:)*180/pi,'r--','lineWidth',lineWidth);
% lh=legend('真实','惯导','视觉轨迹','传统组合','新组合');
% set(lh,'fontsize',legFontsize);
% % title('横滚','fontsize',fontsize)
% xlabel('时间（s）','fontsize',labelFontSize)
% ylabel('横滚（°）','fontsize',labelFontSize)
% 
% saveas(gcf,[path,'\横滚.emf'])
%  %%% 航向
% figure('name','航向')
% set(gcf,'position',figurePosition) ;
% plot(time_VO,true_attitude(3,:)*180/pi,'k','lineWidth',lineWidth);
% hold on
% plot(time_VO(1:Nins),SINS_attitude(3,:)*180/pi,'c-.','lineWidth',lineWidth);
% hold on
% plot(time_VO,VO_attitude(3,:)*180/pi,'g','lineWidth',lineWidth);
% hold on
% plot(time_VO,dQTb_attitude(3,:)*180/pi,'b-.','lineWidth',lineWidth);
% hold on
% plot(time_VO,newdQTb_attitude(3,:)*180/pi,'r--','lineWidth',lineWidth);
% lh=legend('真实','惯导','视觉轨迹','传统组合','新组合');
% set(lh,'fontsize',legFontsize);
% % title('航向','fontsize',fontsize)
% xlabel('时间（s）','fontsize',labelFontSize)
% ylabel('航向（°）','fontsize',labelFontSize)
% 
% saveas(gcf,[path,'\航向.emf'])
%% 视觉+SINS+dQTb+newdQTb：俯仰 横滚 航向 误差 3
%%% 俯仰误差
figure('name','俯仰误差')
set(gcf,'position',figurePosition) ;
plot(time_VO(1:Nins),SINS_attitude_error(1,:)*180/pi,'k-.','lineWidth',lineWidth);
hold on
plot(time_VO,VO_attitude_error(1,:)*180/pi,'g','lineWidth',lineWidth);
hold on
plot(time_VO,dQTb_attitude_error(1,:)*180/pi,'b-.','lineWidth',lineWidth);
lh=legend('惯导','视觉','组合');
set(lh,'fontsize',legFontsize);
% title('俯仰误差','fontsize',fontsize)
xlabel('时间（s）','fontsize',labelFontSize)
ylabel('俯仰误差（°）','fontsize',labelFontSize)

% saveas(gcf,[path,'\俯仰误差.emf'])
 %%% 横滚误差
figure('name','横滚误差')
set(gcf,'position',figurePosition) ;
plot(time_VO(1:Nins),SINS_attitude_error(2,:)*180/pi,'k-.','lineWidth',lineWidth);
hold on
plot(time_VO,VO_attitude_error(2,:)*180/pi,'g','lineWidth',lineWidth);
hold on
plot(time_VO,dQTb_attitude_error(2,:)*180/pi,'b-.','lineWidth',lineWidth);
lh=legend('惯导','视觉','组合');
set(lh,'fontsize',legFontsize);
% title('横滚误差','fontsize',fontsize)
xlabel('时间（s）','fontsize',labelFontSize)
ylabel('横滚误差（°）','fontsize',labelFontSize)

% saveas(gcf,[path,'\横滚误差.emf'])
 %%% 航向误差
figure('name','航向误差')
set(gcf,'position',figurePosition) ;
plot(time_VO(1:Nins),SINS_attitude_error(3,:)*180/pi,'k-.','lineWidth',lineWidth);
hold on
plot(time_VO,VO_attitude_error(3,:)*180/pi,'g','lineWidth',lineWidth);
hold on
plot(time_VO,dQTb_attitude_error(3,:)*180/pi,'b-.','lineWidth',lineWidth);
lh=legend('惯导','视觉','组合');
set(lh,'fontsize',legFontsize);
% title('航向误差','fontsize',fontsize)
xlabel('时间（s）','fontsize',labelFontSize)
ylabel('航向误差（°）','fontsize',labelFontSize)

% saveas(gcf,[path,'\航向误差.emf'])

%% 将封装好的 Result 拆开

function [ position,attitude,position_error,attitude_error ] = resolve_result(Result)

position_error=[];
attitude_error=[];
N = length(Result) ;

for i=1:N    
   switch  Result{i}.name
       
       case 'position(m)'
           position = Result{i}.data ;
       case 'attitude(°)'
           attitude = Result{i}.data*pi/180 ;
       case 'attitudeError(°)'
           attitude_error = Result{i}.data*pi/180 ;
       case 'positionError(m)'
           position_error = Result{i}.data ;
       otherwise
           
   end
end

%% 降低数据频率

function data_new = reduceDataFre( data_old,fre_old,fre_new)
N_old = size(data_old,2);
N_new = fix((N_old-1)*fre_new/fre_old) +1 ;
data_new = zeros(3,N_new) ;
for k=1:N_new
    k_old = fix((k-1)*fre_old/fre_new) +1 ;
    data_new(:,k) = data_old(:,k_old) ;
end

