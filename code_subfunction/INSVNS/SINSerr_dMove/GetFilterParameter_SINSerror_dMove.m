%% 导入初始滤波参数 P、Q、R_const 2014.9.2
%   dataSource 
%% kitti 2011_10_03_drive_0034 数据导航解算的滤波参数调节规律
%%% P0
% 1.位置和速度 P0 影响不大
% 2.加计 P0 影响不大 0.01 mg~10mg 感觉结果都一样
% 3.陀螺 P0 影响非常大，设置得非常小时导航效果比较好，比如
        %    0.01°/h。此时的协方差只会微小的收敛。设置为1°/h或10°/h时，陀螺常漂的估计协方差虽然收敛很好，但是导航结果很差。
        % 这个每调好（比较大）滤波效果马上就发散了
% 4.初始姿态角误差 P0 影响很大，设得尽量小导航效果好，协方差收敛图不好（看）。
%%% Q
%       理论上 加计和陀螺常漂微分方程应该是0，但是考虑到由于噪声不是完全白色的，我试着给一定的值看看影响
% 5.陀螺的 Q 影响比较大，而且还是置0效果最好
% 6.加计的 Q 影响稍微小，规律也不是那么明显，有时给大一点反而某些维的精度更高，但某些维精度又下去了。
        % 	如果简单起见可以置0，但要想在比较好的效果上达到更高的精度的话调这个会有些效果。
% 7.位置微分方程的 Q 理论上是0，实际上影响不大，不要取太大就好。        
% 8.速度微分方程的 Q 影响大。
% 9.失准角微分方程的 Q 要求比较小就行
%%% R
% 1.dRbb 的 R阵 尽量大的时候（如 1e-1 以上），组合的姿态就基本上取接近惯导
% 2.dTbb 的 R阵 
function [ P_ini,Q_const,R_ini,NavFilterParameter ] = GetFilterParameter_SINSerror_dMove( pg,ng,pa,na,NavFilterParameter )
%% 
dataSource = 'visual scence';
msgbox([dataSource,'. 滤波参数设置：传统误差方程']);
    switch dataSource
        case '2011_09_30_drive_0028'
           [ P_ini,Q_const,R_ini,NavFilterParameter ] = GetFilterParameter_SINSerror_dMove_2011_09_30_drive_0028_B( pg,ng,pa,na,NavFilterParameter ) ;
         %    [ P_ini,Q_const,R_ini,NavFilterParameter ] = GetFilterParameter_SINSerror_dMove_2011_09_30_drive_0028_A( pg,ng,pa,na,NavFilterParameter ) ;
        case '2011_10_03_drive_0034'
         	[ P_ini,Q_const,R_ini,NavFilterParameter ] = GetFilterParameter_SINSerror_dMove_2011_10_03_drive_0034_E( pg,ng,pa,na,NavFilterParameter );
        case 'visual scence'
         	[ P_ini,Q_const,R_ini,NavFilterParameter ] = GetFilterParameter_SINSerror_dMove_visualScene_B( pg,ng,pa,na,NavFilterParameter ) ;
        otherwise
            [ P_ini,Q_const,R_ini,NavFilterParameter ] = GetFilterParameter_SINSerror_dMove_2011_09_30_drive_0028_B( pg,ng,pa,na,NavFilterParameter ) ;
    end

    %% 轨迹A5 0.02HZ 用于paper(tradi)
%     	平面：	真实行程：202.66 m	导航行程:202.86 m	行程误差：0.2018 m (0.099575%)
% 		平面距原点最大误差：1.3853 m (0.68355%)
% 		平面终点位置误差：1.2375 m  (0.61062%) 
% 	空间：	真实行程：202.72 m	导航行程:202.95 m	行程误差：0.223 m (0.11%)
% 		空间终点位置误差：1.5333 m  (0.75637%) 
% 		空间 最大距远点最大误差：1.5438 m (0.76152%)
% 	各维 最大 位置误差(x、y、z)：(-1.3054,0.88397,-0.91156)m	(-1.1348%,0.85593%,-40.018%)
% 	各维 终点 位置误差(x、y、z)：(-1.1485,-0.4606,-0.90546)m	(-0.99846%,-0.44599%,-39.75%)
% 	姿态最大误差 (俯仰、横滚、航向):(0.024325,0.068104,0.532)deg
% 	姿态终点误差 (俯仰、横滚、航向):(-0.0084711,9.5191e-05,0.532)deg
% 
% 	初始、最终加计估计误差：(0  0  0  )、(59.9  4.49  0.095  ) ug
% 	初始、最终陀螺估计误差：(0  0  0  )、(-0.141  0.238  -0.299  ) °/h
% 
% 滤波参数：
% 	X(0)=( 0  0  0  0  0  0  0  0  0  4.85e-06  4.84e-06  4.84e-06  0.000162  0.000162  0.000162  0  0  0  0  0  0   )
% 	P(0)=( 2.35e-11  2.35e-11  2.35e-11  1e-06  1e-06  1e-06  1e-08  1e-08  1e-08  2.35e-11  2.35e-11  2.35e-11  9.6e-09  9.6e-09  9.6e-09  7.62e-05  7.62e-05  7.62e-05  0.0025  0.0025  0.0025   )
% 	Qk=( 2e-11  2e-11  2e-11  2e-18  2e-18  2e-18  0  0  0  0  0  0  0  0  0  1e-10  1e-10  1e-10  1e-08  1e-08  1e-08   )
% 	R(0)=( 10  10  10  1e-08  1e-08  1e-08   )
    
function [ P_ini,Q_const,R_ini,NavFilterParameter ] = GetFilterParameter_SINSerror_dMove_visualScene_C( pg,ng,pa,na,NavFilterParameter )
%%
szj = [ 1 1 1 ] * 1/3600*pi/180 *1 ;
vnsRDrift = [1 1 1]*0.5*pi/180 ;
vnsTDrift = [1 1 1]*0.05 ;
pg = [ 1 1 1 ]*pi/180/3600 * 1 ;        % 
pa = [ 1 1 1 ]*1e-6*9.8 *10 ;
P_ini = diag([(szj(1))^2,(szj(2))^2,(szj(3))^2,(1e-3)^2,(1e-3)^2,(1e-3)^2,(1e-4)^2,(1e-4)^2,(1e-4)^2,...
                (pg(1))^2,(pg(2))^2,(pg(3))^2,(pa(1))^2,(pa(2))^2,(pa(3))^2,...
                (vnsRDrift(1))^2,(vnsRDrift(2))^2,(vnsRDrift(3))^2,(vnsTDrift(1))^2,(vnsTDrift(2))^2,(vnsTDrift(3))^2 ]); %  15*15
 NavFilterParameter.P_ini_augment_dRdT =  sprintf('%1.1e ',P_ini) ;

  Q_const = diag([  2e-11 2e-11 2e-11 ...           % 失准角微分方程
                    2e-18 2e-18 2e-18...               % 速度微分方程
                    0  0  0  ...                    % 位置微分方程
                    0  0  0  ...                    % 陀螺常值微分方程
                    0  0  0  ...                    % 加计常值微分方程
                    1e-10 1e-10 1e-10 ...               % 视觉误差角微分方程
                    1e-8 1e-8 1e-8 ]);               	% 视觉平移误差微分方程     

 NavFilterParameter.Q_ini_augment_dRdT = sprintf('%1.1e ',Q_const) ;


if isfield(NavFilterParameter,'R_ini_augment_dRdT')
    R_list_input = {NavFilterParameter.R_ini_augment_dRdT} ;
else
    R_list_input = [];
end
R_list = [R_list_input,{'[[1 1 1]*1e1  [1 1 1]*1e-8]'...
                        '[[1 1 1]*1e1  [1 1 1]*1e-12]'...  % kitti 平面位置终点精度高
                        }];

[Selection,ok] = listdlg('PromptString','噪声方程阵R(前R[3x3]后T[3x1])-subQ\_subT:','SelectionMode','single','ListSize',[350,100],'ListString',R_list);
if ok==0    
    Selection = 1 ;
end
answer = inputdlg('噪声方程阵R(前R[3x3]后T[3x1])-subQ\_subT                     .','R_ini',1,R_list(Selection));
R_ini = diag(eval(answer{1})) ;   % R_const
NavFilterParameter.R_ini_augment_dRdT = answer{1} ;

%% 对轨迹A5 0.1HZ isTrueX0=1和=0 都 效果很好
% 	平面：	真实行程：203.87 m	导航行程:203.95 m	行程误差：0.079368 m (0.038931%)
% 		平面距原点最大误差：0.8242 m (0.40428%)
% 		平面终点位置误差：0.60388 m  (0.29621%) 
% 	空间：	真实行程：203.93 m	导航行程:204.3 m	行程误差：0.36525 m (0.1791%)
% 		空间终点位置误差：2.7629 m  (1.3548%) 
% 		空间 最大距远点最大误差：2.7894 m (1.3678%)
% 	各维 最大 位置误差(x、y、z)：(0.24462,-0.80357,-2.7222)m	(0.21262%,-0.76908%,-119.29%)
% 	各维 终点 位置误差(x、y、z)：(0.14837,-0.58537,-2.6961)m	(0.12896%,-0.56024%,-118.14%)
% 	姿态最大误差 (俯仰、横滚、航向):(0.0031298,0.004071,-0.26028)deg
% 	姿态终点误差 (俯仰、横滚、航向):(0.00057393,0.00062834,-0.080534)deg
% 
% 	初始、最终加计估计误差：(0  0  0  )、(24.5  -18.3  0.0381  ) ug
% 	初始、最终陀螺估计误差：(0  0  0  )、(0.00192  -0.00271  0.0429  ) °/h
% 
% 滤波参数：
% 	X(0)=( 0  0  0  0  0  0  0  0  0  4.86e-06  4.84e-06  4.85e-06  0.000162  0.000163  0.000162  0  0  0  0  0  0   )
% 	P(0)=( 2.35e-11  2.35e-11  2.35e-11  1e-06  1e-06  1e-06  1e-08  1e-08  1e-08  5.88e-12  5.88e-12  5.88e-12  9.6e-09  9.6e-09  9.6e-09  7.62e-05  7.62e-05  7.62e-05  0.0025  0.0025  0.0025   )
% 	Qk=( 2e-19  2e-19  2e-19  2e-08  2e-08  2e-08  0  0  0  0  0  0  0  0  0  1e-10  1e-10  1e-10  1e-08  1e-08  1e-08   )
% 	R(0)=( 1e+04  1e+04  1e+04  0.001  0.001  0.001   )
%% 对轨迹A5 0.02HZ isTrueX0=0 效果也不错
%   平面：	真实行程：202.66 m	导航行程:202.96 m	行程误差：0.30137 m (0.14871%)
% 		平面距原点最大误差：1.2264 m (0.60514%)
% 		平面终点位置误差：0.98611 m  (0.48659%) 
% 	空间：	真实行程：202.72 m	导航行程:203.03 m	行程误差：0.30893 m (0.15239%)
% 		空间终点位置误差：1.3201 m  (0.6512%) 
% 		空间 最大距远点最大误差：1.3386 m (0.66031%)
% 	各维 最大 位置误差(x、y、z)：(-0.96317,1.2118,-0.88298)m	(-0.8373%,1.1734%,-38.763%)
% 	各维 终点 位置误差(x、y、z)：(-0.93971,-0.29892,-0.87772)m	(-0.81691%,-0.28944%,-38.532%)
% 	姿态最大误差 (俯仰、横滚、航向):(0.015432,0.0093093,0.89852)deg
% 	姿态终点误差 (俯仰、横滚、航向):(0.0050735,0.0081683,0.79782)deg
% 
% 	初始、最终加计估计误差：(-100  -99.9  -99.9  )、(117  -128  0.0132  ) ug
% 	初始、最终陀螺估计误差：(-1  -0.998  -0.999  )、(0.00047  -0.00111  -0.428  ) °/h
% 
% 滤波参数：
% 	X(0)=( 0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0   )
% 	P(0)=( 2.35e-11  2.35e-11  2.35e-11  1e-06  1e-06  1e-06  1e-08  1e-08  1e-08  5.88e-12  5.88e-12  5.88e-12  9.6e-09  9.6e-09  9.6e-09  7.62e-05  7.62e-05  7.62e-05  0.0025  0.0025  0.0025   )
% 	Qk=( 2e-19  2e-19  2e-19  2e-08  2e-08  2e-08  0  0  0  0  0  0  0  0  0  1e-10  1e-10  1e-10  1e-08  1e-08  1e-08   )
% 	R(0)=( 1e+04  1e+04  1e+04  0.001  0.001  0.001   )
function [ P_ini,Q_const,R_ini,NavFilterParameter ] = GetFilterParameter_SINSerror_dMove_visualScene_B( pg,ng,pa,na,NavFilterParameter )
%%
szj = [ 1 1 1 ] * 1/3600*pi/180 *1 ;
vnsRDrift = [1 1 1]*0.5*pi/180 ;
vnsTDrift = [1 1 1]*0.05 ;
pg = [ 1 1 1 ]*pi/180/3600 * 0.5 ;        % 
pa = [ 1 1 1 ]*1e-6*9.8 *10 ;
P_ini = diag([(szj(1))^2,(szj(2))^2,(szj(3))^2,(1e-3)^2,(1e-3)^2,(1e-3)^2,(1e-4)^2,(1e-4)^2,(1e-4)^2,...
                (pg(1))^2,(pg(2))^2,(pg(3))^2,(pa(1))^2,(pa(2))^2,(pa(3))^2,...
                (vnsRDrift(1))^2,(vnsRDrift(2))^2,(vnsRDrift(3))^2,(vnsTDrift(1))^2,(vnsTDrift(2))^2,(vnsTDrift(3))^2 ]); %  15*15
 NavFilterParameter.P_ini_augment_dRdT =  sprintf('%1.1e ',P_ini) ;

  Q_const = diag([  2e-19 2e-19 2e-19 ...           % 失准角微分方程
                    2e-8 2e-8 2e-8...               % 速度微分方程
                    0  0  0  ...                    % 位置微分方程
                    0  0  0  ...                    % 陀螺常值微分方程
                    0  0  0  ...                    % 加计常值微分方程
                    1e-10 1e-10 1e-10 ...               % 视觉误差角微分方程
                    1e-8 1e-8 1e-8 ]);               	% 视觉平移误差微分方程     

 NavFilterParameter.Q_ini_augment_dRdT = sprintf('%1.1e ',Q_const) ;


if isfield(NavFilterParameter,'R_ini_augment_dRdT')
    R_list_input = {NavFilterParameter.R_ini_augment_dRdT} ;
else
    R_list_input = [];
end
R_list = [R_list_input,{'[[1 1 1]*1e4  [1 1 1]*1e-3]'...
                        '[[1 1 1]*1e1  [1 1 1]*1e-12]'...  % kitti 平面位置终点精度高
                        }];

[Selection,ok] = listdlg('PromptString','噪声方程阵R(前R[3x3]后T[3x1])-subQ\_subT:','SelectionMode','single','ListSize',[350,100],'ListString',R_list);
if ok==0    
    Selection = 1 ;
end
answer = inputdlg('噪声方程阵R(前R[3x3]后T[3x1])-subQ\_subT                     .','R_ini',1,R_list(Selection));
R_ini = diag(eval(answer{1})) ;   % R_const
NavFilterParameter.R_ini_augment_dRdT = answer{1} ;

%% 对S轨迹测试效果不错

function [ P_ini,Q_const,R_ini,NavFilterParameter ] = GetFilterParameter_SINSerror_dMove_visualScene_A( pg,ng,pa,na,NavFilterParameter )
%%
szj = [ 1 1 1 ] * 1/3600*pi/180 *1 ;
vnsRDrift = [1 1 1]*0.5*pi/180 ;
vnsTDrift = [1 1 1]*0.05 ;
pg = [ 1 1 1 ]*pi/180/3600 * 0.5 ;        % 
pa = [ 1 1 1 ]*1e-6*9.8 *10 ;
P_ini = diag([(szj(1))^2,(szj(2))^2,(szj(3))^2,(1e-3)^2,(1e-3)^2,(1e-3)^2,(1e-4)^2,(1e-4)^2,(1e-4)^2,...
                (pg(1))^2,(pg(2))^2,(pg(3))^2,(pa(1))^2,(pa(2))^2,(pa(3))^2,...
                (vnsRDrift(1))^2,(vnsRDrift(2))^2,(vnsRDrift(3))^2,(vnsTDrift(1))^2,(vnsTDrift(2))^2,(vnsTDrift(3))^2 ]); %  15*15
 NavFilterParameter.P_ini_augment_dRdT =  sprintf('%1.1e ',P_ini) ;

  Q_const = diag([  2e-19 2e-19 2e-19 ...           % 失准角微分方程
                    2e-8 2e-8 2e-8...               % 速度微分方程
                    0  0  0  ...                    % 位置微分方程
                    0  0  0  ...                    % 陀螺常值微分方程
                    0  0  0  ...                    % 加计常值微分方程
                    1e-10 1e-10 1e-10 ...               % 视觉误差角微分方程
                    1e-8 1e-8 1e-8 ]);               	% 视觉平移误差微分方程     

 NavFilterParameter.Q_ini_augment_dRdT = sprintf('%1.1e ',Q_const) ;


if isfield(NavFilterParameter,'R_ini_augment_dRdT')
    R_list_input = {NavFilterParameter.R_ini_augment_dRdT} ;
else
    R_list_input = [];
end
R_list = [R_list_input,{'[[1 1 1]*1e-1  [1 1 1]*1e-12]'...
                        '[[1 1 1]*1e1  [1 1 1]*1e-12]'...  % kitti 平面位置终点精度高
                        }];

[Selection,ok] = listdlg('PromptString','噪声方程阵R(前R[3x3]后T[3x1])-subQ\_subT:','SelectionMode','single','ListSize',[350,100],'ListString',R_list);
if ok==0    
    Selection = 1 ;
end
answer = inputdlg('噪声方程阵R(前R[3x3]后T[3x1])-subQ\_subT                     .','R_ini',1,R_list(Selection));
R_ini = diag(eval(answer{1})) ;   % R_const
NavFilterParameter.R_ini_augment_dRdT = answer{1} ;

%%  kitti 2011_10_03_drive_0034 位置 姿态 精度 高
% 	平面：	真实行程：5060.3 m	导航行程:5065.4 m	行程误差：5.0216 m (0.099234%)
% 		平面距原点最大误差：14.248 m (0.28157%)
% 		平面终点位置误差：9.0699 m  (0.17923%) 
% 	空间：	真实行程：5069.5 m	导航行程:5072.4 m	行程误差：2.898 m (0.057166%)
% 		空间终点位置误差：12.308 m  (0.24278%) 
% 		空间 最大距远点最大误差：71.658 m (1.4135%)
% 	各维 最大 位置误差(x、y、z)：(-8.2428,12.175,-71.396)m	(-0.2797%,0.33439%,-28.255%)
% 	各维 终点 位置误差(x、y、z)：(-8.0746,4.1307,-8.3198)m	(-0.274%,0.11344%,-3.2925%)
% 	姿态最大误差 (俯仰、横滚、航向):(-9.5919,13.608,4.4225)deg
% 	姿态终点误差 (俯仰、横滚、航向):(-2.4857,-3.0771,0.29184)deg
% 
% 	初始、最终加计估计误差：(-50  -50  -50  )、(1.33e+04  -5.48e+04  2.06e+04  ) ug
% 	初始、最终陀螺估计误差：(-5  -5  -5  )、(-4.1  -4.88  0.808  ) °/h
% 
% 滤波参数：
% 	X(0)=( 0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0   )
% 	P(0)=( 2.35e-11  2.35e-11  2.35e-11  1e-08  1e-08  1e-08  1e-08  1e-08  1e-08  2.35e-13  2.35e-13  2.35e-13  9.6e-13  9.6e-13  9.6e-13  7.62e-05  7.62e-05  7.62e-05  0.0025  0.0025  0.0025   )
% 	Qk=( 2e-15  2e-15  2e-15  2e-32  2e-32  2e-32  1e-24  1e-24  1e-24  0  0  0  1e-05  1e-05  0.0001  1e-10  1e-10  1e-10  1e-08  1e-08  1e-08   )
% 	R(0)=( 1e+08  1e+08  1e+08  1e-08  1e-08  1e-08   )
function [ P_ini,Q_const,R_ini,NavFilterParameter ] = GetFilterParameter_SINSerror_dMove_2011_10_03_drive_0034_E( pg,ng,pa,na,NavFilterParameter )
%%
szj = [ 1 1 1 ] * 1/3600*pi/180 *1 ;
vnsRDrift = [1 1 1]*0.5*pi/180 ;
vnsTDrift = [1 1 1]*0.05 ;
pg = [ 1 1 1 ]*pi/180/3600 * 0.02 ;        % 
pa = [ 1 1 1 ]*1e-6*9.8 *0.1 ;
P_ini = diag([(szj(1))^2,(szj(2))^2,(szj(3))^2,(1e-4)^2,(1e-4)^2,(1e-4)^2,(1e-4)^2,(1e-4)^2,(1e-4)^2,...
                (pg(1))^2,(pg(2))^2,(pg(3))^2,(pa(1))^2,(pa(2))^2,(pa(3))^2,...
                (vnsRDrift(1))^2,(vnsRDrift(2))^2,(vnsRDrift(3))^2,(vnsTDrift(1))^2,(vnsTDrift(2))^2,(vnsTDrift(3))^2 ]); %  15*15
 NavFilterParameter.P_ini_augment_dRdT =  sprintf('%1.1e ',P_ini) ;

  Q_const = diag([  2e-15 2e-15 2e-15 ...     	% 失准角微分方程
                2e-32 2e-32 2e-32 ...           % 速度微分方程
                1e-24 1e-24 1e-24 ...           % 位置微分方程
                0   0   0         ...           % 陀螺常值微分方程
                1e-5 1e-5 1e-5    ...                % 加计常值微分方程
                1e-10 1e-10 1e-10 ...               % 视觉误差角微分方程
                1e-8 1e-8 1e-8 ]);               	% 视觉平移误差微分方程     

 NavFilterParameter.Q_ini_augment_dRdT = sprintf('%1.1e ',Q_const) ;


if isfield(NavFilterParameter,'R_ini_augment_dRdT')
    R_list_input = {NavFilterParameter.R_ini_augment_dRdT} ;
else
    R_list_input = [];
end
R_list = [R_list_input,{'[[1 1 1]*1e8  [1 1 1]*1e-8]'...
                        '[[1 1 1]*1e1  [1 1 1]*1e-12]'...  % kitti 平面位置终点精度高
                        }];

[Selection,ok] = listdlg('PromptString','噪声方程阵R(前R[3x3]后T[3x1])-subQ\_subT:','SelectionMode','single','ListSize',[350,100],'ListString',R_list);
if ok==0    
    Selection = 1 ;
end
answer = inputdlg('噪声方程阵R(前R[3x3]后T[3x1])-subQ\_subT                     .','R_ini',1,R_list(Selection));
R_ini = diag(eval(answer{1})) ;   % R_const
NavFilterParameter.R_ini_augment_dRdT = answer{1} ;
    
%% 2011_09_30_drive_0028 

function [ P_ini,Q_const,R_ini,NavFilterParameter ] = GetFilterParameter_SINSerror_dMove_2011_09_30_drive_0028_C( pg,ng,pa,na,NavFilterParameter )
%%
szj = [ 1 1 1 ] * 1/3600*pi/180 *1 ;
vnsRDrift = [1 1 1]*0.5*pi/180 ;
vnsTDrift = [1 1 1]*0.05 ;
pg = [ 1 1 1 ]*pi/180/3600 * 0.01 ;        % 
pa = [ 1 1 1 ]*1e-6*9.8 *1 ;
P_ini = diag([(szj(1))^2,(szj(2))^2,(szj(3))^2,(1e-2)^2,(1e-2)^2,(1e-2)^2,(1e-2)^2,(1e-2)^2,(1e-2)^2,...
                (pg(1))^2,(pg(2))^2,(pg(3))^2,(pa(1))^2,(pa(2))^2,(pa(3))^2,...
                (vnsRDrift(1))^2,(vnsRDrift(2))^2,(vnsRDrift(3))^2,(vnsTDrift(1))^2,(vnsTDrift(2))^2,(vnsTDrift(3))^2 ]); %  15*15
% P_ini = diag([(szj1)^2,(szj2)^2,(szj3)^2,(1e-4)^2,(1e-4)^2,(1e-4)^2,(1e-5)^2,(1e-5)^2,(1e-5)^2,...
%                 (pg(1))^2,(pg(2))^2,(pg(3))^2,(pa(1))^2,(pa(2))^2,(pa(3))^2,...
%                 (vnsRDrift(1))^2,(vnsRDrift(2))^2,(vnsRDrift(3))^2,(vnsTDrift(1))^2,(vnsTDrift(2))^2,(vnsTDrift(3))^2 ]); %  15*15
 NavFilterParameter.P_ini_augment_dRdT =  sprintf('%1.1e ',P_ini) ;

  Q_const = diag([  2e-12 2e-12 2e-12 ...         % 失准角微分方程
                2e-6 2e-6 2e-6...               % 速度微分方程
                1e-18 1e-18 1e-18 ...           % 位置微分方程
                1e-37 1e-37 1e-37 ...           % 陀螺常值微分方程
                0 0 0 ...                       % 加计常值微分方程
                1e-10 1e-10 1e-10 ...                       % 视觉误差角微分方程
                1e-8 1e-8 1e-8 ]);                       % 视觉平移误差微分方程     

 NavFilterParameter.Q_ini_augment_dRdT = sprintf('%1.1e ',Q_const) ;


if isfield(NavFilterParameter,'R_ini_augment_dRdT')
    R_list_input = {NavFilterParameter.R_ini_augment_dRdT} ;
else
    R_list_input = [];
end
R_list = [R_list_input,{'[[1 1 1]*1e-1  [1 1 1]*1e-9]'...
                        '[[1 1 1]*1e-1  [1 1 1]*1e-12]'...                      % kitti
                        }];

[Selection,ok] = listdlg('PromptString','噪声方程阵R(前R[3x3]后T[3x1])-subQ\_subT:','SelectionMode','single','ListSize',[350,100],'ListString',R_list);
if ok==0    
    Selection = 1 ;
end
answer = inputdlg('噪声方程阵R(前R[3x3]后T[3x1])-subQ\_subT                     .','R_ini',1,R_list(Selection));
R_ini = diag(eval(answer{1})) ;   % R_const
NavFilterParameter.R_ini_augment_dRdT = answer{1} ;

%%  空间位置精度特别高的一组  2011_09_30_drive_0028
% 	平面：	真实行程：4128.9 m	导航行程:4115.4 m	行程误差：-13.573 m (-0.32874%)
% 		平面距原点最大误差：24.955 m (0.6044%)
% 		平面终点位置误差：17.81 m  (0.43136%) 
% 	空间：	真实行程：4206.8 m	导航行程:4123.5 m	行程误差：-83.236 m (-1.9786%)
% 		空间终点位置误差：20.515 m  (0.48767%) 
% 		空间 最大距远点最大误差：84.193 m (2.0014%)
% 	各维 最大 位置误差(x、y、z)：(-23.774,24.32,-82.425)m	(-1.077%,0.88445%,-32.602%)
% 	各维 终点 位置误差(x、y、z)：(-13.677,11.409,10.182)m	(-0.61958%,0.41491%,4.0272%)
% 	姿态最大误差 (俯仰、横滚、航向):(6.5736,-7.8969,4.1421)deg
% 	姿态终点误差 (俯仰、横滚、航向):(0.0044517,-0.87462,0.58754)deg
% 
% 	初始、最终加计估计误差：(-200  -200  -200  )、(-200  -200  -200  ) ug
% 	初始、最终陀螺估计误差：(-7  -7  -7  )、(48.9  1.99  -4.85  ) °/h
% 
% 滤波参数：
% 	X(0)=( 0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0   )
% 	P(0)=( 2.35e-11  2.35e-11  2.35e-11  1e-08  1e-08  1e-08  1e-10  1e-10  1e-10  5.88e-14  5.88e-14  5.88e-14  9.6e-15  9.6e-15  9.6e-15  7.62e-05  7.62e-05  7.62e-05  0.0025  0.0025  0.0025   )
% 	Qk=( 2e-22  2e-22  2e-22  2e-06  2e-06  1e-09  1e-38  1e-38  1e-38  1e-37  1e-37  1e-37  0  0  0  1e-10  1e-10  1e-10  1e-08  1e-08  1e-08   )
% 	R(0)=( 1e+04  1e+04  1e+04  0.0001  0.0001  0.0001   )
% 不进行：IMU数据的常值漂移补偿
%  IMU常值漂移初值 置0
    %%   2011_09_30_drive_0028 用于paper(tradi)
% 	平面：	真实行程：4128.9 m	导航行程:4114.3 m	行程误差：-14.626 m (-0.35424%)
% 		平面距原点最大误差：25.261 m (0.6118%)
% 		平面终点位置误差：18.845 m  (0.45641%) 
% 	空间：	真实行程：4206.8 m	导航行程:4117.2 m	行程误差：-89.55 m (-2.1287%)
% 		空间终点位置误差：58.739 m  (1.3963%) 
% 		空间 最大距远点最大误差：66.094 m (1.5711%)
% 	各维 最大 位置误差(x、y、z)：(-20.863,23.743,64.131)m	(-0.94517%,0.86346%,25.366%)
% 	各维 终点 位置误差(x、y、z)：(-14.265,12.314,55.634)m	(-0.64625%,0.44782%,22.005%)
% 	姿态最大误差 (俯仰、横滚、航向):(7.6729,-8.9172,4.1421)deg
% 	姿态终点误差 (俯仰、横滚、航向):(1.6485,-1.4636,0.8099)deg
% 
% 	初始、最终加计估计误差：(-200  -200  -200  )、(-200  -200  -200  ) ug
% 	初始、最终陀螺估计误差：(-7  -7  -7  )、(-6.11  -6.86  -6.95  ) °/h
% 
% 滤波参数：
% 	X(0)=( 0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0   )
% 	P(0)=( 2.35e-11  2.35e-11  2.35e-11  1e-08  1e-08  1e-08  1e-10  1e-10  1e-10  2.35e-15  2.35e-15  2.35e-15  9.6e-15  9.6e-15  9.6e-15  7.62e-05  7.62e-05  7.62e-05  0.0025  0.0025  0.0025   )
% 	Qk=( 2e-20  2e-20  2e-20  1e-05  1e-05  1e-06  1e-38  1e-38  1e-38  1e-37  1e-37  1e-37  0  0  0  1e-10  1e-10  1e-10  1e-08  1e-08  1e-08   )
% 	R(0)=( 1e+04  1e+04  1e+04  0.0004  0.0004  0.0002   )
% 不进行：IMU数据的常值漂移补偿
%  IMU常值漂移初值 置0

function [ P_ini,Q_const,R_ini,NavFilterParameter ] = GetFilterParameter_SINSerror_dMove_2011_09_30_drive_0028_B( pg,ng,pa,na,NavFilterParameter )
%%
szj1 = 1/3600*pi/180 * 1;
szj2 = 1/3600*pi/180 * 1;
szj3 = 1/3600*pi/180 * 1;
vnsRDrift = [1 1 1]*0.5*pi/180 ;
vnsTDrift = [1 1 1]*0.05 ;
pg = [ 1 1 1 ]*pi/180/3600 * 0.01 ;        %  这个设大一点 马上就发散了
pa = [ 1 1 1 ]*1e-6*9.8 *0.01 ;
P_ini = diag([(szj1)^2,(szj2)^2,(szj3)^2,(1e-4)^2,(1e-4)^2,(1e-4)^2,(1e-5)^2,(1e-5)^2,(1e-5)^2,...
                (pg(1))^2,(pg(2))^2,(pg(3))^2,(pa(1))^2,(pa(2))^2,(pa(3))^2,...
                (vnsRDrift(1))^2,(vnsRDrift(2))^2,(vnsRDrift(3))^2,(vnsTDrift(1))^2,(vnsTDrift(2))^2,(vnsTDrift(3))^2 ]); %  15*15
 NavFilterParameter.P_ini_augment_dRdT =  sprintf('%1.1e ',P_ini) ;

  Q_const = diag([  2e-20 2e-20 2e-20 ...           % 失准角微分方程
                    1e-5 1e-5 1e-6...               % 速度微分方程
                    1e-38 1e-38 1e-38 ...           % 位置微分方程
                    1e-37 1e-37 1e-37 ...           % 陀螺常值微分方程
                    0 0 0 ...                       % 加计常值微分方程
                    1e-10 1e-10 1e-10 ...                       % 视觉误差角微分方程
                    1e-8 1e-8 1e-8 ]);                       % 视觉平移误差微分方程     

 NavFilterParameter.Q_ini_augment_dRdT = sprintf('%1.1e ',Q_const) ;


if isfield(NavFilterParameter,'R_ini_augment_dRdT')
    R_list_input = {NavFilterParameter.R_ini_augment_dRdT} ;
else
    R_list_input = [];
end
R_list = [R_list_input,{'[[1 1 1]*1e4  [4 4 2]*1e-4]'...
                        '[[1 1 1]*1e5  [1 1 1]*1e-7]'...                      % kitti
                        }];

[Selection,ok] = listdlg('PromptString','噪声方程阵R(前R[3x3]后T[3x1])-subQ\_subT:','SelectionMode','single','ListSize',[350,100],'ListString',R_list);
if ok==0    
    Selection = 1 ;
end
answer = inputdlg('噪声方程阵R(前R[3x3]后T[3x1])-subQ\_subT                     .','R_ini',1,R_list(Selection));
R_ini = diag(eval(answer{1})) ;   % R_const
NavFilterParameter.R_ini_augment_dRdT = answer{1} ;

%% kitti 2011_09_30_drive_0028 

function [ P_ini,Q_const,R_ini,NavFilterParameter ] = GetFilterParameter_SINSerror_dMove_2011_09_30_drive_0028_A( pg,ng,pa,na,NavFilterParameter )
%%
szj1 = 1/3600*pi/180 * 10;
szj2 = 1/3600*pi/180 * 10;
szj3 = 1/3600*pi/180 * 10;
vnsRDrift = [1 1 1]*0.5*pi/180 ;
vnsTDrift = [1 1 1]*0.05 ;
pg = [ 1 1 1 ]*pi/180/3600 * 0.1 ;        % 
pa = [ 1 1 1 ]*1e-6*9.8 *0.1 ;
P_ini = diag([(szj1)^2,(szj2)^2,(szj3)^2,(1e-4)^2,(1e-4)^2,(1e-4)^2,(1e-5)^2,(1e-5)^2,(1e-5)^2,...
                (pg(1))^2,(pg(2))^2,(pg(3))^2,(pa(1))^2,(pa(2))^2,(pa(3))^2,...
                (vnsRDrift(1))^2,(vnsRDrift(2))^2,(vnsRDrift(3))^2,(vnsTDrift(1))^2,(vnsTDrift(2))^2,(vnsTDrift(3))^2 ]); %  15*15
 NavFilterParameter.P_ini_augment_dRdT =  sprintf('%1.1e ',P_ini) ;

  Q_const = diag([  2e-12 2e-12 2e-12 ...         % 失准角微分方程
                2e-6 2e-6 2e-6...            % 速度微分方程
                1e-18 1e-18 1e-18 ...           % 位置微分方程
                1e-37 1e-37 1e-37 ...           % 陀螺常值微分方程
                0 0 0 ...                       % 加计常值微分方程
                1e-10 1e-10 1e-10 ...                       % 视觉误差角微分方程
                1e-8 1e-8 1e-8 ]);                       % 视觉平移误差微分方程     

 NavFilterParameter.Q_ini_augment_dRdT = sprintf('%1.1e ',Q_const) ;


if isfield(NavFilterParameter,'R_ini_augment_dRdT')
    R_list_input = {NavFilterParameter.R_ini_augment_dRdT} ;
else
    R_list_input = [];
end
R_list = [R_list_input,{'[[1 1 1]*1e-1  [1 1 1]*1e-7]'...
                        '[[1 1 1]*1e-1  [1 1 1]*1e-12]'...                      % kitti
                        }];

[Selection,ok] = listdlg('PromptString','噪声方程阵R(前R[3x3]后T[3x1])-subQ\_subT:','SelectionMode','single','ListSize',[350,100],'ListString',R_list);
if ok==0    
    Selection = 1 ;
end
answer = inputdlg('噪声方程阵R(前R[3x3]后T[3x1])-subQ\_subT                     .','R_ini',1,R_list(Selection));
R_ini = diag(eval(answer{1})) ;   % R_const
NavFilterParameter.R_ini_augment_dRdT = answer{1} ;

    
%% 位置 姿态精度高 kitti 2011_10_03_drive_0034 
% 	平面：	真实行程：5060.3 m	导航行程:5064.9 m	行程误差：4.5475 m (0.089865%)
% 		平面距原点最大误差：12.406 m (0.24517%)
% 		平面终点位置误差：8.9197 m  (0.17627%) 
% 	空间：	真实行程：5069.5 m	导航行程:5072 m	行程误差：2.5262 m (0.049831%)
% 		空间终点位置误差：14.151 m  (0.27914%) 
% 		空间 最大距远点最大误差：71.971 m (1.4197%)
% 	各维 最大 位置误差(x、y、z)：(-8.5404,10.708,-71.723)m	(-0.2898%,0.29408%,-28.384%)
% 	各维 终点 位置误差(x、y、z)：(-8.5404,2.5736,-10.986)m	(-0.2898%,0.07068%,-4.3476%)
% 	姿态最大误差 (俯仰、横滚、航向):(-9.6556,13.316,-4.677)deg
% 	姿态终点误差 (俯仰、横滚、航向):(-2.686,-2.8763,0.47874)deg
% 
% 	初始、最终加计估计误差：(-50  -50  -50  )、(3.23e+05  -3.72e+06  -2.18e+04  ) ug
% 	初始、最终陀螺估计误差：(-5  -5  -5  )、(-2.41  -2.55  -1.6  ) °/h
% 
% 滤波参数：
% 	X(0)=( 0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0   )
% 	P(0)=( 2.35e-11  2.35e-11  2.35e-11  0.0001  0.0001  0.0001  0.0001  0.0001  0.0001  2.35e-15  2.35e-15  2.35e-15  9.6e-11  9.6e-11  9.6e-11  7.62e-05  7.62e-05  7.62e-05  0.0025  0.0025  0.0025   )
% 	Qk=( 2e-35  2e-35  2e-35  2e-22  2e-22  2e-22  1e-24  1e-24  1e-24  0  0  0  1e-05  1e-05  0.0001  1e-10  1e-10  1e-10  1e-08  1e-08  1e-08   )
% 	R(0)=( 0.1  0.1  0.1  1e-12  1e-12  1e-12   )

function [ P_ini,Q_const,R_ini,NavFilterParameter ] = GetFilterParameter_SINSerror_dMove_2011_10_03_drive_0034_D( pg,ng,pa,na,NavFilterParameter )
%%
szj = [ 1 1 1 ] * 1/3600*pi/180 *1 ;
vnsRDrift = [1 1 1]*0.5*pi/180 ;
vnsTDrift = [1 1 1]*0.05 ;
pg = [ 1 1 1 ]*pi/180/3600 * 0.01 ;        % 
pa = [ 1 1 1 ]*1e-6*9.8 *1 ;
P_ini = diag([(szj(1))^2,(szj(2))^2,(szj(3))^2,(1e-2)^2,(1e-2)^2,(1e-2)^2,(1e-2)^2,(1e-2)^2,(1e-2)^2,...
                (pg(1))^2,(pg(2))^2,(pg(3))^2,(pa(1))^2,(pa(2))^2,(pa(3))^2,...
                (vnsRDrift(1))^2,(vnsRDrift(2))^2,(vnsRDrift(3))^2,(vnsTDrift(1))^2,(vnsTDrift(2))^2,(vnsTDrift(3))^2 ]); %  15*15
 NavFilterParameter.P_ini_augment_dRdT =  sprintf('%1.1e ',P_ini) ;

  Q_const = diag([  2e-35 2e-35 2e-35 ...     	% 失准角微分方程
                2e-22 2e-22 2e-22...               % 速度微分方程
                1e-24 1e-24 1e-24 ...           % 位置微分方程
                0   0   0         ...           % 陀螺常值微分方程
                1e-5 1e-5 1e-4 ...       	    % 加计常值微分方程
                1e-10 1e-10 1e-10 ...               % 视觉误差角微分方程
                1e-8 1e-8 1e-8 ]);               	% 视觉平移误差微分方程     

 NavFilterParameter.Q_ini_augment_dRdT = sprintf('%1.1e ',Q_const) ;


if isfield(NavFilterParameter,'R_ini_augment_dRdT')
    R_list_input = {NavFilterParameter.R_ini_augment_dRdT} ;
else
    R_list_input = [];
end
R_list = [R_list_input,{'[[1 1 1]*1e-1  [1 1 1]*1e-12]'...
                        '[[1 1 1]*1e1  [1 1 1]*1e-12]'...  % kitti 平面位置终点精度高
                        }];

[Selection,ok] = listdlg('PromptString','噪声方程阵R(前R[3x3]后T[3x1])-subQ\_subT:','SelectionMode','single','ListSize',[350,100],'ListString',R_list);
if ok==0    
    Selection = 1 ;
end
answer = inputdlg('噪声方程阵R(前R[3x3]后T[3x1])-subQ\_subT                     .','R_ini',1,R_list(Selection));
R_ini = diag(eval(answer{1})) ;   % R_const
NavFilterParameter.R_ini_augment_dRdT = answer{1} ;

%% kitti 2011_10_03_drive_0034  

function [ P_ini,Q_const,R_ini,NavFilterParameter ] = GetFilterParameter_SINSerror_dMove_2011_10_03_drive_0034_B( pg,ng,pa,na,NavFilterParameter )
%%
szj = [ 1 1 1 ] * 1/3600*pi/180 *1 ;
vnsRDrift = [1 1 1]*0.5*pi/180 ;
vnsTDrift = [1 1 1]*0.05 ;
pg = [ 1 1 1 ]*pi/180/3600 * 0.01 ;        % 
pa = [ 1 1 1 ]*1e-6*9.8 *1 ;
P_ini = diag([(szj(1))^2,(szj(2))^2,(szj(3))^2,(1e-2)^2,(1e-2)^2,(1e-2)^2,(1e-2)^2,(1e-2)^2,(1e-2)^2,...
                (pg(1))^2,(pg(2))^2,(pg(3))^2,(pa(1))^2,(pa(2))^2,(pa(3))^2,...
                (vnsRDrift(1))^2,(vnsRDrift(2))^2,(vnsRDrift(3))^2,(vnsTDrift(1))^2,(vnsTDrift(2))^2,(vnsTDrift(3))^2 ]); %  15*15
 NavFilterParameter.P_ini_augment_dRdT =  sprintf('%1.1e ',P_ini) ;

  Q_const = diag([  2e-12 2e-12 2e-12 ...     	% 失准角微分方程
                2e-6 2e-6 2e-6...               % 速度微分方程
                1e-18 1e-18 1e-18 ...           % 位置微分方程
                0   0   0         ...           % 陀螺常值微分方程
                1e-8 1e-8 1e-9 ...       	    % 加计常值微分方程
                1e-10 1e-10 1e-10 ...               % 视觉误差角微分方程
                1e-8 1e-8 1e-8 ]);               	% 视觉平移误差微分方程     

 NavFilterParameter.Q_ini_augment_dRdT = sprintf('%1.1e ',Q_const) ;


if isfield(NavFilterParameter,'R_ini_augment_dRdT')
    R_list_input = {NavFilterParameter.R_ini_augment_dRdT} ;
else
    R_list_input = [];
end
R_list = [R_list_input,{'[[1 1 1]*1e-1  [1 1 1]*1e-7]'...
                        '[[1 1 1]*1e-1  [1 1 1]*1e-12]'...  % kitti
                        }];

[Selection,ok] = listdlg('PromptString','噪声方程阵R(前R[3x3]后T[3x1])-subQ\_subT:','SelectionMode','single','ListSize',[350,100],'ListString',R_list);
if ok==0    
    Selection = 1 ;
end
answer = inputdlg('噪声方程阵R(前R[3x3]后T[3x1])-subQ\_subT                     .','R_ini',1,R_list(Selection));
R_ini = diag(eval(answer{1})) ;   % R_const
NavFilterParameter.R_ini_augment_dRdT = answer{1} ;

%% kitti 2011_10_03_drive_0034 
%  用于paper(tradi)
% 	平面：	真实行程：5060.3 m	导航行程:5065.9 m	行程误差：5.5874 m (0.11041%)
% 		平面距原点最大误差：17.697 m (0.34972%)
% 		平面终点位置误差：17.697 m  (0.34972%) 
% 	空间：	真实行程：5069.5 m	导航行程:5072.2 m	行程误差：2.7401 m (0.054051%)
% 		空间终点位置误差：23.868 m  (0.47081%) 
% 		空间 最大距远点最大误差：37.004 m (0.72994%)
% 	各维 最大 位置误差(x、y、z)：(-16.755,14.352,-35.765)m	(-0.56854%,0.39417%,-14.154%)
% 	各维 终点 位置误差(x、y、z)：(-16.755,5.6979,-16.015)m	(-0.56854%,0.15649%,-6.3378%)
% 	姿态最大误差 (俯仰、横滚、航向):(-10.269,13.511,4.6395)deg
% 	姿态终点误差 (俯仰、横滚、航向):(-6.8189,-2.7589,1.3288)deg
% 
% 	初始、最终加计估计误差：(-50  -50  -50  )、(-43.4  -34.3  -50  ) ug
% 	初始、最终陀螺估计误差：(-5  -5  -5  )、(-5.39  17.9  -6.21  ) °/h
% 
% 滤波参数：
% 	X(0)=( 0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0   )
% 	P(0)=( 2.35e-13  2.35e-13  2.35e-13  0.0001  0.0001  0.0001  0.0001  0.0001  0.0001  2.35e-13  2.35e-13  2.35e-13  9.6e-11  9.6e-11  9.6e-11  7.62e-05  7.62e-05  7.62e-05  0.0025  0.0025  0.0025   )
% 	Qk=( 2e-12  2e-12  2e-12  2e-06  2e-06  2e-06  1e-18  1e-18  1e-18  1e-37  1e-37  1e-37  0  0  0  1e-10  1e-10  1e-10  1e-08  1e-08  1e-08   )
% 	R(0)=( 10  10  10  1e-08  1e-08  1e-08   )
function [ P_ini,Q_const,R_ini,NavFilterParameter ] = GetFilterParameter_SINSerror_dMove_2011_10_03_drive_0034_A( pg,ng,pa,na,NavFilterParameter )
%%
szj = [ 1 1 1 ] * 1/3600*pi/180 *0.1 ;
vnsRDrift = [1 1 1]*0.5*pi/180 ;
vnsTDrift = [1 1 1]*0.05 ;
pg = [ 1 1 1 ]*pi/180/3600 * 0.1 ;        % 
pa = [ 1 1 1 ]*1e-6*9.8 *1 ;
P_ini = diag([(szj(1))^2,(szj(2))^2,(szj(3))^2,(1e-2)^2,(1e-2)^2,(1e-2)^2,(1e-2)^2,(1e-2)^2,(1e-2)^2,...
                (pg(1))^2,(pg(2))^2,(pg(3))^2,(pa(1))^2,(pa(2))^2,(pa(3))^2,...
                (vnsRDrift(1))^2,(vnsRDrift(2))^2,(vnsRDrift(3))^2,(vnsTDrift(1))^2,(vnsTDrift(2))^2,(vnsTDrift(3))^2 ]); %  15*15
 NavFilterParameter.P_ini_augment_dRdT =  sprintf('%1.1e ',P_ini) ;

  Q_const = diag([  2e-12 2e-12 2e-12 ...     	% 失准角微分方程
                2e-6 2e-6 2e-6...               % 速度微分方程
                1e-18 1e-18 1e-18 ...           % 位置微分方程
                1e-37 1e-37 1e-37 ...           % 陀螺常值微分方程
                0 0 0 ...                       % 加计常值微分方程
                1e-10 1e-10 1e-10 ...                       % 视觉误差角微分方程
                1e-8 1e-8 1e-8 ]);                       % 视觉平移误差微分方程     

 NavFilterParameter.Q_ini_augment_dRdT = sprintf('%1.1e ',Q_const) ;


if isfield(NavFilterParameter,'R_ini_augment_dRdT')
    R_list_input = {NavFilterParameter.R_ini_augment_dRdT} ;
else
    R_list_input = [];
end
R_list = [R_list_input,{'[[1 1 1]*1e1  [1 1 1]*1e-8]'...
                        '[[1 1 1]*1e-1  [1 1 1]*1e-12]'...  % kitti
                        }];

[Selection,ok] = listdlg('PromptString','噪声方程阵R(前R[3x3]后T[3x1])-subQ\_subT:','SelectionMode','single','ListSize',[350,100],'ListString',R_list);
if ok==0    
    Selection = 1 ;
end
answer = inputdlg('噪声方程阵R(前R[3x3]后T[3x1])-subQ\_subT                     .','R_ini',1,R_list(Selection));
R_ini = diag(eval(answer{1})) ;   % R_const
NavFilterParameter.R_ini_augment_dRdT = answer{1} ;

