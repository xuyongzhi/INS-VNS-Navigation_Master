%% 导入初始滤波参数 P、Q、R_const 2014.9.2
%   dataSource 
% 默认：isTrueX0=0
%% kitti 2011_10_03_drive_0034 数据导航解算的滤波参数调节规律
%%% P0
% 1.位置和速度 P0 影响不大
% 2.加计 P0 影响不大 0.01 mg~10mg 感觉结果都一样
% 3.陀螺 P0 影响非常大，设置得非常小时导航效果比较好，比如
        %    0.01°/h。此时的协方差只会微小的收敛。设置为1°/h或10°/h时，陀螺常漂的估计协方差虽然收敛很好，但是导航结果很差。
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
function [ P_ini,Q_const,R_ini,NavFilterParameter ] = GetFilterParameter_SINSerror_dMove_new( pg,ng,pa,na,NavFilterParameter )
%% 
dataSource = 'visual scence' ;
msgbox([dataSource,'. 滤波参数设置：新 误差方程']);
    switch dataSource
        case '2011_09_30_drive_0028'
           [ P_ini,Q_const,R_ini,NavFilterParameter ] = GetFilterParameter_SINSerror_dMove_2011_09_30_drive_0028_E( pg,ng,pa,na,NavFilterParameter ) ;
        case '2011_10_03_drive_0034'
           [ P_ini,Q_const,R_ini,NavFilterParameter ] = GetFilterParameter_SINSerror_dMove_2011_10_03_drive_0034_B( pg,ng,pa,na,NavFilterParameter ) ;
        case 'visual scence'
            [ P_ini,Q_const,R_ini,NavFilterParameter ] = GetFilterParameter_SINSerror_dMove_visualScene_B( pg,ng,pa,na,NavFilterParameter ) ;
        otherwise
            [ P_ini,Q_const,R_ini,NavFilterParameter ] = GetFilterParameter_SINSerror_dMove_2011_09_30_drive_0028( pg,ng,pa,na,NavFilterParameter ) ;
    end

function [ P_ini,Q_const,R_ini,NavFilterParameter ] = GetFilterParameter_SINSerror_dMove_visualScene_D( pg,ng,pa,na,NavFilterParameter )
%%
szj = [ 1 1 1 ] * 1/3600*pi/180 *1 ;
vnsRDrift = [1 1 0.5]*pi/180  ;
vnsTDrift = [8 40 -8]*1e-3 ;
pg = [ 1 1 1 ]*pi/180/3600 * 0.7 ;        % 
pa = [ 1 1 1 ]*1e-6*9.8 *30 ;
P_ini = diag([(szj(1))^2,(szj(2))^2,(szj(3))^2,(1e-3)^2,(1e-3)^2,(1e-3)^2,(1e-4)^2,(1e-4)^2,(1e-4)^2,...
                (pg(1))^2,(pg(2))^2,(pg(3))^2,(pa(1))^2,(pa(2))^2,(pa(3))^2,...
                (vnsRDrift(1))^2,(vnsRDrift(2))^2,(vnsRDrift(3))^2,(vnsTDrift(1))^2,(vnsTDrift(2))^2,(vnsTDrift(3))^2 ]); %  15*15
 NavFilterParameter.P_ini_augment_dRdT =  sprintf('%1.1e ',P_ini) ;

  Q_const = diag([  2e-24 2e-24 2e-24 ...           % 失准角微分方程
                    2e-4 2e-4 2e-4...               % 速度微分方程
                    0  0  0  ...                    % 位置微分方程
                    0  0  0  ...                    % 陀螺常值微分方程
                    0  0  0  ...                    % 加计常值微分方程
                    1e0 1e0 1e0 ...               % 视觉误差角微分方程
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

    %% 对 轨迹A5 0.1HZ isTrueX0=0 效果特别好
%     	平面：	真实行程：203.87 m	导航行程:203.53 m	行程误差：-0.33368 m (-0.16368%)
% 		平面距原点最大误差：0.69543 m (0.34112%)
% 		平面终点位置误差：0.1625 m  (0.079711%) 
% 	空间：	真实行程：203.93 m	导航行程:203.72 m	行程误差：-0.21727 m (-0.10654%)
% 		空间终点位置误差：2.7127 m  (1.3302%) 
% 		空间 最大距远点最大误差：2.7237 m (1.3356%)
% 	各维 最大 位置误差(x、y、z)：(0.29045,0.69035,-2.7206)m	(0.25246%,0.66071%,-119.22%)
% 	各维 终点 位置误差(x、y、z)：(-0.15607,0.045258,-2.7078)m	(-0.13566%,0.043315%,-118.66%)
% 	姿态最大误差 (俯仰、横滚、航向):(0.0097787,0.0067189,-0.053744)deg
% 	姿态终点误差 (俯仰、横滚、航向):(0.00034738,0.0010161,-0.012997)deg
% 
% 	初始、最终加计估计误差：(-99.8  -100  -100  )、(25.3  -18.3  -0.101  ) ug
% 	初始、最终陀螺估计误差：(-1  -0.997  -1  )、(0.000822  -0.00281  0.00677  ) °/h
% 
% 滤波参数：
% 	X(0)=( 0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0   )
% 	P(0)=( 2.35e-11  2.35e-11  2.35e-11  1e-06  1e-06  1e-06  1e-08  1e-08  1e-08  9.4e-13  9.4e-13  9.4e-13  9.6e-09  9.6e-09  9.6e-09  7.62e-05  7.62e-05  7.62e-05  0.0025  0.0025  0.0025   )
% 	Qk=( 2e-20  2e-20  2e-20  2e-08  2e-08  2e-08  0  0  0  0  0  0  0  0  0  1e-10  1e-10  1e-10  1e-08  1e-08  1e-08   )
% 	R(0)=( 1e+07  1e+07  1e+07  1e-05  1e-05  1e-05   )
function [ P_ini,Q_const,R_ini,NavFilterParameter ] = GetFilterParameter_SINSerror_dMove_visualScene_C( pg,ng,pa,na,NavFilterParameter )
%%
szj = [ 1 1 1 ] * 1/3600*pi/180 *1 ;
vnsRDrift = [1 1 1]*0.5*pi/180 ;
vnsTDrift = [1 1 1]*0.05 ;
pg = [ 1 1 1 ]*pi/180/3600 * 0.2 ;        % 
pa = [ 1 1 1 ]*1e-6*9.8 *10 ;
P_ini = diag([(szj(1))^2,(szj(2))^2,(szj(3))^2,(1e-3)^2,(1e-3)^2,(1e-3)^2,(1e-4)^2,(1e-4)^2,(1e-4)^2,...
                (pg(1))^2,(pg(2))^2,(pg(3))^2,(pa(1))^2,(pa(2))^2,(pa(3))^2,...
                (vnsRDrift(1))^2,(vnsRDrift(2))^2,(vnsRDrift(3))^2,(vnsTDrift(1))^2,(vnsTDrift(2))^2,(vnsTDrift(3))^2 ]); %  15*15
 NavFilterParameter.P_ini_augment_dRdT =  sprintf('%1.1e ',P_ini) ;

  Q_const = diag([  2e-20 2e-20 2e-20 ...       % 失准角微分方程
                    2e-8 2e-8 2e-8...           % 速度微分方程
                    0  0  0 ...                	% 位置微分方程
                    0  0  0 ...                 % 陀螺常值微分方程
                    0  0  0 ...                 % 加计常值微分方程
                    1e-10 1e-10 1e-10 ...               % 视觉误差角微分方程
                    1e-8 1e-8 1e-8 ]);               	% 视觉平移误差微分方程     

 NavFilterParameter.Q_ini_augment_dRdT = sprintf('%1.1e ',Q_const) ;


if isfield(NavFilterParameter,'R_ini_augment_dRdT')
    R_list_input = {NavFilterParameter.R_ini_augment_dRdT} ;
else
    R_list_input = [];
end
R_list = [R_list_input,{'[[1 1 1]*1e7  [1 1 1]*1e-5]'...
                        '[[1 1 1]*1e5  [1 1 1]*1e-8]'...  % kitti 平面位置终点精度高
                        }];      

[Selection,ok] = listdlg('PromptString','噪声方程阵R(前R[3x3]后T[3x1])-subQ\_subT:','SelectionMode','single','ListSize',[350,100],'ListString',R_list);
if ok==0    
    Selection = 1 ;
end
answer = inputdlg('噪声方程阵R(前R[3x3]后T[3x1])-subQ\_subT                     .','R_ini',1,R_list(Selection));
R_ini = diag(eval(answer{1})) ;   % R_const
NavFilterParameter.R_ini_augment_dRdT = answer{1} ;

    %% 对 A5 0.02HZ（isTrueX0=1） 效果特别好
%     	平面：	真实行程：202.66 m	导航行程:202.91 m	行程误差：0.25506 m (0.12586%)
% 		平面距原点最大误差：0.93428 m (0.46101%)
% 		平面终点位置误差：0.15921 m  (0.078559%) 
% 	空间：	真实行程：202.72 m	导航行程:202.99 m	行程误差：0.26293 m (0.1297%)
% 		空间终点位置误差：0.89119 m  (0.4396%) 
% 		空间 最大距远点最大误差：1.0753 m (0.53045%)
% 	各维 最大 位置误差(x、y、z)：(-0.47781,0.87809,-0.88261)m	(-0.41538%,0.85024%,-38.747%)
% 	各维 终点 位置误差(x、y、z)：(-0.15908,-0.0063288,-0.87685)m	(-0.13829%,-0.0061281%,-38.494%)
% 	姿态最大误差 (俯仰、横滚、航向):(0.01236,0.01141,-0.22065)deg
% 	姿态终点误差 (俯仰、横滚、航向):(0.0084589,0.0070482,-0.22065)deg
% 
% 	初始、最终加计估计误差：(0  0  0  )、(79.1  -114  -0.0483  ) ug
% 	初始、最终陀螺估计误差：(0  0  0  )、(0.00636  -0.01  0.119  ) °/h
% 
% 滤波参数：
% 	X(0)=( 0  0  0  0  0  0  0  0  0  4.85e-06  4.84e-06  4.84e-06  0.000162  0.000162  0.000162  0  0  0  0  0  0   )
% 	P(0)=( 2.35e-11  2.35e-11  2.35e-11  1e-06  1e-06  1e-06  1e-08  1e-08  1e-08  5.88e-12  5.88e-12  5.88e-12  9.6e-09  9.6e-09  9.6e-09  7.62e-05  7.62e-05  7.62e-05  0.0025  0.0025  0.0025   )
% 	Qk=( 2e-19  2e-19  2e-19  2e-05  2e-05  2e-05  0  0  0  0  0  0  0  0  0  1e-10  1e-10  1e-10  1e-08  1e-08  1e-08   )
% 	R(0)=( 1e+04  1e+04  1e+04  1e-07  1e-07  1e-07   )
    
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
                    2e-5 2e-5 2e-5...               % 速度微分方程
                    0  0  0  ...                    % 位置微分方程
                    0  0  0  ...                    % 陀螺常值微分方程
                    0  0  0  ...                    % 加计常值微分方程
                    1e-4 1e-4 1e-4 ...               % 视觉误差角微分方程
                    1e-8 1e-8 1e-8 ]);               	% 视觉平移误差微分方程     

 NavFilterParameter.Q_ini_augment_dRdT = sprintf('%1.1e ',Q_const) ;


if isfield(NavFilterParameter,'R_ini_augment_dRdT')
    R_list_input = {NavFilterParameter.R_ini_augment_dRdT} ;
else
    R_list_input = [];
end
R_list = [R_list_input,{'[[1 1 1]*1e4  [1 1 1]*1e-7]'...
                        '[[1 1 1]*1e1  [1 1 1]*1e-12]'...  % kitti 平面位置终点精度高
                        }];     

[Selection,ok] = listdlg('PromptString','噪声方程阵R(前R[3x3]后T[3x1])-subQ\_subT:','SelectionMode','single','ListSize',[350,100],'ListString',R_list);
if ok==0    
    Selection = 1 ;
end
answer = inputdlg('噪声方程阵R(前R[3x3]后T[3x1])-subQ\_subT                     .','R_ini',1,R_list(Selection));
R_ini = diag(eval(answer{1})) ;   % R_const
NavFilterParameter.R_ini_augment_dRdT = answer{1} ;


%% 对S轨迹还不错  
% 	平面：	真实行程：540 m	导航行程:539.53 m	行程误差：-0.46409 m (-0.085942%)
% 		平面距原点最大误差：3.2205 m (0.59639%)
% 		平面终点位置误差：3.0089 m  (0.5572%) 
% 	空间：	真实行程：540 m	导航行程:539.56 m	行程误差：-0.43685 m (-0.080899%)
% 		空间终点位置误差：3.6781 m  (0.68114%) 
% 		空间 最大距远点最大误差：3.7906 m (0.70197%)
% 	各维 最大 位置误差(x、y、z)：(-3.1525,-0.71611,-2.1155)m	(-2.2267%,-0.139%,-Inf%)
% 	各维 终点 位置误差(x、y、z)：(-2.9404,-0.63829,-2.1155)m	(-2.0769%,-0.1239%,-Inf%)
% 	姿态最大误差 (俯仰、横滚、航向):(0.0067024,0.006762,-0.89525)deg
% 	姿态终点误差 (俯仰、横滚、航向):(0.0050308,0.00645,-0.20391)deg
% 
% 	初始、最终加计估计误差：(0  0  0  )、(28.4  -105  0.0277  ) ug
% 	初始、最终陀螺估计误差：(0  0  0  )、(0.00157  -0.000234  0.0412  ) °/h
% 
% 滤波参数：
% 	X(0)=( 0  0  0  0  0  0  0  0  0  4.86e-06  4.85e-06  4.85e-06  0.000162  0.000162  0.000163  0  0  0  0  0  0   )
% 	P(0)=( 2.35e-11  2.35e-11  2.35e-11  1e-06  1e-06  1e-06  1e-08  1e-08  1e-08  2.35e-13  2.35e-13  2.35e-13  9.6e-09  9.6e-09  9.6e-09  7.62e-05  7.62e-05  7.62e-05  0.0025  0.0025  0.0025   )
% 	Qk=( 2e-19  2e-19  2e-19  2e-08  2e-08  2e-08  0  0  0  0  0  0  0  0  0  1e-10  1e-10  1e-10  1e-08  1e-08  1e-08   )
% 	R(0)=( 10  10  10  1e-12  1e-12  1e-12   )
%% 对 A5 0.1HZ isTrueX0=0 还不错 （姿态还不够满意）
% 	平面：	真实行程：203.87 m	导航行程:204.2 m	行程误差：0.3341 m (0.16388%)
% 		平面距原点最大误差：0.69401 m (0.34042%)
% 		平面终点位置误差：0.57088 m  (0.28003%) 
% 	空间：	真实行程：203.93 m	导航行程:204.53 m	行程误差：0.60048 m (0.29445%)
% 		空间终点位置误差：2.791 m  (1.3686%) 
% 		空间 最大距远点最大误差：2.8136 m (1.3797%)
% 	各维 最大 位置误差(x、y、z)：(0.26543,-0.66352,-2.7575)m	(0.23071%,-0.63503%,-120.84%)
% 	各维 终点 位置误差(x、y、z)：(-0.010376,-0.57079,-2.732)m	(-0.009019%,-0.54628%,-119.72%)
% 	姿态最大误差 (俯仰、横滚、航向):(0.0080135,-0.0081431,0.3746)deg
% 	姿态终点误差 (俯仰、横滚、航向):(-0.00077435,0.00211,0.23826)deg
% 
% 	初始、最终加计估计误差：(-99.8  -100  -100  )、(32  -17.3  -0.208  ) ug
% 	初始、最终陀螺估计误差：(-1  -0.997  -1  )、(-0.00211  -0.00278  -0.127  ) °/h
% 
% 滤波参数：
% 	X(0)=( 0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0   )
% 	P(0)=( 2.35e-11  2.35e-11  2.35e-11  1e-06  1e-06  1e-06  1e-08  1e-08  1e-08  2.35e-13  2.35e-13  2.35e-13  9.6e-09  9.6e-09  9.6e-09  7.62e-05  7.62e-05  7.62e-05  0.0025  0.0025  0.0025   )
% 	Qk=( 2e-19  2e-19  2e-19  2e-08  2e-08  2e-08  0  0  0  0  0  0  0  0  0  1e-10  1e-10  1e-10  1e-08  1e-08  1e-08   )
% 	R(0)=( 10  10  10  1e-12  1e-12  1e-12   )

function [ P_ini,Q_const,R_ini,NavFilterParameter ] = GetFilterParameter_SINSerror_dMove_visualScene_A( pg,ng,pa,na,NavFilterParameter )
%%
szj = [ 1 1 1 ] * 1/3600*pi/180 *1 ;
vnsRDrift = [1 1 1]*0.5*pi/180 ;
vnsTDrift = [1 1 1]*0.05 ;
pg = [ 1 1 1 ]*pi/180/3600 * 0.1 ;        % 
pa = [ 1 1 1 ]*1e-6*9.8 *10 ;
P_ini = diag([(szj(1))^2,(szj(2))^2,(szj(3))^2,(1e-3)^2,(1e-3)^2,(1e-3)^2,(1e-4)^2,(1e-4)^2,(1e-4)^2,...
                (pg(1))^2,(pg(2))^2,(pg(3))^2,(pa(1))^2,(pa(2))^2,(pa(3))^2,...
                (vnsRDrift(1))^2,(vnsRDrift(2))^2,(vnsRDrift(3))^2,(vnsTDrift(1))^2,(vnsTDrift(2))^2,(vnsTDrift(3))^2 ]); %  15*15
 NavFilterParameter.P_ini_augment_dRdT =  sprintf('%1.1e ',P_ini) ;

  Q_const = diag([  2e-19 2e-19 2e-19 ...       % 失准角微分方程
                    2e-8 2e-8 2e-8...           % 速度微分方程
                    0  0  0 ...                	% 位置微分方程
                    0  0  0 ...                 % 陀螺常值微分方程
                    0  0  0 ...                 % 加计常值微分方程
                    1e-10 1e-10 1e-10 ...               % 视觉误差角微分方程
                    1e-8 1e-8 1e-8 ]);               	% 视觉平移误差微分方程     

 NavFilterParameter.Q_ini_augment_dRdT = sprintf('%1.1e ',Q_const) ;


if isfield(NavFilterParameter,'R_ini_augment_dRdT')
    R_list_input = {NavFilterParameter.R_ini_augment_dRdT} ;
else
    R_list_input = [];
end
R_list = [R_list_input,{'[[1 1 1]*1e1  [1 1 1]*1e-12]'...
                        '[[1 1 1]*1e1  [1 1 1]*1e-12]'...  % kitti 平面位置终点精度高
                        }];     

[Selection,ok] = listdlg('PromptString','噪声方程阵R(前R[3x3]后T[3x1])-subQ\_subT:','SelectionMode','single','ListSize',[350,100],'ListString',R_list);
if ok==0    
    Selection = 1 ;
end
answer = inputdlg('噪声方程阵R(前R[3x3]后T[3x1])-subQ\_subT                     .','R_ini',1,R_list(Selection));
R_ini = diag(eval(answer{1})) ;   % R_const
NavFilterParameter.R_ini_augment_dRdT = answer{1} ;

%% 2011_09_30_drive_0028 位置 姿态 精度都高  （空间的位置精度不行）
% *** new_dQTb 解算误差：
% 	平面：	真实行程：4128.9 m	导航行程:4124.4 m	行程误差：-4.54 m (-0.10996%)
% 		平面距原点最大误差：31.397 m (0.76042%)
% 		平面终点位置误差：5.7822 m  (0.14004%) 
% 	空间：	真实行程：4206.8 m	导航行程:4125 m	行程误差：-81.823 m (-1.945%)
% 		空间终点位置误差：42.511 m  (1.0105%) 
% 		空间 最大距远点最大误差：56.603 m (1.3455%)
% 	各维 最大 位置误差(x、y、z)：(22.716,21.674,-47.097)m	(1.0291%,0.78823%,-18.628%)
% 	各维 终点 位置误差(x、y、z)：(0.83971,5.7209,42.116)m	(0.038041%,0.20805%,16.658%)
% 	姿态最大误差 (俯仰、横滚、航向):(-5.8685,-7.1184,1.8065)deg
% 	姿态终点误差 (俯仰、横滚、航向):(-1.0455,0.28911,0.0028473)deg
% 
% 	初始、最终加计估计误差：(-50  -50  -50  )、(-46.4  -46.2  -48.4  ) ug
% 	初始、最终陀螺估计误差：(-5  -5  -5  )、(-4.57  -4.85  -4.94  ) °/h
% 
% 滤波参数：
% 	X(0)=( 0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0   )
% 	P(0)=( 2.35e-11  2.35e-11  2.35e-11  0.0001  0.0001  0.0001  0.0001  0.0001  0.0001  2.35e-15  2.35e-15  2.35e-15  9.6e-11  9.6e-11  9.6e-11  7.62e-05  7.62e-05  7.62e-05  0.0025  0.0025  0.0025   )
% 	Qk=( 2e-12  2e-12  2e-12  2e-06  2e-06  2e-06  1e-18  1e-18  1e-18  1e-37  1e-37  1e-37  0  0  0  1e-10  1e-10  1e-10  1e-08  1e-08  1e-08   )
% 	R(0)=( 1  1  1  1e-12  1e-12  1e-12   )

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

    %% 位置精度高 2011_09_30_drive_0028
%     	平面：	真实行程：4128.9 m	导航行程:4124.6 m	行程误差：-4.2882 m (-0.10386%)
% 		平面距原点最大误差：31.152 m (0.75449%)
% 		平面终点位置误差：5.5584 m  (0.13462%) 
% 	空间：	真实行程：4206.8 m	导航行程:4125 m	行程误差：-81.745 m (-1.9432%)
% 		空间终点位置误差：36.792 m  (0.87458%) 
% 		空间 最大距远点最大误差：58.119 m (1.3816%)
% 	各维 最大 位置误差(x、y、z)：(22.211,21.844,-49.065)m	(1.0062%,0.79439%,-19.407%)
% 	各维 终点 位置误差(x、y、z)：(1.1253,5.4433,36.37)m	(0.050979%,0.19796%,14.385%)
% 	姿态最大误差 (俯仰、横滚、航向):(-5.5966,-7.0534,1.8043)deg
% 	姿态终点误差 (俯仰、横滚、航向):(-1.1447,0.19832,-0.26548)deg
function [ P_ini,Q_const,R_ini,NavFilterParameter ] = GetFilterParameter_SINSerror_dMove_2011_09_30_drive_0028_B( pg,ng,pa,na,NavFilterParameter )
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
R_list = [R_list_input,{'[[1 1 1]*1e-1  [1 1 1]*1e-8]'...
                        '[[1 1 1]*1e-1  [1 1 1]*1e-12]'...                      % kitti
                        }];      

[Selection,ok] = listdlg('PromptString','噪声方程阵R(前R[3x3]后T[3x1])-subQ\_subT:','SelectionMode','single','ListSize',[350,100],'ListString',R_list);
if ok==0    
    Selection = 1 ;
end
answer = inputdlg('噪声方程阵R(前R[3x3]后T[3x1])-subQ\_subT                     .','R_ini',1,R_list(Selection));
R_ini = diag(eval(answer{1})) ;   % R_const
NavFilterParameter.R_ini_augment_dRdT = answer{1} ;

%% kitti 2011_09_30_drive_0028 效果比较好的一组数据
% INS_VNS_new_dQTb 解算误差：
% 	平面：	真实行程：4128.9 m	导航行程:4124.6 m	行程误差：-4.3509 m (-0.10538%)
% 		平面距原点最大误差：31.563 m (0.76445%)
% 		平面终点位置误差：6.1607 m  (0.14921%) 
% 	空间：	真实行程：4206.8 m	导航行程:4125 m	行程误差：-81.823 m (-1.945%)
% 		空间终点位置误差：36.102 m  (0.85818%) 
% 		空间 最大距远点最大误差：59.077 m (1.4043%)
% 	各维 最大 位置误差(x、y、z)：(22.915,21.706,-49.938)m	(1.0381%,0.78937%,-19.752%)
% 	各维 终点 位置误差(x、y、z)：(2.869,5.4519,35.572)m	(0.12997%,0.19827%,14.07%)
% 	姿态最大误差 (俯仰、横滚、航向):(-5.5383,-7.0616,1.7843)deg
% 	姿态终点误差 (俯仰、横滚、航向):(-1.1275,0.19229,-0.2733)deg
% 
% 	初始、最终加计估计误差：(-50  -50  -50  )、(-50  -49.9  -50  ) ug
% 	初始、最终陀螺估计误差：(-5  -5  -5  )、(10.9  -1.53  -2.32  ) °/h
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

  Q_const = diag([  2e-12 2e-12 2e-12 ...           % 失准角微分方程
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

%% kitti 2011_09_30_drive_0028 姿态位置 精度高 3D位置不好
% 	平面：	真实行程：4128.9 m	导航行程:4123.8 m	行程误差：-5.1043 m (-0.12362%)
% 		平面距原点最大误差：31.432 m (0.76126%)
% 		平面终点位置误差：5.9971 m  (0.14525%) 
% 	空间：	真实行程：4206.8 m	导航行程:4125 m	行程误差：-81.823 m (-1.945%)
% 		空间终点位置误差：47.542 m  (1.1301%) 
% 		空间 最大距远点最大误差：57.657 m (1.3706%)
% 	各维 最大 位置误差(x、y、z)：(22.741,21.698,49.387)m	(1.0302%,0.78911%,19.534%)
% 	各维 终点 位置误差(x、y、z)：(0.41747,5.9825,47.162)m	(0.018912%,0.21757%,18.654%)
% 	姿态最大误差 (俯仰、横滚、航向):(6.6128,-7.5604,1.8406)deg
% 	姿态终点误差 (俯仰、横滚、航向):(0.16529,-0.46303,0.00041717)deg
% 
% 	初始、最终加计估计误差：(-50  -50  -50  )、(-50  -50  -50  ) ug
% 	初始、最终陀螺估计误差：(-5  -5  -5  )、(14.3  1.79  -3.73  ) °/h
% 
% 滤波参数：
% 	X(0)=( 0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0   )
% 	P(0)=( 2.35e-09  2.35e-09  2.35e-09  1e-08  1e-08  1e-08  1e-10  1e-10  1e-10  2.35e-13  2.35e-13  2.35e-13  9.6e-15  9.6e-15  9.6e-15  7.62e-05  7.62e-05  7.62e-05  0.0025  0.0025  0.0025   )
% 	Qk=( 1e-26  1e-26  1e-26  2e-05  2e-05  2e-06  0  0  0  0  0  0  0  0  0  1e-10  1e-10  1e-10  1e-08  1e-08  1e-08   )
% 	R(0)=( 1e+05  1e+05  1e+05  1e-15  1e-15  1e-15   )

%% isTrueX0=1 时可得到特别高的二维位置精度 但3D位置和姿态效果并不好
% 	平面：	真实行程：4128.9 m	导航行程:4122.8 m	行程误差：-6.1627 m (-0.14926%)
% 		平面距原点最大误差：31.299 m (0.75804%)
% 		平面终点位置误差：2.6888 m  (0.06512%) 
% 	空间：	真实行程：4206.8 m	导航行程:4124.8 m	行程误差：-81.978 m (-1.9487%)
% 		空间终点位置误差：51.422 m  (1.2224%) 
% 		空间 最大距远点最大误差：56.948 m (1.3537%)
% 	各维 最大 位置误差(x、y、z)：(22.803,21.439,55.502)m	(1.033%,0.77967%,21.953%)
% 	各维 终点 位置误差(x、y、z)：(-1.0415,2.4788,51.352)m	(-0.047183%,0.090148%,20.311%)
% 	姿态最大误差 (俯仰、横滚、航向):(7.5238,-8.0823,-2.2699)deg
% 	姿态终点误差 (俯仰、横滚、航向):(1.2961,-0.97813,-0.88389)deg
% 
% 	初始、最终加计估计误差：(0  0  0  )、(7.16e-06  -4.38e-06  -0.000221  ) ug
% 	初始、最终陀螺估计误差：(0  0  0  )、(0.0411  0.00978  0.00236  ) °/h
% 
% 滤波参数：
% 	X(0)=( 0  0  0  0  0  0  0  0  0  3.39e-05  3.39e-05  3.39e-05  0.00196  0.00196  0.00196  0  0  0  0  0  0   )
% 	P(0)=( 2.35e-09  2.35e-09  2.35e-09  1e-08  1e-08  1e-08  1e-10  1e-10  1e-10  2.35e-15  2.35e-15  2.35e-15  9.6e-15  9.6e-15  9.6e-15  7.62e-05  7.62e-05  7.62e-05  0.0025  0.0025  0.0025   )
% 	Qk=( 1e-26  1e-26  1e-26  0.0002  0.0002  2e-06  0  0  0  0  0  0  0  0  0  1e-10  1e-10  1e-10  1e-08  1e-08  1e-08   )
% 	R(0)=( 1e+05  1e+05  1e+05  1e-07  1e-07  1e-07   )
% 不进行：IMU数据的常值漂移补偿
%  IMU常值漂移初值给 真值 （仿真真值/实验经验值）: pa=200 na=100 ug, pg=7.000 ng=6.000 °/h
 
function [ P_ini,Q_const,R_ini,NavFilterParameter ] = GetFilterParameter_SINSerror_dMove_2011_09_30_drive_0028_D( pg,ng,pa,na,NavFilterParameter )
%%
szj1 = 1/3600*pi/180 * 10;
szj2 = 1/3600*pi/180 * 10;
szj3 = 1/3600*pi/180 * 10;
vnsRDrift = [1 1 1]*0.5*pi/180 ;
vnsTDrift = [1 1 1]*0.05 ;
pg = [ 1 1 1 ]*pi/180/3600 * 0.01 ;        % 
pa = [ 1 1 1 ]*1e-6*9.8 *0.01 ;
P_ini = diag([(szj1)^2,(szj2)^2,(szj3)^2,(1e-4)^2,(1e-4)^2,(1e-4)^2,(1e-5)^2,(1e-5)^2,(1e-5)^2,...
                (pg(1))^2,(pg(2))^2,(pg(3))^2,(pa(1))^2,(pa(2))^2,(pa(3))^2,...
                (vnsRDrift(1))^2,(vnsRDrift(2))^2,(vnsRDrift(3))^2,(vnsTDrift(1))^2,(vnsTDrift(2))^2,(vnsTDrift(3))^2 ]); %  15*15
 NavFilterParameter.P_ini_augment_dRdT =  sprintf('%1.1e ',P_ini) ;

  Q_const = diag([  1e-26 1e-26 1e-26 ...           % 失准角微分方程
                    2e-4 2e-4 2e-6...               % 速度微分方程
                    0  0  0 ...                     % 位置微分方程
                    0  0  0 ...                     % 陀螺常值微分方程
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

%% 2011_09_30_drive_0028 效果好 用于paper
% 	平面：	真实行程：4128.9 m	导航行程:4126.4 m	行程误差：-2.505 m (-0.060669%)
% 		平面距原点最大误差：30.992 m (0.7506%)
% 		平面终点位置误差：5.9084 m  (0.1431%) 
% 	空间：	真实行程：4206.8 m	导航行程:4126.7 m	行程误差：-80.076 m (-1.9035%)
% 		空间终点位置误差：30.351 m  (0.72148%) 
% 		空间 最大距远点最大误差：59.92 m (1.4244%)
% 	各维 最大 位置误差(x、y、z)：(21.966,21.863,-51.283)m	(0.99512%,0.79509%,-20.284%)
% 	各维 终点 位置误差(x、y、z)：(-0.090663,5.9077,29.77)m	(-0.0041072%,0.21484%,11.775%)
% 	姿态最大误差 (俯仰、横滚、航向):(-6.1865,-6.9942,1.8283)deg
% 	姿态终点误差 (俯仰、横滚、航向):(-1.381,0.64827,0.0050421)deg
% 
% 	初始、最终加计估计误差：(-200  -200  -200  )、(-200  -200  -200  ) ug
% 	初始、最终陀螺估计误差：(-7  -7  -7  )、(-6.93  -6.92  -6.2  ) °/h
% 
% 滤波参数：
% 	X(0)=( 0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0   )
% 	P(0)=( 2.35e-09  2.35e-09  2.35e-09  1e-06  1e-06  1e-06  1e-08  1e-08  1e-08  5.88e-14  5.88e-14  5.88e-14  9.6e-15  9.6e-15  9.6e-15  7.62e-05  7.62e-05  7.62e-05  0.0025  0.0025  0.0025   )
% 	Qk=( 2e-10  2e-10  2e-25  2e-06  2e-06  2e-06  1e-18  1e-18  1e-18  1e-37  1e-37  1e-37  0  0  0  1e-10  1e-10  1e-10  1e-08  1e-08  1e-08   )
% 	R(0)=( 100  100  100  1e-07  1e-07  1e-07   )
% 不进行：IMU数据的常值漂移补偿
%  IMU常值漂移初值 置0
 
function [ P_ini,Q_const,R_ini,NavFilterParameter ] = GetFilterParameter_SINSerror_dMove_2011_09_30_drive_0028_E( pg,ng,pa,na,NavFilterParameter )
%%
szj1 = 1/3600*pi/180 * 10;
szj2 = 1/3600*pi/180 * 10;
szj3 = 1/3600*pi/180 * 10;
vnsRDrift = [1 1 1]*0.5*pi/180 ;
vnsTDrift = [1 1 1]*0.05 ;
pg = [ 1 1 1 ]*pi/180/3600 * 0.05 ;        % 
pa = [ 1 1 1 ]*1e-6*9.8 *0.01 ;
P_ini = diag([(szj1)^2,(szj2)^2,(szj3)^2,(1e-3)^2,(1e-3)^2,(1e-3)^2,(1e-4)^2,(1e-4)^2,(1e-4)^2,...
                (pg(1))^2,(pg(2))^2,(pg(3))^2,(pa(1))^2,(pa(2))^2,(pa(3))^2,...
                (vnsRDrift(1))^2,(vnsRDrift(2))^2,(vnsRDrift(3))^2,(vnsTDrift(1))^2,(vnsTDrift(2))^2,(vnsTDrift(3))^2 ]); %  15*15
 NavFilterParameter.P_ini_augment_dRdT =  sprintf('%1.1e ',P_ini) ;
% 2e-12 2e-12 2e-18
  Q_const = diag([  2e-10 2e-10 2e-25 ...           % 失准角微分方程
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
R_list = [R_list_input,{'[[1 1 1]*1e2  [1 1 1]*1e-7]'...
                        '[[1 1 1]*1e-1  [1 1 1]*1e-12]'}];      % 圆周360m Rbb 20.6"

[Selection,ok] = listdlg('PromptString','噪声方程阵R(前R[3x3]后T[3x1])-subQ\_subT:','SelectionMode','single','ListSize',[350,100],'ListString',R_list);
if ok==0    
    Selection = 1 ;
end
answer = inputdlg('噪声方程阵R(前R[3x3]后T[3x1])-subQ\_subT                     .','R_ini',1,R_list(Selection));
R_ini = diag(eval(answer{1})) ;   % R_const
NavFilterParameter.R_ini_augment_dRdT = answer{1} ;

%% 位置精度高 kitti 2011_10_03_drive_0034 效果比较好的一组数据
% INS_VNS_new_dQTb 解算误差：
% 	平面：	真实行程：5060.3 m	导航行程:5065 m	行程误差：4.6864 m (0.092611%)
% 		平面距原点最大误差：10.35 m (0.20452%)
% 		平面终点位置误差：3.7733 m  (0.074566%) 
% 	空间：	真实行程：5069.5 m	导航行程:5072.3 m	行程误差：2.8324 m (0.055871%)
% 		空间终点位置误差：17.398 m  (0.34319%) 
% 		空间 最大距远点最大误差：71.672 m (1.4138%)
% 	各维 最大 位置误差(x、y、z)：(-6.3145,8.8241,-71.577)m	(-0.21427%,0.24234%,-28.326%)
% 	各维 终点 位置误差(x、y、z)：(-3.7063,0.70816,-16.984)m	(-0.12577%,0.019449%,-6.7211%)
% 	姿态最大误差 (俯仰、横滚、航向):(-9.931,13.193,2.3417)deg
% 	姿态终点误差 (俯仰、横滚、航向):(-2.9433,-2.8028,-0.31443)deg
% 
% 	初始、最终加计估计误差：(-50  -50  -50  )、(9.07e+04  -3.65e+06  -5.33e+04  ) ug
% 	初始、最终陀螺估计误差：(-5  -5  -5  )、(-2.91  -2.96  -0.184  ) °/h
% 
% 滤波参数：
% 	X(0)=( 0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0   )
% 	P(0)=( 2.35e-11  2.35e-11  2.35e-11  0.0001  0.0001  0.0001  0.0001  0.0001  0.0001  2.35e-15  2.35e-15  2.35e-15  9.6e-11  9.6e-11  9.6e-11  7.62e-05  7.62e-05  7.62e-05  0.0025  0.0025  0.0025   )
% 	Qk=( 2e-35  2e-35  2e-35  2e-22  2e-22  2e-22  1e-24  1e-24  1e-24  0  0  0  1e-05  1e-05  0.0001  1e-10  1e-10  1e-10  1e-08  1e-08  1e-08   )
% 	R(0)=( 0.01  0.01  0.01  1e-12  1e-12  1e-12   )
% 不进行：IMU数据的常值漂移补偿

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

%% 位置精度高 kitti 2011_10_03_drive_0034 效果比较好的一组数据
% INS_VNS_new_dQTb 解算误差：
% 	平面：	真实行程：5060.3 m	导航行程:5065.4 m	行程误差：5.0985 m (0.10075%)
% 		平面距原点最大误差：12.174 m (0.24058%)
% 		平面终点位置误差：6.3946 m  (0.12637%) 
% 	空间：	真实行程：5069.5 m	导航行程:5071.8 m	行程误差：2.2768 m (0.044913%)
% 		空间终点位置误差：26.314 m  (0.51908%) 
% 		空间 最大距远点最大误差：56.247 m (1.1095%)
% 	各维 最大 位置误差(x、y、z)：(8.7047,11.517,-55.599)m	(0.29538%,0.31632%,-22.003%)
% 	各维 终点 位置误差(x、y、z)：(-5.6853,-2.9273,-25.526)m	(-0.19292%,-0.080394%,-10.102%)
% 	姿态最大误差 (俯仰、横滚、航向):(-9.8917,13.726,2.9696)deg
% 	姿态终点误差 (俯仰、横滚、航向):(-5.3511,-2.9112,0.65247)deg
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
                1e-14 1e-14 1e-14 ...               % 视觉误差角微分方程
                1e-18 1e-18 1e-18 ]);               	% 视觉平移误差微分方程     

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

%% kitti 2011_10_03_drive_0034 效果比较好的一组数据
% INS_VNS_new_dQTb 解算误差：
% 	平面：	真实行程：5060.3 m	导航行程:5069.8 m	行程误差：9.4274 m (0.1863%)
% 		平面距原点最大误差：12.916 m (0.25523%)
% 		平面终点位置误差：10.331 m  (0.20416%) 
% 	空间：	真实行程：5069.5 m	导航行程:5076.3 m	行程误差：6.8492 m (0.13511%)
% 		空间终点位置误差：24.678 m  (0.4868%) 
% 		空间 最大距远点最大误差：40.834 m (0.80549%)
% 	各维 最大 位置误差(x、y、z)：(-7.9403,12.735,-40.402)m	(-0.26944%,0.34975%,-15.989%)
% 	各维 终点 位置误差(x、y、z)：(-7.9403,-6.6096,-22.412)m	(-0.26944%,-0.18153%,-8.8692%)
% 	姿态最大误差 (俯仰、横滚、航向):(-10.123,13.262,2.9459)deg
% 	姿态终点误差 (俯仰、横滚、航向):(-6.6651,-2.5152,0.46895)deg
function [ P_ini,Q_const,R_ini,NavFilterParameter ] = GetFilterParameter_SINSerror_dMove_2011_10_03_drive_0034_A( pg,ng,pa,na,NavFilterParameter )
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
R_list = [R_list_input,{'[[1 1 1]*1e-5  [1 1 1]*1e-5]'...
                        '[[1 1 1]*1e-1  [1 1 1]*1e-12]'...  % kitti
                        }];      

[Selection,ok] = listdlg('PromptString','噪声方程阵R(前R[3x3]后T[3x1])-subQ\_subT:','SelectionMode','single','ListSize',[350,100],'ListString',R_list);
if ok==0    
    Selection = 1 ;
end
answer = inputdlg('噪声方程阵R(前R[3x3]后T[3x1])-subQ\_subT                     .','R_ini',1,R_list(Selection));
R_ini = diag(eval(answer{1})) ;   % R_const
NavFilterParameter.R_ini_augment_dRdT = answer{1} ;



%% 位置精度高 kitti 2011_10_03_drive_0034
% INS_VNS_new_dQTb 解算误差：
% 	平面：	真实行程：5060.3 m	导航行程:5065.4 m	行程误差：5.0985 m (0.10075%)
% 		平面距原点最大误差：12.174 m (0.24058%)
% 		平面终点位置误差：6.3946 m  (0.12637%) 
% 	空间：	真实行程：5069.5 m	导航行程:5071.8 m	行程误差：2.2768 m (0.044913%)
% 		空间终点位置误差：26.314 m  (0.51908%) 
% 		空间 最大距远点最大误差：56.247 m (1.1095%)
% 	各维 最大 位置误差(x、y、z)：(8.7047,11.517,-55.599)m	(0.29538%,0.31632%,-22.003%)
% 	各维 终点 位置误差(x、y、z)：(-5.6853,-2.9273,-25.526)m	(-0.19292%,-0.080394%,-10.102%)
% 	姿态最大误差 (俯仰、横滚、航向):(-9.8917,13.726,2.9696)deg
% 	姿态终点误差 (俯仰、横滚、航向):(-5.3511,-2.9112,0.65247)deg
function [ P_ini,Q_const,R_ini,NavFilterParameter ] = GetFilterParameter_SINSerror_dMove_2011_10_03_drive_0034_C( pg,ng,pa,na,NavFilterParameter )
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