% 从地球直角坐标系到地球大地坐标系（经纬度）下的转换
% 直角坐标系 -->  经纬度极坐标系

% 输入 position (m) ：x,y,z
% 输出 P
%   P(1)：经度（rad）
%   P(2)：纬度（rad）
%   P(3)：高度(m)

function P = FZJtoJW(position,planet)
format long
if ~exist('planet','var')
   errordlg('未输入所在星体(Fdtoe)')
   P=[];
   return;
else
    if ~strcmp(planet,'e') && ~strcmp(planet,'m')
        errordlg('planet参数不对(Fdtoe)')
        P=[];
        return;
    end
end
if strcmp(planet,'e')
    earthConst = getEarthConst;
    e = earthConst.e ;
    Ra = earthConst.Re ;
else
    moonConst = getMoonConst;
    e = moonConst.e ;   % 月球扁率
    Ra = moonConst.Rm ;  % 月球椭球长轴半径，m
end

x = position(1);
y = position(2);
z = position(3);
% a = 1737400;  % 地球椭球长轴半径，m
% e = 0.006;  % 地球扁率
r = sqrt(x^2 + y^2 +z^2);

latitude = asin(z/r);

longitude = atan(y/x); %   经度多值，
%% 经度范围[0 360]
if longitude<0
    if  x<0
        longitude=longitude+pi;     % [90 180]
    else
        longitude=longitude+2*pi;   % [270 360]
    end
else
    if y<0 % [180 270]
        longitude=longitude+pi;
    end
end

% %% 经度范围[-180 180]
% if longitude<0
%     if x<0 % [90 180]
%         longitude=longitude+pi;
%     end
% else
%     if y<0 % [-180 -90]
%         longitude=longitude-pi;
%     end
% end

%% 椭球体
% RN = Ra*(1-e^2)/(1-e^2*sin(latitude)^2)^(3/2) ; % 子午面主曲率半径
% RE = Ra/(1-e^2*sin(latitude)^2)^(1/2) ; % 横向主曲率半径
% R0 = sqrt(RE*RN) ;  % 平均主曲率半径

%% 近似为球体
R0 = Ra ;

h=r-R0;

P=[longitude;latitude;h];

% 
% B = atan(z/sqrt(x^2+y^2));
% longitude = atan(y/x);  % 经度 
% fai2 = 0;
% while 1
%     latitude = atan(tan(B)*(1+a*e^2/z*sin(fai2)/sqrt(1-e^2*sin(fai2)^2)));
%     h = r * cos(B) / cos(fai2) - a/sqrt(1-e^2*sin(fai2)^2);
%     if latitude-fai2<1e-18
%         break;
%     else
%         fai2=latitude;
%     end
% end
% if x < 0 && longitude < 0
% %     sym E
%     longitude = longitude + pi;
% elseif y < 0 && longitude < 0
%     longitude = longitude + 2*pi;
% end
% P=[longitude;latitude;h];
