function varargout = ResultDisplay(varargin)
% miaRESULTDISPLAY M-file for ResultDisplay.fig
%      RESULTDISPLAY, by itself, creates a new RESULTDISPLAY or raises the existing
%      singleton*.
%
%      H = RESULTDISPLAY returns the handle to a new RESULTDISPLAY or the handle to
%      the existing singleton*.
%
%      RESULTDISPLAY('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in RESULTDISPLAY.M with the given input arguments.
%
%      RESULTDISPLAY('Property','Value',...) creates a new RESULTDISPLAY or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before ResultDisplay_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to ResultDisplay_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help ResultDisplay

% Last Modified by GUIDE v2.5 10-Nov-2013 17:19:16

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @ResultDisplay_OpeningFcn, ...
                   'gui_OutputFcn',  @ResultDisplay_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before ResultDisplay is made visible.
function ResultDisplay_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to ResultDisplay (see VARARGIN)

% Choose default command line output for ResultDisplay
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);
handles.OpenSelected_DoingFlag = 'ready';   % 绘图准备ready
guidata(hObject,handles);

% OpenAll(handles) ;
% ColseAll(handles) ;
setFigurePosition(handles) ;
% UIWAIT makes ResultDisplay wait for user response (see UIRESUME)
%uiwait(handles.ResultDisplay);
% global ResultDisplayFirst
% if ResultDisplayFirst == 1
%      OpenAll(handles) ;
%      ColseAll() ;
%      ResultDisplayFirst = 0 ;        %第一次先绘制所有图片
% end


% OpenAll_Callback([],[],handles);       
% CloseAll_Callback;      %打开所有后再关闭所有，从而保存图形


function setFigurePosition(handles)
% 设置GUI界面的大小和位置
set(handles.ResultDisplay,'Position',[15 50 180 370]);%设置初始位置和大小

%% handles 中暂存的数据变量
%  Result,figureOutputPath,SelectedResult,OpenSelected_DoingFlag,FigureType
function OpenAll(handles)
%功能:打开所有图形
%方法：先判断变量是否已经打开，若没打开判断变量是否已经绘图。
Result = handles.Result;
figureOutputPath = handles.figureOutputPath;
n = length(Result) ;
%%%%%当重复进入结果界面时，图形已经绘制好，只需打开即可。因此为了保证结果已更新，在仿真前必须先清空Result文件夹
%%%%%根据文件名判断图形存在与否，存在则打开，不存在则绘制并保存
for i=1:n  
    %%%%根据图形名查找是否存在相应的图形
    h = findobj('Name',Result{i}.comment);%存在则获得句柄，不存在则为空
    if isempty(h)   %确认未打开。判断是否需要绘图
        path = [figureOutputPath,'\',Result{i}.project,'-',Result{i}.name,'.fig'];
        if exist(path,'file')  %根据文件名判断，相应的fig图形已存在，直接打开            
            open(path);
        else        %判断为不存在，先绘图，再保存
            DoPlotResult(Result,i,handles);
        end
    end
end

if isfield(handles,'ResultDisplay')
    figure(handles.ResultDisplay);      %将结果查看控制界面置于最前
end

function DoPlotResult(Result,index,handles)

isSubPlot = 1;

name = Result{index}.name ;     % 变量名
% 如果是组合轨迹数据独立绘制
if strcmp(name,'track')
    % 轨迹组合数据不符合正常Result格式
   PlotCombineTrack(Result,index,handles); 
else
    % 符合正常Result格式
    if isSubPlot==1
        if strcmp(Result{index}.project,'contrast')
            PlotResult(Result,index,handles);
        else
            SubPlotResult(Result,index,handles);
        end
    else
        PlotResult(Result,index,handles);
    end
end



function PlotResult(Result,index,handles)
%功能：绘制Result中Result{number}的图形
%参数 number ：所需绘制变量在Result中的下标

figureOutputPath = handles.figureOutputPath ;
%% 绘制独立变量的方法
% 允许这个变量具有多行/列，行列存储均可
data = Result{index}.data ;     % 待绘制数据
name = Result{index}.name ;     % 变量名
comment = Result{index}.comment ;% 图片的中文解释
project = Result{index}.project; % 方案名
frequency = Result{index}.frequency ;  % 频率
if isfield(Result{index},'subName')
    subName = Result{index}.subName ;
else
    if min (size(data,1),size(data,2) )==3
        subName = {'x','y','z'};
    else
        subName = [];
    end
end

time = ( (1:length(data))-1 )/frequency;  % 行存储
% 使 time 和 data 的存储形式一致，都用行存储,一行一个时刻
if size(data,1)>size(data,2)    % data为列存储,一列一个时刻
    data = data';
end
h = figure('name',[project,'-',name]) ;
dataNum = min(size(data,1),size(data,2));
[lineStyles,colors,makers] = GenerateLineStyle(dataNum) ;
hold on
line_h=zeros(1,dataNum);
for k=1:dataNum
    line_h(k)=plot(time,data(k,:),[lineStyles{k},makers{k}],'Color',colors{k},'LineWidth',1, 'MarkerSize',5); 
   %line_h(k)=plot(time,data(k,:),['-',makers{k}],'Color',colors{k},'LineWidth',1, 'MarkerSize',5); 
    %line_h(k)=plot(time,data(k,:),[lineStyles{k},makers{k}],'Color',colors{k},'LineWidth',1, 'MarkerSize',5); 
end

xlabel('时间/S');
title(AddBias([project,'-',comment]));     %以中文说明为plot标题
ylabel(name);    %以变量名为轴名
if ~isempty(subName) 
   legend(line_h,AddBias(subName)); 
end

% 添加注释文字
if isfield(Result{index},'text')
   textToAdd = Result{index} .text ;
   
   xTickLabel = get(gca,'XTickLabel');
   XLim = get(gca,'XLim');   
   dXLim = (XLim(2)-XLim(1))/50;% 空格
   xTickLabel_End = str2double(xTickLabel(size(xTickLabel,1),:)) ;
   text_x_1 = str2double(xTickLabel(1,:))*XLim(2)/xTickLabel_End+dXLim ;   % 前4个cell的显示位置
   text_x_2 = str2double(xTickLabel(3,:))*XLim(2)/xTickLabel_End+2*dXLim;
   
   yTickLabel = get(gca,'YTickLabel');
   YLim = get(gca,'YLim');
   yTickLabel_toText = str2double(yTickLabel(2,:)) ;
   yTickLabel_Min = str2double(yTickLabel(1,:)) ;
   text_y = yTickLabel_toText*YLim(1)/yTickLabel_Min;
   
   for j=1:length(textToAdd)
       textToAdd{j} = AddBias(textToAdd{j});
   end
   % 当 textToAdd 中超过4个cell时，分开显示
   if length(textToAdd)>4
       text(text_x_1,text_y,textToAdd(1:4));
       text(text_x_2,text_y,textToAdd(5:length(textToAdd)));
   else
       text(text_x_1,text_y,textToAdd);
   end
   
end

saveas(h,saveHand([figureOutputPath,'\',project,'-',name,'.fig']));   % 以“方案名-变量名”作为.fig名
saveas(h,saveHand([figureOutputPath,'\',project,'-',name,'.emf']));



function strOut = saveHand( str )
% 将保持路径中的 /转化成／
strOut = strrep(str, '/', '／');

function PlotCombineTrack(Result,index,handles)
figureOutputPath = handles.figureOutputPath ;
datai = Result{index}.data ;     % 待绘制数据
name = Result{index}.name ;     % 变量名
subName = Result{index}.subName ;     % 方案
comment = Result{index}.comment ;% 图片的中文解释
project = Result{index}.project; % 方案名

track_h = figure('name',[project,'-',name]) ;
hold on
[lineStyles,colors,makers] = GenerateLineStyle(length(datai)) ;
for k=1:length(datai)
   data = datai{k}; 
   plot(data(1,:),data(2,:),[lineStyles{k},makers{k}],'Color',colors{k},'LineWidth',1, 'MarkerSize',4);
end
legend(subName)
xlabel('x方向/m');
ylabel('y方向/m');
title('track\_contrast');     
saveas(track_h,saveHand([figureOutputPath,'\',project,'-',name,'.fig']));   % 以“方案名-变量名”作为.fig名
saveas(track_h,saveHand([figureOutputPath,'\',project,'-',name,'.emf']));

function SubPlotResult(Result,index,handles)
%功能：绘制Result中Result{number}的图形
%参数 number ：所需绘制变量在Result中的下标

figureOutputPath = handles.figureOutputPath ;
%% 绘制独立变量的方法
% 允许这个变量具有多行/列，行列存储均可
data = Result{index}.data ;     % 待绘制数据
name = Result{index}.name ;     % 变量名
comment = Result{index}.comment ;% 图片的中文解释
project = Result{index}.project; % 方案名
frequency = Result{index}.frequency ;  % 频率

if isfield(Result{index},'subName')
    subName = Result{index}.subName ;
else
    if min (size(data,1),size(data,2) )==3
        subName = {'x','y','z'};
    else
        subName = [];
    end
end

    
time = ( (1:length(data))-1 )/frequency;  % 行存储
% 使 time 和 data 的存储形式一致，都用行存储,一行一个时刻
if size(data,1)>size(data,2)    % data为列存储,一列一个时刻
    data = data';
end
h = figure('name',[project,'-',name]) ;
dataNum = min(size(data,1),size(data,2));
[lineStyles,colors,makers] = GenerateLineStyle(dataNum) ;
% hold on
line_h=zeros(1,dataNum);
for k=1:dataNum
    % 分别绘图
	subplot(dataNum,1,k);
    line_h(k)=plot(time,data(k,:),[lineStyles{k},''],'Color',colors{k},'LineWidth',1, 'MarkerSize',4); 
   %line_h(k)=plot(time,data(k,:),['-',makers{k}],'Color',colors{k},'LineWidth',1, 'MarkerSize',5); 
    %line_h(k)=plot(time,data(k,:),[lineStyles{k},makers{k}],'Color',colors{k},'LineWidth',1, 'MarkerSize',5); 
    if ~isempty(subName)
        ylabel(subName{k});
    else
        ylabel(name);
    end
    if k==1
        title(AddBias([project,'-',comment]));     %以中文说明为plot标题
    end
    AddText(Result,index,k);
end

xlabel('时间/S');
% 
% % 添加注释文字
% if isfield(Result{index},'text')
%    textToAdd = Result{index} .text ;
%    
%    xTickLabel = get(gca,'XTickLabel');
%    XLim = get(gca,'XLim');   
%    dXLim = (XLim(2)-XLim(1))/50;% 空格
%    xTickLabel_End = str2double(xTickLabel(size(xTickLabel,1),:)) ;
%    text_x_1 = str2double(xTickLabel(1,:))*XLim(2)/xTickLabel_End+dXLim ;   % 前4个cell的显示位置
%    text_x_2 = str2double(xTickLabel(3,:))*XLim(2)/xTickLabel_End+2*dXLim;
%    
%    
%    
%    yTickLabel = get(gca,'YTickLabel');
%    YLim = get(gca,'YLim');
%    yTickLabel_toText = str2double(yTickLabel(2,:)) ;
%    yTickLabel_Min = str2double(yTickLabel(1,:)) ;
%    yTickLabel_Max = str2double(yTickLabel(length(yTickLabel),:)) ;
%    text_y = yTickLabel_Min+(yTickLabel_Max-yTickLabel_Min)*0.2;
%    %text_y = yTickLabel_toText*YLim(1)/yTickLabel_Min;
% 
%     disp('')
%    disp(name)
%    for i=1:length(textToAdd)
%         disp(textToAdd{i});
%    end
%    
%    for j=1:length(textToAdd)
%        textToAdd{j} = AddBias(textToAdd{j});
%    end
%    text(text_x_1,text_y,textToAdd(2));
%    % 当 textToAdd 中超过4个cell时，分开显示
%    if length(textToAdd)>4
%        text(text_x_1,text_y,textToAdd(1:4));
%        text(text_x_2,text_y,textToAdd(5:length(textToAdd)));
%    else
%        text(text_x_1,text_y,textToAdd);
%    end
   
% end

saveas(h,saveHand([figureOutputPath,'\',project,'-',name,'.fig']));   % 以“方案名-变量名”作为.fig名
saveas(h,saveHand([figureOutputPath,'\',project,'-',name,'.emf']));

%% 如果是位置组合，绘制轨迹图
if strcmp(name,'position(m)') && ~strcmp(project,'contrast')
    track_h = figure('name',[project,'-',name]) ;
    plot(data(1,:),data(2,:));
    xlabel('x方向/m');
    ylabel('y方向/m');
    title(AddBias([project,'-轨迹']));     %以中文说明为plot标题
    saveas(track_h,saveHand([figureOutputPath,'\',project,'-',name,'.fig']));   % 以“方案名-变量名”作为.fig名
    saveas(track_h,saveHand([figureOutputPath,'\',project,'-',name,'.emf']));
end

function AddText(Result,index,k)
% 添加注释文字
if isfield(Result{index},'text')
   textToAdd = Result{index} .text ;
   
   xTickLabel = get(gca,'XTickLabel');
   XLim = get(gca,'XLim');   
   dXLim = (XLim(2)-XLim(1))/50;% 空格
   xTickLabel_End = str2double(xTickLabel(size(xTickLabel,1),:)) ;
   text_x_1 = str2double(xTickLabel(1,:))*XLim(2)/xTickLabel_End+dXLim ;   % 前4个cell的显示位置
   text_x_2 = str2double(xTickLabel(3,:))*XLim(2)/xTickLabel_End+2*dXLim;
   
   
   
   yTickLabel = get(gca,'YTickLabel');
   YLim = get(gca,'YLim');
   yTickLabel_toText = str2double(yTickLabel(2,:)) ;
   yTickLabel_Min = str2double(yTickLabel(1,:)) ;
   yTickLabel_Max = str2double(yTickLabel(size(yTickLabel,1),:)) ;
   text_y = yTickLabel_Min+(yTickLabel_Max-yTickLabel_Min)*0.2;
   %text_y = yTickLabel_toText*YLim(1)/yTickLabel_Min;

%     disp('')
%    disp(name)
%    for i=1:length(textToAdd)
%         disp(textToAdd{i});
%    end
   
   for j=1:length(textToAdd)
       textToAdd{j} = AddBias(textToAdd{j});
   end
   text(text_x_1,text_y,textToAdd(1+k));
%    % 当 textToAdd 中超过4个cell时，分开显示
%    if length(textToAdd)>4
%        text(text_x_1,text_y,textToAdd(1:4));
%        text(text_x_2,text_y,textToAdd(5:length(textToAdd)));
%    else
%        text(text_x_1,text_y,textToAdd);
%    end
   
end

function ColseAll(handles)
%功能：关闭所有图形
Result = handles.Result ;
%关闭所有独立图形
n = length(Result);
for i=1:n
    name = Result{i}.name ;     % 变量名
    project = Result{i}.project; % 方案名
    FigName = [project,'-',name];
    h = findobj('Name',FigName);
    if ~isempty(h)
        close(h);
    end
end
%关闭所有组合图形

function OpenSelected(handles)
%功能：打开ResultList中被选中的变量
%将最后选择的那个变量的图形置于当前
%%先检查是否已经打开，防止重复打开。未打开时再检查该图形是否存在，不存在则绘制、保存
% 列表中顺序与 Result  成员序号一致
global   AxisRange
SelectedResult = get(handles. ResultList,'Value') ;
Result = handles.Result ;
for i=1:numel(SelectedResult)
    name = Result{SelectedResult(i)}.name ;     % 变量名
    project = Result{SelectedResult(i)}.project; % 方案名
    FigName = [project,'-',name];
    %%%%根据图形名查找是否存在相应的图形
    h = findobj('Name',FigName);%存在则获得句柄，不存在则为空
    if isempty(h)   %确认不存在。判断是否需要绘图
        %disp('未打开');
        path = [handles.figureOutputPath,'\',FigName,'.fig'];
        if exist(path,'file')  %根据文件名判断，相应的fig图形已存在，直接打开
            %disp('存在，直接打开');
            try
                open(path);
            catch 
                disp('打开失败')
                DoPlotResult(Result,SelectedResult(i),handles);
            end
        else
            DoPlotResult(Result,SelectedResult(i),handles);
            %disp('不存在，绘制');
        end
    else
        %disp('已打开');
        figure(h);      %存在，将其置于前
    end
end
% AxisRange = axis;
% RefreshAxisRange(0,handles);

% --- Outputs from this function are returned to the command line.
function varargout = ResultDisplay_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;
guidata(hObject,handles);


function CombineResult = GetCombineProjectResult(originalResult)
%% 根据原始同一方案各自独立的数据生成方案组合数据
% 生成方法见说明文档

% 寻找所有name相同，project不同的数据cell
% 有多少中name，就有多少个组合细胞
% 得到所有的name列表
orNum = length(originalResult);
allName = cell(1,orNum) ;
difNameNum = 0;         %  不同name的个数
firstProjectIndex = zeros(1,orNum);     % Result中同一name的第一个变量下标
for i=1:orNum       % 计算 difNameNum firstProjectIndex
    name = originalResult{i}.name ;
    if isNewName(name,allName)
        % 为新 name
        difNameNum = difNameNum+1 ;
        firstProjectIndex(difNameNum) = i ;
        allName{difNameNum} = name ;
    end
end
allName = allName(1:difNameNum);
firstProjectIndex = firstProjectIndex(1:difNameNum);
combineLength = zeros(1,difNameNum);  % 各个组合数据的有效长度
combineFre = zeros(1,difNameNum);  % 各个组合数据的频率
for j=1:difNameNum
    name_j = originalResult{firstProjectIndex(j)}.name ;
    length_j = zeros(1,5);   % 所有name相同的第j分量的数据长度
    frequency_j = zeros(1,5);% 所有name相同的第j分量的数据频率
    project_num = 1;
    length_j(1) = size(makeRow(originalResult{firstProjectIndex(j)}.data,3),2);
    frequency_j(1) = originalResult{firstProjectIndex(j)}.frequency;
    for k=firstProjectIndex(j)+1:orNum        
        if strcmp(name_j,originalResult{k}.name)
            project_num = project_num+1 ;
            length_j(project_num) = size(makeRow(originalResult{k}.data,3),2);
            frequency_j(project_num) = originalResult{k}.frequency;
        end
    end
    length_j = length_j(1:project_num);
    frequency_j = frequency_j(1:project_num);
    % 得到组合数据的平率和长度
    [validLenth,combineK,combineLength(j),combineFre(j)] = GetValidLength(length_j,frequency_j);
end
combineResultNum = 0 ;  % 新的细胞变量的个数=name个数*每个name对应的子变量
CombineResult = cell(1,20);
% 根据 name 查找所有需要组合的曲线
for j=1:difNameNum
    % 每个j将产生新的细胞变量的个数为 originalResult{firstProjectIndex(j)}.data 的子变量数
    firstData = originalResult{firstProjectIndex(j)}.data ;
    j_newCellNum = min( size(firstData,1),size(firstData,2) );
    
    dataFlag = 'xyz result display format';
    frequency_j_first = originalResult{firstProjectIndex(j)}.frequency ;
    name_j = originalResult{firstProjectIndex(j)}.name ;
    subName = originalResult{firstProjectIndex(j)}.subName ;
    comment_j = originalResult{firstProjectIndex(j)}.comment ;
    project_j = originalResult{firstProjectIndex(j)}.project ;
    % 到此已经确定j对应的新细胞的如下成员：dataFlag,frequency,name,project
    data_j = originalResult{firstProjectIndex(j)}.data ;
    data_j = makeRow(data_j) ;  % 使为行存储
    if j_newCellNum>1
        for j_sub=1:j_newCellNum
            % 此循环中将产生一个确定的新cell变量
            if ~isempty(subName)
                name_j_sub = [name_j,'-',subName{j_sub}] ;      % 最终确定 name
            else
                subName = [];
            end
            comment_j_sub = [comment_j,'-',subName{j_sub}]; % 最终确定 comment
            subName_j_sub = cell(1,5);  % 行数未知，预设为5
            subName_j_sub{1} = project_j;
            
            data_j_sub = zeros(5,combineLength(j));
            %%%%%%%%%%%%%%%%%%%%%% ************ %%%%%%%%%%%%%%
            toCombineData  = GetDataToComine(data_j(j_sub,:),frequency_j_first,combineFre(j));
            data_j_sub(1,:) = toCombineData(1:combineLength(j)) ;  % 数据合并第一步
            %%%%%%%%%%%%%%%%%%%%% ************ %%%%%%%%%%%%%%%
            difProjectNum = 1;  % 不同的project的个数        
            % 获取data：寻找所有name为name_j的cell，并合并其数据
            for k=firstProjectIndex(j)+1:orNum
                if strcmp(name_j,originalResult{k}.name)
                    difProjectNum = difProjectNum+1 ;
                    %%%%%%%%%%%%%%%%%%%%%%
                    % 做有效性检查：长度、频率一致等
                    %%%%%%%%%%%%%%%%%%%%%%
                    data_k = originalResult{k}.data ;
                    toCombineData  = GetDataToComine(data_k(j_sub,:),originalResult{k}.frequency,combineFre(j));
                    data_j_sub(difProjectNum,:) = toCombineData(1:combineLength(j)) ;  % 数据合并第k步% 最终确定 data
                    subName_j_sub{difProjectNum} = originalResult{k}.project ;
                end
            end
            subName_j_sub = subName_j_sub(1:difProjectNum);     % 去除无效
            data_j_sub = data_j_sub(1:difProjectNum,:);
            % 确定一个新的细胞变量
            aNewCombineResult.dataFlag = dataFlag ;
            aNewCombineResult.frequency = combineFre(j) ;
            aNewCombineResult.name = name_j_sub ;
            aNewCombineResult.comment = comment_j_sub ;
            aNewCombineResult.data = data_j_sub ;
            aNewCombineResult.project = 'contrast' ;
            aNewCombineResult.subName = subName_j_sub ;
            % 确定了一个新的细胞，更新
            combineResultNum = combineResultNum+1 ;
            CombineResult{combineResultNum} = aNewCombineResult ; 
            
        end
    end
    
end
CombineResult = CombineResult(1:combineResultNum);
%% 添加组合轨迹数据（格式特殊，独立绘图）
combineTrack.dataFlag = dataFlag ;
combineTrack.frequency = 1 ;    % 无意义
combineTrack.name = 'track' ;
combineTrack.comment = 'contrast_track';
combineTrack.project = 'contrast' ;

difProjectNum = 0;  % 不同的project的个数   
data_temp=cell(1);
subName_temp=cell(1);

for k=1:orNum
    if strcmp('position(m)',originalResult{k}.name)
        difProjectNum = difProjectNum+1 ;
        data_temp{difProjectNum} = originalResult{k}.data ;
        subName_temp{difProjectNum} = originalResult{k}.project ;
    end                
end        

combineTrack.data = data_temp;
combineTrack.subName = subName_temp;
combineResultNum = combineResultNum+1 ;
CombineResult{combineResultNum} = combineTrack ; 


function isNew = isNewName(name,allName)
%% allName 中不包含 name则返回1
for i=1:length(allName)
   if strcmp(allName{i},name) 
      isNew = 0;
      return
   end
end
isNew = 1;

function new = makeRow(old,n)
% 使数据为列存储 ：一列一个时刻，正常时 行数<列数(时刻数)
% 输入n时n表示分量数（行数已知）
new = old ;
if exist('n','var')
   momentNum = n;       % 分量个数
else
    momentNum = length(old);
end
if size(old,1)>momentNum
   new = old'; 
end

% --- Executes during object creation, after setting all properties.
function Open_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Open (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% --- Executes on button press in Close.
function Close_Callback(hObject, eventdata, handles)
% hObject    handle to Close (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%功能：关闭被选中的图形
%方法：根据FigureHandle数组关闭，先判断是否打开
%备：另一种方法——先判断图形是否打开，根据图形名查找句柄
global Result   ResultNum   SelectedResult   FigureHandle   
n = length(SelectedResult) ;
for i=1:n
    VarOrder = SelectedResult(i)+ResultNum.time ;    %SelectedResult(i)+ResultNum.time为所选变量在Result数组中的标号
    FigName = Result{VarOrder,3};
    h = findobj('Name',FigName);
    if ~isempty(h)
        close(h);
    end
    FigureHandle(VarOrder) = 0;     %关闭某个图形后，一定要记得将FigureHandle数组中的相应位清0
end         

% --- Executes on button press in SupCombine.
function SupCombine_Callback(hObject, eventdata, handles)
% hObject    handle to SupCombine (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in CloseAll.
function CloseAll_Callback(~, ~,handles)
% hObject    handle to CloseAll (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
ColseAll(handles) ;

% --- Executes on button press in SubCombine.
function SubCombine_Callback(hObject, eventdata, handles)
% hObject    handle to SubCombine (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

function isResult = isResultFormat(data)
% 判断 data 是不是Result专用格式
if iscell(data) && isfield(data{1},'dataFlag') && strcmp(data{1}.dataFlag,'xyz result display format')
    % 确认为有效的Result文件
    isResult = 1 ;
else
    isResult = 0 ;
end

% --- Executes during object creation, after setting all properties.
function ResultDisplay_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ResultDisplay (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
global projectDataPath 

if isempty(projectDataPath)
    ResultPath = uigetdir([pwd,'\resultInput'],'选择待绘制结果文件');
else
    ResultPath = [projectDataPath,'\navResult'];
end

%ResultPath = pwd;   % 可执行文件时这么用

if ~isdir(ResultPath)
   ResultPath = uigetdir('选择待绘制结果文件');
end

% disp('结果保存路径')
% display(ResultPath)
figureOutputPath = [ResultPath,'\figureOutput'] ;  % 绘图结果保存目录
if isdir(figureOutputPath)
    button = questdlg('是否清空原图片','图片存储文件操作','是','否','否') ;
    if strcmp(button,'是')
        delete([figureOutputPath,'\*']);     % 存在则清空
    end
else
    mkdir(figureOutputPath);    % 不存在则建立
end
handles.figureOutputPath = figureOutputPath ;
guidata(hObject,handles);
% 读取文件中所有.mat文件，并逐一判断是否为result格式的文件，将有效数据合并到 Result
allFillName = ls([ResultPath,'\*.mat']);
fileNum = size(allFillName,1);
validFileNum = fileNum;
Result = cell(1,20);    % 变量数未知，随便定义先，等确定后再定大小
varNum = 0; % 记录所有待绘制的变量的个数
for j=1:fileNum
   data = importdata([ResultPath,'\',allFillName(j,:)]); 
   if isResultFormat(data)  % 此文件被判断为有效结果格式文件
       for k=1:length(data)
           varNum = varNum+1 ;
           Result{varNum} = data{k};    % 合并
       end
   else
       validFileNum=validFileNum-1;
   end
end
Result = Result(1:varNum);
if(validFileNum>1)
    CombineResult = GetCombineProjectResult(Result) ;   % 得到方案组合数据
    assignin('base','CombineResult',CombineResult);
    Result = [Result,CombineResult];    % 将方案组合结果合并到待绘制的变量中
end

handles.Result = Result ;   % 保存 Result 到GUIData中

handles.figureOutputPath = figureOutputPath ;
guidata(hObject,handles);
% Result 中此时已经包含所有待绘制的变量
assignin('base','Result',Result);
save([handles.figureOutputPath,'\Result.mat'],'Result');
disp('Result 文件提取结束，保存到figureOutput 和基本工作空间')

% --- Executes during object creation, after setting all properties.
function ResultList_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ResultList (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
% 更新列表
Result = handles.Result ;
varNum = length(Result);
ResDis = cell(1,varNum);
for j=1:varNum
    ResDis{j} = [Result{j}.project,'-',Result{j}.comment] ;
end
set(hObject,'String',ResDis);
SelectedResult = 1;
set(hObject,'Value',SelectedResult);
handles.SelectedResult = SelectedResult ;
guidata(hObject,handles);   % 保存被选择序号

% --- Executes on selection change in ResultList.
function ResultList_Callback(hObject, eventdata, handles)
% hObject    handle to ResultList (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns ResultList contents as cell array
%        contents{get(hObject,'Value')} returns selected item from ResultList
% 独立模式下直接左击打开，组合模式下采用右击打开

FigureType = handles.FigureType ;
if ~strcmp(handles.OpenSelected_DoingFlag,'busy')
    handles.OpenSelected_DoingFlag = 'busy' ;       %进入忙状态，执行完下述语句前不响应新的左击
    guidata(hObject,handles);
    if strcmp(FigureType,'alone')
        %ColseAll(handles) ;
        OpenSelected(handles) ;         %独立模式
    end
    figure(handles.ResultDisplay);      %将结果查看控制界面置于最前
    setFigurePosition(handles) ;
    handles.OpenSelected_DoingFlag = 'ready' ;    %恢复正常状态，可以响应新的命令
    guidata(hObject,handles);
else
    %disp('wait last OpenSelected function finished')
end

% --- Executes on button press in OpenAll.
function OpenAll_Callback(~, ~, handles)
% hObject    handle to OpenAll (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
 OpenAll(handles) ;

% --- Executes during object creation, after setting all properties.
function ResultListPane_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ResultListPane (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
set(hObject,'Title','结果列表( 左击打开 )')

function x_axis_Callback(hObject, eventdata, handles)
% hObject    handle to x_axis (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of x_axis as text
%        str2double(get(hObject,'String')) returns contents of x_axis as a double
%输入两个数字时，直接设定轴的范围
%输入一个数字时，以中间为中心，按数字缩放范围
%从编辑框中的字符串中读取数字，数字之间以任何非数字元素隔开（如空格,小数点除外）
global AxisRange
xv = get(hObject,'String');

j=1;
temp=[];
%%%%%%%
%读数字，适用于任何长度与数字个数的形式
n = length(xv);
for i=1:n
    if i<n
         if xv(i)>44 && xv(i)<58  %数字或小数点
            temp = [temp,xv(i)];
         else %空格
             if ~isempty(temp)
                temp = str2double(temp) ;
                xvdata(j) = temp;
                j=j+1;
                temp = [];
             end
         end
    else
        temp = [temp,xv(i)];
        temp = str2double(temp) ;
        xvdata(j) = temp;
    end
end
%%%%%%%
n = length(xvdata);
if n~=2 && n~=1
    errordlg('时间轴范围数据有误！');
    return;
end
if n==1     %缩放
    middle = (AxisRange(1) + AxisRange(2))/2 ;
    middle = fix(middle);
    xrange = AxisRange(2) - AxisRange(1) ;
    xrange = fix(xrange*xvdata) ;
    AxisRange(1) = middle - xrange/2;
    AxisRange(2) = middle + xrange/2;
    if AxisRange(1)<-60
        AxisRange(1) = -60;
        AxisRange(2) = AxisRange(1)+xrange ;
    end
    if AxisRange(2)>xlabelTime
        AxisRange(2)=xlabelTime+60;
        AxisRange(1) = AxisRange(2)-xrange ;
    end
end

Result = handles.Result ;
SelectedResult = get(handles. ResultList,'Value') ;
data = Result{SelectedResult(1)}.data ;
xlabelTime = ceil( length(data)/Result{SelectedResult(1)}.frequency );  %  时间总长度

if n==2  %直接设置范围
    if xvdata(2)<xvdata(1)
        errordlg('X轴范围设置错误！前面的应小于后面的。');
        return;
    end
    AxisRange(1) = xvdata(1);
    AxisRange(2) = xvdata(2);  
    if AxisRange(1)<0
        AxisRange(1) = 0;
    end
    if AxisRange(2)>xlabelTime
        AxisRange(2)=xlabelTime;
    end
end
RefreshAxisRange(1,handles);
        
function RefreshAxisRange(SourceNum,handles)
%根据 AxisRange的值更新：图形、编辑框、拖动条（有一个为数据来源不需更新）
%参数SourceNum表示数据来源：图形（0）；编辑框（1）；拖动条（2）
global   AxisRange 
Result = handles.Result ;
SelectedResult = get(handles. ResultList,'Value') ;
data = Result{SelectedResult(1)}.data ;
xlabelTime = ceil( length(data)/Result{SelectedResult(1)}.frequency );  %  时间总长度

if SourceNum~=0         %不为0时需更新图形
    %只改被选中的第一个图形
    FigName = [ Result{SelectedResult(1)}.project,'-',Result{SelectedResult(1)}.name ];
    h = findobj('Name',FigName);
    if isempty(h)
         DoPlotResult(Result,SelectedResult(1),handles);
    else
        figure(h);
    end
    axis(AxisRange);
end
%总更新编辑框
xstr = [num2str(AxisRange(1)),' ',num2str(AxisRange(2))];
set(handles.x_axis,'String',xstr);
ystr = [num2str(AxisRange(3)),' ',num2str(AxisRange(4))];
set(handles.y_axis,'String',ystr);

if SourceNum~=2         %不为2时需更新拖动条
    xposition = (AxisRange(1)+AxisRange(2))/xlabelTime/2;
    if xposition>1
        xposition=1;
    end
    if xposition<0
        xposition=0;
    end
    set(handles.Xslider,'Value',xposition);
end

% --- Executes during object creation, after setting all properties.
function x_axis_CreateFcn(hObject, eventdata, handles)
% hObject    handle to x_axis (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function y_axis_Callback(hObject, eventdata, handles)
% hObject    handle to y_axis (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of y_axis as text
%        str2double(get(hObject,'String')) returns contents of y_axis as a double
%输入两个数字时，直接设定轴的范围
%输入一个数字时，以中间为中心，按数字缩放范围
%从编辑框中的字符串中读取数字，数字之间以任何非数字元素隔开（如空格,小数点除外）
global AxisRange 
yv = get(hObject,'String');

j=1;
temp=[];
%%%%%%%
%读数字，适用于任何长度与数字个数的形式
n = length(yv);
for i=1:n
    if i<n
         if yv(i)>44 && yv(i)<58  %数字或小数点
            temp = [temp,yv(i)];
         else %空格
             if ~isempty(temp)
                temp = str2double(temp) ;
                yvdata(j) = temp;
                j=j+1;
                temp = [];
             end
         end
    else
        temp = [temp,yv(i)];
        temp = str2double(temp) ;
        yvdata(j) = temp;
    end
end
%%%%%%%
n = length(yvdata);
if n~=2 && n~=1
    errordlg('Y轴范围数据有误！');
    return;
end
if n==1     %缩放
    middle = (AxisRange(3) + AxisRange(4))/2 ;
    middle = fix(middle);
    yrange = AxisRange(4) - AxisRange(3) ;
    yrange = fix(yrange*yvdata) ;
    AxisRange(3) = middle - yrange/2;
    AxisRange(4) = middle + yrange/2;
end

if n==2  %直接设置范围
    if yvdata(2)<yvdata(1)
        errordlg('Y轴范围设置错误！前面的应小于后面的。');
        return;
    end
    AxisRange(3) = yvdata(1);
    AxisRange(4) = yvdata(2);  
end
RefreshAxisRange(1,handles);

% --- Executes during object creation, after setting all properties.
function y_axis_CreateFcn(hObject, eventdata, handles)
% hObject    handle to y_axis (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on slider movement.
function Xslider_Callback(hObject, eventdata, handles)
% hObject    handle to Xslider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
%在x轴方向，保持大小比例不变的情况下移动图形
%由silder的值确定中间值
global AxisRange

Result = handles.Result ;
SelectedResult = get(handles. ResultList,'Value') ;
data = Result{SelectedResult(1)}.data ;
xlabelTime = ceil( length(data)/Result{SelectedResult(1)}.frequency );  %  时间总长度

position = get(hObject,'Value');
middle = xlabelTime * position ; %求得中间值
xrange = AxisRange(2)-AxisRange(1);
xmin = middle - xrange/2 ;
xmax = middle + xrange/2 ;
AxisRange(1) = xmin;
AxisRange(2) = xmax;
if AxisRange(1)<-60
    AxisRange(1) = -60 ;
    AxisRange(2) = AxisRange(1)+xrange ;
    if AxisRange(2)>xlabelTime
        AxisRange(2)=xlabelTime+100 ;
    end
end
if AxisRange(2)>xlabelTime
    AxisRange(2)=xlabelTime+60;
    AxisRange(1) = AxisRange(2)-xrange ;
    if AxisRange(1)<-60
        AxisRange(1)=-60;
    end
end

RefreshAxisRange(2,handles);

% --- Executes during object creation, after setting all properties.
function Xslider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Xslider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes during object creation, after setting all properties.
function uipanel4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to uipanel4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over ResultList.
function ResultList_ButtonDownFcn(~, ~, handles)
% hObject    handle to ResultList (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% 独立模式下直接左击打开，组合模式下采用右击打开
% global FigureType OpenSelectedHold_DoingFlag
% % OpenSelectedHold_DoingFlag用于防止快速重复右击时，回调函数被自己中断出错。要求执行完当前再响应新的右击
% FigureType = handles.FigureType ;
% if ~strcmp(OpenSelectedHold_DoingFlag,'busy')
%     OpenSelectedHold_DoingFlag = 'busy' ;       %进入忙状态，执行完下述语句前不响应新的右击
%     if strcmp(FigureType,'combine')
%         ColseAll() ;
%         PlotCombineFig(handles) ; %组合模式
%     end
%     figure(handles.ResultDisplay);      %将结果查看控制界面置于最前
%     setFigurePosition(handles) ;
%     OpenSelectedHold_DoingFlag = 'ready' ;    %恢复正常状态，可以响应新的命令
% else
%     %disp('wait last OpenSelectedHold function finished')
% end

function PlotDiResult(Result)
%% 绘制组合图形
% combineProject 的格式参考说明书
resultIndex = combineProject.resultIndex ;
subIndex = combineProject.subIndex ;
% 将以上两个下标表示的所有数据合并到一起

function PlotCombineFig(handles)
% 一组合模式绘图
% 不管是否存在，直接重新绘制
%功能：绘制Result中的图形
%参数 number ：所需绘制变量在Result中的下标
global Result  HoldNameList ResultNum OpenSelectedHold_DoingFlag


SelectedResult = get(handles. ResultList,'Value') ;

timeLength = length(Result{1,1}) ;  %时间长度
dataNum = length(SelectedResult) ;  %组合个数

holdData = zeros(timeLength,dataNum) ;   %列存储，一行一个时刻
holdTitle = cell(dataNum,1) ;
WholeExplain = '' ; %组合解释
WholeName = '' ;
%组合数据、说明和变量名
for k=1:dataNum
    if k==1
        data_N = SelectedResult(k)+ResultNum.time ;
        holdData(:,k) = Result{data_N,k} ;
        holdTitle{k} = Result{data_N,3} ;   %中文说明
        WholeExplain = Result{data_N,3} ;
        WholeName = Result{data_N,2} ;
    else
      	data_N = SelectedResult(k)+ResultNum.time ;
        holdData(:,k) = Result{data_N,1} ;
        holdTitle{k} = Result{data_N,3} ;   %中文说明
        WholeExplain = [WholeExplain,'-',Result{data_N,3}] ;
        WholeName = [WholeName,'-',Result{data_N,2}] ;
    end
end

    figh = figure('Name',WholeExplain) ;    %句柄数组中的下标与Result的下标相一致  %以中文说明为figure标题
    ploth = plot(Result{1,1},holdData);        %Result{1}为仿真时间，Result{number,1}中为待绘制的变量数据
    title(AddBias(WholeExplain));     %以中文说明为plot标题
    legend(ploth,holdTitle) ;
    xlabel('时间/S');
    ylabel(WholeName);    %以变量名为轴名
    v =axis;
    v(3)=v(3)*0.95;
    if v(3)==0
        v(3)=-v(4)/50;
    end
    v(4)=v(4)*1.05;
    axis(v);        
    %%%%%%%%保存：只在组合情况下保存
    if dataNum>1
        path = [pwd,'\result\figure\',WholeName,'.fig'];
        saveas(figh,path);
        path = [pwd,'\result\jpeg\',WholeName,'.jpg'];
        saveas(figh,path);
    end
% 将组合图形名记录到HoldNameList，用于关闭图形时查找图形
num = length(HoldNameList) ;
HoldNameList{num+1} = WholeExplain ;



% --- Executes during object creation, after setting all properties.
function OpenAll_CreateFcn(hObject, eventdata, handles)
% hObject    handle to OpenAll (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes on button press in HoldMode.
function HoldMode_Callback(hObject, ~, handles)
% hObject    handle to HoldMode (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of HoldMode

FigureTypeNum = get(hObject,'Value') ;
if FigureTypeNum == 1    %组合模式
    handles.FigureType = 'combine';
    set(handles.ResultListPane,'Title','结果列表(左击选中,右击打开)')
else    %独立模式
    handles.FigureType = 'alone';
    set(handles.ResultListPane,'Title','结果列表( 左击打开 )')
end
guidata(hObject,handles);
% --- Executes during object creation, after setting all properties.
function HoldMode_CreateFcn(hObject, ~, handles)
% hObject    handle to HoldMode (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
handles.FigureType = 'alone'; % 默认独立绘图模式
guidata(hObject,handles);
set(hObject,'Value',0)


% --- Executes on key press with focus on ResultList and none of its controls.
function ResultList_KeyPressFcn(hObject, eventdata, handles)
% hObject    handle to ResultList (see GCBO)
% eventdata  structure with the following fields (see UICONTROL)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)
disp('')


% --- Executes on button press in refrash.
function refrash_Callback(hObject, eventdata, handles)
% hObject    handle to refrash (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

delete(handles.ResultDisplay) ;
ResultDisplay ;


% --- Executes during object creation, after setting all properties.
function refrash_CreateFcn(hObject, eventdata, handles)
% hObject    handle to refrash (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes during object deletion, before destroying properties.
function CloseAll_DeleteFcn(hObject, eventdata, handles)
% hObject    handle to CloseAll (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
ColseAll(handles) ;
