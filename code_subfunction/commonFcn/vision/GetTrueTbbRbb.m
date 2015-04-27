%% 从真实轨迹计算真实的 Tbb Rbb
% isTbb_last=1 : Tbb 在上一时刻分解，
% isTbb_last=0 : Tbb 在后一时刻分解
function [ trueTbb,trueRbb  ] = GetTrueTbbRbb(trueTrace,visualFre,isTbb_last)
format long

position = trueTrace.position ;
attitude = trueTrace.attitude ;
trueFre = trueTrace.frequency;

if isempty(visualFre)
    answer = inputdlg('视觉信息频率');
    visualFre = str2double(answer);
end
visualNum = fix( (length(position)-1)*visualFre/trueFre);
Rbb = zeros(3,3,visualNum);
Tbb = zeros(3,visualNum);
% 根据真实轨迹生成真实 Tbb Rbb
for k=1:visualNum
    k_true_last = 1+fix((k-1)*trueFre/visualFre) ;
    k_true = 1+fix((k)*trueFre/visualFre) ;
    if isTbb_last==1    % 得到 Tbb_last 在上一时刻分解
        Tbb(:,k) = FCbn(attitude(:,k_true_last))' * ( position(:,k_true)-position(:,k_true_last) ) ;
        
    else                % 得到 Tbb 在后一时刻分解
        Tbb(:,k) = FCbn(attitude(:,k_true))' * ( position(:,k_true)-position(:,k_true_last) ) ;
    end
    Rbb(:,:,k) =  FCbn(attitude(:,k_true))' * FCbn(attitude(:,k_true_last)) ;     % R:b(k)->b(k+1)

end
trueTbb = Tbb ;
trueRbb = Rbb;