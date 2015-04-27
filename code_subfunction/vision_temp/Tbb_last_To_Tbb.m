% Tbb 将Tbb中的 Tbb_last 生成 Tbb（在后一时刻的Tbb）

visualInputData = importdata('visualInputData.mat');
VisualRT = visualInputData.VisualRT ;
Tbb_last = VisualRT.Tbb_last ;
Rbb = VisualRT.Rbb ;
Tbb = zeros(size(Tbb_last));

for k=1:length(Tbb_last)
    Tbb(:,k) = Rbb(:,:,k)*Tbb_last(:,k) ;
end

VisualRT.Tbb=Tbb;
visualInputData.VisualRT=VisualRT ;

save visualInputData visualInputData
save VisualRT VisualRT