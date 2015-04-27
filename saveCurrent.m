name=get(gcf,'name');

figurePosition = [300 300 450 412] ;
set(gcf,'position',figurePosition)

saveas(gcf,[name,'.fig'])
saveas(gcf,[name,'.jpg'])
saveas(gcf,[name,'.emf'])
disp(['save ',name,'OK'])