% function [estimates,trajectoryEstimates] = estimator(MBM,model)
% 
% trajectoryEstimates = [];
% 
% estimates.g = zeros(1,0);
% estimates.x = zeros(4,0);
% estimates.X = zeros(2,2,0);
% 
% m=1;%形状分量的数量，本实验中固定为2，若不需要固定值需修改代码以避免使用cat函数保存不一致维度的数组
% estimates.gshape=zeros(1,m,0);%形状分量的泊松比率
% estimates.xshape=zeros(4,m,0);%形状分量的偏移状态
% estimates.Xshape=zeros(2,2,m,0);%形状分量代表的拓展椭圆
% 
% d = 2;
% 
% [~,idx] = max(MBM.w);
% table_entry = MBM.table(idx,:);
% for i = 1:length(table_entry)
% %     if table_entry(i) > 0 && MBM.track{i}(table_entry(i)).Bern.r >= 0%测试用
%     if table_entry(i) > 0 && MBM.track{i}(table_entry(i)).Bern.r >= model.exist_r
%         [~,ideath] = max(MBM.track{i}(table_entry(i)).Bern.w_death);
%         if ideath == length(MBM.track{i}(table_entry(i)).Bern.w_death)
%             estimates.g = [estimates.g MBM.track{i}(table_entry(i)).Bern.GGIW(end).a/MBM.track{i}(table_entry(i)).Bern.GGIW(end).b];
%             estimates.x = [estimates.x MBM.track{i}(table_entry(i)).Bern.GGIW(end).m];
%             X = MBM.track{i}(table_entry(i)).Bern.GGIW(end).V/(MBM.track{i}(table_entry(i)).Bern.GGIW(end).v-2*d-2);
%             estimates.X = cat(3,estimates.X,X);
%             %形状分量估计
%             g=zeros(1,0);
%             x=zeros(4,0);
%             Xshape=zeros(2,2,0);
%             shape=MBM.track{i}(table_entry(i)).Bern.GGIW.shape;
%             for j=1:length(shape)
%                 g = [g MBM.track{i}(table_entry(i)).Bern.GGIW(end).shape(j).a/MBM.track{i}(table_entry(i)).Bern.GGIW(end).shape(j).b];
%                 x = [x MBM.track{i}(table_entry(i)).Bern.GGIW(end).shape(j).m];
%                 Xj = MBM.track{i}(table_entry(i)).Bern.GGIW(end).shape(j).V/(MBM.track{i}(table_entry(i)).Bern.GGIW(end).shape(j).v-2*d-2);
%                 Xshape=cat(3,Xshape,Xj);
%             end
%             %形状估计结果叠放保存
%             estimates.gshape=cat(3,estimates.gshape,g);
%             estimates.xshape=cat(3,estimates.xshape,x);
%             estimates.Xshape=cat(4,estimates.Xshape,Xshape);
%         end
%         t_death = MBM.track{i}(table_entry(i)).Bern.t_death(ideath);
%         tlen = t_death - MBM.track{i}(table_entry(i)).Bern.t_birth + 1;
%         
%         trajectoryEstimates(end+1,1).t_birth = [MBM.track{i}(table_entry(i)).assocHistory(1).t];
%         trajectoryEstimates(end,1).t_death = t_death;
%         trajectoryEstimates(end,1).g = [MBM.track{i}(table_entry(i)).Bern.GGIW(1:tlen).a]./[MBM.track{i}(table_entry(i)).Bern.GGIW(1:tlen).b];
%         trajectoryEstimates(end,1).x = [MBM.track{i}(table_entry(i)).Bern.GGIW(1:tlen).m];
%         temp = arrayfun(@(x) x.V/(x.v-2*d-2), MBM.track{i}(table_entry(i)).Bern.GGIW(1:tlen), 'un', false);
%         trajectoryEstimates(end,1).X = zeros(d,d,length(temp));
%         for j = 1:length(temp)
%             trajectoryEstimates(end,1).X(:,:,j) = temp{j};
%         end
%     end
% end  
%     
%     
% end



function [estimates,trajectoryEstimates] = estimator(MBM,model)

trajectoryEstimates = [];

estimates.g = zeros(1,0);
estimates.x = zeros(4,0);
estimates.X = zeros(2,2,0);
d = 2;

[~,idx] = max(MBM.w);
table_entry = MBM.table(idx,:);
for i = 1:length(table_entry)
    if table_entry(i) > 0 && MBM.track{i}(table_entry(i)).Bern.r >= model.exist_r
        [~,ideath] = max(MBM.track{i}(table_entry(i)).Bern.w_death);
        if ideath == length(MBM.track{i}(table_entry(i)).Bern.w_death)
            estimates.g = [estimates.g MBM.track{i}(table_entry(i)).Bern.GGIW(end).a/MBM.track{i}(table_entry(i)).Bern.GGIW(end).b];
            estimates.x = [estimates.x MBM.track{i}(table_entry(i)).Bern.GGIW(end).m];
            X = MBM.track{i}(table_entry(i)).Bern.GGIW(end).V/(MBM.track{i}(table_entry(i)).Bern.GGIW(end).v-2*d-2);
            estimates.X = cat(3,estimates.X,X);
        end
        t_death = MBM.track{i}(table_entry(i)).Bern.t_death(ideath);
        tlen = t_death - MBM.track{i}(table_entry(i)).Bern.t_birth + 1;
        
        trajectoryEstimates(end+1,1).t_birth = [MBM.track{i}(table_entry(i)).assocHistory(1).t];
        trajectoryEstimates(end,1).t_death = t_death;
        trajectoryEstimates(end,1).g = [MBM.track{i}(table_entry(i)).Bern.GGIW(1:tlen).a]./[MBM.track{i}(table_entry(i)).Bern.GGIW(1:tlen).b];
        trajectoryEstimates(end,1).x = [MBM.track{i}(table_entry(i)).Bern.GGIW(1:tlen).m];
        temp = arrayfun(@(x) x.V/(x.v-2*d-2), MBM.track{i}(table_entry(i)).Bern.GGIW(1:tlen), 'un', false);
        trajectoryEstimates(end,1).X = zeros(d,d,length(temp));
        for j = 1:length(temp)
            trajectoryEstimates(end,1).X(:,:,j) = temp{j};
        end
    end
end  
    
    
end

