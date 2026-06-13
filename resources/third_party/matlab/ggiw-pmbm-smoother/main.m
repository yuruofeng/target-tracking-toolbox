clear;clc;close all;
dbstop if error
%Choose a scenario: Scenario 1: 27 targets born at four different
%locations; Scenario 2: targets move in proximity.
scenario = 1;

%Parameter setting
if scenario == 1
    %交叉曲线
    modelparas1;
elseif scenario == 2
    modelparas2;
elseif scenario == 3
    %单目标曲线
    modelparas3;
else
    modelparas4;
end

%Number of Monte Carlo Simulations
numMC = 1;
%numMC = length(Scenario.Z);

%Parameters used in GOSPA metric
c = 20;
p = 1;

%Number of time steps
K = model.K;

%每smooth_step次，平滑一次
smooth_step=6;

GOSPA = zeros(K,4,numMC);
trajectoryEstimates = cell(numMC,1);
simulation_time = zeros(numMC,1);

T_smoothUpdate_partition = zeros(1,K);
T_smooth_partition=zeros(1,K);
 X_s=[];Y_s=[];X_o=[];Y_o=[];
%tic
for t = 1:numMC
    Z = Scenario.Z{t};
    % Initialisation
    PPP.w = log(model.birth.w);
    PPP.GGIW = model.birth.GGIW;
    MBM.w = [];     % Global hypotheses weights
    MBM.track = {}; % Locl hypotheses trees
    MBM.table = []; % Global hypotheses look-up table
    
    estimates = cell(K,1);
    tic
    for k = 1:K
         pause(0);
         [t,k]
         hold on
       % plot(Z{k}(1,:),Z{k}(2,:),"*");
        
       %平滑的update时间代价
        tic
        %Update step
        [PPP,MBM] = updatePMBM(PPP,MBM,Z{k},k,model);
        %平滑
        
        for i=1:length(MBM.track)
            for j=1:length(MBM.track{i})
                if mod(length(MBM.track{i}(j).Bern.GGIW),smooth_step)==0
                    GGIW_=SmootherAdapter(MBM.track{i}(j).Bern,model);
                    for s=0:smooth_step-1
                       MBM.track{i}(j).Bern.GGIW(end-s).m= GGIW_(end-s).m;
                       MBM.track{i}(j).Bern.GGIW(end-s).P= GGIW_(end-s).P;
                       MBM.track{i}(j).Bern.GGIW(end-s).v= GGIW_(end-s).v;
                       MBM.track{i}(j).Bern.GGIW(end-s).V= GGIW_(end-s).V;
                    end
                end
            end
        end
        T_smoothUpdate_partition(k) = toc;
        T_smooth=sum(T_smoothUpdate_partition);

%         %画当前量测
%         for I=1:length(MBM.track)
%             for J=1:length(MBM.track{I})
%                 plot(Z{k}(1,MBM.track{I}(J).assocHistory(end).meas(:)),Z{k}(2,MBM.track{I}(J).assocHistory(end).meas(:)),".");
%             end
%         end
        %Extract estimates (both estimate of the current time and the
        %estimate of the full trajectory) 
        [estimates{k},trajectoryEstimates{t}{k}] = estimator(MBM,model);
        %[estimates{t,1}{k},trajectoryEstimates{t}{k}] = estimator(MBM,model);

        %Evaluate filtering performance using GOSPA
        GOSPA(k,:,t) = GOSPAmetric(estimates{k},groundTruth{k},c,p);
        %GOSPA(k,:,t) = GOSPAmetric(estimates{t,1}{k},groundTruth{k},c,p);

% 
%         %smooth-Prediction Step
         i0=size(estimates{k}.g,2);
         for j = 1:i0
           %  figure(1);
             [x_s, y_s] = Sigmacircle(estimates{k}.x(1,j),estimates{k}.x(2,j),estimates{k}.X(:,:,j),2,1);
             %[x_s,y_s]=[[x_s,y_s] [x, y]];
             X_s=[X_s x_s];
             Y_s=[Y_s y_s];
         end
         drawnow;
        if k < K
            [PPP,MBM] = predictPMBM(PPP,MBM,model);
        end
    end
   % simulation_time(t) = toc;
end
GOSPA02_smooth = sum(GOSPA,3) / numMC;

% %smooth——GOSPA%%%%%%%%%%%%%%%%%%%%%
% for i=1:4
%     figure(1+i)
%     plot(1:K,GOSPA02(:,i),'-r');
%     hold on;
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%原算法
T_Update_partition=zeros(1,K);
for t = 1:numMC
    Z = Scenario.Z{t};
    % Initialisation
    PPP.w = log(model.birth.w);
    PPP.GGIW = model.birth.GGIW;
    MBM.w = [];     % Global hypotheses weights
    MBM.track = {}; % Locl hypotheses trees
    MBM.table = []; % Global hypotheses look-up table
    
    estimates = cell(K,1);
    
   % tic
    for k = 1:K
        %Print info
        pause(0);
       % figure(1);
%         clf(1);
%         axis([-200,200,-200,200]);
%         
        hold on;
         [t,k]
%         plot(Z{k}(1,:),Z{k}(2,:),"*");

        tic
        %Update step
        [PPP,MBM] = updatePMBM(PPP,MBM,Z{k},k,model);
        T_Update_partition(k)=toc;
        T_Update=sum(T_Update_partition);

        %Extract estimates (both estimate of the current time and the
        %estimate of the full trajectory) 
        [estimates{k},trajectoryEstimates{t}{k}] = estimator(MBM,model);
        %Evaluate filtering performance using GOSPA
        GOSPA(k,:,t) = GOSPAmetric(estimates{k},groundTruth{k},c,p);
        %Prediction Step
        
        %原算法跟踪图
         i0=size(estimates{k}.g,2);
         for j = 1:i0
           %figure(1);
             [x_o, y_o] = Sigmacircle_e(estimates{k}.x(1,j),estimates{k}.x(2,j),estimates{k}.X(:,:,j),2,3);
             X_o=[X_o x_o];
             Y_o=[Y_o y_o];
         end
         drawnow;
        if k < K
            [PPP,MBM] = predictPMBM(PPP,MBM,model);
        end
    end
    %simulation_time(t) = toc;
end
GOSPA02 = sum(GOSPA,3) / numMC;

% % PMBM-GOSPA %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% for i=1:4
%     figure(1+i)
%     plot(1:K,GOSPA02(:,i),'-b');
%     hold on;
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%全部GOSPA%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%gospa
figure(2)
plot(1:K,GOSPA02_smooth(:,1),'-r','linewidth',1.8);hold on;
plot(1:K,GOSPA02(:,1),':b','linewidth',1.8);hold on;
set(legend('$MTS-PMBM$',...
           '$PMBM$',...
           'Location','northeast'),'Interpreter','latex','FontSize',10,'fontname','Times New Roman','FontWeight','no')

set(gca,'FontSize',8,'fontname','Times New Roman')  %是设置刻度字体大小
xlabel('$Time(s)$','Interpreter','latex','FontSize',10,'fontname','Times New Roman')
ylabel('$GOSPA$','Interpreter','latex','FontSize',10,'fontname','Times New Roman')

%LocationError
figure(3)
plot(1:K,GOSPA02_smooth(:,2),'-r','linewidth',1.8);hold on;
plot(1:K,GOSPA02(:,2),':b','linewidth',1.8);hold on;
set(legend('$MTS-PMBM$',...
           '$PMBM$',...
           'Location','northeast'),'Interpreter','latex','FontSize',10,'fontname','Times New Roman','FontWeight','no')

set(gca,'FontSize',8,'fontname','Times New Roman')  %是设置刻度字体大小
xlabel('$Time(s)$','Interpreter','latex','FontSize',10,'fontname','Times New Roman')
ylabel('$LocationError$','Interpreter','latex','FontSize',10,'fontname','Times New Roman')

%MissedError
figure(4)
plot(1:K,GOSPA02_smooth(:,3),'-r','linewidth',1.8);hold on;
plot(1:K,GOSPA02(:,3),':b','linewidth',1.8);hold on;
set(legend('$MTS-PMBM$',...
           '$PMBM$',...
           'Location','northeast'),'Interpreter','latex','FontSize',10,'fontname','Times New Roman','FontWeight','no')

set(gca,'FontSize',8,'fontname','Times New Roman')  %是设置刻度字体大小
xlabel('$Time(s)$','Interpreter','latex','FontSize',10,'fontname','Times New Roman')
ylabel('$MissedError$','Interpreter','latex','FontSize',10,'fontname','Times New Roman')

%FalseError
figure(5)
plot(1:K,GOSPA02_smooth(:,3),'-r','linewidth',1.8);hold on;
plot(1:K,GOSPA02(:,3),':b','linewidth',1.8);hold on;
set(legend('$MTS-PMBM$',...
           '$PMBM$',...
           'Location','northeast'),'Interpreter','latex','FontSize',10,'fontname','Times New Roman','FontWeight','no')

set(gca,'FontSize',8,'fontname','Times New Roman')  %是设置刻度字体大小
xlabel('$Time(s)$','Interpreter','latex','FontSize',10,'fontname','Times New Roman')
ylabel('$FalseError$','Interpreter','latex','FontSize',10,'fontname','Times New Roman')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%时间代价图%%%%%%%%%%%%%%%%%%%%%%%%%%%%
YY1=[];
YY2=[];
X=1:100;
Y1=T_Update_partition(1,:);
Y2=T_smoothUpdate_partition(1,:);
YY1=[YY1 ;[T_Update_partition(1,:)]];
YY2=[YY2 ;[T_smoothUpdate_partition(1,:)]];

Y1_bar=mean(YY1,1);
Y2_bar=mean(YY2,1);
figure(7)
plot(X,Y2_bar,'r-','LineWidth',1.8);hold on;
plot(X,Y1_bar,'b:','LineWidth',1.8);hold on;
%plot(X,Y1_bar,'b:',X,Y2_bar,'r-');

set(legend('$MTS-PMBM$',...
           '$PMBM$',...
           'Location','northeast'),'Interpreter','latex','FontSize',10,'fontname','Times New Roman','FontWeight','no')

set(gca,'FontSize',8,'fontname','Times New Roman')  %是设置刻度字体大小
set(gca, 'XLim', [0,100])
set(gca, 'YLim', [0,0.1])
xlabel('$Time(s)$','Interpreter','latex','FontSize',12,'fontname','Times New Roman')
ylabel('$Time Cost(s)$','Interpreter','latex','FontSize',12,'fontname','Times New Roman')

n_LOW_rate=0;
for i=1:length(Y1_bar)
    if Y2_bar(1,i)<=Y1_bar(1,i)
        n_LOW_rate=n_LOW_rate+1;  
    end
end
n_LOW_rate = n_LOW_rate/K;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%画全部量测和跟踪结果%%%%%%%%%%%%%%%%%%%%%%
figure(1)
     c = ['r' 'g' 'b' 'c' 'm' 'y' 'k' 'w'];
%     %%  红   绿  蓝 青绿 洋红 黄  黑  白
  ZZ=[];
 for i=1:length(Scenario.Z{1,1})
    ZZ=[ZZ [Scenario.Z{1,1}{i,1}(1,:);Scenario.Z{1,1}{i,1}(2,:)]];
 end

plot(ZZ(1,1),ZZ(2,1),'r-','linewidth',2)
plot(ZZ(1,1),ZZ(2,1),'b:','linewidth',2)
plot(ZZ(1,1),ZZ(2,1),'k.')
plot(X_s,Y_s,'r-','linewidth',2);
plot(X_o,Y_o,'b:','linewidth',2);
plot(ZZ(1,:),ZZ(2,:),'k.');

set(legend('$MTS-PMBM$',...
           '$PMBM$',...
           '$MEASUREMENTS$',...
           'Location','northeast'),'Interpreter','latex','FontSize',8,'fontname','Times New Roman')

set(gca,'FontSize',8,'fontname','Times New Roman')  %是设置刻度字体大小
set(gca, 'XLim', [-200,200])
set(gca, 'YLim', [-200,200])
xlabel('$X(m)$','Interpreter','latex','FontSize',10,'fontname','Times New Roman')
ylabel('$Y(m)$','Interpreter','latex','FontSize',10,'fontname','Times New Roman')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
