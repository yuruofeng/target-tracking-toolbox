function est = run_filter(model,meas)

% This is the MATLAB code for the (joint prediction and update) implementation of the Generalized Labeled Multi-Bernoulli filter proposed in
% B.-T. Vo, and B.-N. Vo, "An Efficient Implementation of the Generalized Labeled Multi-Bernoulli Filter," IEEE Trans. Signal Processing, Vol. 65, No. 8, pp. 1975-1987, 2017.
% http://ba-ngu.vo-au.com/vo/VVH_FastGLMB_TSP17.pdf
% corresponding to the (separate prediction and update) implementation proposed in
% B.-N. Vo, B.-T. Vo, and D. Phung, "Labeled Random Finite Sets and the Bayes Multi-Target Tracking Filter," IEEE Trans. Signal Processing, Vol. 62, No. 24, pp. 6554-6567, 2014
% http://ba-ngu.vo-au.com/vo/VVP_GLMB_TSP14.pdf
% of the original filter proposed in
% B.-T. Vo, and B.-N. Vo, "Labeled Random Finite Sets and Multi-Object Conjugate Priors," IEEE Trans. Signal Processing, Vol. 61, No. 13, pp. 3460-3475, 2013.
% http://ba-ngu.vo-au.com/vo/VV_Conjugate_TSP13.pdf
%
% ---BibTeX entry
% @ARTICLE{GLMB3,
% author={B.-N. Vo and B.-T. Vo and H. Hung},
% journal={IEEE Transactions on Signal Processing},
% title={An Efficient Implementation of the Generalized Labeled Multi-Bernoulli Filter},
% year={2017},
% month={Apr}
% volume={65},
% number={8},
% pages={1975-1987}}
%
% @ARTICLE{GLMB2,
% author={B.-T. Vo and B.-N. Vo and D. Phung},
% journal={IEEE Transactions on Signal Processing},
% title={Labeled Random Finite Sets and the Bayes Multi-Target Tracking Filter},
% year={2014},
% month={Dec}
% volume={62},
% number={24},
% pages={6554-6567}}
%
% @ARTICLE{GLMB1,
% author={B.-T. Vo and B.-N. Vo
% journal={IEEE Transactions on Signal Processing},
% title={Labeled Random Finite Sets and Multi-Object Conjugate Priors},
% year={2013},
% month={Jul}
% volume={61},
% number={13},
% pages={3460-3475}}
%---

%=== Setup

%output variables
est.X= cell(meas.K,1);
est.N= zeros(meas.K,1);
est.L= cell(meas.K,1);
est.T= {};
est.M= 0;
est.J= []; est.H= {};

%filter parameters
filter.H_upd= 1000;                 %requested number of updated components/hypotheses
filter.H_max= 1000;                 %cap on number of posterior components/hypotheses
filter.hyp_threshold= 1e-15;         %pruning threshold for components/hypotheses

filter.L_max= 100;                  %limit on number of Gaussians in each track - not implemented yet
filter.elim_threshold= 1e-5;        %pruning threshold for Gaussians in each track - not implemented yet
filter.merge_threshold= 4;          %merging threshold for Gaussians in each track - not implemented yet

filter.P_G= 0.9999999;                           %gate size in percentage
filter.gamma= chi2inv(filter.P_G,model.z_dim);   %inv chi^2 dn gamma value
filter.gate_flag= 1;                             %gating on or off 1/0

filter.run_flag= 'disp';            %'disp' or 'silence' for on the fly output

est.filter= filter;


%=== Filtering

%initial prior
glmb_update.tt= cell(0,1);      %track table for GLMB (cell array of structs for individual tracks)
glmb_update.w= 1;               %vector of GLMB component/hypothesis weights
glmb_update.I= {[]};            %cell of GLMB component/hypothesis labels (labels are indices/entries in track table)
glmb_update.n= 0;               %vector of GLMB component/hypothesis cardinalities
glmb_update.cdn= 1;             %cardinality distribution of GLMB (vector of cardinality distribution probabilities)

%recursive filtering
for k=1:meas.K
    
    %joint prediction and update
    glmb_update= jointpredictupdate(glmb_update,model,filter,meas,k);      H_posterior= length(glmb_update.w);
    
    %pruning and truncation
    glmb_update= prune(glmb_update,filter);                     H_prune= length(glmb_update.w);
    glmb_update= cap(glmb_update,filter);                       H_cap= length(glmb_update.w);
    
    %state estimation and display diagnostics
    est= extract_estimates_recursive(glmb_update,model,meas,est); %[est.X{k},est.N(k),est.L{k}]= extract_estimates(glmb_update,model);
    display_diaginfo(glmb_update,k,est,filter,H_posterior,H_posterior,H_prune,H_cap);
    
end
end

function glmb_nextupdate= jointpredictupdate(glmb_update,model,filter,meas,k)
%---generate next update
%create birth tracks
tt_birth= cell(length(model.r_birth),1);                                            %initialize cell array
for tabbidx=1:length(model.r_birth)
    tt_birth{tabbidx}.m= model.m_birth{tabbidx};                                   %means of Gaussians for birth track
    tt_birth{tabbidx}.P= model.P_birth{tabbidx};                                   %covs of Gaussians for birth track
    tt_birth{tabbidx}.w= model.w_birth{tabbidx}(:);                                %weights of Gaussians for birth track
    tt_birth{tabbidx}.l= [k;tabbidx];                                              %track label
    tt_birth{tabbidx}.ah= [];                                                      %track association history (empty at birth)
end

%create surviving tracks - via time prediction (single target CK)
tt_survive= cell(length(glmb_update.tt),1);                                                                                 %initialize cell array
for tabsidx=1:length(glmb_update.tt)
    [mtemp_predict,Ptemp_predict]= kalman_predict_multiple(model,glmb_update.tt{tabsidx}.m,glmb_update.tt{tabsidx}.P);      %kalman prediction for GM
    tt_survive{tabsidx}.m= mtemp_predict;                                                                                   %means of Gaussians for surviving track
    tt_survive{tabsidx}.P= Ptemp_predict;                                                                                   %covs of Gaussians for surviving track
    tt_survive{tabsidx}.w= glmb_update.tt{tabsidx}.w;                                                                       %weights of Gaussians for surviving track
    tt_survive{tabsidx}.l= glmb_update.tt{tabsidx}.l;                                                                       %track label
    tt_survive{tabsidx}.ah= glmb_update.tt{tabsidx}.ah;                                                                     %track association history (no change at prediction)
end

%create predicted tracks - concatenation of birth and survival
glmb_predict.tt= cat(1,tt_birth,tt_survive);                                                                                %copy track table back to GLMB struct

%gating by tracks
if filter.gate_flag
    for tabidx=1:length(glmb_predict.tt)
        glmb_predict.tt{tabidx}.gatemeas= gate_meas_gms_idx(meas.Z{k},filter.gamma,model,glmb_predict.tt{tabidx}.m,glmb_predict.tt{tabidx}.P);
    end
else
    for tabidx=1:length(glmb_predict.tt)
        glmb_predict.tt{tabidx}.gatemeas= 1:size(meas.Z{k},2);
    end
end

%precalculation loop for average survival/death probabilities
avps= [model.r_birth; zeros(length(glmb_update.tt),1)];
for tabidx=1:length(glmb_update.tt)
    avps(model.T_birth+tabidx)= model.P_S;
end
avqs= 1-avps;

%precalculation loop for average detection/missed probabilities
avpd= zeros(length(glmb_predict.tt),1);
for tabidx=1:length(glmb_predict.tt)
    avpd(tabidx)= model.P_D;
end
avqd= 1-avpd;

%create updated tracks (single target Bayes update)
m= size(meas.Z{k},2);                                   %number of measurements
tt_update= cell((1+m)*length(glmb_predict.tt),1);       %initialize cell array
%missed detection tracks (legacy tracks)
for tabidx= 1:length(glmb_predict.tt)
    tt_update{tabidx}= glmb_predict.tt{tabidx};         %same track table
    tt_update{tabidx}.ah= [tt_update{tabidx}.ah; 0];    %track association history (updated for missed detection)
end
%measurement updated tracks (all pairs)
allcostm= zeros(length(glmb_predict.tt),m);
for tabidx= 1:length(glmb_predict.tt)
    for emm= glmb_predict.tt{tabidx}.gatemeas
            stoidx= length(glmb_predict.tt)*emm + tabidx; %index of predicted track i updated with measurement j is (number_predicted_tracks*j + i)
            [qz_temp,m_temp,P_temp] = kalman_update_multiple(meas.Z{k}(:,emm),model,glmb_predict.tt{tabidx}.m,glmb_predict.tt{tabidx}.P);   %kalman update for this track and this measurement
            w_temp= qz_temp.*glmb_predict.tt{tabidx}.w+eps;                                                                                 %unnormalized updated weights
            tt_update{stoidx}.m= m_temp;                                                                                                    %means of Gaussians for updated track
            tt_update{stoidx}.P= P_temp;                                                                                                    %covs of Gaussians for updated track
            tt_update{stoidx}.w= w_temp/sum(w_temp);                                                                                        %weights of Gaussians for updated track
            tt_update{stoidx}.l = glmb_predict.tt{tabidx}.l;                                                                                %track label
            tt_update{stoidx}.ah= [glmb_predict.tt{tabidx}.ah; emm];                                                                        %track association history (updated with new measurement)
            allcostm(tabidx,emm)= sum(w_temp);                                                                                              %predictive likelihood
    end
end
glmb_nextupdate.tt= tt_update;                                                                                                          %copy track table back to GLMB struct
%joint cost matrix
jointcostm= [diag(avqs) ...
             diag(avps.*avqd) ...
             repmat(avps.*avpd,[1 m]).*allcostm/(model.lambda_c*model.pdf_c)];
%gated measurement index matrix
gatemeasidxs= zeros(length(glmb_predict.tt),m);
for tabidx= 1:length(glmb_predict.tt)
    gatemeasidxs(tabidx,1:length(glmb_predict.tt{tabidx}.gatemeas))= glmb_predict.tt{tabidx}.gatemeas;
end
gatemeasindc= gatemeasidxs>0;
         

%component updates
runidx= 1;
for pidx=1:length(glmb_update.w)
    %calculate best updated hypotheses/components
    cpreds= length(glmb_predict.tt);
    nbirths= model.T_birth;
    nexists= length(glmb_update.I{pidx});
    ntracks= nbirths + nexists;
    tindices= [1:nbirths nbirths+glmb_update.I{pidx}'];                                                                                 %indices of all births and existing tracks  for current component
    lselmask= false(length(glmb_predict.tt),m); lselmask(tindices,:)= gatemeasindc(tindices,:);                                         %logical selection mask to index gating matrices
    mindices= unique_faster(gatemeasidxs(lselmask));                                                                                    %union indices of gated measurements for corresponding tracks
    costm= jointcostm(tindices,[tindices cpreds+tindices 2*cpreds+mindices]);                                                           %cost matrix - [no_birth/is_death | born/survived+missed | born/survived+detected]
    neglogcostm= -log(costm);                                                                                                           %negative log cost
    [uasses,nlcost]= gibbswrap_jointpredupdt_custom(neglogcostm,round(filter.H_upd*sqrt(glmb_update.w(pidx))/sum(sqrt(glmb_update.w))));%murty's algo/gibbs sampling to calculate m-best assignment hypotheses/components
    uasses(uasses<=ntracks)= -inf;                                                                                                      %set not born/track deaths to -inf assignment
    uasses(uasses>ntracks & uasses<= 2*ntracks)= 0;                                                                                     %set survived+missed to 0 assignment
    uasses(uasses>2*ntracks)= uasses(uasses>2*ntracks)-2*ntracks;                                                                       %set survived+detected to assignment of measurement index from 1:|Z|    
    uasses(uasses>0)= mindices(uasses(uasses>0));                                                                                       %restore original indices of gated measurements
    
    %generate corrresponding jointly predicted/updated hypotheses/components
    for hidx=1:length(nlcost)
        update_hypcmp_tmp= uasses(hidx,:)'; 
        update_hypcmp_idx= cpreds.*update_hypcmp_tmp+[(1:nbirths)'; nbirths+glmb_update.I{pidx}];
        glmb_nextupdate.w(runidx)= -model.lambda_c+m*log(model.lambda_c*model.pdf_c)+log(glmb_update.w(pidx))-nlcost(hidx);                                             %hypothesis/component weight
        glmb_nextupdate.I{runidx}= update_hypcmp_idx(update_hypcmp_idx>0);                                                                                              %hypothesis/component tracks (via indices to track table)
        glmb_nextupdate.n(runidx)= sum(update_hypcmp_idx>0);                                                                                                            %hypothesis/component cardinality
        runidx= runidx+1;
    end
end

glmb_nextupdate.w= exp(glmb_nextupdate.w-logsumexp(glmb_nextupdate.w));                                                                                                                 %normalize weights

%extract cardinality distribution
for card=0:max(glmb_nextupdate.n)
    glmb_nextupdate.cdn(card+1)= sum(glmb_nextupdate.w(glmb_nextupdate.n==card));                                                                                                       %extract probability of n targets
end

%remove duplicate entries and clean track table
glmb_nextupdate= clean_update(clean_predict(glmb_nextupdate));
end



function glmb_temp= clean_predict(glmb_raw)
%hash label sets, find unique ones, merge all duplicates
for hidx= 1:length(glmb_raw.w)
    glmb_raw.hash{hidx}= sprintf('%i*',sort(glmb_raw.I{hidx}(:)'));
end

[cu,~,ic]= unique(glmb_raw.hash);

glmb_temp.tt= glmb_raw.tt;
glmb_temp.w= zeros(length(cu),1);
glmb_temp.I= cell(length(cu),1);
glmb_temp.n= zeros(length(cu),1);
for hidx= 1:length(ic)
        glmb_temp.w(ic(hidx))= glmb_temp.w(ic(hidx))+glmb_raw.w(hidx);
        glmb_temp.I{ic(hidx)}= glmb_raw.I{hidx};
        glmb_temp.n(ic(hidx))= glmb_raw.n(hidx);
end
glmb_temp.cdn= glmb_raw.cdn;
end



function glmb_clean= clean_update(glmb_temp)
%flag used tracks
usedindicator= zeros(length(glmb_temp.tt),1);
for hidx= 1:length(glmb_temp.w)
    usedindicator(glmb_temp.I{hidx})= usedindicator(glmb_temp.I{hidx})+1;
end
trackcount= sum(usedindicator>0);

%remove unused tracks and reindex existing hypotheses/components
newindices= zeros(length(glmb_temp.tt),1); newindices(usedindicator>0)= 1:trackcount;
glmb_clean.tt= glmb_temp.tt(usedindicator>0);
glmb_clean.w= glmb_temp.w;
for hidx= 1:length(glmb_temp.w)
    glmb_clean.I{hidx}= newindices(glmb_temp.I{hidx});
end
glmb_clean.n= glmb_temp.n;
glmb_clean.cdn= glmb_temp.cdn;
end



function glmb_out= prune(glmb_in,filter)
%prune components with weights lower than specified threshold
idxkeep= find(glmb_in.w > filter.hyp_threshold);
glmb_out.tt= glmb_in.tt;
glmb_out.w= glmb_in.w(idxkeep);
glmb_out.I= glmb_in.I(idxkeep);
glmb_out.n= glmb_in.n(idxkeep);

glmb_out.w= glmb_out.w/sum(glmb_out.w);
for card=0:max(glmb_out.n)
    glmb_out.cdn(card+1)= sum(glmb_out.w(glmb_out.n==card));
end
end



function glmb_out= cap(glmb_in,filter)
%cap total number of components to specified maximum
if length(glmb_in.w) > filter.H_max
    [~,idxsort]= sort(glmb_in.w,'descend');
    idxkeep=idxsort(1:filter.H_max);
    glmb_out.tt= glmb_in.tt;
    glmb_out.w= glmb_in.w(idxkeep);
    glmb_out.I= glmb_in.I(idxkeep);
    glmb_out.n= glmb_in.n(idxkeep);
    
    glmb_out.w= glmb_out.w/sum(glmb_out.w);
    for card=0:max(glmb_out.n)
        glmb_out.cdn(card+1)= sum(glmb_out.w(glmb_out.n==card));
    end
else
    glmb_out= glmb_in;
end
end



function est=extract_estimates_recursive(glmb,model,meas,est)
%extract estimates via recursive estimator, where  
%trajectories are extracted via association history, and
%track continuity is guaranteed with a non-trivial estimator

%extract MAP cardinality and corresponding highest weighted component
[~,mode] = max(glmb.cdn); 
M = mode-1;
T= cell(M,1);
J= zeros(2,M);

[~,idxcmp]= max(glmb.w.*(glmb.n==M));
for m=1:M
    idxptr= glmb.I{idxcmp}(m);
    T{m,1}= glmb.tt{idxptr}.ah;
    J(:,m)= glmb.tt{idxptr}.l;
end

H= cell(M,1);
for m=1:M
   H{m}= [num2str(J(1,m)),'.',num2str(J(2,m))]; 
end

%compute dead & updated & new tracks
[~,io,is]= intersect(est.H,H);
[~,id,in]= setxor(est.H,H);

est.M= M;
est.T= cat(1,est.T(id),T(is),T(in));
est.J= cat(2,est.J(:,id),J(:,is),J(:,in));
est.H= cat(1,est.H(id),H(is),H(in));

%write out estimates in standard format
est.N= zeros(meas.K,1);
est.X= cell(meas.K,1);
est.L= cell(meas.K,1);
for t=1:length(est.T)
    ks= est.J(1,t);
    bidx= est.J(2,t);
    tah= est.T{t};
    
    w= model.w_birth{bidx};
    m= model.m_birth{bidx};
    P= model.P_birth{bidx};
    for u=1:length(tah)
        [m,P] = kalman_predict_multiple(model,m,P);
        k= ks+u-1;
        emm= tah(u);
        if emm > 0
            [qz,m,P] = kalman_update_multiple(meas.Z{k}(:,emm),model,m,P);
            w= qz.*w+eps;
            w= w/sum(w);
        end

        [~,idxtrk]= max(w);
        est.N(k)= est.N(k)+1;
        est.X{k}= cat(2,est.X{k},m(:,idxtrk));
        est.L{k}= cat(2,est.L{k},est.J(:,t));
    end
end
end



function [X,N,L]=extract_estimates(glmb,model)
%extract estimates via best cardinality, then 
%best component/hypothesis given best cardinality, then
%best means of tracks given best component/hypothesis and cardinality
[~,mode] = max(glmb.cdn);
N = mode-1;
X= zeros(model.x_dim,N);
L= zeros(2,N);

[~,idxcmp]= max(glmb.w.*(glmb.n==N));
for n=1:N
    [~,idxtrk]= max(glmb.tt{glmb.I{idxcmp}(n)}.w);
    X(:,n)= glmb.tt{glmb.I{idxcmp}(n)}.m(:,idxtrk);
    L(:,n)= glmb.tt{glmb.I{idxcmp}(n)}.l;
end
end



function display_diaginfo(glmb,k,est,filter,H_predict,H_posterior,H_prune,H_cap)
if ~strcmp(filter.run_flag,'silence')
    disp([' time= ',num2str(k),...
        ' #eap cdn=' num2str((0:(length(glmb.cdn)-1))*glmb.cdn(:)),...
        ' #var cdn=' num2str((0:(length(glmb.cdn)-1)).^2*glmb.cdn(:)-((0:(length(glmb.cdn)-1))*glmb.cdn(:))^2,4),...
        ' #est card=' num2str(est.N(k),4),...
        ' #comp pred=' num2str(H_predict,4),...
        ' #comp post=' num2str(H_posterior,4),...
        ' #comp updt=',num2str(H_cap),...
        ' #trax updt=',num2str(length(glmb.tt),4)   ]);
end
end