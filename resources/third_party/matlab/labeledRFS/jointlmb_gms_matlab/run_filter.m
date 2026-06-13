function est = run_filter(model,meas)

% This is the MATLAB code for the Labeled Multi-Bernoulli filter proposed in
% S. Reuter, B.-T. Vo, B.-N. Vo, and K. Dietmayer, "The labelled multi-Bernoulli filter," IEEE Trans. Signal Processing, Vol. 62, No. 12, pp. 3246-3260, 2014
% http://ba-ngu.vo-au.com/vo/RVVD_LMB_TSP14.pdf
% which propagates an LMB approximation of the GLMB update proposed in
% B.-T. Vo, and B.-N. Vo, "Labeled Random Finite Sets and Multi-Object Conjugate Priors," IEEE Trans. Signal Processing, Vol. 61, No. 13, pp. 3460-3475, 2013.
% http://ba-ngu.vo-au.com/vo/VV_Conjugate_TSP13.pdf
% and
% B.-N. Vo, B.-T. Vo, and D. Phung, "Labeled Random Finite Sets and the Bayes Multi-Target Tracking Filter," IEEE Trans. Signal Processing, Vol. 62, No. 24, pp. 6554-6567, 2014
% http://ba-ngu.vo-au.com/vo/VVP_GLMB_TSP14.pdf
% using an efficient implementation of the GLMB filter proposed in
% B.-T. Vo, and B.-N. Vo, "An Efficient Implementation of the Generalized Labeled Multi-Bernoulli Filter," IEEE Trans. Signal Processing, Vol. 65, No. 8, pp. 1975-1987, 2017.
% http://ba-ngu.vo-au.com/vo/VVH_FastGLMB_TSP17.pdf
%
%
% Note 1: no dynamic grouping or adaptive birth is implemented in this code, only the standard filter with static birth is given
% Note 2: the simple example used here is the same as in the CB-MeMBer filter code for a quick demonstration and comparison purposes
% Note 3: more difficult scenarios require more components/hypotheses (thus exec time)
% ---BibTeX entry
% @ARTICLE{LMB,
% author={S. Reuter and B.-T. Vo and B.-N. Vo and K. Dietmayer},
% journal={IEEE Transactions on Signal Processing},
% title={The Labeled Multi-Bernoulli Filter},
% year={2014},
% month={Jun}
% volume={62},
% number={12},
% pages={3246-3260}}
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
% @ARTICLE{GLMB3,
% author={B.-N. Vo and B.-T. Vo and H. Hung},
% journal={IEEE Transactions on Signal Processing},
% title={An Efficient Implementation of the Generalized Labeled Multi-Bernoulli Filter},
% year={2017},
% month={Apr}
% volume={65},
% number={8},
% pages={1975-1987}}
%---

%=== Setup

%output variables
est.X= cell(meas.K,1);
est.N= zeros(meas.K,1);
est.L= cell(meas.K,1);

%filter parameters
filter.T_max= 100;                  %maximum number of tracks
filter.track_threshold= 1e-3;       %threshold to prune tracks
filter.H_upd= 1000;                  %requested number of updated components/hypotheses (for GLMB update)

filter.L_max= 10;                   %limit on number of Gaussians in each track 
filter.elim_threshold= 1e-5;        %pruning threshold for Gaussians in each track 
filter.merge_threshold= 4;          %merging threshold for Gaussians in each track

filter.P_G= 0.9999999;                           %gate size in percentage
filter.gamma= chi2inv(filter.P_G,model.z_dim);   %inv chi^2 dn gamma value
filter.gate_flag= 1;                             %gating on or off 1/0

filter.run_flag= 'disp';            %'disp' or 'silence' for on the fly output

est.filter= filter;

%=== Filtering

%initial prior
tt_lmb_update= cell(0,1);      %track table for LMB (cell array of structs for individual tracks)

%recursive filtering
for k=1:meas.K
    
    %joint predict and update, results in GLMB, convert to LMB
    glmb_update= jointlmbpredictupdate(tt_lmb_update,model,filter,meas,k);              T_predict= length(tt_lmb_update)+model.T_birth;
    tt_lmb_update= glmb2lmb(glmb_update);                                               T_posterior= length(tt_lmb_update);
       
    %pruning, truncation and track cleanup
    tt_lmb_update= clean_lmb(tt_lmb_update,filter);                                     T_clean= length(tt_lmb_update);
    
    %state estimation
    [est.X{k},est.N(k),est.L{k}]= extract_estimates(tt_lmb_update,model);
    
    %display diagnostics
    display_diaginfo(tt_lmb_update,k,est,filter,T_predict,T_posterior,T_clean);
    
end
end



function glmb_nextupdate= jointlmbpredictupdate(tt_lmb_update,model,filter,meas,k)
%---generate birth tracks
tt_birth= cell(length(model.r_birth),1);                                           %initialize cell array
for tabbidx=1:length(model.r_birth)
    tt_birth{tabbidx}.r= model.r_birth(tabbidx);                                   %birth prob for birth track
    tt_birth{tabbidx}.m= model.m_birth{tabbidx};                                   %means of Gaussians for birth track
    tt_birth{tabbidx}.P= model.P_birth{tabbidx};                                   %covs of Gaussians for birth track
    tt_birth{tabbidx}.w= model.w_birth{tabbidx}(:);                                %weights of Gaussians for birth track
    tt_birth{tabbidx}.l= [k;tabbidx];                                              %track label
end

%---generate surviving tracks
tt_survive= cell(length(tt_lmb_update),1);                                                                              %initialize cell array
for tabsidx=1:length(tt_lmb_update)
    tt_survive{tabsidx}.r= model.P_S*tt_lmb_update{tabsidx}.r;                                                          %predicted existence probability for surviving track
    [mtemp_predict,Ptemp_predict]= kalman_predict_multiple(model,tt_lmb_update{tabsidx}.m,tt_lmb_update{tabsidx}.P);        %kalman prediction
    tt_survive{tabsidx}.m= mtemp_predict;                                                                               %means of Gaussians for surviving track
    tt_survive{tabsidx}.P= Ptemp_predict;                                                                               %covs of Gaussians for predicted track
    tt_survive{tabsidx}.w= tt_lmb_update{tabsidx}.w;                                                                    %weights of Gaussians for predicted track
    tt_survive{tabsidx}.l= tt_lmb_update{tabsidx}.l;                                                                    %track label
end

%create predicted tracks - concatenation of birth and survival
tt_predict= cat(1,tt_birth,tt_survive);                                                                                %copy track table back to GLMB struct

%gating by tracks
if filter.gate_flag
    for tabidx=1:length(tt_predict)
        tt_predict{tabidx}.gatemeas= gate_meas_gms_idx(meas.Z{k},filter.gamma,model,tt_predict{tabidx}.m,tt_predict{tabidx}.P);
    end
else
    for tabidx=1:length(tt_predict)
        tt_predict{tabidx}.gatemeas= 1:size(meas.Z{k},2);
    end
end

%precalculation loop for average survival/death probabilities
avps= zeros(length(tt_predict),1);
for tabidx=1:length(tt_predict)
    avps(tabidx)= tt_predict{tabidx}.r;
end
avqs= 1-avps;

%precalculation loop for average detection/missed probabilities
avpd= zeros(length(tt_predict),1);
for tabidx=1:length(tt_predict)
    avpd(tabidx)= model.P_D;
end
avqd= 1-avpd;

%create updated tracks (single target Bayes update)
m= size(meas.Z{k},2);                                   %number of measurements
tt_update= cell((1+m)*length(tt_predict),1);            %initialize cell array
%missed detection tracks (legacy tracks)
for tabidx= 1:length(tt_predict)
    tt_update{tabidx}= tt_predict{tabidx};              %same track table
end
%measurement updated tracks (all pairs)
allcostm= zeros(length(tt_predict),m);
for tabidx= 1:length(tt_predict)
    for emm= tt_predict{tabidx}.gatemeas
            stoidx= length(tt_predict)*emm + tabidx; %index of predicted track i updated with measurement j is (number_predicted_tracks*j + i)
            [qz_temp,m_temp,P_temp] = kalman_update_multiple(meas.Z{k}(:,emm),model,tt_predict{tabidx}.m,tt_predict{tabidx}.P);             %kalman update for this track and this measurement
            w_temp= qz_temp.*tt_predict{tabidx}.w+eps;                                                                                      %unnormalized updated weights
            tt_update{stoidx}.m= m_temp;                                                                                                    %means of Gaussians for updated track
            tt_update{stoidx}.P= P_temp;                                                                                                    %covs of Gaussians for updated track
            tt_update{stoidx}.w= w_temp/sum(w_temp);                                                                                        %weights of Gaussians for updated track
            tt_update{stoidx}.l = tt_predict{tabidx}.l;                                                                                     %track label
            allcostm(tabidx,emm)= sum(w_temp);                                                                                              %predictive likelihood
    end
end
glmb_nextupdate.tt= tt_update;                                                                                                              %copy track table back to GLMB struct
%joint cost matrix
jointcostm= [diag(avqs) ...
             diag(avps.*avqd) ...
             repmat(avps.*avpd,[1 m]).*allcostm/(model.lambda_c*model.pdf_c)];
%gated measurement index matrix
gatemeasidxs= zeros(length(tt_predict),m);
for tabidx= 1:length(tt_predict)
    gatemeasidxs(tabidx,1:length(tt_predict{tabidx}.gatemeas))= tt_predict{tabidx}.gatemeas;
end
gatemeasindc= gatemeasidxs>0;
         

%component updates

    %calculate best updated hypotheses/components
    cpreds= length(tt_predict);
    nbirths= model.T_birth;
    nexists= length(tt_lmb_update);
    ntracks= nbirths + nexists;
    tindices= [(1:nbirths) nbirths+(1:nexists)];                                                                                          %indices of all births and existing tracks  for current component
    lselmask= false(length(tt_predict),m); lselmask(tindices,:)= gatemeasindc(tindices,:);                                              %logical selection mask to index gating matrices
    mindices= unique_faster(gatemeasidxs(lselmask));                                                                                    %union indices of gated measurements for corresponding tracks
    costm= jointcostm(tindices,[tindices cpreds+tindices 2*cpreds+mindices]);                                                           %cost matrix - [no_birth/is_death | born/survived+missed | born/survived+detected]
    neglogcostm= -log(costm);                                                                                                           %negative log cost
    [uasses,nlcost]= gibbswrap_jointpredupdt_custom(neglogcostm,round(filter.H_upd));                                                   %murty's algo/gibbs sampling to calculate m-best assignment hypotheses/components
    uasses(uasses<=ntracks)= -inf;                                                                                                      %set not born/track deaths to -inf assignment
    uasses(uasses>ntracks & uasses<= 2*ntracks)= 0;                                                                                     %set survived+missed to 0 assignment
    uasses(uasses>2*ntracks)= uasses(uasses>2*ntracks)-2*ntracks;                                                                       %set survived+detected to assignment of measurement index from 1:|Z|    
    uasses(uasses>0)= mindices(uasses(uasses>0));                                                                                       %restore original indices of gated measurements
    
    %generate corrresponding jointly predicted/updated hypotheses/components
    for hidx=1:length(nlcost)
        update_hypcmp_tmp= uasses(hidx,:)'; 
        update_hypcmp_idx= cpreds.*update_hypcmp_tmp+[(1:nbirths)'; nbirths+(1:nexists)'];
        glmb_nextupdate.w(hidx)= -model.lambda_c+m*log(model.lambda_c*model.pdf_c)-nlcost(hidx);                                             %hypothesis/component weight
        glmb_nextupdate.I{hidx}= update_hypcmp_idx(update_hypcmp_idx>0);                                                                                              %hypothesis/component tracks (via indices to track table)
        glmb_nextupdate.n(hidx)= sum(update_hypcmp_idx>0);                                                                                                            %hypothesis/component cardinality
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



function tt_lmb= glmb2lmb(glmb)

%find unique labels (with different possibly different association histories)
lmat= zeros(2,length(glmb.tt),1);
for tabidx= 1:length(glmb.tt)
    lmat(:,tabidx)= glmb.tt{tabidx}.l;
end
lmat= lmat';

[cu,~,ic]= unique(lmat,'rows'); cu= cu';

%initialize LMB struct
tt_lmb= cell(size(cu,2),1);
for tabidx=1:length(tt_lmb)
   tt_lmb{tabidx}.r= 0;
   tt_lmb{tabidx}.m= [];
   tt_lmb{tabidx}.P= [];
   tt_lmb{tabidx}.w= [];
   tt_lmb{tabidx}.l= cu(:,tabidx);
end

%extract individual tracks
for hidx=1:length(glmb.w)
   for t= 1:glmb.n(hidx)
      trkidx= glmb.I{hidx}(t);
      newidx= ic(trkidx);
      tt_lmb{newidx}.m= cat(2,tt_lmb{newidx}.m,glmb.tt{trkidx}.m);
      tt_lmb{newidx}.P= cat(3,tt_lmb{newidx}.P,glmb.tt{trkidx}.P);
      tt_lmb{newidx}.w= cat(1,tt_lmb{newidx}.w,glmb.w(hidx)*glmb.tt{trkidx}.w);
   end
end

%extract existence probabilities and normalize track weights
for tabidx=1:length(tt_lmb)
   tt_lmb{tabidx}.r= sum(tt_lmb{tabidx}.w);
   tt_lmb{tabidx}.w= tt_lmb{tabidx}.w/tt_lmb{tabidx}.r;
end

end



function tt_lmb_out= clean_lmb(tt_lmb_in,filter)
%prune tracks with low existence probabilities
rvect= get_rvals(tt_lmb_in);
idxkeep= find(rvect > filter.track_threshold);
rvect= rvect(idxkeep);
tt_lmb_out= tt_lmb_in(idxkeep);

%enforce cap on maximum number of tracks
if length(tt_lmb_out) > filter.T_max
    [~,idxkeep]= sort(rvect,'descend');
    tt_lmb_out= tt_lmb_out(idxkeep);   
end

%cleanup tracks
for tabidx=1:length(tt_lmb_out)
    [tt_lmb_out{tabidx}.w,tt_lmb_out{tabidx}.m,tt_lmb_out{tabidx}.P]= gaus_prune(tt_lmb_out{tabidx}.w,tt_lmb_out{tabidx}.m,tt_lmb_out{tabidx}.P,filter.elim_threshold);
    [tt_lmb_out{tabidx}.w,tt_lmb_out{tabidx}.m,tt_lmb_out{tabidx}.P]= gaus_merge(tt_lmb_out{tabidx}.w,tt_lmb_out{tabidx}.m,tt_lmb_out{tabidx}.P,filter.merge_threshold);
    [tt_lmb_out{tabidx}.w,tt_lmb_out{tabidx}.m,tt_lmb_out{tabidx}.P]= gaus_cap(tt_lmb_out{tabidx}.w,tt_lmb_out{tabidx}.m,tt_lmb_out{tabidx}.P,filter.L_max);
end
end



function rvect= get_rvals(tt_lmb)                           %function to extract vector of existence probabilities from LMB track table
rvect= zeros(length(tt_lmb),1);
for tabidx=1:length(tt_lmb)
   rvect(tabidx)= tt_lmb{tabidx}.r; 
end
end



function [X,N,L]=extract_estimates(tt_lmb,model)
%extract estimates via MAP cardinality and corresponding tracks
rvect= get_rvals(tt_lmb); rvect= min(rvect,0.999); rvect= max(rvect,0.001);
cdn= prod(1-rvect)*esf(rvect./(1-rvect));
[~,mode] = max(cdn);
N = min(length(rvect),mode-1);
X= zeros(model.x_dim,N);
L= zeros(2,N);

[~,idxcmp]= sort(rvect,'descend');
for n=1:N
    [~,idxtrk]= max(tt_lmb{idxcmp(n)}.w);
    X(:,n)= tt_lmb{idxcmp(n)}.m(:,idxtrk);
    L(:,n)= tt_lmb{idxcmp(n)}.l;
end
end



function display_diaginfo(tt_lmb,k,est,filter,T_predict,T_posterior,T_clean)
rvect= get_rvals(tt_lmb); rvect= min(rvect,0.999); rvect= max(rvect,0.001);
cdn= prod(1-rvect)*esf(rvect./(1-rvect));
eap= (0:(length(cdn)-1))*cdn(:);
var= (0:(length(cdn)-1)).^2*cdn(:)-((0:(length(cdn)-1))*cdn(:))^2;
if ~strcmp(filter.run_flag,'silence')
    disp([' time= ',num2str(k),...
        ' #eap cdn=' num2str(eap),...
        ' #var cdn=' num2str(var,4),...
        ' #est card=' num2str(est.N(k),4),...
        ' #trax pred=' num2str(T_predict,4),...
        ' #trax post=' num2str(T_posterior,4),...
        ' #trax updt=',num2str(T_clean,4)   ]);
end
end