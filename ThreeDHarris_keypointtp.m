function [keypoint] = ThreeDHarris_keypointtp(PC,Rk)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
[n m]=size(PC);
[idx dist]=rangesearch(PC,PC,Rk);
MnormalV=[];
for i=1:n
    KNN=PC(idx{i},:);
    [h1 l1]=size(KNN);
    Cov=(1/h1)*(KNN-ones(h1,1)*mean(KNN))'*(KNN-ones(h1,1)*mean(KNN));
    [U S V]=svd(Cov);
    normalV=V(:,3);
    MnormalV=[MnormalV normalV];
end
MRH=[];
for i=1:n
    nKNN=MnormalV(:,idx{i});
    [h1 l1]=size(nKNN);
    COV=(1/l1)*(nKNN)*(nKNN)';
    RH=det(COV);
    MRH=[MRH RH];
end
keypoint=[];
for i=1:n
    id=idx{i};
    h=min(5,length(id));   %%参数
    KRH=MRH(id(1:h));
    if KRH(1)==max(KRH) && KRH(1)>0.00001 && KRH(1)<0.01   %%参数
        keypoint=[keypoint;PC(i,:)];
    end
end
end

