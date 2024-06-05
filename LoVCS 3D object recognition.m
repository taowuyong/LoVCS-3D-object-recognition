%LoVCS 3D object recognition 
%UWAOR dataset
%offline
%model representation
list=dir(['E:\compile document\matlab\data\UWAOR model\','*.ply']);
kk=length(list);
for count=1:kk
    str= strcat ('E:\compile document\matlab\data\UWAOR model\', list(count).name);
    pcloud=pcread(str);
%     pcshow(pcloud);
    PC=pcloud.Location;
    pr=0.49491739;              %point cloud resolution
    [keypoint] = ThreeDHarris_keypointtp(PC,5*pr);
     % plot3(PC(:,1),PC(:,2),PC(:,3),'.b','MarkerSize',1);
     % hold on;
     % plot3(keypoint(:,1),keypoint(:,2),keypoint(:,3),'.r','MarkerSize',10);
     % set(gca,'DataAspectRatio',[1 1 1]);
     % axis off;
    [n2 m3]=size(keypoint);
    RR=15*pr;
    [idx,dist]=rangesearch(PC,keypoint,RR);
    MV=[];
    MDV=[];
    for i=1:n2
        KNN=PC(idx{i},:);
        d=dist{i};
        [V] = LRF_LVCS(KNN,RR,d,keypoint(i,:));
        MV=[MV;V(:)'];
        [DV] = LVC(KNN,keypoint(i,:),V,RR);
        MDV=[MDV;DV];
    end
    keypointMVMDV=[keypoint MV MDV];
    name=list(count).name;
    strout= strcat ('E:\文本\LVCS目标识别\UWAOR model LVCS descriptor\', name(1:end-4),'keypointMVMDV.txt');
    dlmwrite(strout,keypointMVMDV,'delimiter',' ','precision','%f');
end





%LVCS 3D object recognition
%UWAOR dataset
%online
%model and pose hypothesis generation
list0=dir(['E:\compile document\matlab\data\UWAOR GroundTruth_3Dscenes\','*.xf']);
kkkk=length(list0);
for count=1:kkkk
    name=list0(count).name;
    idname=strfind(name,'-');
    name1=name(1:idname(end)-1);
    name2=name(idname(end)+1:end-3);
    str= strcat ('E:\compile document\matlab\data\UWAOR scene\', name2,'.ply');
    pcloud1=pcread(str);
    % pcshow(pcloud1);
    PC1=pcloud1.Location;
    pr=0.49491739;               %point cloud resolution
    [keypoint1] = ThreeDHarris_keypointtp(PC1,5*pr);
    [n3 m3]=size(keypoint1);
    RR=15*pr;
    [idx,dist]=rangesearch(PC1,keypoint1,RR);
    MV1=[];
    MDV1=[];
    for i=1:n3
        KNN=PC1(idx{i},:);
        d=dist{i};
        [V] = LRF_LVCS(KNN,RR,d,keypoint1(i,:));
        MV1=[MV1;V];
        [DV] = LVC(KNN,keypoint1(i,:),V,RR);
        MDV1=[MDV1;DV];
    end
    str1= strcat ('E:\compile document\matlab\data\UWAOR model\', name1,'.ply');
    str2= strcat ('E:\文本\LoVCS目标识别\UWAOR model LVCS descriptor\', name1,'keypointMVMDV.txt');
    MMMMMMM = importdata(str2);
    pcloud=pcread(str1);
    % pcshow(pcloud);
    PC=pcloud.Location;
    [n1 m1]=size(PC);
    keypoint=MMMMMMM(:,1:3);
    [n2 m2]=size(keypoint);
    MDV=MMMMMMM(:,13:end);
    MVv=MMMMMMM(:,4:12);
    MV=[];
    for i=1:n2
        Vv=MVv(i,:);
        V=[Vv(1) Vv(4) Vv(7);Vv(2) Vv(5) Vv(8);Vv(3) Vv(6) Vv(9)];
        MV=[MV;V];
    end
    %transformation heypothesis generation
    [idxx distt]=knnsearch(MDV,MDV1,'k',2);
    Mmatch=[];
    MDratio=[];
    for i=1:n3
        Dratio=distt(i,1)/distt(i,2);
        if Dratio<=0.9
            match=[idxx(i,1) i];
            Mmatch=[Mmatch;match];
            MDratio=[MDratio Dratio];
        end
    end
    [n5 m5]=size(Mmatch);
    %%1-point traversal
    tic
    [SMDratio,I] = sort(MDratio,'ascend');
    K=floor(4*n5/10);       %参数需调节
    Mmatch=Mmatch(I(1:K),:);
    ninlier0=0;
    R=eye(3);
    t=zeros(1,3);
    for i=1:K
        Sidx=i;
        Spoint=keypoint(Mmatch(Sidx,1),:);
        Spoint1=keypoint1(Mmatch(Sidx,2),:);
        V=MV(3*Mmatch(Sidx,1)-2:3*Mmatch(Sidx,1),:);
        V1=MV1(3*Mmatch(Sidx,2)-2:3*Mmatch(Sidx,2),:);
        R1=V1*V';
        t1=Spoint1-Spoint*R1';
        keypointt=keypoint*R1'+ones(n2,1)*t1;
        [idx1 dist1]=rangesearch(PC1,keypointt,3*pr);
        ninlier=0;
        for count1=1:n2
            if ~isempty(idx1{count1})
                ninlier=ninlier+1;
            end
        end
        if ninlier>ninlier0
            R=R1;
            t=t1;
            ninlier0=ninlier;
        end
    end
    T=[R t';zeros(1,3) 1];
    PCt=PC*R'+ones(n1,1)*t;
    %transformation hypothesis verification
    [Tg,PCt,RMS,overlapPCt,iteration] = partialoverlapICP3(PC1,PCt,pr);
    T=Tg*T;
    % plot3(PCt(:,1),PCt(:,2),PCt(:,3),'.b','MarkerSize',1);
    % hold on;
    % plot3(PC1(:,1),PC1(:,2),PC1(:,3),'.r','MarkerSize',1);
    % set(gca,'DataAspectRatio',[1 1 1]);
    % axis off
    overlap=length(overlapPCt)/n1;
    if overlap>0.2 && RMS<1.5*pr
        strout= strcat ('E:\文本\LoVCS目标识别\UWAOR LVCS recognition result4\',name(1:end-3),'.txt');
        fid=fopen(strout,'wt');
        fprintf(fid,'%f %f %f %f\n',T');
        fclose(fid);
        continue;
    end
    [idx dist]=rangesearch(PCt,overlapPCt,15*pr);
    [idx1 dist1]=rangesearch(PC1,overlapPCt,15*pr);
    MmaxD=[];
    for i=1:length(overlapPCt)
        KNN=PCt(idx{i},:);
        KNN1=PC1(idx1{i},:);
        [idK distK]=knnsearch(KNN,KNN1,'k',1);
        maxD=max(distK);
        MmaxD=[MmaxD;maxD];
    end
    if median(MmaxD)<2*pr
        strout= strcat ('E:\文本\LoVCS目标识别\UWAOR LVCS recognition result4\',name(1:end-3),'.txt');
        fid=fopen(strout,'wt');
        fprintf(fid,'%f %f %f %f\n',T');
        fclose(fid);
    end
    time=toc;
end