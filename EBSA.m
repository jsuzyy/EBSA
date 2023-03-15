function [BestCost,BestValue,BestP,Optimal_UAV]=EBSA(CostFunction,nPop,nVar,VarMin,VarMax,MaxIt,X,data_task,graph_link1,pos_link1,graph_link2,pos_link2,graph_link3,pos_link3,pro,uav_overall,charging_station_overall,index_data,NUM)
DIM_RATE=1;
BestValue=inf;
for i=1:nPop
    [charging_station_overall,pro(i).uav,uav_overall,flag] = cal_initial_route(uav_overall,X(i,:),NUM,pro(i).uav,data_task,pos_link1,pos_link2,pos_link3,graph_link1,graph_link2,graph_link3,index_data,charging_station_overall);
    val_X(i) = CostFunction(pro(i).uav,uav_overall);
    if val_X(i)<BestValue
        BestValue=val_X(i);
        Optimal_UAV=pro(i).uav;
        BestP=X(i,:);
    end
end
historical_X = repmat(VarMin, nPop, 1) + rand(nPop, nVar) .* (repmat(VarMax-VarMin, nPop, 1));
it=1;
BestCost(it)=min(val_X);
for it=2:MaxIt
    if rand<rand
        historical_X=X;
    end
    historical_X=historical_X(randperm(nPop),:);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
    map=ones(nPop,nVar);     %%%
    if rand<rand
        for i=1:nPop
            u=randperm(nVar);
            map(i,u(1:ceil(DIM_RATE*rand*nVar)))=0;
        end
    else
        for i=1:nPop
            map(i,randi(nVar))=0;
        end
    end
    pros=pro;
    [pro,charging_station_overall,uav_overall] = Initilize_uav(pro,charging_station_overall,uav_overall);
    MM=mean(X);
    for i=1:nPop
        F=3*randn;
        fi=rand;
        a=randperm(nPop,1);
        while a==i
            a=randperm(nPop,1);
        end
        if val_X(i)<val_X(a)
            vv=X(i,:)-X(a,:);
        else
            vv=X(a,:)-X(i,:);
        end
        if fi<=1/3
            Xi=X(i,:)+F.*map(i,:).*(historical_X(i,:)-X(i,:)).*rand(1,nVar);
        elseif fi>=2/3
            Xi=X(i,:);
            kn=max([ceil(rand*size(data_task,1)) 1]);
            kx=randperm(size(data_task,1),kn);
            for jj=1:length(kx)
                if kx(jj)==1
                    Xi(1:NUM(1))=BestP(1:NUM(1));
                else
                    Xi(sum(NUM(1:kx(jj)-1))+1:sum(NUM(1:kx(jj))))=BestP(sum(NUM(1:kx(jj)-1))+1:sum(NUM(1:kx(jj))));
                end
            end
        else
            beta=rand;
            gi=round(rand);
            Xi=X(i,:)+rand(1,nVar).*(BestP-(beta*MM+(1-beta).*(gi*X(i,:)+(1-gi)*X(a,:))));
        end
        Xi = boundConstraint_absorb(Xi, VarMin, VarMax);
        
        [pro(i).uav,flag] =update_position_uav(Xi,X(i,:),NUM,uav_overall,data_task,graph_link1,graph_link2,graph_link3,pos_link1,pos_link2,pos_link3,index_data,charging_station_overall,pros(i).uav,pro(i).uav);
        if sum(flag)>=1
            val_Xi = CostFunction(pro(i).uav,uav_overall);
            if val_Xi<val_X(i)
                val_X(i) = val_Xi;
                X(i,:) = Xi;
                if val_X(i)<BestValue
                    BestValue=val_X(i);
                    Optimal_UAV=pro(i).uav;
                    BestP=X(i,:);
                end
            end
        end
        Optimal_UAVX=Optimal_UAV;
        [Optimal_UAV,charging_station_overall,uav_overall] = Initilize_uav_optimal(Optimal_UAV,charging_station_overall,uav_overall);
        Xtemp=BestP;
        kk=randperm(size(data_task,1),max([ceil((rand)*size(data_task,1)) 1]));
        for jj=1:length(kk)
            AUM=max([ceil((rand)*NUM(kk(jj))) 1]);
            if kk(jj)==1
                xtp=randperm(NUM(kk(jj)),AUM);
                XX=Xtemp(1:NUM(1));
                XX(xtp)=XX(xtp).*randn;
                Xtemp(1:NUM(1))=XX;
            else
                xtp=randperm(NUM(kk(jj)),AUM);
                XX=Xtemp(sum(NUM(1:kk(jj)-1))+1:sum(NUM(1:kk(jj))));
                XX(xtp)=XX(xtp).*randn;
                Xtemp(sum(NUM(1:kk(jj)-1))+1:sum(NUM(1:kk(jj))))=XX;
            end
        end
        Xtemp = boundConstraint_absorb(Xtemp, VarMin, VarMax);
        [Optimal_UAV,flag] =update_position_uav(Xtemp,BestP,NUM,uav_overall,data_task,graph_link1,graph_link2,graph_link3,pos_link1,pos_link2,pos_link3,index_data,charging_station_overall,Optimal_UAVX,Optimal_UAV);
        if sum(flag)>=1
            val_Xi = CostFunction(Optimal_UAV,uav_overall);
            if val_Xi<BestValue
                BestValue = val_Xi;
                BestP = Xtemp;
                Optimal_UAVX=Optimal_UAV;
            end
        end
        Optimal_UAV=Optimal_UAVX;
    end
    BestCost(it)=BestValue;
end
end