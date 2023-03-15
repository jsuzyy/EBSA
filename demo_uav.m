clc
clear all
data_task=load('Task_Schedular.txt');%Task_Schedular
data=load('Sort_Orginal_Data.txt');%Latitudes and longitudes of node 1 to node 46
index_data=load('Classification.txt');
Orignal_Data1=data(index_data(1,1:length(find((index_data(1,:))>0))),:);
Orignal_Data2=data(index_data(2,1:length(find((index_data(2,:))>0))),:);
Orignal_Data3=data(index_data(3,1:length(find((index_data(3,:))>0))),:);
[graph_link1,pos_link1]=Calculate_distance(Orignal_Data1);
[graph_link2,pos_link2]=Calculate_distance(Orignal_Data2);
[graph_link3,pos_link3]=Calculate_distance(Orignal_Data3);
nPop=50;
MaxIt=500;  
[uav_overall,pro,VarMin,VarMax,nVar,NUM] = intilize_parameter(data_task,nPop,graph_link1,graph_link2,graph_link3);
charging_station_overall.number=size(data,1);
charging_station_overall.charging_pile=1;
for i=1:charging_station_overall.number
    charging_station_overall.charge_table(i).busy_time=zeros(charging_station_overall.charging_pile,2);
end
pro_optimal3=pro;
for i=1:30
    disp(['The ',num2str(i),'th loop is executing:'])
    [X] = generate_initial_route(nPop,uav_overall,VarMax,VarMin,NUM,data_task,graph_link1,graph_link2,graph_link3,index_data);
    [BestCost3(i,:),BestValue3(i),BestP3(i,:),pro_optimal3(i).uav] =EBSA(@CostFunction,nPop,nVar,VarMin,VarMax,0.5*MaxIt,X,data_task,graph_link1,pos_link1,graph_link2,pos_link2,graph_link3,pos_link3,pro,uav_overall,charging_station_overall,index_data,NUM);
    disp(['EBSA    ' num2str(i) ': Best Value = ' num2str(min(BestValue3),15) '; Mean Best Value = ' num2str(mean(BestValue3),15)]);
    disp(['**************************************************************************************************************************************************']);
end