function[new_matrix_cells,new_v]=border_control2(matrix_cells,a,b,v,vmax)
%Copyright @Miracle. http://www.tzq.me
%对左车道进行出口控制，不考虑车辆进入
%边界条件，开口边界，控制车辆出入
%出口边界，若头车在道路边界，则以一定概率0.9离去
n=length(matrix_cells);
if a~=0&&a+v(a)>n
    %rand('state',sum(100*clock)*rand(1));%
    %p_1=rand(1); %产生随机概率
    %if p_1<=1 %如果随机概率小于0.9，则车辆离开路段，否则不离口
    matrix_cells(a)=0;
    v(a)=0;    
    %end
end
new_matrix_cells=matrix_cells;
new_v=v;