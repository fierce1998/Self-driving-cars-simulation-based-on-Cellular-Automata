function [matrix_cells_start]=roadstart(matrix_cells,n)
%Copyright @Miracle. http://www.tzq.me
%道路上的车辆初始化状态，元胞矩阵随机为0或1，matrix_cells初始矩阵，n初始车辆数
k=length(matrix_cells);
z=round(k*rand(1,n));
for i=1:n
    j=z(i);
    if j==0 
       matrix_cells(i)=0;
    else
       matrix_cells(i)=1;
    end
end
matrix_cells_start=matrix_cells;