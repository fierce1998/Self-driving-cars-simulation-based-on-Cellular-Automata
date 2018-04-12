function [location_leadcar]=searchleadcar(matrix_cells)
%Copyright @Miracle. http://www.tzq.me
%ËÑË÷Ê×³µÎ»ÖÃ
i=length(matrix_cells);
for j=1:i
    if matrix_cells(i-j+1)~=0
       location_leadcar=i-j+1;
       break;
    else
       location_leadcar=0;
    end
end