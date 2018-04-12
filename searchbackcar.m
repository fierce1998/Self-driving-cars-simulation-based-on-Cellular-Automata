function [location_backcar]=searchbackcar(current_location,matrix_cells)
%Copyright @Miracle. http://www.tzq.me
%搜索某车后方首个非空元胞位置
i=length(matrix_cells);
if current_location==1
   location_backcar=0;
else
    for j=(current_location-1):-1:1
       if matrix_cells(j)~=0
          location_backcar=j;
       break;
       else
          location_backcar=0;
       end
    end
end