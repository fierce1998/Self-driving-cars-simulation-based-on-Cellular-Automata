function  [location_frontcar]=searchfrontcar(current_location,matrix_cells)
%Copyright @Miracle. http://www.tzq.me
%搜索某车前方首个非空元胞位置
i=length(matrix_cells);
if current_location==i
   location_frontcar=0;
else
    for j=current_location+1:i
       if matrix_cells(j)~=0
          location_frontcar=j;
       break;
       else
          location_frontcar=0;
       end
    end
end