function[new_matrix_cells,new_v]=border_control2(matrix_cells,a,b,v,vmax)
%Copyright @Miracle. http://www.tzq.me
%���󳵵����г��ڿ��ƣ������ǳ�������
%�߽����������ڱ߽磬���Ƴ�������
%���ڱ߽磬��ͷ���ڵ�·�߽磬����һ������0.9��ȥ
n=length(matrix_cells);
if a~=0&&a+v(a)>n
    %rand('state',sum(100*clock)*rand(1));%
    %p_1=rand(1); %�����������
    %if p_1<=1 %����������С��0.9�������뿪·�Σ��������
    matrix_cells(a)=0;
    v(a)=0;    
    %end
end
new_matrix_cells=matrix_cells;
new_v=v;