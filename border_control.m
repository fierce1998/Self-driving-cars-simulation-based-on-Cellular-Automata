function[new_matrix_cells,new_v]=border_control(matrix_cells,a,b,v,vmax,T_poiss,current_location)
%Copyright @Miracle. http://www.tzq.me
%�߽����������ڱ߽磬���Ƴ�������
%���ڱ߽磬��ͷ���ڵ�·�߽磬����һ������0.9��ȥ
n=length(matrix_cells);
if a+v(a)>n
    %rand('state',sum(100*clock)*rand(1));%
    %p_1=rand(1); %�����������
    %if p_1<=1 %����������С��0.9�������뿪·�Σ��������
    matrix_cells(a)=0;
    v(a)=0;    
    %end
end
%��ڱ߽磬���ɷֲ����1s��ƽ�����ﳵ����Ϊq��tΪ1s
if b>1
    %t=1;
    %q=0.75;
    %x=1;
    %p=(q*t)^x*exp(-q*t)/prod(x); %1s����1��������ĸ���
    %rand('state',sum(100*clock)*rand(1));
    %p_2=rand(1);
    %if p_2<=p 
    %   m=min(b-vmax,vmax);
    %   matrix_cells(m)=1;
    %   v(m)=m;    
    %end
    m=round(vmax*rand(1));
    matrix_cells(1)=T_poiss(current_location);
    if matrix_cells(1)~=0
        v(1)=m;
    else
        v(1)=0;
    end
end
new_matrix_cells=matrix_cells;
new_v=v;   