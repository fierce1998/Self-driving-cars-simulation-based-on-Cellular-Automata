function [new_v]=randfast(d2,vmax)
%Copyright @Miracle. http://www.tzq.me
p=0.3; %���ٵ�d2�ĸ���
rand('state',sum(100*clock)*rand(1));%
p_rand=rand; %�����������
if p_rand<=p
   new_v=d2;
else
   new_v=vmax;
end