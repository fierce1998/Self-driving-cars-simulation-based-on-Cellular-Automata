function [new_v]=randfast(d2,vmax)
%Copyright @Miracle. http://www.tzq.me
p=0.3; %加速到d2的概率
rand('state',sum(100*clock)*rand(1));%
p_rand=rand; %产生随机概率
if p_rand<=p
   new_v=d2;
else
   new_v=vmax;
end