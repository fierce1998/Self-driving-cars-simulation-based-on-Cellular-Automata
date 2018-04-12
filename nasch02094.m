function [jj,Accident,F_t]=nasch02094()
clear all
%Copyright @Miracle. http://www.tzq.me
%初始化数据

%左行   



n=500;%元胞个数

%cells用来判断
%z用来实时更新
z1=zeros(1,n);%右车道元胞个数
z2=zeros(1,n);%左车道元胞个数
z1=roadstart(z1,200);%道路状态初始化，路段随机分布200辆车
z2=roadstart(z2,20);%左车道随机分布20辆车
cells1=z1;
cells2=z2;
T= 60 ;%一共运行的时间
vmax= 10 ;%最大速度=================变量===============
v1=speedstart(cells1,vmax);%速度初始化
v2=speedstart(cells2,vmax);
a1= 5 ;%刹车时的加速度
L= 7.5  ;%每个元胞的长度
Accident=0;%记录撞车次数
Jam=zeros(1,T); %记录每个时刻的堵车次数
flag_change=zeros(1,n);%换道记录



%产生泊松分布的时间间隔
%T_p记录t时刻到达的车辆数，0或1，时间间隔按照泊松分布
k=1;
for i=1:1000
    T_poiss(i)=poissrnd(5);%间隔平均值为3=========变量===========
end
for i=1:1000    %随便定一个数
    for j=1:T_poiss(i)
        T_p(k)=0;
        k=k+1;
    end
    T_p(k)=1;
    k=k+1;
end


flag=T;%循环次数,也就是取多长的时间



%循环起来
while flag>0,
    
Nt(T+1-flag)=sum(cells1)+sum(cells2);%车辆的数目
p(T+1-flag)=Nt(T+1-flag)/(n*L);%时刻t的密度
v_average(T+1-flag)=(sum(v1)+sum(v2))/Nt(T+1-flag);%平均速度
Flow(T+1-flag)=p(T+1-flag)*v_average(T+1-flag);%流量

Flag_jam=0;%堵车标记
%堵车记录
for i=1:(n-1)
	if Flag_jam==1
		if cells1(i+1)==1
			continue
		else
			Flag_jam=0;
	    end
	else
		if cells1(i)==1&&cells1(i+1)==1
			Jam(T+1-flag)=Jam(T+1-flag)+1;
			Flag_jam=1;
		end
	end
end





%边界条件处理，控制进出
a1=searchleadcar(cells1);%搜索右边首车位置
b1=searchlastcar(cells1);%搜索右边末车位置
a2=searchleadcar(cells2);
b2=searchlastcar(cells2);

[cells1,v1]=border_control(cells1,a1,b1,v1,vmax,T_p,(i-j+1));%控制进出
[cells2,v2]=border_control2(cells2,a2,b2,v2,vmax);%控制左车道车辆出，此时的车辆是来不及换回右车道的车

z1=cells1;
z2=cells2;

i=searchleadcar(cells1);%搜索右边首车位置
for j=1:i

    if i-j+1==n
        [z1,v1]=leadcarupdate(z1,v1);
        continue;
    else
        if cells1(i-j+1)~=0;%判断当前位置是否非空(有车) 
            k1=searchfrontcar((i-j+1),cells1);%搜索前方首个非空元胞位置
            if k1==0;%确定与前车之间的距离
                d1=n-(i-j+1);
            else d1=k1-(i-j+1);
            end   
            
            %判断换道
			k2=searchfrontcar((i-j+1),cells2);%搜索左车道对应位置前方首个非空元胞位置
            if k2==0;%确定左车道对应位置与前车之间的距离
                d2=n-(i-j+1);
            else d2=k2-(i-j+1);
            end
			
			%搜索左车道后方首个车辆位置和速度
			k3=searchbackcar((i-j+1),cells2);
			if k3==0;
				d3=(i-j+1)-1;
				vback=0;
			else
				d3=(i-j+1)-k3;
				vback=v2(k3);
			end

			
            if d1<v1(i-j+1)&&d2>=v1(i-j+1)&&d3>=vback&&cells2(i-j+1)==0  %满足三个条件进行向左换道
			
                p_change=0.8; %换道概率=========================变量==============================
                rand('state',sum(100*clock)*rand(1));%
                p_change_rand=rand; %产生随机概率
                if p_change_rand<=p_change %换道
                    z1(i-j+1)=0;
                    z2(i-j+1)=1;
                    v2(i-j+1)=0;
                    v1(i-j+1)=0;
                    flag_change(i-j+1)=2;%换道标记
                else
                    %若不换道，则按照常规来更新
                    
                    v1(i-j+1)=min(v1(i-j+1)+1,vmax);%加速

                    v1_a=v1(i-j+1)-a1;%能减到的最小速度
                    if v1(i-j+1)>d1
                        if d1>v1_a
                            v1(i-j+1)=d1;%减速
							v1(i-j+1)=randslow(v1(i-j+1));%随机慢化
							new_v1=v1(i-j+1);
							%更新车辆位置
							z1(i-j+1)=0;
							z1(i-j+1+new_v1)=1;
							%更新速度
							v1(i-j+1)=0;
							v1(i-j+1+new_v1)=new_v1;
                        else
                            %撞车更新位置和速度
                            z1(i-j+1)=0;
                            z1(i-j+1+d1+v1(i-j+1+1))=0;%被撞车的位置也为0，消失了两辆车
                            v1(i-j+1)=0;
                            v1(i-j+1+d1+v1(i-j+1+1))=0;
                            Accident=Accident+1;%更新撞车次数
                        end
                    end
                end
			else
			    %不满足条件，直接按照常规来更新
				v1(i-j+1)=min(v1(i-j+1)+1,vmax);%加速

                v1_a=v1(i-j+1)-a1;%能减到的最小速度
				if v1(i-j+1)>d1
                    if d1>v1_a
                        v1(i-j+1)=d1;%减速
						v1(i-j+1)=randslow(v1(i-j+1));%随机慢化
						new_v1=v1(i-j+1);
						%更新车辆位置
						z1(i-j+1)=0;
						z1(i-j+1+new_v1)=1;
						%更新速度
						v1(i-j+1)=0;
						v1(i-j+1+new_v1)=new_v1;
                    else
                        %撞车更新位置和速度
                        z1(i-j+1)=0;
                        z1(i-j+1+d1+v2(i-j+1+1))=0;%被撞车的位置也为0，消失了两辆车
                        v1(i-j+1)=0;
                        v1(i-j+1+d1+v2(i-j+1+1))=0;
                        Accident=Accident+1;%更新撞车次数
                    end
                end
            end
        end
    end
    
    
     
    if cells2(i-j+1)~=0;%判断当前位置是否非空
	
	
		%先判断是否刚换道过来的
		
		
		if flag_change(i-j+1)~=0
			%更新位置和速度，均为0；
			flag_change(i-j+1)=flag_change(i+j-1)-1;
		else
            
			%左车道，按照FI模型

			k2=searchfrontcar((i-j+1),cells2);%搜索前方首个非空元胞位置
			if k2==0;%确定与前车之间的距离
				d2=n-(i-j+1);
			else d2=k2-(i-j+1);
			end
				
			%搜索右车道前方车辆
			k1=searchfrontcar((i-j+1),cells1);
			if k1==0;
				d1=n-(i-j+1);
			else 
				d1=k2-(i-j+1);
			end
				
			%搜索右车道后方车辆和速度
			k3=searchbackcar((i-j+1),cells1);
			if k3==0;
				d3=(i-j+1)-1;
				vback=0;
			else
				d3=(i-j+1)-k3;
				vback=v1(k3);
			end
				
			if vback<=d3&&v2(i-j+1)<=d2&&cells1(i-j+1)==0  %并道条件满足，进行并道
				%p_change=0.5; %并道概率
				%rand('state',sum(100*clock)*rand(1));%
				%p_change_rand=rand; %产生随机概率
				%if p_change_rand<=p_change %并道
				z1(i-j+1)=1;
				z2(i-j+1)=0;
				v2(i-j+1)=0;
				v1(i-j+1)=0;
				%flag_change=1;%换道标记
			else			
				%加速时随机加到最大速度或者实际最大速度
				v2(i-j+1)=randfast(d2,vmax);

				if v2(i-j+1)>d2
					%撞车  更新位置和速度
					z2(i-j+1)=0;
					z2(i-j+1+d2)=0;%被撞车的位置也为0，消失了两辆车
					v2(i-j+1)=0;
					v2(i-j+1+d2)=0;
					Accident=Accident+1;%更新撞车次数
				else
						
				v2(i-j+1)=randslow2(v2(i-j+1),vmax);%随机慢化
				new_v2=v2(i-j+1);
				%更新车辆位置
				z2(i-j+1)=0;
				z2(i-j+1+new_v2)=1;
				%更新速度
				v2(i-j+1)=0;
				v2(i-j+1+new_v2)=new_v2;
				end
			end
		end
    end
    
end
cells1=z1;
cells2=z2;         
flag=flag-1;
end   %至此更新完一轮
F_t=sum(Flow)/T;  %在这一轮模拟中的平均流量
jj=sum(Jam);