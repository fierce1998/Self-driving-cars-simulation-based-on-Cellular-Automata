function [Accident,F_t]=nasch4()
clear all
%Copyright @Miracle. http://www.tzq.me
%初始化数据
%三车道
n=500;%元胞个数
density=[];%空矩阵，用来储存每次循环的平均密度
flow=[];%空矩阵，用来储存每次循环的平均流量
%cells用来判断
%z用来实时更新
z1=zeros(1,n);%右车道元胞个数
z2=zeros(1,n);%中右车道元胞个数
z3=zeros(1,n);%中左车道元胞个数
z4=zeros(1,n);%左车道元胞个数
z1=roadstart(z1,100);%道路状态初始化，路段随机分布若干辆车
z2=roadstart(z2,10);%中右车道随机分布5辆车
z3=roadstart(z3,5);%中左车道随机分布车辆
z4=roadstart(z4,3);%左车道随机分布车辆
cells1=z1;
cells2=z2;
cells3=z3;
cells4=z4;
T= 60 ;%一共运行的时间
vmax= 10 ;%最大速度=================变量============
v1=speedstart(cells1,vmax);%速度初始化
v2=speedstart(cells2,vmax);
v3=speedstart(cells3,vmax);
v4=speedstart(cells4,vmax);
a1= 1 ;%刹车时的加速度
L= 7.5  ;%每个元胞的长度
Accident=0;%记录撞车次数
Jam=zeros(1,T); %记录每个时刻的堵车次数

%产生泊松分布的时间间隔
%T_p记录t时刻到达的车辆数，0或1，时间间隔按照泊松分布
k=1;
for i=1:1000
    T_poiss(i)=poissrnd(10);%间隔平均值为3=======lambda=====
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
while flag>0
    
Nt(T+1-flag)=sum(cells1)+sum(cells2)+sum(cells3)+sum(cells4);%车辆的数目
p(T+1-flag)=Nt(T+1-flag)/(n*L);%时刻t的密度
v_average(T+1-flag)=(sum(v1)+sum(v2)+sum(v3)+sum(v4))/Nt(T+1-flag);%平均速度
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
Flag_jam=0;
for i=1:(n-1)
	if Flag_jam==1
		if cells2(i+1)==1
			continue
		else
			Flag_jam=0;
	    end
	else
		if cells2(i)==1&&cells2(i+1)==1
			Jam(T+1-flag)=Jam(T+1-flag)+1;
			Flag_jam=1;
		end
	end
end
Flag_jam=0;
for i=1:(n-1)
	if Flag_jam==1
		if cells3(i+1)==1
			continue
		else
			Flag_jam=0;
	    end
	else
		if cells3(i)==1&&cells3(i+1)==1
			Jam(T+1-flag)=Jam(T+1-flag)+1;
			Flag_jam=1;
		end
	end
end
Flag_jam=0;
for i=1:(n-1)
	if Flag_jam==1
		if cells4(i+1)==1
			continue
		else
			Flag_jam=0;
	    end
	else
		if cells4(i)==1&&cells4(i+1)==1
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
a3=searchleadcar(cells3);
b3=searchlastcar(cells3);
a4=searchleadcar(cells4);
b4=searchlastcar(cells4);

[cells1,v1]=border_control(cells1,a1,b1,v1,vmax,T_p,(i-j+1));%控制进出
[cells2,v2]=border_control2(cells2,a2,b2,v2,vmax);%控制左车道车辆出，此时的车辆是来不及换回右车道的车
[cells3,v3]=border_control2(cells3,a3,b3,v3,vmax);
[cells4,v4]=border_control2(cells4,a4,b4,v4,vmax);


z1=cells1;
z2=cells2;
z3=cells3;
z4=cells4;

i=searchleadcar(cells1);%搜索右边首车位置
for j=1:i


   %==========================================右车道=============================================
    if i-j+1==n
        [z1,v1]=leadcarupdate(z1,v1);
        continue;
    else
        if cells1(i-j+1)~=0 %判断当前位置是否非空(有车) 
            k1=searchfrontcar((i-j+1),cells1);%搜索前方首个非空元胞位置
            if k1==0 %确定与前车之间的距离
                d1=n-(i-j+1);
            else d1=k1-(i-j+1);
            end   
            
            %判断换道
			k2=searchfrontcar((i-j+1),cells2);%搜索左车道对应位置前方首个非空元胞位置
            if k2==0 %确定左车道对应位置与前车之间的距离
                d2=n-(i-j+1);
            else d2=k2-(i-j+1);
            end
			
			%搜索左车道后方首个车辆位置和速度
			k3=searchbackcar((i-j+1),cells2);
			if k3==0
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
                    %flag_change=1;%换道标记
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
                        z1(i-j+1+d1+v1(i-j+1+1))=0;%被撞车的位置也为0，消失了两辆车
                        v1(i-j+1)=0;
                        v1(i-j+1+d1+v1(i-j+1+1))=0;
                        Accident=Accident+1;%更新撞车次数
                    end
                end
            end
        end
    end
    
    
     %==============================================中右车道======================================
     if cells2(i-j+1)~=0 %判断当前位置是否非空
            
        %中车道，按照NS-FI混合模型

		%搜索中车道前方车辆
        k2_middle=searchfrontcar((i-j+1),cells2);%搜索前方首个非空元胞位置
        if k2_middle==0 %确定与前车之间的距离
            d2_middle=n-(i-j+1);
        else d2_middle=k2_middle-(i-j+1);
        end
			
		%搜索右车道前方车辆
		k1_middle=searchfrontcar((i-j+1),cells1);
		if k1_middle==0
			d1_middle=n-(i-j+1);
		else 
			d1_middle=k2_middle-(i-j+1);
		end
			
		%搜索右车道后方车辆和速度
		k3_middle=searchbackcar((i-j+1),cells1);
		if k3_middle==0
			d3_middle=(i-j+1)-1;
			vback_middle=0;
		else
			d3_middle=(i-j+1)-k3_middle;
			vback_middle=v1(k3_middle);
		end
			
		if vback_middle<=d3_middle&&v2(i-j+1)<=d2_middle&&cells1(i-j+1)==0  %并道条件满足，进行并道
			%按照随机概率进行并道
			p_change=0.7; %并道概率===========================变量===========================
			rand('state',sum(100*clock)*rand(1));%
			p_change_rand=rand; %产生随机概率
			
			if p_change_rand<=p_change %并道
			z1(i-j+1)=1;
			z2(i-j+1)=0;
			v2(i-j+1)=0;
			v1(i-j+1)=0;
			%flag_change=1;%换道标记
			
			else       %不并道
				
				v2(i-j+1)=min(v2(i-j+1)+2,vmax);   %按照NS-FI混合模型进行加速

				if v2(i-j+1)>d2_middle
					%撞车  更新位置和速度
					z2(i-j+1)=0;
					z2(i-j+1+d2_middle+v2(i-j+1+1))=0;%被撞车的位置也为0，消失了两辆车
					v2(i-j+1)=0;
					v2(i-j+1+d2_middle+v2(i-j+1+1))=0;
					Accident=Accident+1;%更新撞车次数
				
				else		
				v2(i-j+1)=randslow2(v2(i-j+1),vmax);%随机慢化  此处慢化方式和左车道一样
				new_v2=v2(i-j+1);
				%更新车辆位置
				z2(i-j+1)=0;
				z2(i-j+1+new_v2)=1;
				%更新速度
				v2(i-j+1)=0;
				v2(i-j+1+new_v2)=new_v2;
				end
			end
			
		else	 %不满足并道条件
			
			
			%判断是否满足超车条件
			
			%搜索前方首个非空元胞位置
			k1_middle_2=searchfrontcar((i-j+1),cells2);
            if k1_middle_2==0 %确定与前车之间的距离
                d1_middle_2=n-(i-j+1);
            else d1_middle_2=k1_middle_2-(i-j+1);
            end   
            
			%搜索左车道对应位置前方首个非空元胞位置
			k2_middle_2=searchfrontcar((i-j+1),cells3);
            if k2_middle_2==0 %确定左车道对应位置与前车之间的距离
                d2_middle_2=n-(i-j+1);
            else d2_middle_2=k2_middle_2-(i-j+1);
            end
			
			%搜索左车道后方首个车辆位置和速度
			k3_middle_2=searchbackcar((i-j+1),cells3);
			if k3_middle_2==0
				d3_middle_2=(i-j+1)-1;
				vback_middle_2=0;
			else
				d3_middle_2=(i-j+1)-k3_middle_2;
				vback_middle_2=v2(k3_middle_2);
			end

			
			%满足超车条件
			if d1_middle_2<v1(i-j+1)&&d2_middle_2>=v1(i-j+1)&&d3_middle_2>=vback_middle_2&&cells3(i-j+1)==0  %满足三个条件进行向左换道
				p_change=0.7; %超车概率===========================变量===========================
				rand('state',sum(100*clock)*rand(1));%
				p_change_rand=rand; %产生随机概率
				
				if p_change_rand<=p_change %超车
					z2(i-j+1)=0;
                    z3(i-j+1)=1;
                    v3(i-j+1)=0;
                    v2(i-j+1)=0;
					
				else   %不超车
					%进行减速
					v2(i-j+1)=min(v2(i-j+1)+2,vmax);   %按照NS-FI混合模型进行加速

					if v2(i-j+1)>d2_middle
						%撞车  更新位置和速度
						z2(i-j+1)=0;
						z2(i-j+1+d2_middle+v2(i-j+1+1))=0;%被撞车的位置也为0，消失了两辆车
						v2(i-j+1)=0;
						v2(i-j+1+d2_middle+v2(i-j+1+1))=0;
						Accident=Accident+1;%更新撞车次数
					
					else		
					v2(i-j+1)=randslow2(v2(i-j+1),vmax);%随机慢化  此处慢化方式和左车道一样
					new_v2=v2(i-j+1);
					%更新车辆位置
					z2(i-j+1)=0;
					z2(i-j+1+new_v2)=1;
					%更新速度
					v2(i-j+1)=0;
					v2(i-j+1+new_v2)=new_v2;
					end
				end

			else   %不满足超车条件
			       %进行减速
				v2(i-j+1)=min(v2(i-j+1)+2,vmax);   %按照NS-FI混合模型进行加速

				if v2(i-j+1)>d2_middle
					%撞车  更新位置和速度
					z2(i-j+1)=0;
					z2(i-j+1+d2_middle+v2(i-j+1+1))=0;%被撞车的位置也为0，消失了两辆车
					v2(i-j+1)=0;
					v2(i-j+1+d2_middle+v2(i-j+1+1))=0;
					Accident=Accident+1;%更新撞车次数
				
				else		
				v2(i-j+1)=randslow2(v2(i-j+1),vmax);%随机慢化  此处慢化方式和左车道一样
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
	%==============================================中左车道======================================
     if cells3(i-j+1)~=0 %判断当前位置是否非空
            
        %中车道，按照NS-FI混合模型

		%搜索中车道前方车辆
        kl2_middle=searchfrontcar((i-j+1),cells3);%搜索前方首个非空元胞位置
        if kl2_middle==0 %确定与前车之间的距离
            dl2_middle=n-(i-j+1);
        else dl2_middle=kl2_middle-(i-j+1);
        end
			
		%搜索右车道前方车辆
		kl1_middle=searchfrontcar((i-j+1),cells2);
		if kl1_middle==0
			dl1_middle=n-(i-j+1);
		else 
			dl1_middle=kl2_middle-(i-j+1);
		end
			
		%搜索右车道后方车辆和速度
		kl3_middle=searchbackcar((i-j+1),cells2);
		if kl3_middle==0
			dl3_middle=(i-j+1)-1;
			vlback_middle=0;
		else
			dl3_middle=(i-j+1)-kl3_middle;
			vlback_middle=v2(kl3_middle);
		end
			
		if vlback_middle<=dl3_middle&&v3(i-j+1)<=dl2_middle&&cells2(i-j+1)==0  %并道条件满足，进行并道
			%按照随机概率进行并道
			p_change=0.7; %并道概率===========================变量===========================
			rand('state',sum(100*clock)*rand(1));%
			p_change_rand=rand; %产生随机概率
			
			if p_change_rand<=p_change %并道
			z2(i-j+1)=1;
			z3(i-j+1)=0;
			v3(i-j+1)=0;
			v2(i-j+1)=0;
			%flag_change=1;%换道标记
			
			else       %不并道
				
				v3(i-j+1)=min(v3(i-j+1)+3,vmax);   %按照NS-FI混合模型进行加速   加速值为3！！！！！！！！

				if v3(i-j+1)>dl2_middle
					%撞车  更新位置和速度
					z3(i-j+1)=0;
					z3(i-j+1+dl2_middle+v3(i-j+1+1))=0;%被撞车的位置也为0，消失了两辆车
					v3(i-j+1)=0;
					v3(i-j+1+dl2_middle+v3(i-j+1+1))=0;
					Accident=Accident+1;%更新撞车次数
				
				else		
				v3(i-j+1)=randslow2(v3(i-j+1),vmax);%随机慢化  此处慢化方式和左车道一样
				new_v3=v3(i-j+1);
				%更新车辆位置
				z3(i-j+1)=0;
				z3(i-j+1+new_v3)=1;
				%更新速度
				v3(i-j+1)=0;
				v3(i-j+1+new_v3)=new_v3;
				end
			end
			
		else	 %不满足并道条件
			
			
			%判断是否满足超车条件
			
			%搜索前方首个非空元胞位置
			kl1_middle_2=searchfrontcar((i-j+1),cells3);
            if kl1_middle_2==0 %确定与前车之间的距离
                dl1_middle_2=n-(i-j+1);
            else dl1_middle_2=kl1_middle_2-(i-j+1);
            end   
            
			%搜索左车道对应位置前方首个非空元胞位置
			kl2_middle_2=searchfrontcar((i-j+1),cells4);
            if kl2_middle_2==0 %确定左车道对应位置与前车之间的距离
                dl2_middle_2=n-(i-j+1);
            else dl2_middle_2=kl2_middle_2-(i-j+1);
            end
			
			%搜索左车道后方首个车辆位置和速度
			kl3_middle_2=searchbackcar((i-j+1),cells4);
			if kl3_middle_2==0
				dl3_middle_2=(i-j+1)-1;
				vlback_middle_2=0;
			else
				dl3_middle_2=(i-j+1)-kl3_middle_2;
				vlback_middle_2=v2(kl3_middle_2);
			end

			
			%满足超车条件
			if dl1_middle_2<v2(i-j+1)&&dl2_middle_2>=v2(i-j+1)&&dl3_middle_2>=vlback_middle_2&&cells4(i-j+1)==0  %满足三个条件进行向左换道
				p_change=0.7; %超车概率===========================变量===========================
				rand('state',sum(100*clock)*rand(1));%
				p_change_rand=rand; %产生随机概率
				
				if p_change_rand<=p_change %超车
					z3(i-j+1)=0;
                    z4(i-j+1)=1;
                    v4(i-j+1)=0;
                    v3(i-j+1)=0;
					
				else   %不超车
					%进行减速
					v3(i-j+1)=min(v3(i-j+1)+3,vmax);   %按照NS-FI混合模型进行加速      !!!!!减速变加速

					if v3(i-j+1)>dl2_middle
						%撞车  更新位置和速度
						z3(i-j+1)=0;
						z3(i-j+1+dl2_middle+v3(i-j+1+1))=0;%被撞车的位置也为0，消失了两辆车
						v3(i-j+1)=0;
						v3(i-j+1+dl2_middle+v3(i-j+1+1))=0;
						Accident=Accident+1;%更新撞车次数
					
					else		
					v3(i-j+1)=randslow2(v3(i-j+1),vmax);%随机慢化  此处慢化方式和左车道一样
					new_v3=v3(i-j+1);
					%更新车辆位置
					z3(i-j+1)=0;
					z3(i-j+1+new_v3)=1;
					%更新速度
					v3(i-j+1)=0;
					v3(i-j+1+new_v3)=new_v3;
					end
				end

			else   %不满足超车条件
			       %进行减速
				v3(i-j+1)=min(v3(i-j+1)+3,vmax);   %按照NS-FI混合模型进行加速           !!!!!减速变加速

				if v3(i-j+1)>dl2_middle
					%撞车  更新位置和速度
					z3(i-j+1)=0;
					z3(i-j+1+dl2_middle+v3(i-j+1+1))=0;%被撞车的位置也为0，消失了两辆车
					v3(i-j+1)=0;
					v3(i-j+1+dl2_middle+v3(i-j+1+1))=0;
					Accident=Accident+1;%更新撞车次数
				
				else		
				v3(i-j+1)=randslow2(v3(i-j+1),vmax);%随机慢化  此处慢化方式和左车道一样
				new_v3=v3(i-j+1);
				%更新车辆位置
				z3(i-j+1)=0;
				z3(i-j+1+new_v3)=1;
				%更新速度
				v3(i-j+1)=0;
				v3(i-j+1+new_v3)=new_v3;
				end
			end
		end
	end
	
	%========================================左车道========================================
	if cells4(i-j+1)~=0 %判断当前位置是否非空
            
        %左车道，按照FI模型

        k2_left=searchfrontcar((i-j+1),cells4);%搜索前方首个非空元胞位置
        if k2_left==0 %确定与前车之间的距离
            d2_left=n-(i-j+1);
        else d2_left=k2_left-(i-j+1);
        end
			
		%搜索中车道前方车辆
		k1_left=searchfrontcar((i-j+1),cells3);
		if k1_left==0
			d1_left=n-(i-j+1);
		else 
			d1_left=k2_left-(i-j+1);
		end
			
		%搜索中车道后方车辆和速度
		k3_left=searchbackcar((i-j+1),cells3);
		if k3_left==0
			d3_left=(i-j+1)-1;
			vback_left=0;
		else
			d3_left=(i-j+1)-k3_left;
			vback_left=v3(k3_left);
		end
			
		if vback_left<=d3_left&&v4(i-j+1)<=d2_left&&cells3(i-j+1)==0  %并道条件满足，进行并道
			%p_change=0.5; %并道概率
			%rand('state',sum(100*clock)*rand(1));%
			%p_change_rand=rand; %产生随机概率
			%if p_change_rand<=p_change %并道
			z3(i-j+1)=1;
			z4(i-j+1)=0;
			v4(i-j+1)=0;
			v3(i-j+1)=0;
			%flag_change=1;%换道标记
		else			
			%加速时随机加到最大速度或者实际最大速度
			v4(i-j+1)=randfast(d2_left,vmax);

			if v4(i-j+1)>d2_left
				%撞车  更新位置和速度
				z4(i-j+1)=0;
				z4(i-j+1+d2_left+v4(i-j+1+1))=0;%被撞车的位置也为0，消失了两辆车
				v4(i-j+1)=0;
				v4(i-j+1+d2_left+v4(i-j+1+1))=0;
				Accident=Accident+1;%更新撞车次数
			else
					
			v4(i-j+1)=randslow2(v4(i-j+1),vmax);%随机慢化
			new_v4=v4(i-j+1);
			%更新车辆位置
			z4(i-j+1)=0;
			z4(i-j+1+new_v4)=1;
			%更新速度
			v4(i-j+1)=0;
			v4(i-j+1+new_v4)=new_v4;
            end
		end
    end
	
    
end
cells1=z1;
cells2=z2;  
cells3=z3;   
cells4=z4;
flag=flag-1;
F_t=mean(Flow)  %在这一轮模拟中的平均流量
p_t=mean(p)%这一轮模拟中的平均密度
flow=[flow;F_t];
density=[density;p_t];
end   %至此更新完一轮
flow
density
plot(density,flow)