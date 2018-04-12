function [jj,Accident,F_t]=nasch02094()
clear all
%Copyright @Miracle. http://www.tzq.me
%��ʼ������

%����   



n=500;%Ԫ������

%cells�����ж�
%z����ʵʱ����
z1=zeros(1,n);%�ҳ���Ԫ������
z2=zeros(1,n);%�󳵵�Ԫ������
z1=roadstart(z1,200);%��·״̬��ʼ����·������ֲ�200����
z2=roadstart(z2,20);%�󳵵�����ֲ�20����
cells1=z1;
cells2=z2;
T= 60 ;%һ�����е�ʱ��
vmax= 10 ;%����ٶ�=================����===============
v1=speedstart(cells1,vmax);%�ٶȳ�ʼ��
v2=speedstart(cells2,vmax);
a1= 5 ;%ɲ��ʱ�ļ��ٶ�
L= 7.5  ;%ÿ��Ԫ���ĳ���
Accident=0;%��¼ײ������
Jam=zeros(1,T); %��¼ÿ��ʱ�̵Ķ³�����
flag_change=zeros(1,n);%������¼



%�������ɷֲ���ʱ����
%T_p��¼tʱ�̵���ĳ�������0��1��ʱ�������ղ��ɷֲ�
k=1;
for i=1:1000
    T_poiss(i)=poissrnd(5);%���ƽ��ֵΪ3=========����===========
end
for i=1:1000    %��㶨һ����
    for j=1:T_poiss(i)
        T_p(k)=0;
        k=k+1;
    end
    T_p(k)=1;
    k=k+1;
end


flag=T;%ѭ������,Ҳ����ȡ�೤��ʱ��



%ѭ������
while flag>0,
    
Nt(T+1-flag)=sum(cells1)+sum(cells2);%��������Ŀ
p(T+1-flag)=Nt(T+1-flag)/(n*L);%ʱ��t���ܶ�
v_average(T+1-flag)=(sum(v1)+sum(v2))/Nt(T+1-flag);%ƽ���ٶ�
Flow(T+1-flag)=p(T+1-flag)*v_average(T+1-flag);%����

Flag_jam=0;%�³����
%�³���¼
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





%�߽������������ƽ���
a1=searchleadcar(cells1);%�����ұ��׳�λ��
b1=searchlastcar(cells1);%�����ұ�ĩ��λ��
a2=searchleadcar(cells2);
b2=searchlastcar(cells2);

[cells1,v1]=border_control(cells1,a1,b1,v1,vmax,T_p,(i-j+1));%���ƽ���
[cells2,v2]=border_control2(cells2,a2,b2,v2,vmax);%�����󳵵�����������ʱ�ĳ����������������ҳ����ĳ�

z1=cells1;
z2=cells2;

i=searchleadcar(cells1);%�����ұ��׳�λ��
for j=1:i

    if i-j+1==n
        [z1,v1]=leadcarupdate(z1,v1);
        continue;
    else
        if cells1(i-j+1)~=0;%�жϵ�ǰλ���Ƿ�ǿ�(�г�) 
            k1=searchfrontcar((i-j+1),cells1);%����ǰ���׸��ǿ�Ԫ��λ��
            if k1==0;%ȷ����ǰ��֮��ľ���
                d1=n-(i-j+1);
            else d1=k1-(i-j+1);
            end   
            
            %�жϻ���
			k2=searchfrontcar((i-j+1),cells2);%�����󳵵���Ӧλ��ǰ���׸��ǿ�Ԫ��λ��
            if k2==0;%ȷ���󳵵���Ӧλ����ǰ��֮��ľ���
                d2=n-(i-j+1);
            else d2=k2-(i-j+1);
            end
			
			%�����󳵵����׸�����λ�ú��ٶ�
			k3=searchbackcar((i-j+1),cells2);
			if k3==0;
				d3=(i-j+1)-1;
				vback=0;
			else
				d3=(i-j+1)-k3;
				vback=v2(k3);
			end

			
            if d1<v1(i-j+1)&&d2>=v1(i-j+1)&&d3>=vback&&cells2(i-j+1)==0  %�������������������󻻵�
			
                p_change=0.8; %��������=========================����==============================
                rand('state',sum(100*clock)*rand(1));%
                p_change_rand=rand; %�����������
                if p_change_rand<=p_change %����
                    z1(i-j+1)=0;
                    z2(i-j+1)=1;
                    v2(i-j+1)=0;
                    v1(i-j+1)=0;
                    flag_change(i-j+1)=2;%�������
                else
                    %�������������ճ���������
                    
                    v1(i-j+1)=min(v1(i-j+1)+1,vmax);%����

                    v1_a=v1(i-j+1)-a1;%�ܼ�������С�ٶ�
                    if v1(i-j+1)>d1
                        if d1>v1_a
                            v1(i-j+1)=d1;%����
							v1(i-j+1)=randslow(v1(i-j+1));%�������
							new_v1=v1(i-j+1);
							%���³���λ��
							z1(i-j+1)=0;
							z1(i-j+1+new_v1)=1;
							%�����ٶ�
							v1(i-j+1)=0;
							v1(i-j+1+new_v1)=new_v1;
                        else
                            %ײ������λ�ú��ٶ�
                            z1(i-j+1)=0;
                            z1(i-j+1+d1+v1(i-j+1+1))=0;%��ײ����λ��ҲΪ0����ʧ��������
                            v1(i-j+1)=0;
                            v1(i-j+1+d1+v1(i-j+1+1))=0;
                            Accident=Accident+1;%����ײ������
                        end
                    end
                end
			else
			    %������������ֱ�Ӱ��ճ���������
				v1(i-j+1)=min(v1(i-j+1)+1,vmax);%����

                v1_a=v1(i-j+1)-a1;%�ܼ�������С�ٶ�
				if v1(i-j+1)>d1
                    if d1>v1_a
                        v1(i-j+1)=d1;%����
						v1(i-j+1)=randslow(v1(i-j+1));%�������
						new_v1=v1(i-j+1);
						%���³���λ��
						z1(i-j+1)=0;
						z1(i-j+1+new_v1)=1;
						%�����ٶ�
						v1(i-j+1)=0;
						v1(i-j+1+new_v1)=new_v1;
                    else
                        %ײ������λ�ú��ٶ�
                        z1(i-j+1)=0;
                        z1(i-j+1+d1+v2(i-j+1+1))=0;%��ײ����λ��ҲΪ0����ʧ��������
                        v1(i-j+1)=0;
                        v1(i-j+1+d1+v2(i-j+1+1))=0;
                        Accident=Accident+1;%����ײ������
                    end
                end
            end
        end
    end
    
    
     
    if cells2(i-j+1)~=0;%�жϵ�ǰλ���Ƿ�ǿ�
	
	
		%���ж��Ƿ�ջ���������
		
		
		if flag_change(i-j+1)~=0
			%����λ�ú��ٶȣ���Ϊ0��
			flag_change(i-j+1)=flag_change(i+j-1)-1;
		else
            
			%�󳵵�������FIģ��

			k2=searchfrontcar((i-j+1),cells2);%����ǰ���׸��ǿ�Ԫ��λ��
			if k2==0;%ȷ����ǰ��֮��ľ���
				d2=n-(i-j+1);
			else d2=k2-(i-j+1);
			end
				
			%�����ҳ���ǰ������
			k1=searchfrontcar((i-j+1),cells1);
			if k1==0;
				d1=n-(i-j+1);
			else 
				d1=k2-(i-j+1);
			end
				
			%�����ҳ����󷽳������ٶ�
			k3=searchbackcar((i-j+1),cells1);
			if k3==0;
				d3=(i-j+1)-1;
				vback=0;
			else
				d3=(i-j+1)-k3;
				vback=v1(k3);
			end
				
			if vback<=d3&&v2(i-j+1)<=d2&&cells1(i-j+1)==0  %�����������㣬���в���
				%p_change=0.5; %��������
				%rand('state',sum(100*clock)*rand(1));%
				%p_change_rand=rand; %�����������
				%if p_change_rand<=p_change %����
				z1(i-j+1)=1;
				z2(i-j+1)=0;
				v2(i-j+1)=0;
				v1(i-j+1)=0;
				%flag_change=1;%�������
			else			
				%����ʱ����ӵ�����ٶȻ���ʵ������ٶ�
				v2(i-j+1)=randfast(d2,vmax);

				if v2(i-j+1)>d2
					%ײ��  ����λ�ú��ٶ�
					z2(i-j+1)=0;
					z2(i-j+1+d2)=0;%��ײ����λ��ҲΪ0����ʧ��������
					v2(i-j+1)=0;
					v2(i-j+1+d2)=0;
					Accident=Accident+1;%����ײ������
				else
						
				v2(i-j+1)=randslow2(v2(i-j+1),vmax);%�������
				new_v2=v2(i-j+1);
				%���³���λ��
				z2(i-j+1)=0;
				z2(i-j+1+new_v2)=1;
				%�����ٶ�
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
end   %���˸�����һ��
F_t=sum(Flow)/T;  %����һ��ģ���е�ƽ������
jj=sum(Jam);