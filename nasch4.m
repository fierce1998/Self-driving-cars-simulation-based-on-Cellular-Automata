function [Accident,F_t]=nasch4()
clear all
%Copyright @Miracle. http://www.tzq.me
%��ʼ������
%������
n=500;%Ԫ������
density=[];%�վ�����������ÿ��ѭ����ƽ���ܶ�
flow=[];%�վ�����������ÿ��ѭ����ƽ������
%cells�����ж�
%z����ʵʱ����
z1=zeros(1,n);%�ҳ���Ԫ������
z2=zeros(1,n);%���ҳ���Ԫ������
z3=zeros(1,n);%���󳵵�Ԫ������
z4=zeros(1,n);%�󳵵�Ԫ������
z1=roadstart(z1,100);%��·״̬��ʼ����·������ֲ���������
z2=roadstart(z2,10);%���ҳ�������ֲ�5����
z3=roadstart(z3,5);%���󳵵�����ֲ�����
z4=roadstart(z4,3);%�󳵵�����ֲ�����
cells1=z1;
cells2=z2;
cells3=z3;
cells4=z4;
T= 60 ;%һ�����е�ʱ��
vmax= 10 ;%����ٶ�=================����============
v1=speedstart(cells1,vmax);%�ٶȳ�ʼ��
v2=speedstart(cells2,vmax);
v3=speedstart(cells3,vmax);
v4=speedstart(cells4,vmax);
a1= 1 ;%ɲ��ʱ�ļ��ٶ�
L= 7.5  ;%ÿ��Ԫ���ĳ���
Accident=0;%��¼ײ������
Jam=zeros(1,T); %��¼ÿ��ʱ�̵Ķ³�����

%�������ɷֲ���ʱ����
%T_p��¼tʱ�̵���ĳ�������0��1��ʱ�������ղ��ɷֲ�
k=1;
for i=1:1000
    T_poiss(i)=poissrnd(10);%���ƽ��ֵΪ3=======lambda=====
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
while flag>0
    
Nt(T+1-flag)=sum(cells1)+sum(cells2)+sum(cells3)+sum(cells4);%��������Ŀ
p(T+1-flag)=Nt(T+1-flag)/(n*L);%ʱ��t���ܶ�
v_average(T+1-flag)=(sum(v1)+sum(v2)+sum(v3)+sum(v4))/Nt(T+1-flag);%ƽ���ٶ�
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


%�߽������������ƽ���
a1=searchleadcar(cells1);%�����ұ��׳�λ��
b1=searchlastcar(cells1);%�����ұ�ĩ��λ��
a2=searchleadcar(cells2);
b2=searchlastcar(cells2);
a3=searchleadcar(cells3);
b3=searchlastcar(cells3);
a4=searchleadcar(cells4);
b4=searchlastcar(cells4);

[cells1,v1]=border_control(cells1,a1,b1,v1,vmax,T_p,(i-j+1));%���ƽ���
[cells2,v2]=border_control2(cells2,a2,b2,v2,vmax);%�����󳵵�����������ʱ�ĳ����������������ҳ����ĳ�
[cells3,v3]=border_control2(cells3,a3,b3,v3,vmax);
[cells4,v4]=border_control2(cells4,a4,b4,v4,vmax);


z1=cells1;
z2=cells2;
z3=cells3;
z4=cells4;

i=searchleadcar(cells1);%�����ұ��׳�λ��
for j=1:i


   %==========================================�ҳ���=============================================
    if i-j+1==n
        [z1,v1]=leadcarupdate(z1,v1);
        continue;
    else
        if cells1(i-j+1)~=0 %�жϵ�ǰλ���Ƿ�ǿ�(�г�) 
            k1=searchfrontcar((i-j+1),cells1);%����ǰ���׸��ǿ�Ԫ��λ��
            if k1==0 %ȷ����ǰ��֮��ľ���
                d1=n-(i-j+1);
            else d1=k1-(i-j+1);
            end   
            
            %�жϻ���
			k2=searchfrontcar((i-j+1),cells2);%�����󳵵���Ӧλ��ǰ���׸��ǿ�Ԫ��λ��
            if k2==0 %ȷ���󳵵���Ӧλ����ǰ��֮��ľ���
                d2=n-(i-j+1);
            else d2=k2-(i-j+1);
            end
			
			%�����󳵵����׸�����λ�ú��ٶ�
			k3=searchbackcar((i-j+1),cells2);
			if k3==0
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
                    %flag_change=1;%�������
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
                        z1(i-j+1+d1+v1(i-j+1+1))=0;%��ײ����λ��ҲΪ0����ʧ��������
                        v1(i-j+1)=0;
                        v1(i-j+1+d1+v1(i-j+1+1))=0;
                        Accident=Accident+1;%����ײ������
                    end
                end
            end
        end
    end
    
    
     %==============================================���ҳ���======================================
     if cells2(i-j+1)~=0 %�жϵ�ǰλ���Ƿ�ǿ�
            
        %�г���������NS-FI���ģ��

		%�����г���ǰ������
        k2_middle=searchfrontcar((i-j+1),cells2);%����ǰ���׸��ǿ�Ԫ��λ��
        if k2_middle==0 %ȷ����ǰ��֮��ľ���
            d2_middle=n-(i-j+1);
        else d2_middle=k2_middle-(i-j+1);
        end
			
		%�����ҳ���ǰ������
		k1_middle=searchfrontcar((i-j+1),cells1);
		if k1_middle==0
			d1_middle=n-(i-j+1);
		else 
			d1_middle=k2_middle-(i-j+1);
		end
			
		%�����ҳ����󷽳������ٶ�
		k3_middle=searchbackcar((i-j+1),cells1);
		if k3_middle==0
			d3_middle=(i-j+1)-1;
			vback_middle=0;
		else
			d3_middle=(i-j+1)-k3_middle;
			vback_middle=v1(k3_middle);
		end
			
		if vback_middle<=d3_middle&&v2(i-j+1)<=d2_middle&&cells1(i-j+1)==0  %�����������㣬���в���
			%����������ʽ��в���
			p_change=0.7; %��������===========================����===========================
			rand('state',sum(100*clock)*rand(1));%
			p_change_rand=rand; %�����������
			
			if p_change_rand<=p_change %����
			z1(i-j+1)=1;
			z2(i-j+1)=0;
			v2(i-j+1)=0;
			v1(i-j+1)=0;
			%flag_change=1;%�������
			
			else       %������
				
				v2(i-j+1)=min(v2(i-j+1)+2,vmax);   %����NS-FI���ģ�ͽ��м���

				if v2(i-j+1)>d2_middle
					%ײ��  ����λ�ú��ٶ�
					z2(i-j+1)=0;
					z2(i-j+1+d2_middle+v2(i-j+1+1))=0;%��ײ����λ��ҲΪ0����ʧ��������
					v2(i-j+1)=0;
					v2(i-j+1+d2_middle+v2(i-j+1+1))=0;
					Accident=Accident+1;%����ײ������
				
				else		
				v2(i-j+1)=randslow2(v2(i-j+1),vmax);%�������  �˴�������ʽ���󳵵�һ��
				new_v2=v2(i-j+1);
				%���³���λ��
				z2(i-j+1)=0;
				z2(i-j+1+new_v2)=1;
				%�����ٶ�
				v2(i-j+1)=0;
				v2(i-j+1+new_v2)=new_v2;
				end
			end
			
		else	 %�����㲢������
			
			
			%�ж��Ƿ����㳬������
			
			%����ǰ���׸��ǿ�Ԫ��λ��
			k1_middle_2=searchfrontcar((i-j+1),cells2);
            if k1_middle_2==0 %ȷ����ǰ��֮��ľ���
                d1_middle_2=n-(i-j+1);
            else d1_middle_2=k1_middle_2-(i-j+1);
            end   
            
			%�����󳵵���Ӧλ��ǰ���׸��ǿ�Ԫ��λ��
			k2_middle_2=searchfrontcar((i-j+1),cells3);
            if k2_middle_2==0 %ȷ���󳵵���Ӧλ����ǰ��֮��ľ���
                d2_middle_2=n-(i-j+1);
            else d2_middle_2=k2_middle_2-(i-j+1);
            end
			
			%�����󳵵����׸�����λ�ú��ٶ�
			k3_middle_2=searchbackcar((i-j+1),cells3);
			if k3_middle_2==0
				d3_middle_2=(i-j+1)-1;
				vback_middle_2=0;
			else
				d3_middle_2=(i-j+1)-k3_middle_2;
				vback_middle_2=v2(k3_middle_2);
			end

			
			%���㳬������
			if d1_middle_2<v1(i-j+1)&&d2_middle_2>=v1(i-j+1)&&d3_middle_2>=vback_middle_2&&cells3(i-j+1)==0  %�������������������󻻵�
				p_change=0.7; %��������===========================����===========================
				rand('state',sum(100*clock)*rand(1));%
				p_change_rand=rand; %�����������
				
				if p_change_rand<=p_change %����
					z2(i-j+1)=0;
                    z3(i-j+1)=1;
                    v3(i-j+1)=0;
                    v2(i-j+1)=0;
					
				else   %������
					%���м���
					v2(i-j+1)=min(v2(i-j+1)+2,vmax);   %����NS-FI���ģ�ͽ��м���

					if v2(i-j+1)>d2_middle
						%ײ��  ����λ�ú��ٶ�
						z2(i-j+1)=0;
						z2(i-j+1+d2_middle+v2(i-j+1+1))=0;%��ײ����λ��ҲΪ0����ʧ��������
						v2(i-j+1)=0;
						v2(i-j+1+d2_middle+v2(i-j+1+1))=0;
						Accident=Accident+1;%����ײ������
					
					else		
					v2(i-j+1)=randslow2(v2(i-j+1),vmax);%�������  �˴�������ʽ���󳵵�һ��
					new_v2=v2(i-j+1);
					%���³���λ��
					z2(i-j+1)=0;
					z2(i-j+1+new_v2)=1;
					%�����ٶ�
					v2(i-j+1)=0;
					v2(i-j+1+new_v2)=new_v2;
					end
				end

			else   %�����㳬������
			       %���м���
				v2(i-j+1)=min(v2(i-j+1)+2,vmax);   %����NS-FI���ģ�ͽ��м���

				if v2(i-j+1)>d2_middle
					%ײ��  ����λ�ú��ٶ�
					z2(i-j+1)=0;
					z2(i-j+1+d2_middle+v2(i-j+1+1))=0;%��ײ����λ��ҲΪ0����ʧ��������
					v2(i-j+1)=0;
					v2(i-j+1+d2_middle+v2(i-j+1+1))=0;
					Accident=Accident+1;%����ײ������
				
				else		
				v2(i-j+1)=randslow2(v2(i-j+1),vmax);%�������  �˴�������ʽ���󳵵�һ��
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
	%==============================================���󳵵�======================================
     if cells3(i-j+1)~=0 %�жϵ�ǰλ���Ƿ�ǿ�
            
        %�г���������NS-FI���ģ��

		%�����г���ǰ������
        kl2_middle=searchfrontcar((i-j+1),cells3);%����ǰ���׸��ǿ�Ԫ��λ��
        if kl2_middle==0 %ȷ����ǰ��֮��ľ���
            dl2_middle=n-(i-j+1);
        else dl2_middle=kl2_middle-(i-j+1);
        end
			
		%�����ҳ���ǰ������
		kl1_middle=searchfrontcar((i-j+1),cells2);
		if kl1_middle==0
			dl1_middle=n-(i-j+1);
		else 
			dl1_middle=kl2_middle-(i-j+1);
		end
			
		%�����ҳ����󷽳������ٶ�
		kl3_middle=searchbackcar((i-j+1),cells2);
		if kl3_middle==0
			dl3_middle=(i-j+1)-1;
			vlback_middle=0;
		else
			dl3_middle=(i-j+1)-kl3_middle;
			vlback_middle=v2(kl3_middle);
		end
			
		if vlback_middle<=dl3_middle&&v3(i-j+1)<=dl2_middle&&cells2(i-j+1)==0  %�����������㣬���в���
			%����������ʽ��в���
			p_change=0.7; %��������===========================����===========================
			rand('state',sum(100*clock)*rand(1));%
			p_change_rand=rand; %�����������
			
			if p_change_rand<=p_change %����
			z2(i-j+1)=1;
			z3(i-j+1)=0;
			v3(i-j+1)=0;
			v2(i-j+1)=0;
			%flag_change=1;%�������
			
			else       %������
				
				v3(i-j+1)=min(v3(i-j+1)+3,vmax);   %����NS-FI���ģ�ͽ��м���   ����ֵΪ3����������������

				if v3(i-j+1)>dl2_middle
					%ײ��  ����λ�ú��ٶ�
					z3(i-j+1)=0;
					z3(i-j+1+dl2_middle+v3(i-j+1+1))=0;%��ײ����λ��ҲΪ0����ʧ��������
					v3(i-j+1)=0;
					v3(i-j+1+dl2_middle+v3(i-j+1+1))=0;
					Accident=Accident+1;%����ײ������
				
				else		
				v3(i-j+1)=randslow2(v3(i-j+1),vmax);%�������  �˴�������ʽ���󳵵�һ��
				new_v3=v3(i-j+1);
				%���³���λ��
				z3(i-j+1)=0;
				z3(i-j+1+new_v3)=1;
				%�����ٶ�
				v3(i-j+1)=0;
				v3(i-j+1+new_v3)=new_v3;
				end
			end
			
		else	 %�����㲢������
			
			
			%�ж��Ƿ����㳬������
			
			%����ǰ���׸��ǿ�Ԫ��λ��
			kl1_middle_2=searchfrontcar((i-j+1),cells3);
            if kl1_middle_2==0 %ȷ����ǰ��֮��ľ���
                dl1_middle_2=n-(i-j+1);
            else dl1_middle_2=kl1_middle_2-(i-j+1);
            end   
            
			%�����󳵵���Ӧλ��ǰ���׸��ǿ�Ԫ��λ��
			kl2_middle_2=searchfrontcar((i-j+1),cells4);
            if kl2_middle_2==0 %ȷ���󳵵���Ӧλ����ǰ��֮��ľ���
                dl2_middle_2=n-(i-j+1);
            else dl2_middle_2=kl2_middle_2-(i-j+1);
            end
			
			%�����󳵵����׸�����λ�ú��ٶ�
			kl3_middle_2=searchbackcar((i-j+1),cells4);
			if kl3_middle_2==0
				dl3_middle_2=(i-j+1)-1;
				vlback_middle_2=0;
			else
				dl3_middle_2=(i-j+1)-kl3_middle_2;
				vlback_middle_2=v2(kl3_middle_2);
			end

			
			%���㳬������
			if dl1_middle_2<v2(i-j+1)&&dl2_middle_2>=v2(i-j+1)&&dl3_middle_2>=vlback_middle_2&&cells4(i-j+1)==0  %�������������������󻻵�
				p_change=0.7; %��������===========================����===========================
				rand('state',sum(100*clock)*rand(1));%
				p_change_rand=rand; %�����������
				
				if p_change_rand<=p_change %����
					z3(i-j+1)=0;
                    z4(i-j+1)=1;
                    v4(i-j+1)=0;
                    v3(i-j+1)=0;
					
				else   %������
					%���м���
					v3(i-j+1)=min(v3(i-j+1)+3,vmax);   %����NS-FI���ģ�ͽ��м���      !!!!!���ٱ����

					if v3(i-j+1)>dl2_middle
						%ײ��  ����λ�ú��ٶ�
						z3(i-j+1)=0;
						z3(i-j+1+dl2_middle+v3(i-j+1+1))=0;%��ײ����λ��ҲΪ0����ʧ��������
						v3(i-j+1)=0;
						v3(i-j+1+dl2_middle+v3(i-j+1+1))=0;
						Accident=Accident+1;%����ײ������
					
					else		
					v3(i-j+1)=randslow2(v3(i-j+1),vmax);%�������  �˴�������ʽ���󳵵�һ��
					new_v3=v3(i-j+1);
					%���³���λ��
					z3(i-j+1)=0;
					z3(i-j+1+new_v3)=1;
					%�����ٶ�
					v3(i-j+1)=0;
					v3(i-j+1+new_v3)=new_v3;
					end
				end

			else   %�����㳬������
			       %���м���
				v3(i-j+1)=min(v3(i-j+1)+3,vmax);   %����NS-FI���ģ�ͽ��м���           !!!!!���ٱ����

				if v3(i-j+1)>dl2_middle
					%ײ��  ����λ�ú��ٶ�
					z3(i-j+1)=0;
					z3(i-j+1+dl2_middle+v3(i-j+1+1))=0;%��ײ����λ��ҲΪ0����ʧ��������
					v3(i-j+1)=0;
					v3(i-j+1+dl2_middle+v3(i-j+1+1))=0;
					Accident=Accident+1;%����ײ������
				
				else		
				v3(i-j+1)=randslow2(v3(i-j+1),vmax);%�������  �˴�������ʽ���󳵵�һ��
				new_v3=v3(i-j+1);
				%���³���λ��
				z3(i-j+1)=0;
				z3(i-j+1+new_v3)=1;
				%�����ٶ�
				v3(i-j+1)=0;
				v3(i-j+1+new_v3)=new_v3;
				end
			end
		end
	end
	
	%========================================�󳵵�========================================
	if cells4(i-j+1)~=0 %�жϵ�ǰλ���Ƿ�ǿ�
            
        %�󳵵�������FIģ��

        k2_left=searchfrontcar((i-j+1),cells4);%����ǰ���׸��ǿ�Ԫ��λ��
        if k2_left==0 %ȷ����ǰ��֮��ľ���
            d2_left=n-(i-j+1);
        else d2_left=k2_left-(i-j+1);
        end
			
		%�����г���ǰ������
		k1_left=searchfrontcar((i-j+1),cells3);
		if k1_left==0
			d1_left=n-(i-j+1);
		else 
			d1_left=k2_left-(i-j+1);
		end
			
		%�����г����󷽳������ٶ�
		k3_left=searchbackcar((i-j+1),cells3);
		if k3_left==0
			d3_left=(i-j+1)-1;
			vback_left=0;
		else
			d3_left=(i-j+1)-k3_left;
			vback_left=v3(k3_left);
		end
			
		if vback_left<=d3_left&&v4(i-j+1)<=d2_left&&cells3(i-j+1)==0  %�����������㣬���в���
			%p_change=0.5; %��������
			%rand('state',sum(100*clock)*rand(1));%
			%p_change_rand=rand; %�����������
			%if p_change_rand<=p_change %����
			z3(i-j+1)=1;
			z4(i-j+1)=0;
			v4(i-j+1)=0;
			v3(i-j+1)=0;
			%flag_change=1;%�������
		else			
			%����ʱ����ӵ�����ٶȻ���ʵ������ٶ�
			v4(i-j+1)=randfast(d2_left,vmax);

			if v4(i-j+1)>d2_left
				%ײ��  ����λ�ú��ٶ�
				z4(i-j+1)=0;
				z4(i-j+1+d2_left+v4(i-j+1+1))=0;%��ײ����λ��ҲΪ0����ʧ��������
				v4(i-j+1)=0;
				v4(i-j+1+d2_left+v4(i-j+1+1))=0;
				Accident=Accident+1;%����ײ������
			else
					
			v4(i-j+1)=randslow2(v4(i-j+1),vmax);%�������
			new_v4=v4(i-j+1);
			%���³���λ��
			z4(i-j+1)=0;
			z4(i-j+1+new_v4)=1;
			%�����ٶ�
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
F_t=mean(Flow)  %����һ��ģ���е�ƽ������
p_t=mean(p)%��һ��ģ���е�ƽ���ܶ�
flow=[flow;F_t];
density=[density;p_t];
end   %���˸�����һ��
flow
density
plot(density,flow)