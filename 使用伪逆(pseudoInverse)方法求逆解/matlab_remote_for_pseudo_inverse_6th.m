% �����;��Ȼ�Ӱ�� ����Ŀ��λ�õ�ʱ��  ����ԽС ����Խ�� ����Ŀ��λ�õ�ʱ���Խ����
% ������3rd �� 4th �Ѱ汾�в��� ���ǲ�һ����  �����Ĵ����Ͳ�ͬ��Ҳ��Ӱ�쵽���յ���Ŀ����λ��
% 3rd��2nd��������޸��ǽ� �����Ļ�÷�ʽ�ǽ���������1.6 apha=norm_deta_pXY/10;
% 4th �� 3rd ��� �ǽ� ��������1.5 ��ͬ��ϵ����С������������ͬ
 %��ֵ������ô��� orientation
 % simx_opmode_oneshot_wait �Ƿ���Ҫ�Ľ���Ҫ����һ��
%6th  ���Ч��Ҳ�ܺ�
%1.Ϊʲôerror��������ģ�����������Ч��  Ч���޴� ��Ҫ��������������Ѿ����ڱʼ����ˡ�
%2.ʹ�� deta=deta_pXY ������ deta=deta_pXY/norm_deta_pXY  �����̶�Ϊ1.Ч��Ҳ�ܺ�
%Ч���õ�ԭ������ ����ʱ�䵱������С�������������д�ڱʼ��
%��ʵ���������ٶȲ�������ô�졣���Բ���Ҳ�Ͳ����ܵ���1.
 %��������С  �����Ĵ����ͻ����ӡ����ǹؽڵ��ٶȻ��½�����������ʵ����
function matlab_remote_for_pseudo_inverse_6th()
    global r1; global r2;  
    disp('����ʼ');
    vrep=remApi('remoteApi');%����һ������ remApi��һ����
    vrep.simxFinish(-1)%��һ��ʼ��ʱ��ص����е�socket���� just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5)
    motorHandles=[-1,-1];
    %�����洢�ؽ�ֵ������Ҫ���Ե���ע��
    r1=[];
    r2=[];
	%���������
	error_tolerance=0.0001
	%���� apha
	%apha=0.1
	%��������
	i=0;
	%��е������  
	%������ô��֤���� ��ô��֤��ȷ��С�������һλ��Ҫ����һ��
	l1=0.5;  %���˳̶� ��λ����
	l2=0.5;  %���˳̶� ��λ����
    if(clientID>-1)
        disp('���ӵ�vrep��������')
        vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot);
        % Now try to retrieve data in a blocking fashion (i.e. a service call):
        [res,objs]=vrep.simxGetObjects(clientID,vrep.sim_handle_all,vrep.simx_opmode_blocking);
        if (res==vrep.simx_return_ok)
            fprintf('Number of objects in the scene: %d\n',length(objs));
        else
            fprintf('Remote API function call returned with error code: %d\n',res);
        end
        %�����ص�obje��handle
        [res1,j1]=vrep.simxGetObjectHandle(clientID,'j1#',vrep.simx_opmode_blocking);
        [res2,j2]=vrep.simxGetObjectHandle(clientID,'j2#',vrep.simx_opmode_blocking);
		[res,target]=vrep.simxGetObjectHandle(clientID,'target#',vrep.simx_opmode_blocking);
		[res2,actual]=vrep.simxGetObjectHandle(clientID,'tip#',vrep.simx_opmode_blocking);
        %ԭ����֤�Ƿ���Object��handle
        fprintf(' error code: %d\n',res1);
		res8=vrep.simxSetJointPosition(clientID,j2, pi/4 ,vrep.simx_opmode_oneshot);
		 
		%���Ŀ��λ�ú�ʵ��λ��
		[res,theta1]=vrep.simxGetJointPosition(clientID,j1,vrep.simx_opmode_oneshot_wait);
        [res,theta2]=vrep.simxGetJointPosition(clientID,j2,vrep.simx_opmode_oneshot_wait);
		
		[res,targetXYZ]=vrep.simxGetObjectPosition(clientID,target,-1,vrep.simx_opmode_blocking);
		[actual_x,actual_y]= get_tip_position(theta1,theta2);
		%���Ŀ��λ�ú�ʵ��λ�õ����
		deta_pXY=targetXYZ([1 2])-[actual_x,actual_y]
		 	 
		fprintf(' deta_pXY: %8.6f\n',deta_pXY);
		%���error�ķ���
		norm_deta_pXY=norm(deta_pXY)		

		%ֻҪ����û��С�ڹ涨��ֵ��Ҫһֱ������ȥ
		while(norm_deta_pXY > error_tolerance)
		
		i=i+1;
		
		[res,theta1]=vrep.simxGetJointPosition(clientID,j1,vrep.simx_opmode_oneshot_wait);
        [res,theta2]=vrep.simxGetJointPosition(clientID,j2,vrep.simx_opmode_oneshot_wait);
		%���Ŀ��λ�ú�ʵ��λ��
		[res,targetXYZ]=vrep.simxGetObjectPosition(clientID,target,-1,vrep.simx_opmode_blocking);
		[actual_x,actual_y]= get_tip_position(theta1,theta2);
		%���Ŀ��λ�ú�ʵ��λ�õĲ�ķ���
		deta_pXY=targetXYZ([1 2])-[actual_x,actual_y]
		%fprintf(' deta_pXY: %8.6f\n',deta_pXY);
		norm_deta_pXY=norm(deta_pXY)
		%��Vrep���л�ȡ�ؽڽǶ� ��λ�ǻ���
	
        %������ķ�����ʵ����˵�λ����
		% 4th �� 3rd ��� �ǽ� ��������1.5 ��ͬ��ϵ����С������������ͬ
        %deta=deta_pXY/norm_deta_pXY
        deta=deta_pXY
		%α�淨���¹ؽ�ֵ
		J_pinv_svd=getPseudoInverse(theta1,theta2)
		
		%�����Ļ�÷�ʽ
		% 4th �� 3rd ��� �ǽ� ��������1.5 ��ͬ��ϵ����С������������ͬ
	    %��������С  �����Ĵ����ͻ�����
		apha=1;	
      
		deta_theta=J_pinv_svd*deta'.*apha;
        fprintf(' deta_theta: %8.6f\n',deta_theta);	
		theta1=theta1+deta_theta(1);
		theta2=theta2+deta_theta(2);
        res7=vrep.simxSetJointPosition(clientID,j1, theta1 ,vrep.simx_opmode_oneshot); 
        res8=vrep.simxSetJointPosition(clientID,j2, theta2 ,vrep.simx_opmode_oneshot);
		%��ӡ�ؽڽ�
		%fprintf(' theta1: %8.6f\n',theta1);
		%��ӡ��������		
		fprintf('����������%d\n',i);
        end
 
        fprintf('����Ŀ��λ��')  
        % Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
        %vrep.simxGetPingTime(clientID) 
        pause(0.1)
    else
        disp('���ӵ�verpʧ��')
    end 
    %vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait);
    vrep.simxFinish(clientID);
    vrep.delete(); % call the destructor!
    disp('�������')
 end
 
 function J_pinv_svd=getPseudoInverse(theta1,theta2)
 
    %��е������  
	%������ô��֤���� ��ô��֤��ȷ��С�������һλ��Ҫ����һ��
	l1=0.5;  %���˳̶� ��λ����
	l2=0.5;  %���˳̶� ��λ����

    %��û�е�۵��ſ˱Ⱦ���
	J11=-l1*sin(theta1)-l2*sin(theta1+theta2);
	J12=-l2*sin(theta1+theta2);
	J21= l1*cos(theta1)+l2*cos(theta1+theta2);
	J22= l2*cos(theta1+theta2);
	J=[J11,J12;J21,J22];
	%����ſ˱Ⱦ����α��
	%���ſ˱Ⱦ����������ʱ���������ȵģ����ſ˱Ⱦ���������һ������
	%��1 ʹ��MATLAB���� ������svd����
	J_pinv_svd=pinv(J);
    %fprintf('α��ʹ��svd��%08.8f\n',J_pinv_svd)
	
	%��2 ֱ����
    
	J_pinv_direct=J'*inv(J*J');
	%fprintf('α��ʹ��ֱ�ӷ���%08.8f\n',J_pinv_direct)
	
 end
%��û�е��ĩ��λ�õĺ���
function [positionX,positionY]=get_tip_position(theta1,theta2)
    %��е������  
	%������ô��֤���� ��ô��֤��ȷ��С�������һλ��Ҫ����һ��
	l1=0.5;  %���˳̶� ��λ����
	l2=0.5;  %���˳̶� ��λ����
	positionX=l1*cos(theta1)+l2*cos(theta1+theta2);
	positionY=l1*sin(theta1)+l2*sin(theta1+theta2);	
end
%