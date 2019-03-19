%雅克比矩阵转置方法2nd
%雅克比矩阵转置方法在奇异点的性能很差 需要迭代的次数非常多
%2nd对转置方法的步长做一些探索
function matlab_remote_for_jacboian_tranpose_2nd()
     
    disp('程序开始');
    vrep=remApi('remoteApi');%生成一个对象 remApi是一个类
    vrep.simxFinish(-1)%在一开始的时候关掉所有的socket连接 just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5)
    motorHandles=[-1,-1];
	%可允许误差
	error_tolerance=0.0001
	%步长 apha
	%apha=0.1
	%迭代次数
	i=0;
	%机械臂数据  
	%这里怎么保证精度 怎么保证精确到小数点的哪一位需要考虑一下
	l1=0.5;  %连杆程度 单位是米
	l2=0.5;  %连杆程度 单位是米
    if(clientID>-1)
        disp('连接到vrep服务器端')
        vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot);
        % Now try to retrieve data in a blocking fashion (i.e. a service call):
        [res,objs]=vrep.simxGetObjects(clientID,vrep.sim_handle_all,vrep.simx_opmode_blocking);
        if (res==vrep.simx_return_ok)
            fprintf('Number of objects in the scene: %d\n',length(objs));
        else
            fprintf('Remote API function call returned with error code: %d\n',res);
        end
        %获得相关的obje的handle
        [res1,j1]=vrep.simxGetObjectHandle(clientID,'j1#',vrep.simx_opmode_blocking);
        [res2,j2]=vrep.simxGetObjectHandle(clientID,'j2#',vrep.simx_opmode_blocking);
		[res,target]=vrep.simxGetObjectHandle(clientID,'target#',vrep.simx_opmode_blocking);
		[res2,actual]=vrep.simxGetObjectHandle(clientID,'tip#',vrep.simx_opmode_blocking);
        %原来验证是否获得Object的handle
        fprintf(' error code: %d\n',res1);
		res8=vrep.simxSetJointPosition(clientID,j2, pi/4 ,vrep.simx_opmode_oneshot);
		 
		%获得目标位置和实际位置
		[res,theta1]=vrep.simxGetJointPosition(clientID,j1,vrep.simx_opmode_oneshot_wait);
        [res,theta2]=vrep.simxGetJointPosition(clientID,j2,vrep.simx_opmode_oneshot_wait);
		
		[res,targetXYZ]=vrep.simxGetObjectPosition(clientID,target,-1,vrep.simx_opmode_blocking);
		[actual_x,actual_y]= get_tip_position(theta1,theta2);
		%获得目标位置和实际位置的误差
		deta_pXY=targetXYZ([1 2])-[actual_x,actual_y]
		 	 
		fprintf(' deta_pXY: %8.6f\n',deta_pXY);
		%误差error的范数
		norm_deta_pXY=norm(deta_pXY)		

		%只要误差范数没有小于规定的值就要一直迭代下去
		while(norm_deta_pXY > error_tolerance)
		
		i=i+1;
		
		[res,theta1]=vrep.simxGetJointPosition(clientID,j1,vrep.simx_opmode_oneshot_wait);
        [res,theta2]=vrep.simxGetJointPosition(clientID,j2,vrep.simx_opmode_oneshot_wait);
		%获得目标位置和实际位置
		[res,targetXYZ]=vrep.simxGetObjectPosition(clientID,target,-1,vrep.simx_opmode_blocking);
		[actual_x,actual_y]= get_tip_position(theta1,theta2);
		%获得目标位置和实际位置的差的范数
		deta_pXY=targetXYZ([1 2])-[actual_x,actual_y]
		%fprintf(' deta_pXY: %8.6f\n',deta_pXY);
		norm_deta_pXY=norm(deta_pXY)
		%从Vrep当中获取关节角度 单位是弧度
		% simx_opmode_oneshot_wait 是否需要改进需要再想一想
       
		 
		%获得雅克比矩阵  
		 J=getPseudoInverse(theta1,theta2);
		
		%步长的获得方式
		aaaa=dot(J*J'*deta_pXY',J*J'*deta_pXY')
		apha=dot(deta_pXY,J*J'*deta_pXY')/dot(J*J'*deta_pXY',J*J'*deta_pXY')
        
       %apha=1
		deta_theta=apha*J'*deta_pXY';
        fprintf(' deta_theta: %8.6f\n',deta_theta);	
		theta1=theta1+deta_theta(1);
		theta2=theta2+deta_theta(2);
        res7=vrep.simxSetJointPosition(clientID,j1, theta1 ,vrep.simx_opmode_oneshot); 
        res8=vrep.simxSetJointPosition(clientID,j2, theta2 ,vrep.simx_opmode_oneshot);
		%打印关节角
		%fprintf(' theta1: %8.6f\n',theta1);
		%打印迭代次数		
		fprintf('迭代次数：%d\n',i);
        end
 
        fprintf('到达目标位置')  
        % Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
        %vrep.simxGetPingTime(clientID) 
        pause(0.1)
    else
        disp('连接到verp失败')
    end 
    %vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait);
    vrep.simxFinish(clientID);
    vrep.delete(); % call the destructor!
    disp('程序结束')
 end
 
%获得雅克比矩阵 
 function J=getPseudoInverse(theta1,theta2)
 
    %机械臂数据  
	%这里怎么保证精度 怎么保证精确到小数点的哪一位需要考虑一下
	l1=0.5;  %连杆程度 单位是米
	l2=0.5;  %连杆程度 单位是米

    %获得机械臂的雅克比矩阵
	J11=-l1*sin(theta1)-l2*sin(theta1+theta2);
	J12=-l2*sin(theta1+theta2);
	J21= l1*cos(theta1)+l2*cos(theta1+theta2);
	J22= l2*cos(theta1+theta2);
	J=[J11,J12;J21,J22];
	%获得雅克比矩阵的伪逆
	%在雅克比矩阵不是奇异的时候二者是相等的（此雅克比矩阵正好是一个方阵）
	%法1 使用MATLAB函数 利用了svd方法
	J_pinv_svd=pinv(J);
    %fprintf('伪逆使用svd：%08.8f\n',J_pinv_svd)
	
	%法2 直接求法
    
	J_pinv_direct=J'*inv(J*J');
	%fprintf('伪逆使用直接法：%08.8f\n',J_pinv_direct)
	
 end
%获得机械臂末端位置的函数
function [positionX,positionY]=get_tip_position(theta1,theta2)
    %机械臂数据  
	%这里怎么保证精度 怎么保证精确到小数点的哪一位需要考虑一下
	l1=0.5;  %连杆程度 单位是米
	l2=0.5;  %连杆程度 单位是米
	positionX=l1*cos(theta1)+l2*cos(theta1+theta2);
	positionY=l1*sin(theta1)+l2*sin(theta1+theta2);	
end
%