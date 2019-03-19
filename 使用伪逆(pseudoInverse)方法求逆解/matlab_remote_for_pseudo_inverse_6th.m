% 步长和精度会影响 到达目标位置的时间  步长越小 精度越高 到达目标位置的时间就越长。
% 比如在3rd 和 4th 把版本中步长 就是不一样的  迭代的次数就不同，也会影响到最终到达目标点的位姿
% 3rd和2nd相比做的修改是将 步长的获得方式是将误差范数除以1.6 apha=norm_deta_pXY/10;
% 4th 和 3rd 相比 是将 误差范数除以1.5 不同的系数大小带来的能力不同
 %数值方法怎么解决 orientation
 % simx_opmode_oneshot_wait 是否需要改进需要再想一想
%6th  这版效果也很好
%1.为什么error除以他的模会带来这样的效果  效果巨大 。要考虑清楚：这我已经记在笔记里了。
%2.使用 deta=deta_pXY 而不是 deta=deta_pXY/norm_deta_pXY  步长固定为1.效果也很好
%效果好的原因大概是 这样时间当做无限小。具体的内容我写在笔记里。
%现实世界电机的速度不可能那么快。所以步长也就不可能等于1.
 %将步长变小  迭代的次数就会增加。但是关节的速度会下降，更符合现实意义
function matlab_remote_for_pseudo_inverse_6th()
    global r1; global r2;  
    disp('程序开始');
    vrep=remApi('remoteApi');%生成一个对象 remApi是一个类
    vrep.simxFinish(-1)%在一开始的时候关掉所有的socket连接 just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5)
    motorHandles=[-1,-1];
    %用来存储关节值根据需要予以调整注释
    r1=[];
    r2=[];
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
	
        %获得误差的方向，其实获得了单位向量
		% 4th 和 3rd 相比 是将 误差范数除以1.5 不同的系数大小带来的能力不同
        %deta=deta_pXY/norm_deta_pXY
        deta=deta_pXY
		%伪逆法更新关节值
		J_pinv_svd=getPseudoInverse(theta1,theta2)
		
		%步长的获得方式
		% 4th 和 3rd 相比 是将 误差范数除以1.5 不同的系数大小带来的能力不同
	    %将步长变小  迭代的次数就会增加
		apha=1;	
      
		deta_theta=J_pinv_svd*deta'.*apha;
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
 
 function J_pinv_svd=getPseudoInverse(theta1,theta2)
 
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