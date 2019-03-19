function matlab_remote_test()
    disp('程序开始');
    vrep=remApi('remoteApi');%生成一个对象 remApi是一个类
    vrep.simxFinish(-1)%在一开始的时候关掉所有的socket连接 just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5)
    motorHandles=[-1,-1,-1,-1];
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
        %开始写小车控制的程序
        
        
        
        %-----------------------------------------------------------
        [res1,joint_front_left_wheel]=vrep.simxGetObjectHandle(clientID,'joint_front_left_wheel#',vrep.simx_opmode_blocking);
        [res2,joint_front_right_wheel]=vrep.simxGetObjectHandle(clientID,'joint_front_right_wheel#',vrep.simx_opmode_blocking);
        [res3,joint_back_right_wheel]=vrep.simxGetObjectHandle(clientID,'joint_back_right_wheel#',vrep.simx_opmode_blocking);
        [res4,joint_back_left_wheel]=vrep.simxGetObjectHandle(clientID,'joint_back_left_wheel#',vrep.simx_opmode_blocking);
        
        %-----------------------------------------------------------
       
        
         
        vrep.simxSetJointTargetVelocity(clientID,joint_front_left_wheel, 1 ,vrep.simx_opmode_oneshot); 
        vrep.simxSetJointTargetVelocity(clientID,joint_front_right_wheel,-1 ,vrep.simx_opmode_oneshot); 
        vrep.simxSetJointTargetVelocity(clientID,joint_back_right_wheel,-1 ,vrep.simx_opmode_oneshot);
        vrep.simxSetJointTargetVelocity(clientID,joint_back_left_wheel, 1 ,vrep.simx_opmode_oneshot);
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
