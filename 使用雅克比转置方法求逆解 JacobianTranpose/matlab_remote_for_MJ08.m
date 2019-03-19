function matlab_remote_for_MJ08()
    disp('����ʼ');
    vrep=remApi('remoteApi');%����һ������ remApi��һ����
    vrep.simxFinish(-1)%��һ��ʼ��ʱ��ص����е�socket���� just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5)
    motorHandles=[-1,-1,-1,-1];
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
        %��ʼд���ƻ�е�۵ĳ���  ��Ҫ�����е�۵�ÿһ���ؽڷ��͹ؽ�ֵ
        
        [res1,joint_1]=vrep.simxGetObjectHandle(clientID,'joint_1#',vrep.simx_opmode_blocking);
        [res2,joint_2]=vrep.simxGetObjectHandle(clientID,'joint_2#',vrep.simx_opmode_blocking);
        [res3,joint_3]=vrep.simxGetObjectHandle(clientID,'joint_3#',vrep.simx_opmode_blocking);
        [res4,joint_4]=vrep.simxGetObjectHandle(clientID,'joint_4#',vrep.simx_opmode_blocking);
        [res5,joint_5]=vrep.simxGetObjectHandle(clientID,'joint_5#',vrep.simx_opmode_blocking);
        [res6,joint_6]=vrep.simxGetObjectHandle(clientID,'joint_6#',vrep.simx_opmode_blocking);
       
   
        
        %������Ҫע�����  ����joint�Ǵ���inverse mode �ؽڽǶȺ�����Ҫѡ��ΪsimxSetJointPosition
        %��������������ؽ�ֵ
        res7=vrep.simxSetJointPosition(clientID,joint_1, 0.1 ,vrep.simx_opmode_oneshot); 
        res8=vrep.simxSetJointPosition(clientID,joint_2, 0.1 ,vrep.simx_opmode_oneshot); 
        res9=vrep.simxSetJointPosition(clientID,joint_3, 0.1 ,vrep.simx_opmode_oneshot); 
        res10=vrep.simxSetJointPosition(clientID,joint_4, 0.5 ,vrep.simx_opmode_oneshot); 
        res11=vrep.simxSetJointPosition(clientID,joint_5, 0.1 ,vrep.simx_opmode_oneshot); 
        res12=vrep.simxSetJointPosition(clientID,joint_6, 3 ,vrep.simx_opmode_oneshot);
        
       
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
