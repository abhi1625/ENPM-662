vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
target = [0,-pi/6,0.1,pi/2,pi/3,pi/3];
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

if (clientID>-1)
    disp('Connection Sucessful')
    
    %code here
    %Joint1
    [returnCode1, Right_Shoulder_Motor]=vrep.simxGetObjectHandle(clientID,'Right_Shoulder',vrep.simx_opmode_blocking);
    [returnCode1]=vrep.simxSetJointTargetPosition(clientID,Right_Shoulder_Motor,target(1),vrep.simx_opmode_blocking);
    %Joint2
    [returnCode2, Right_Shoulder1_Motor]=vrep.simxGetObjectHandle(clientID,'Right_Shoulder1',vrep.simx_opmode_blocking);
    [returnCode2]=vrep.simxSetJointTargetPosition(clientID,Right_Shoulder1_Motor,target(2),vrep.simx_opmode_oneshot_wait);
    %Joint3
    [returnCode3, Right_Elbow_Motor]=vrep.simxGetObjectHandle(clientID,'Right_Elbow',vrep.simx_opmode_blocking);
    [returnCode3]=vrep.simxSetJointTargetPosition(clientID,Right_Elbow_Motor,target(3),vrep.simx_opmode_oneshot_wait);
    %Joint4
    [returnCode4, Right_Wrist1_Motor]=vrep.simxGetObjectHandle(clientID,'Right_Wrist1',vrep.simx_opmode_blocking);
    [returnCode4]=vrep.simxSetJointTargetPosition(clientID,Right_Wrist1_Motor,target(4),vrep.simx_opmode_oneshot_wait);
    %Joint5
    [returnCode5, Right_Wrist2_Motor]=vrep.simxGetObjectHandle(clientID,'Right_Wrist2',vrep.simx_opmode_blocking);
    [returnCode5]=vrep.simxSetJointTargetPosition(clientID,Right_Wrist2_Motor,target(5),vrep.simx_opmode_oneshot_wait);
    %Joint6
    [returnCode6, Right_Wrist3_Motor]=vrep.simxGetObjectHandle(clientID,'Right_Wrist3',vrep.simx_opmode_blocking);
    [returnCode6]=vrep.simxSetJointTargetPosition(clientID,Right_Wrist3_Motor,target(6),vrep.simx_opmode_oneshot_wait);
    
    pause(10)
    %[returnCode]=vrep.simxSetJointTargetVelocity(clientID,Left_Motor,0,vrep.simx_opmode_oneshot_wait
    vrep.simxFinish(-1); 
end

vrep.delete();
