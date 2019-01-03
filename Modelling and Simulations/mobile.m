%% Simulation

vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
%target = [0,-pi/6,0.1,pi/2,pi/3,pi/3];
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
[returncode] = vrep.simxSynchronous(clientID,1);
vrep.simxStartSimulation(clientID,vrep.simx_opmode_blocking);
if (clientID>-1)
    disp('Connection Sucessful')
    run('Forward_Kinematics.m')
    r =  0.15;
    [~,dt] = vrep.simxGetFloatingParameter(clientID,vrep.sim_floatparam_simulation_time_step,vrep.simx_opmode_blocking);
    while d_target > 0.2
     
             [numberreturnCode, WALLE_BASE1]=vrep.simxGetObjectHandle(clientID,'WALLE_BASE',vrep.simx_opmode_blocking);
             [numberreturnCode,ang]= vrep.simxGetObjectOrientation(clientID,WALLE_BASE1,-1,vrep.simx_opmode_blocking);
             J = [0.8579, -0.5139,-0.47;0.8579,0.5139,0.47]*[cos(ang(3)),sin(ang(3)),0;-sin(ang(3)),cos(ang(3)),0;0,0,1]*[0.1;0.1;0];
             w1_d = J./r
        
    %code here
    %for i = 1:3
        %[returnCode]=vrep.simxSetObjectPosition(clientID,'Cuboid2',-1,[1.35,-0.5,0.25],vrep.simx_opmode_oneshot)
        %Joint1 Arm 1 and Arm 2
        [returnCode1, Right_Wheel_Motor]=vrep.simxGetObjectHandle(clientID,'WALLE_Right_Axle',vrep.simx_opmode_blocking);
        [returnCode1]=vrep.simxSetJointTargetVelocity(clientID,Right_Wheel_Motor,w1_d(2),vrep.simx_opmode_blocking);
        
        [returnCode2, Left_Wheel_Motor]=vrep.simxGetObjectHandle(clientID,'WALLE_Left_Axle',vrep.simx_opmode_blocking);
        [returnCode2]=vrep.simxSetJointTargetVelocity(clientID,Left_Wheel_Motor,w1_d(1),vrep.simx_opmode_blocking);
        pause(0.1) 
        i = i+1;
        
    end
    
    [~] = vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot);
    vrep.simxGetPingTime(clientID);
    
    vrep.simxFinish(-1); 
end

vrep.delete();