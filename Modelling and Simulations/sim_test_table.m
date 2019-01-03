%% Simulation

vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
%target = [0,-pi/6,0.1,pi/2,pi/3,pi/3];
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
[returncode] = vrep.simxSynchronous(clientID,1);
vrep.simxStartSimulation(clientID,vrep.simx_opmode_blocking);
if (clientID>-1)
    disp('Connection Sucessful')
    r =  0.15;
    [~,dt] = vrep.simxGetFloatingParameter(clientID,vrep.sim_floatparam_simulation_time_step,vrep.simx_opmode_blocking);
    [returncode, target] = vrep.simxGetObjectHandle(clientID,'target',vrep.simx_opmode_blocking);
    d_target = 100;
    while d_target > 0.75
     
             [numberreturnCode, WALLE_BASE1]=vrep.simxGetObjectHandle(clientID,'WALLE_BASE',vrep.simx_opmode_blocking);
             [numberreturnCode,ang]= vrep.simxGetObjectOrientation(clientID,WALLE_BASE1,-1,vrep.simx_opmode_blocking);
             J = [0.8579,-0.5139,-0.47;0.8579,0.5139,0.47]*[cos(ang(3)),sin(ang(3)),0;-sin(ang(3)),cos(ang(3)),0;0,0,1]*[0.1;0;0];
             w1_d = J./r;
        
    %code here
    %for i = 1:3
        %[returnCode]=vrep.simxSetObjectPosition(clientID,'Cuboid2',-1,[1.35,-0.5,0.25],vrep.simx_opmode_oneshot)
        %Joint1 Arm 1 and Arm 2
        [returnCode1, Right_Wheel_Motor]=vrep.simxGetObjectHandle(clientID,'WALLE_Right_Axle',vrep.simx_opmode_blocking);
        [returnCode1]=vrep.simxSetJointTargetVelocity(clientID,Right_Wheel_Motor,w1_d(2),vrep.simx_opmode_blocking);
        
        [returnCode2, Left_Wheel_Motor]=vrep.simxGetObjectHandle(clientID,'WALLE_Left_Axle',vrep.simx_opmode_blocking);
        [returnCode2]=vrep.simxSetJointTargetVelocity(clientID,Left_Wheel_Motor,w1_d(1),vrep.simx_opmode_blocking);
        [returnCode,pos]=vrep.simxGetObjectPosition(clientID,target,WALLE_BASE1,vrep.simx_opmode_blocking);
        d_target = pos(1)
    end
    [returnCode1]=vrep.simxSetJointTargetVelocity(clientID,Right_Wheel_Motor,0,vrep.simx_opmode_blocking);
    [returnCode1]=vrep.simxSetJointTargetVelocity(clientID,Left_Wheel_Motor,0,vrep.simx_opmode_blocking);
            
    
    run('Forward_Kinematics.m')
    
    for i = 1:4
        if i == 1
            %For Arm1
            Twn_1 = [1 0 0 0;0 cos(-pi/2) -sin(-pi/2) 0;0 sin(-pi/2) cos(-pi/2) 0;0 0 0 1]*[cos(pi) -sin(pi) 0 0;sin(pi) cos(pi) 0 0;0 0 1 0;0 0 0 1];
            Twn_1(1:3,4) = [0.7;-0.5;0.5];
    
            %For Arm2
            Twn_2 = [1 0 0 0;0 cos(pi/2) -sin(pi/2) 0;0 sin(pi/2) cos(pi/2) 0;0 0 0 1]*[cos(pi) -sin(pi) 0 0;sin(pi) cos(pi) 0 0;0 0 1 0;0 0 0 1];
            Twn_2(1:3,4) = [0.7;0.5;0.5];
        elseif i == 2
            %For Arm1
            Twn_1 = [1 0 0 0;0 cos(-pi/2) -sin(-pi/2) 0;0 sin(-pi/2) cos(-pi/2) 0;0 0 0 1]*[cos(pi) -sin(pi) 0 0;sin(pi) cos(pi) 0 0;0 0 1 0;0 0 0 1];
            Twn_1(1:3,4) = [0.7;-0.267;0.5];
    
            %For Arm2
            Twn_2 = [1 0 0 0;0 cos(pi/2) -sin(pi/2) 0;0 sin(pi/2) cos(pi/2) 0;0 0 0 1]*[cos(pi) -sin(pi) 0 0;sin(pi) cos(pi) 0 0;0 0 1 0;0 0 0 1];
            Twn_2(1:3,4) = [0.7;0.267;0.5];
            
        elseif i == 3     
            %For Arm1
            Twn_1 = [1 0 0 0;0 cos(-pi/2) -sin(-pi/2) 0;0 sin(-pi/2) cos(-pi/2) 0;0 0 0 1]*[cos(pi) -sin(pi) 0 0;sin(pi) cos(pi) 0 0;0 0 1 0;0 0 0 1];
            Twn_1(1:3,4) = [0.7;-0.267;0.5];
    
            %For Arm2
            Twn_2 = [1 0 0 0;0 cos(pi/2) -sin(pi/2) 0;0 sin(pi/2) cos(pi/2) 0;0 0 0 1]*[cos(pi) -sin(pi) 0 0;sin(pi) cos(pi) 0 0;0 0 1 0;0 0 0 1];
            Twn_2(1:3,4) = [0.7;0.267;0.5];
        
        elseif i == 4     
            %For Arm1
            Twn_1 = [1 0 0 0;0 cos(-pi/2) -sin(-pi/2) 0;0 sin(-pi/2) cos(-pi/2) 0;0 0 0 1]*[cos(pi) -sin(pi) 0 0;sin(pi) cos(pi) 0 0;0 0 1 0;0 0 0 1];
            Twn_1(1:3,4) = [0.7;-0.267;0.55];
    
            %For Arm2
            Twn_2 = [1 0 0 0;0 cos(pi/2) -sin(pi/2) 0;0 sin(pi/2) cos(pi/2) 0;0 0 0 1]*[cos(pi) -sin(pi) 0 0;sin(pi) cos(pi) 0 0;0 0 1 0;0 0 0 1];
            Twn_2(1:3,4) = [0.7;0.267;0.55];
        
        end
%% Inverse Position Problem
%%
        %For Arm 1
        Rd1 = double(Twn_1(1:3,1:3));

        xd1 = Twn_1(1,4);
        yd1 = Twn_1(2,4);
        zd1 = Twn_1(3,4);

        oc_x1 = xd1 - (l5+l6)*Rd1(1,3);
        oc_y1 = yd1 - (l5+l6)*Rd1(2,3);
        oc_z1 = zd1 - (l5+l6)*Rd1(3,3);

        R1 = A1(1:3,1:3);
        d1 = A1(1:3,4);

        xc1 = oc_x1;yc1 =oc_y1; zc1 = oc_z1;

        t1_ik1 = atan2((zc1-d1(3)),sqrt((xc1-d1(1))^2 +(yc1+l1-d1(2))^2));
        t2_ik1 = atan2(yc1+l1-d1(2),xc1-d1(1));
        d3_ik1 = sqrt((xc1-d1(1))^2 + (yc1+l1-d1(2))^2 + (zc1-d1(3))^2)-l2-l3-l4;
%% Inverse Orientation Problem
%%
        R36_1 = subs(((R1*T01_1(1:3,1:3)*T12_1(1:3,1:3)*T23_1(1:3,1:3)).'*Rd1),{t1,t2,d3},{t1_ik1,t2_ik1,d3_ik1});
        if R36_1(3,3) >=1 
            R36_1(3,3) = 1;
        end

        t5_ik1 = atan2(sqrt(1-(R36_1(3,3))^2),R36_1(3,3));

        if double(t5_ik1) > 0
            t4_ik1 = atan2(R36_1(2,3),R36_1(1,3));
            t6_ik1 = atan2(-R36_1(3,2),R36_1(3,1));
        elseif double(t5_ik1) < 0
            t4_ik1 = atan2(-R36_1(2,3),-R36_1(1,3));
            t6_ik1 = atan2(R36_1(3,2),-R36_1(3,1));
        elseif double(t5_ik1) == 0
            t4_ik1 = 0;
            t6_ik1 = atan2(R36_1(1,2),-R36_1(1,1));
        end

        %For Arm 2
        Rd2 = double(Twn_2(1:3,1:3));

        xd2 = Twn_2(1,4);
        yd2 = Twn_2(2,4);
        zd2 = Twn_2(3,4);

        oc_x2 = xd2 - (l5+l6)*Rd2(1,3);
        oc_y2 = yd2 - (l5+l6)*Rd2(2,3);
        oc_z2 = zd2 - (l5+l6)*Rd2(3,3);

        R2 = A2(1:3,1:3);
        d2 = A2(1:3,4);

        xc2 = oc_x2;yc2 =oc_y2; zc2 = oc_z2;
        t1_ik2 = atan2((zc2-d2(3)),sqrt((xc2-d2(1))^2 +(yc2-l1-d2(2))^2));
        t2_ik2 = atan2(yc2-l1-d2(2),xc2-d2(1));
        d3_ik2 = sqrt((xc2-d2(1))^2 + (yc2-l1-d2(2))^2 + (zc2-d2(3))^2)-l2-l3-l4;
%% Inverse Orientation Problem
%%
        R36_2 = subs(((R2*T01_2(1:3,1:3)*T12_2(1:3,1:3)*T23_2(1:3,1:3)).'*Rd2),{t1,t2,d3},{t1_ik2,t2_ik2,d3_ik2});
        if R36_2(3,3) >=1 
            R36_2(3,3) = 1;
        end

        t5_ik2 = atan2(sqrt(1-(R36_2(3,3))^2),R36_2(3,3));
        if double(t5_ik2) > 0
            t4_ik2 = atan2(R36_2(2,3),R36_2(1,3));
            t6_ik2 = atan2(-R36_2(3,2),R36_2(3,1));
        elseif double(t5_ik2) < 0
            t4_ik2 = atan2(-R36_2(2,3),-R36_2(1,3));
            t6_ik2 = atan2(R36_2(3,2),-R36_2(3,1));
        elseif double(t5_ik2) == 0
            t4_ik2 = 0;
            t6_ik2 = atan2(R36_2(1,2),-R36_2(1,1));
        end
%% Verification
%%
        %For Arm 1
        Tik1 = vpa(subs(A1*T0n_1,{t1,t2,d3,t4,t5,t6},{t1_ik1,t2_ik1,d3_ik1,t4_ik1,t5_ik1,t6_ik1}),4);
        q1 = vpa([t1_ik1,t2_ik1,d3_ik1,t4_ik1,t5_ik1,t6_ik1]',4);

        %For Arm 2
        Tik2 = vpa(subs(A2*T0n_2,{t1,t2,d3,t4,t5,t6},{t1_ik2,t2_ik2,d3_ik2,t4_ik2,t5_ik2,t6_ik2}),4);
        q2 = vpa([t1_ik2,t2_ik2,d3_ik2,t4_ik2,t5_ik2,t6_ik2]',4);


    
    %code here
    %for i = 1:3
        %[returnCode]=vrep.simxSetObjectPosition(clientID,'Cuboid2',-1,[1.35,-0.5,0.25],vrep.simx_opmode_oneshot)
        %Joint1 Arm 1 and Arm 2
        [returnCode1_1, Right_Shoulder_Motor]=vrep.simxGetObjectHandle(clientID,'Right_Shoulder',vrep.simx_opmode_blocking);
        [returnCode1_1]=vrep.simxSetJointTargetPosition(clientID,Right_Shoulder_Motor,q1(1),vrep.simx_opmode_blocking);
        [returnCode1_2, Left_Shoulder_Motor]=vrep.simxGetObjectHandle(clientID,'Left_Shoulder',vrep.simx_opmode_blocking);
        [returnCode1_2]=vrep.simxSetJointTargetPosition(clientID,Left_Shoulder_Motor,q2(1),vrep.simx_opmode_blocking);
         
        %Joint2
        [returnCode2_1, Right_Shoulder1_Motor]=vrep.simxGetObjectHandle(clientID,'Right_Shoulder1',vrep.simx_opmode_blocking);
        [returnCode2_1]=vrep.simxSetJointTargetPosition(clientID,Right_Shoulder1_Motor,q1(2),vrep.simx_opmode_oneshot_wait);
        [returnCode2_2, Left_Shoulder1_Motor]=vrep.simxGetObjectHandle(clientID,'Left_Shoulder1',vrep.simx_opmode_blocking);
        [returnCode2_2]=vrep.simxSetJointTargetPosition(clientID,Left_Shoulder1_Motor,q2(2),vrep.simx_opmode_oneshot_wait);
%         if i == 4 
%             [returnCode2_1]=vrep.simxSetJointTargetVelocity(clientID,Right_Shoulder1_Motor,1000,vrep.simx_opmode_oneshot_wait);
%             [returnCode2_2]=vrep.simxSetJointTargetVelocity(clientID,Left_Shoulder1_Motor,-1000,vrep.simx_opmode_oneshot_wait);
%         end
        
        %Joint3
        [returnCode3_1, Right_Elbow_Motor]=vrep.simxGetObjectHandle(clientID,'Right_Elbow',vrep.simx_opmode_blocking);
        [returnCode3_1]=vrep.simxSetJointTargetPosition(clientID,Right_Elbow_Motor,q1(3),vrep.simx_opmode_oneshot_wait);
        [returnCode3_2, Left_Elbow_Motor]=vrep.simxGetObjectHandle(clientID,'Left_Elbow',vrep.simx_opmode_blocking);
        [returnCode3_2]=vrep.simxSetJointTargetPosition(clientID,Left_Elbow_Motor,q2(3),vrep.simx_opmode_oneshot_wait);
        
        %Joint4
        [returnCode4_1, Right_Wrist1_Motor]=vrep.simxGetObjectHandle(clientID,'Right_Wrist1',vrep.simx_opmode_blocking);
        [returnCode4_1]=vrep.simxSetJointTargetPosition(clientID,Right_Wrist1_Motor,q1(4),vrep.simx_opmode_oneshot_wait);
        [returnCode4_2, Left_Wrist1_Motor]=vrep.simxGetObjectHandle(clientID,'Left_Wrist1',vrep.simx_opmode_blocking);
        [returnCode4_2]=vrep.simxSetJointTargetPosition(clientID,Left_Wrist1_Motor,q2(4),vrep.simx_opmode_oneshot_wait);
        
        %Joint5
        [returnCode5_1, Right_Wrist2_Motor]=vrep.simxGetObjectHandle(clientID,'Right_Wrist2',vrep.simx_opmode_blocking);
        [returnCode5_1]=vrep.simxSetJointTargetPosition(clientID,Right_Wrist2_Motor,q1(5),vrep.simx_opmode_oneshot_wait);
        [returnCode5_2, Left_Wrist2_Motor]=vrep.simxGetObjectHandle(clientID,'Left_Wrist2',vrep.simx_opmode_blocking);
        [returnCode5_2]=vrep.simxSetJointTargetPosition(clientID,Left_Wrist2_Motor,q2(5),vrep.simx_opmode_oneshot_wait);
       
        %Joint6
        [returnCode6_1, Right_Wrist3_Motor]=vrep.simxGetObjectHandle(clientID,'Right_Wrist3',vrep.simx_opmode_blocking);
        [returnCode6_1]=vrep.simxSetJointTargetPosition(clientID,Right_Wrist3_Motor,q1(6),vrep.simx_opmode_oneshot_wait);
        [returnCode6_2, Left_Wrist3_Motor]=vrep.simxGetObjectHandle(clientID,'Left_Wrist3',vrep.simx_opmode_blocking);
        [returnCode6_2]=vrep.simxSetJointTargetPosition(clientID,Left_Wrist3_Motor,q2(6),vrep.simx_opmode_oneshot_wait);
        
        pause(5)
        
        i = i+1;
        
    end
    d_target1 = 0;
    while d_target1 < 1.5
     
             [numberreturnCode, WALLE_BASE2]=vrep.simxGetObjectHandle(clientID,'Table',vrep.simx_opmode_blocking);
             [numberreturnCode,ang]= vrep.simxGetObjectOrientation(clientID,WALLE_BASE1,-1,vrep.simx_opmode_blocking);
             J = [0.8579,-0.5139,-0.47;0.8579,0.5139,0.47]*[cos(ang(3)),sin(ang(3)),0;-sin(ang(3)),cos(ang(3)),0;0,0,1]*[-0.1;0;0];
             w1_d = J./r;
        
    %code here
    %for i = 1:3
        %[returnCode]=vrep.simxSetObjectPosition(clientID,'Cuboid2',-1,[1.35,-0.5,0.25],vrep.simx_opmode_oneshot)
        %Joint1 Arm 1 and Arm 2
        [returnCode1, Right_Wheel_Motor]=vrep.simxGetObjectHandle(clientID,'WALLE_Right_Axle',vrep.simx_opmode_blocking);
        [returnCode1]=vrep.simxSetJointTargetVelocity(clientID,Right_Wheel_Motor,w1_d(2),vrep.simx_opmode_blocking);
        
        [returnCode2, Left_Wheel_Motor]=vrep.simxGetObjectHandle(clientID,'WALLE_Left_Axle',vrep.simx_opmode_blocking);
        [returnCode2]=vrep.simxSetJointTargetVelocity(clientID,Left_Wheel_Motor,w1_d(1),vrep.simx_opmode_blocking);
        [returnCode,pos1]=vrep.simxGetObjectPosition(clientID,target,WALLE_BASE2,vrep.simx_opmode_blocking);
        d_target1 = abs(pos1(1));
    end
    [returnCode1]=vrep.simxSetJointTargetVelocity(clientID,Right_Wheel_Motor,0,vrep.simx_opmode_blocking);
    [returnCode1]=vrep.simxSetJointTargetVelocity(clientID,Left_Wheel_Motor,0,vrep.simx_opmode_blocking);
    for i = 1:4
        if i == 1
            %For Arm1
            Twn_1 = [1 0 0 0;0 cos(-pi/2) -sin(-pi/2) 0;0 sin(-pi/2) cos(-pi/2) 0;0 0 0 1]*[cos(pi) -sin(pi) 0 0;sin(pi) cos(pi) 0 0;0 0 1 0;0 0 0 1];
            Twn_1(1:3,4) = [0.7;-0.267;0.55];
    
            %For Arm2
            Twn_2 = [1 0 0 0;0 cos(pi/2) -sin(pi/2) 0;0 sin(pi/2) cos(pi/2) 0;0 0 0 1]*[cos(pi) -sin(pi) 0 0;sin(pi) cos(pi) 0 0;0 0 1 0;0 0 0 1];
            Twn_2(1:3,4) = [0.7;0.267;0.55];
        elseif i == 2
            %For Arm1
            Twn_1 = [1 0 0 0;0 cos(-pi/2) -sin(-pi/2) 0;0 sin(-pi/2) cos(-pi/2) 0;0 0 0 1]*[cos(pi) -sin(pi) 0 0;sin(pi) cos(pi) 0 0;0 0 1 0;0 0 0 1];
            Twn_1(1:3,4) = [0.7;-0.27;0.25];
    
            %For Arm2
            Twn_2 = [1 0 0 0;0 cos(pi/2) -sin(pi/2) 0;0 sin(pi/2) cos(pi/2) 0;0 0 0 1]*[cos(pi) -sin(pi) 0 0;sin(pi) cos(pi) 0 0;0 0 1 0;0 0 0 1];
            Twn_2(1:3,4) = [0.7;0.27;0.25];
            
        elseif i == 3     
            %For Arm1
            Twn_1 = [1 0 0 0;0 cos(-pi/2) -sin(-pi/2) 0;0 sin(-pi/2) cos(-pi/2) 0;0 0 0 1]*[cos(pi) -sin(pi) 0 0;sin(pi) cos(pi) 0 0;0 0 1 0;0 0 0 1];
            Twn_1(1:3,4) = [0.7;-0.45;0.25];
    
            %For Arm2
            Twn_2 = [1 0 0 0;0 cos(pi/2) -sin(pi/2) 0;0 sin(pi/2) cos(pi/2) 0;0 0 0 1]*[cos(pi) -sin(pi) 0 0;sin(pi) cos(pi) 0 0;0 0 1 0;0 0 0 1];
            Twn_2(1:3,4) = [0.7;0.45;0.25];
        
        elseif i == 4     
            %For Arm1
            Twn_1 = [1 0 0 0;0 cos(-pi/2) -sin(-pi/2) 0;0 sin(-pi/2) cos(-pi/2) 0;0 0 0 1]*[cos(pi) -sin(pi) 0 0;sin(pi) cos(pi) 0 0;0 0 1 0;0 0 0 1];
            Twn_1(1:3,4) = [0.7;-0.39;0.595];
    
            %For Arm2
            Twn_2 = [1 0 0 0;0 cos(pi/2) -sin(pi/2) 0;0 sin(pi/2) cos(pi/2) 0;0 0 0 1]*[cos(pi) -sin(pi) 0 0;sin(pi) cos(pi) 0 0;0 0 1 0;0 0 0 1];
            Twn_2(1:3,4) = [0.7;0.39;0.595];
        
        end
%% Inverse Position Problem
%%
        %For Arm 1
        Rd1 = double(Twn_1(1:3,1:3));

        xd1 = Twn_1(1,4);
        yd1 = Twn_1(2,4);
        zd1 = Twn_1(3,4);

        oc_x1 = xd1 - (l5+l6)*Rd1(1,3);
        oc_y1 = yd1 - (l5+l6)*Rd1(2,3);
        oc_z1 = zd1 - (l5+l6)*Rd1(3,3);

        R1 = A1(1:3,1:3);
        d1 = A1(1:3,4);

        xc1 = oc_x1;yc1 =oc_y1; zc1 = oc_z1;

        t1_ik1 = atan2((zc1-d1(3)),sqrt((xc1-d1(1))^2 +(yc1+l1-d1(2))^2));
        t2_ik1 = atan2(yc1+l1-d1(2),xc1-d1(1));
        d3_ik1 = sqrt((xc1-d1(1))^2 + (yc1+l1-d1(2))^2 + (zc1-d1(3))^2)-l2-l3-l4;
%% Inverse Orientation Problem
%%
        R36_1 = subs(((R1*T01_1(1:3,1:3)*T12_1(1:3,1:3)*T23_1(1:3,1:3)).'*Rd1),{t1,t2,d3},{t1_ik1,t2_ik1,d3_ik1});
        if R36_1(3,3) >=1 
            R36_1(3,3) = 1;
        end

        t5_ik1 = atan2(sqrt(1-(R36_1(3,3))^2),R36_1(3,3));

        if double(t5_ik1) > 0
            t4_ik1 = atan2(R36_1(2,3),R36_1(1,3));
            t6_ik1 = atan2(-R36_1(3,2),R36_1(3,1));
        elseif double(t5_ik1) < 0
            t4_ik1 = atan2(-R36_1(2,3),-R36_1(1,3));
            t6_ik1 = atan2(R36_1(3,2),-R36_1(3,1));
        elseif double(t5_ik1) == 0
            t4_ik1 = 0;
            t6_ik1 = atan2(R36_1(1,2),-R36_1(1,1));
        end

        %For Arm 2
        Rd2 = double(Twn_2(1:3,1:3));

        xd2 = Twn_2(1,4);
        yd2 = Twn_2(2,4);
        zd2 = Twn_2(3,4);

        oc_x2 = xd2 - (l5+l6)*Rd2(1,3);
        oc_y2 = yd2 - (l5+l6)*Rd2(2,3);
        oc_z2 = zd2 - (l5+l6)*Rd2(3,3);

        R2 = A2(1:3,1:3);
        d2 = A2(1:3,4);

        xc2 = oc_x2;yc2 =oc_y2; zc2 = oc_z2;
        t1_ik2 = atan2((zc2-d2(3)),sqrt((xc2-d2(1))^2 +(yc2-l1-d2(2))^2));
        t2_ik2 = atan2(yc2-l1-d2(2),xc2-d2(1));
        d3_ik2 = sqrt((xc2-d2(1))^2 + (yc2-l1-d2(2))^2 + (zc2-d2(3))^2)-l2-l3-l4;
%% Inverse Orientation Problem
%%
        R36_2 = subs(((R2*T01_2(1:3,1:3)*T12_2(1:3,1:3)*T23_2(1:3,1:3)).'*Rd2),{t1,t2,d3},{t1_ik2,t2_ik2,d3_ik2});
        if R36_2(3,3) >=1 
            R36_2(3,3) = 1;
        end

        t5_ik2 = atan2(sqrt(1-(R36_2(3,3))^2),R36_2(3,3));
        if double(t5_ik2) > 0
            t4_ik2 = atan2(R36_2(2,3),R36_2(1,3));
            t6_ik2 = atan2(-R36_2(3,2),R36_2(3,1));
        elseif double(t5_ik2) < 0
            t4_ik2 = atan2(-R36_2(2,3),-R36_2(1,3));
            t6_ik2 = atan2(R36_2(3,2),-R36_2(3,1));
        elseif double(t5_ik2) == 0
            t4_ik2 = 0;
            t6_ik2 = atan2(R36_2(1,2),-R36_2(1,1));
        end
%% Verification
%%
        %For Arm 1
        Tik1 = vpa(subs(A1*T0n_1,{t1,t2,d3,t4,t5,t6},{t1_ik1,t2_ik1,d3_ik1,t4_ik1,t5_ik1,t6_ik1}),4);
        q1 = vpa([t1_ik1,t2_ik1,d3_ik1,t4_ik1,t5_ik1,t6_ik1]',4);

        %For Arm 2
        Tik2 = vpa(subs(A2*T0n_2,{t1,t2,d3,t4,t5,t6},{t1_ik2,t2_ik2,d3_ik2,t4_ik2,t5_ik2,t6_ik2}),4);
        q2 = vpa([t1_ik2,t2_ik2,d3_ik2,t4_ik2,t5_ik2,t6_ik2]',4);


    
    %code here
    %for i = 1:3
        %[returnCode]=vrep.simxSetObjectPosition(clientID,'Cuboid2',-1,[1.35,-0.5,0.25],vrep.simx_opmode_oneshot)
        %Joint1 Arm 1 and Arm 2
        [returnCode1_1, Right_Shoulder_Motor]=vrep.simxGetObjectHandle(clientID,'Right_Shoulder',vrep.simx_opmode_blocking);
        [returnCode1_1]=vrep.simxSetJointTargetPosition(clientID,Right_Shoulder_Motor,q1(1),vrep.simx_opmode_blocking);
        [returnCode1_2, Left_Shoulder_Motor]=vrep.simxGetObjectHandle(clientID,'Left_Shoulder',vrep.simx_opmode_blocking);
        [returnCode1_2]=vrep.simxSetJointTargetPosition(clientID,Left_Shoulder_Motor,q2(1),vrep.simx_opmode_blocking);
         
        %Joint2
        [returnCode2_1, Right_Shoulder1_Motor]=vrep.simxGetObjectHandle(clientID,'Right_Shoulder1',vrep.simx_opmode_blocking);
        [returnCode2_1]=vrep.simxSetJointTargetPosition(clientID,Right_Shoulder1_Motor,q1(2),vrep.simx_opmode_oneshot_wait);
        [returnCode2_2, Left_Shoulder1_Motor]=vrep.simxGetObjectHandle(clientID,'Left_Shoulder1',vrep.simx_opmode_blocking);
        [returnCode2_2]=vrep.simxSetJointTargetPosition(clientID,Left_Shoulder1_Motor,q2(2),vrep.simx_opmode_oneshot_wait);
%         if i == 4 
%             [returnCode2_1]=vrep.simxSetJointTargetVelocity(clientID,Right_Shoulder1_Motor,1000,vrep.simx_opmode_oneshot_wait);
%             [returnCode2_2]=vrep.simxSetJointTargetVelocity(clientID,Left_Shoulder1_Motor,-1000,vrep.simx_opmode_oneshot_wait);
%         end
        
        %Joint3
        [returnCode3_1, Right_Elbow_Motor]=vrep.simxGetObjectHandle(clientID,'Right_Elbow',vrep.simx_opmode_blocking);
        [returnCode3_1]=vrep.simxSetJointTargetPosition(clientID,Right_Elbow_Motor,q1(3),vrep.simx_opmode_oneshot_wait);
        [returnCode3_2, Left_Elbow_Motor]=vrep.simxGetObjectHandle(clientID,'Left_Elbow',vrep.simx_opmode_blocking);
        [returnCode3_2]=vrep.simxSetJointTargetPosition(clientID,Left_Elbow_Motor,q2(3),vrep.simx_opmode_oneshot_wait);
        
        %Joint4
        [returnCode4_1, Right_Wrist1_Motor]=vrep.simxGetObjectHandle(clientID,'Right_Wrist1',vrep.simx_opmode_blocking);
        [returnCode4_1]=vrep.simxSetJointTargetPosition(clientID,Right_Wrist1_Motor,q1(4),vrep.simx_opmode_oneshot_wait);
        [returnCode4_2, Left_Wrist1_Motor]=vrep.simxGetObjectHandle(clientID,'Left_Wrist1',vrep.simx_opmode_blocking);
        [returnCode4_2]=vrep.simxSetJointTargetPosition(clientID,Left_Wrist1_Motor,q2(4),vrep.simx_opmode_oneshot_wait);
        
        %Joint5
        [returnCode5_1, Right_Wrist2_Motor]=vrep.simxGetObjectHandle(clientID,'Right_Wrist2',vrep.simx_opmode_blocking);
        [returnCode5_1]=vrep.simxSetJointTargetPosition(clientID,Right_Wrist2_Motor,q1(5),vrep.simx_opmode_oneshot_wait);
        [returnCode5_2, Left_Wrist2_Motor]=vrep.simxGetObjectHandle(clientID,'Left_Wrist2',vrep.simx_opmode_blocking);
        [returnCode5_2]=vrep.simxSetJointTargetPosition(clientID,Left_Wrist2_Motor,q2(5),vrep.simx_opmode_oneshot_wait);
       
        %Joint6
        [returnCode6_1, Right_Wrist3_Motor]=vrep.simxGetObjectHandle(clientID,'Right_Wrist3',vrep.simx_opmode_blocking);
        [returnCode6_1]=vrep.simxSetJointTargetPosition(clientID,Right_Wrist3_Motor,q1(6),vrep.simx_opmode_oneshot_wait);
        [returnCode6_2, Left_Wrist3_Motor]=vrep.simxGetObjectHandle(clientID,'Left_Wrist3',vrep.simx_opmode_blocking);
        [returnCode6_2]=vrep.simxSetJointTargetPosition(clientID,Left_Wrist3_Motor,q2(6),vrep.simx_opmode_oneshot_wait);
        
        pause(5)
        
        i = i+1;
        
    end
    
    [~] = vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot);
    vrep.simxGetPingTime(clientID);
    
    vrep.simxFinish(-1); 
end

vrep.delete();