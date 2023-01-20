%% Initialize Connection
clc
disp('Program started');
vrep = remApi('remoteApi');
vrep.simxFinish(-1);

clientID = vrep.simxStart('127.0.0.1',19997,true,true,5000,5); %establish connection
vrep.simxSynchronous(clientID,true); % enable the synchronous mode on the client:
vrep.simxStartSimulation(clientID,vrep.simx_opmode_blocking); % start the simulation:

joints = zeros(6,3);

if (clientID>-1)
    disp('Connected to simulator');
    % Handles
        for limb = 1:3
        for joint = 1:3
            [ret_code1,joints(limb,joint)] = vrep.simxGetObjectHandle(clientID,['L',num2str(limb),'_joint',num2str(joint)],vrep.simx_opmode_blocking);
            [ret_code2,joints(limb+3,joint)] = vrep.simxGetObjectHandle(clientID,['R',num2str(limb),'_joint',num2str(joint)],vrep.simx_opmode_blocking);
        end
        end
end
vrep.simxSynchronousTrigger(clientID);

%% Actuation

angle = rad(30);

if (clientID>-1)
    disp('Moving joints');

    
    for limb = 1:6
        for joint = 1:3
            vrep.simxSetJointTargetPosition(clientID,joints(limb,joint),0,vrep.simx_opmode_blocking);
        end
    end
    
    for step = 1:100
        vrep.simxSynchronousTrigger(clientID);
        pause(0.01);
    end
    
    for limb = 1:3
        for joint = 2
            vrep.simxSetJointTargetPosition(clientID,joints(limb,joint),angle,vrep.simx_opmode_blocking);
            vrep.simxSetJointTargetPosition(clientID,joints(limb+3,joint),angle,vrep.simx_opmode_blocking);
        end
        for joint = 3
            vrep.simxSetJointTargetPosition(clientID,joints(limb,joint),angle,vrep.simx_opmode_blocking);
            vrep.simxSetJointTargetPosition(clientID,joints(limb+3,joint),-angle,vrep.simx_opmode_blocking);
        end
    end
    
    for step = 1:100
        vrep.simxSynchronousTrigger(clientID);
        pause(0.01);
    end
    
    for limb = 1:3
        for joint = 2
            vrep.simxSetJointTargetPosition(clientID,joints(limb,joint),-angle,vrep.simx_opmode_blocking);
            vrep.simxSetJointTargetPosition(clientID,joints(limb+3,joint),-angle,vrep.simx_opmode_blocking);
        end
        for joint = 3
            vrep.simxSetJointTargetPosition(clientID,joints(limb,joint),-angle,vrep.simx_opmode_blocking);
            vrep.simxSetJointTargetPosition(clientID,joints(limb+3,joint),angle,vrep.simx_opmode_blocking);
        end
    end
    
    for step = 1:100
        vrep.simxSynchronousTrigger(clientID);
        pause(0.01);
    end
    
    for limb = 1:3
        for joint = 1:3
            vrep.simxSetJointTargetPosition(clientID,joints(limb,joint),0,vrep.simx_opmode_blocking);
            vrep.simxSetJointTargetPosition(clientID,joints(limb+3,joint),0,vrep.simx_opmode_blocking);
        end
    end
    
    for step = 1:100
        vrep.simxSynchronousTrigger(clientID);
        pause(0.01);
    end
    
else
    disp('Client not connected');
end

%% Disconnect
vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking);
vrep.simxFinish(-1); %close all opened connections
vrep.delete(); % call the destructor!
disp('Program ended');


%% Functions

function angle = rad(deg)
angle = deg*pi/180;
end