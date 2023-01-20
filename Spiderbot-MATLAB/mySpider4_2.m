%% Initialize Connection
% scene1.ttt
clc
disp('Program started');
vrep = remApi('remoteApi');
vrep.simxFinish(-1);

sim_len = 930;

clientID = vrep.simxStart('127.0.0.1',19997,true,true,5000,5); %establish connection
vrep.simxSynchronous(clientID,true); % enable the synchronous mode on the client:
vrep.simxStartSimulation(clientID,vrep.simx_opmode_blocking); % start the simulation:

if (clientID>-1)
    disp('Connected to simulator');
    joints = init_joints(vrep, clientID);
    angles = zeros(6,3,sim_len);
    ANG = zeros(6,3);
end

dt = 15;
[hexapod,xyz,aby,vel,angV] = init_states(vrep,clientID, sim_len);

vrep.simxSynchronousTrigger(clientID);

%% Actuation

angle1 = rad(10);
angle2 = rad(20);

if (clientID>-1)
    disp('Moving joints');
    
    for iteration = 1:10
        disp(['iteration ',int2str(iteration)]);
        
        angles(1,2,((iteration-1)*6+2)*dt:end) = -angle2;
        angles(1,3,((iteration-1)*6+2)*dt:end) = -angle2;
        angles(3,2,((iteration-1)*6+2)*dt:end) = -angle2;
        angles(3,3,((iteration-1)*6+2)*dt:end) = -angle2;
        angles(5,2,((iteration-1)*6+2)*dt:end) = -angle2;
        angles(5,3,((iteration-1)*6+2)*dt:end) = -angle2;
        
        angles(1,1,((iteration-1)*6+3)*dt:end) = -angle1;
        angles(3,1,((iteration-1)*6+3)*dt:end) = -angle1;
        angles(5,1,((iteration-1)*6+3)*dt:end) = -angle1;
        angles(2,1,((iteration-1)*6+3)*dt:end) = angle1;
        angles(4,1,((iteration-1)*6+3)*dt:end) = angle1;
        angles(6,1,((iteration-1)*6+3)*dt:end) = angle1;
        
        angles(1,2,((iteration-1)*6+4)*dt:end) = 0;
        angles(1,3,((iteration-1)*6+4)*dt:end) = 0;
        angles(3,2,((iteration-1)*6+4)*dt:end) = 0;
        angles(3,3,((iteration-1)*6+4)*dt:end) = 0;
        angles(5,2,((iteration-1)*6+4)*dt:end) = 0;
        angles(5,3,((iteration-1)*6+4)*dt:end) = 0;
        
        angles(2,2,((iteration-1)*6+5)*dt:end) = -angle2;
        angles(2,3,((iteration-1)*6+5)*dt:end) = -angle2;
        angles(4,2,((iteration-1)*6+5)*dt:end) = -angle2;
        angles(4,3,((iteration-1)*6+5)*dt:end) = -angle2;
        angles(6,2,((iteration-1)*6+5)*dt:end) = -angle2;
        angles(6,3,((iteration-1)*6+5)*dt:end) = -angle2;
        
        angles(1,1,((iteration-1)*6+6)*dt:end) = angle1;
        angles(3,1,((iteration-1)*6+6)*dt:end) = angle1;
        angles(5,1,((iteration-1)*6+6)*dt:end) = angle1;
        angles(2,1,((iteration-1)*6+6)*dt:end) = -angle1;
        angles(4,1,((iteration-1)*6+6)*dt:end) = -angle1;
        angles(6,1,((iteration-1)*6+6)*dt:end) = -angle1;
        
        angles(2,2,((iteration-1)*6+7)*dt:end) = 0;
        angles(2,3,((iteration-1)*6+7)*dt:end) = 0;
        angles(4,2,((iteration-1)*6+7)*dt:end) = 0;
        angles(4,3,((iteration-1)*6+7)*dt:end) = 0;
        angles(6,2,((iteration-1)*6+7)*dt:end) = 0;
        angles(6,3,((iteration-1)*6+7)*dt:end) = 0;
    end
    angles(1,1,(iteration*6+1)*dt:end) = 0;
    angles(3,1,(iteration*6+1)*dt:end) = 0;
    angles(5,1,(iteration*6+1)*dt:end) = 0;
    angles(2,1,(iteration*6+1)*dt:end) = 0;
    angles(4,1,(iteration*6+1)*dt:end) = 0;
    angles(6,1,(iteration*6+1)*dt:end) = 0;

    for step = 1:sim_len
        ANG(:,:) = angles(:,:,step);
        actuate(vrep, clientID, joints, ANG);
        vrep.simxSynchronousTrigger(clientID);
        [xyz(step,:),aby(step,:),vel(step,:),angV(step,:)] = get_states(vrep,clientID,hexapod);
    end
    
    disp('stopped actuation');
    
else
    disp('Client not connected');
end

plot_data(step, xyz, aby, vel, angV);

%% Disconnect
vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking);
vrep.simxFinish(-1); %close all opened connections
vrep.delete(); % call the destructor!
disp('Program ended');


%% Functions

function angle = rad(deg)
angle = deg*pi/180;
end

function joints = init_joints(vrep, clientID)
joints = zeros(6,3);
for limb = 1:3
    for joint = 1:3
        [~,joints(limb,joint)] = vrep.simxGetObjectHandle(clientID,['L',num2str(limb),'_joint',num2str(joint)],vrep.simx_opmode_blocking);
        [~,joints(limb+3,joint)] = vrep.simxGetObjectHandle(clientID,['R',num2str(limb),'_joint',num2str(joint)],vrep.simx_opmode_blocking);
    end
end
end

function [hexapod,xyz,aby,vel,angV] = init_states(vrep,clientID, sim_len)

[~,hexapod] = vrep.simxGetObjectHandle(clientID,'Hexapod',vrep.simx_opmode_blocking);

xyz = zeros(sim_len,3);
aby = xyz;
vel = xyz;
angV = xyz;

[~, ~] = vrep.simxGetObjectPosition(clientID, hexapod, -1, vrep.simx_opmode_streaming);
[~, ~] = vrep.simxGetObjectOrientation(clientID, hexapod, -1, vrep.simx_opmode_streaming);
[~, ~, ~] = vrep.simxGetObjectVelocity(clientID, hexapod, vrep.simx_opmode_streaming);

end

function actuate(vrep, clientID, joints, angles)
% left legs
for limb = 1:3
    vrep.simxSetJointTargetPosition(clientID,joints(limb,1),angles(limb,1)+rad(20*(2-limb)),vrep.simx_opmode_oneshot);
    vrep.simxSetJointTargetPosition(clientID,joints(limb,2),angles(limb,2),vrep.simx_opmode_oneshot);
    vrep.simxSetJointTargetPosition(clientID,joints(limb,3),angles(limb,3),vrep.simx_opmode_oneshot);
end
% right legs
for limb = 4:6
    vrep.simxSetJointTargetPosition(clientID,joints(limb,1),-angles(limb,1)-rad(20*(5-limb)),vrep.simx_opmode_oneshot);
    vrep.simxSetJointTargetPosition(clientID,joints(limb,2),angles(limb,2),vrep.simx_opmode_oneshot);
    vrep.simxSetJointTargetPosition(clientID,joints(limb,3),-angles(limb,3),vrep.simx_opmode_oneshot);
end
end

function [position,orientation,linearVelocity,angularVelocity] = get_states(vrep,clientID,hexapod)
[~, position] = vrep.simxGetObjectPosition(clientID, hexapod, -1, vrep.simx_opmode_buffer);
[~, orientation] = vrep.simxGetObjectOrientation(clientID, hexapod, -1, vrep.simx_opmode_buffer);
[~, linearVelocity, angularVelocity] = vrep.simxGetObjectVelocity(clientID, hexapod, vrep.simx_opmode_buffer);
end

function plot_data(sim_len, position, orientation, linearVelocity, angularVelocity)

figure(1);

n = 1:sim_len;

subplot(311)
plot(n,position(:,1),'r',n,position(:,2),'g',n,position(:,3),'b')
title('Position'), legend('x','y','z'), xlabel('Time');

subplot(312)
plot(n,orientation(:,1),'r',n,orientation(:,2),'g',n,orientation(:,3),'b')
title('Orientation'), legend('alpha','beta','gamma'), xlabel('Time');

subplot(313)
plot(n,linearVelocity(:,1),'r',n,linearVelocity(:,2),'g',n,linearVelocity(:,3),'b')
title('Velocity'), legend('Vx','Vy','Vz'), xlabel('Time');

end