clc
disp('Program started');
vrep=remApi('remoteApi');
vrep.simxFinish(-1);
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

if (clientID>-1)
    disp('Connected to remote API server');
    
    angles = [0,-30,-30,0,-30,-30,0,-30,-30,0,-30,-30,0,-30,-30,0,-30,-30];
    packedData = vrep.simxPackInts(angles);
    error_code = vrep.simxSetStringSignal(clientID,'positions',packedData,vrep.simx_opmode_oneshot);
    %error_code = vrep.simxWriteStringStream(clientID,'positions',packedData,vrep.simx_opmode_oneshot);
    angles = vrep.simxUnpackInts(packedData);
    if error_code == 0
        disp('Data sent successfully');
    else
        disp('Failure to send data');
    end
    
    vrep.simxFinish(clientID);
else
    disp('Failed connecting to remote API server');
end
vrep.delete(); % call the destructor!

disp('Program ended');



