clear all; close all; clc;

%% Initialization

bot = robot();

%% Coppeliasim

if (bot.clientID > -1)
    disp('Connection to robot successful');
    [~, ~, ~, ~] = bot.initialize_robot();
 
    bot.core_routine();
    %bot.arm_routine("pick");
    %bot.arm_routine("place");
    %demo_door(bot)
    %demo_movement(bot)
    %demo_arm(bot)
   
    bot.terminate_robot();
    
else
    disp('Failed connecting to remote API server');
end

disp("Simulation ended");

%% Termination

bot.destructor();

disp('Connection terminated');


%% Demonstration functions

% Demo door control
function [] = demo_door(bot)
    
    disp("Demo door control routine started");

    for i = 1:8
        bot.control_door(i, 1);
        pause(1);
        bot.control_door(i, 0);
        pause(1);
    end
    
    disp("Demo door control routine finished");

end

function [] = demo_movement(bot)
    disp("Demo movement routine started");

    figure()
    % demo code for going back and forth 5 times
    for i = 1:5 % change with while loop later
        
        tic
        wheel_velocity = [6, 6, 6, 6];
        bot.set_wheel_velocity(wheel_velocity);
        
        while (toc < 2)
            bot.update_cameras();
            % bot.save_images('D:/output_images');
            imshow(bot.frame_left);
            drawnow;
        end
        
        
        tic
        wheel_velocity = [-6, -6, -6, -6];
        bot.set_wheel_velocity(wheel_velocity);
        
        while (toc < 2)
            bot.update_cameras();
            imshow(bot.frame_left);
            drawnow;
        end
        
    end
    
    % Halt robot after 5 iterations
    [wheel_velocity] = bot.lfr_routine(0);
    bot.set_wheel_velocity(wheel_velocity);
    
    close all;
    disp("Demo movement routine finished");

end

% Demo arm control
function [] = demo_arm(bot)

    disp("Demo arm control routine started");
    
    [~] = bot.update_joint_angle();
    angle = [0.5, 0.6, 0.7, 0.8, 0, 0, 0];
    bot.set_joint_position(angle);
    pause(2);
    
    %{
    [~] = bot.update_joint_angle();
    angle = bot.joint_angle + [-1, -0.5, -0.5, 0, 0, 0, 0];
    bot.set_joint_position(angle);
    pause(2);
    
    [~] = bot.update_joint_angle();
    angle = [0, 0, 0, 0, 0, 0, 0];
    bot.set_joint_position(angle);
    pause(2);
    %}
     
    disp("Demo arm control routine finished");
    
end












