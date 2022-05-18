function Callback(~, message)
    %exampleHelperROSPoseCallback Subscriber callback function for pose data    
    %   exampleHelperROSPoseCallback(~,MESSAGE) returns no arguments - it instead sets 
    %   global variables to the values of position and orientation that are
    %   received in the ROS message MESSAGE.
    %   
    %   See also ROSPublishAndSubscribeExample.
    
    %   Copyright 2014-2015 The MathWorks, Inc.
    
    % Declare global variables to store position and orientation
    %global vel
    global pos_all;
    global pos_used;

    % Extract position and orientation from the ROS message and assign the
    % data to the global variables.
    %vel = [vel, [message.Twist.Twist.Linear.X message.Twist.Twist.Linear.Y message.Twist.Twist.Linear.Z]];
           pos_all = [message.Pose.Pose.Position.X message.Pose.Pose.Position.Y message.Pose.Pose.Position.Z];
           if 
           p = [p; pos];
   if (pos(2) < 0)
       
       rosshutdown;
   end
        %end
   % end
%     if pos(2) < 00
% 
%         pos = current;
%     end
    %pos = [message.Pose.Twist.Linear.X message.Twist.Twist.Linear.Y message.Twist.Twist.Linear.Z];
    %orient = [message.Angular.X message.Angular.Y message.Angular.Z];
end