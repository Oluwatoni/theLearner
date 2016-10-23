function [throttle, steering] = generate_command(time)
%drive in a straight line
    throttle = time/10; %m/s
    steering = sin(time)/2 +.001; %radians
end
