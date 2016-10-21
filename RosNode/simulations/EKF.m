%%%% Extended Kalman Filter implentation for an ackerrman vehicle
time_step = 0.025; %40 hertz update frequency
state = zeros(8,1); %x_position, y_position, heading,
                %x_linear velocity, y_linear velocity, angular velocity,
                %x_angular acceleration,y_angular acceleration
command = zeros(2,1);%throttle, steering
imu_measurements = zeros(3,1); %yaw, rotational velocity,
                              %x_linear_acceleration, y_linear_acceleration
vehicle_noise_vector = [[1,0];%throttle variance and steering variance
                        [0,1]];%across the diagonal
dwb = 0.25; %distance between wheels

figure(1);
for ii=0:25
    %display(ii);
    [command(1), command(2)] = generate_command(ii);

    %prediction step
    last_position = [state(1), state(2)];      % last Point


    omega = (command(1) * tan(command(2))) / dwb;%angular velocity
    state(1) = state(1) - ((command(1)/omega) * sin(state(3)));% + ((command(1)/omega) * sin(state(3) + (omega*time_step)));
    state(2) = state(2) + ((command(1)/omega) * cos(state(3)))- ((command(1)/omega) * cos(state(3) + (omega*time_step)));
    state(3) = state(3) + omega*time_step;
    
    state(3) = atan2(sin(state(3)),cos(state(3)));

    quiver(last_position(1),last_position(2),state(1) - last_position(1),state(2) - last_position(2),0,'MaxHeadSize',0.8);
    hold on;
    grid
end