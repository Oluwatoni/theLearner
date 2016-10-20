%%%% Extended Kalman Filter implentation for an ackerrman vehicle
time_step = 0.025; %40 hertz update frequency
state = zeros(8,1); %x_position, y_position, heading,
                %x_linear velocity, y_linear velocity, rotational velocity,
                %x_linear_acceleration,y_linear_acceleration
command = zeros(2,1); %throttle, steering
imu_measurements = zeros(3,1); %yaw, rotational velocity,
                              %x_linear_acceleration, y_linear_acceleration
for ii=0:500
   %display(ii);
   command = generate_command(ii);
   
   %prediction step
   
end