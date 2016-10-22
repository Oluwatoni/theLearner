figure(1);

%%%% Extended Kalman Filter implentation for an ackerrman vehicle
dt = 0.025; %40 hertz update frequency (timestep)
state_est = zeros(8,1); %estimate of x_position, y_position, heading,
                %x_linear velocity, y_linear velocity, angular velocity,
                %x_angular acceleration,y_angular acceleration
state = zeros(8,1); %actual x_position, y_position, heading
                    %x_linear_velocity, y_linear velocity, angular velocity,
                    %x_angular acceleration,y_angular acceleration
new_state = zeros(8,1);
state_covariance = [[10,0,0,0,0,0,0,0]; %state uncertainty covariance matrix
                    [0,10,0,0,0,0,0,0];
                    [0,0,10,0,0,0,0,0];
                    [0,0,0,10,0,0,0,0];
                    [0,0,0,0,10,0,0,0];
                    [0,0,0,0,0,10,0,0];
                    [0,0,0,0,0,0,10,0];
                    [0,0,0,0,0,0,0,10]];
command = zeros(2,1);%throttle, steering
imu_measurements = zeros(3,1); %yaw, rotational velocity,
                              %x_linear_acceleration, y_linear_acceleration
vehicle_noise_vector = [[.3,0];%throttle variance and steering variance
                        [0,.25]];%across the diagonal
dwb = 0.25; %distance between wheels


for ii = 0:15
    [command(1), command(2)] = generate_command(ii);

    %%%%%%%%%%%% prediction step %%%%%%%%%%%%%%%%%
    last_position = [state_est(1), state_est(2)];      % last Point

    state_transition = [[1,0,0,sin(state_est(3)*dt),sin(state_est(3)- pi/2)*dt,0,0,0];
                        [0,1,0,cos(state_est(3)*dt),cos(state_est(3)- pi/2)*dt,0,0,0];
                        [0,0,1,0,0,dt,0,0];
                        [0,0,0,1,0,0,dt,0];
                        [0,0,0,0,1,0,0,dt];
                        [0,0,0,0,0,1,0,0];
                        [0,0,0,0,0,0,1,0];
                        [0,0,0,0,0,0,0,1]];
    %propagate the new state forward
    new_state = state_transition * state_est;

    %add control command
    omega = (command(1) * tan(command(2))) / dwb;%angular velocity
    state_est(1) = state_est(1) - ((command(1)/omega) * sin(state_est(3))) + ((command(1)/omega) * sin(state_est(3) + (omega*time_step)));
    state_est(2) = state_est(2) + ((command(1)/omega) * cos(state_est(3)))- ((command(1)/omega) * cos(state_est(3) + (omega*dt)));
    state_est(3) = atan2(sin(state_est(3) + omega*dt),cos(state_est(3) + omega*dt));

    %propagate state uncertainty
    dx_dtheta = state_est(4)*cos(state_est(3))*dt + state_est(5)*cos(state_est(3)-pi/2)*dt;
    dy_dtheta = state_est(4)*sin(state_est(3))*dt + state_est(5)*sin(state_est(3)-pi/2)*dt;
    state_transition_jacobian = [[1,0,dx_dtheta,sin(state_est(3))*dt,sin(state_est(3)-pi/2)*dt,0,0,0];
                                 [0,1,dy_dtheta,cos(state_est(3))*dt,cos(state_est(3)-pi/2)*dt,0,0,0];
                                 [0,0,1,0,0,dt,0,0];
                                 [0,0,0,1,0,0,dt,0];
                                 [0,0,0,0,1,0,0,dt];
                                 [0,0,0,0,0,0,0,0];
                                 [0,0,0,0,0,0,0,0];
                                 [0,0,0,0,0,0,0,0];];
    state_covariance = state_transition_jacobian * state_covariance * transpose(state_transition_jacobian);
    %visualize results
    quiver(last_position(1),last_position(2),state_est(1) - last_position(1),state_est(2) - last_position(2),0,'MaxHeadSize',0.8,'color','red');
    hold on;
    grid

    %%%%%%%%%%%% model update %%%%%%%%%%%%%%%%%
    %cite http://web.cecs.pdx.edu/~mperkows/temp/KALMAN/KALMAN-PAPERS/EKF-mobile-iran-GOOD.pdf
    last_position = [state(1), state(2)];      % last Point
    command(1) = add_gauss_noise(command(1),vehicle_noise_vector(1,1));
    command(2) = add_gauss_noise(command(2),vehicle_noise_vector(2,2));

    state(1) = state(1) + dt*command(1)*cos(state(3));
    state(2) = state(2) + dt*command(1)*sin(state(3));
    state(3) = state(3) + (dt*command(1)*tan(command(2)))/dwb;
    %visualize results
    quiver(last_position(1),last_position(2),state(1) - last_position(1),state(2) - last_position(2),0,'MaxHeadSize',0.8,'color','blue');
    hold on;
    grid
    %%%%%%%%%%%% prediction step %%%%%%%%%%%%%%%%%

end
