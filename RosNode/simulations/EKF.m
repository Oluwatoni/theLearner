figure(1);

%%%% Extended Kalman Filter implentation for an ackerrman vehicle
dt = 0.025; %40 hertz update frequency (timestep)
state_est = zeros(8,1); %estimate of x_position, y_position, heading,
                %x_linear velocity, y_linear velocity, angular velocity,
                %x_angular acceleration,y_angular acceleration
state = zeros(8,1); %actual x_position, y_position, heading
                    %x_linear_velocity, y_linear velocity, angular velocity,
                    %x_angular acceleration,y_angular acceleration
% state(3) = pi;
% state_est(3) = state(3);
state_covariance = [[10,0,0,0,0,0,0,0]; %state uncertainty covariance matrix
                    [0,10,0,0,0,0,0,0];
                    [0,0,10,0,0,0,0,0];
                    [0,0,0,10,0,0,0,0];
                    [0,0,0,0,10,0,0,0];
                    [0,0,0,0,0,10,0,0];
                    [0,0,0,0,0,0,10,0];
                    [0,0,0,0,0,0,0,10]];
motion_noise_covariance = [[0.01,0,0,0,0,0,0,0]; %process noise covariance matrix
                           [0,0.01,0,0,0,0,0,0];
                           [0,0,0.001,0,0,0,0,0];
                           [0,0,0,0.01,0,0,0,0];
                           [0,0,0,0,0.01,0,0,0];
                           [0,0,0,0,0,0.01,0,0];
                           [0,0,0,0,0,0,0.01,0];
                           [0,0,0,0,0,0,0,0.01]];
command = zeros(2,1);%throttle, steering
vehicle_noise_vector = [[.3,0];%throttle variance and steering variance
                        [0,.25]];%across the diagonal
dwb = 0.25; %distance between wheels
imu_covariance = [[.0025,0,0,0];%imu measurement covariance matrix
                  [0,.02,0,0];
                  [0,0,.04,0];
                  [0,0,0,.04]];
imu_jacobian= [[0,0,1,0,0,0,0,0];
               [0,0,0,0,0,1,0,0];
               [0,0,0,0,0,0,1,0];
               [0,0,0,0,0,0,0,1]];
imu_measurements = zeros(4,1);%yaw, rotational velocity,
                              %x_linear_acceleration, y_linear_acceleration
K_imu = zeros(8,1);
I = [[1,0,0,0,0,0,0,0]; %state uncertainty covariance matrix
     [0,1,0,0,0,0,0,0];
     [0,0,1,0,0,0,0,0];
     [0,0,0,1,0,0,0,0];
     [0,0,0,0,1,0,0,0];
     [0,0,0,0,0,1,0,0];
     [0,0,0,0,0,0,1,0];
     [0,0,0,0,0,0,0,1]];
 
txt = 'Starting point';
text(0,0,txt);
hold on;
for ii = 1:100
    
   [command(1), command(2)] = generate_command(ii);

    %%%%%%%%%%%% prediction step %%%%%%%%%%%%%%%%%
    last_position = [state_est(1), state_est(2)];      % last Point

    state_transition = [[1,0,0,dt,0,0,(dt^2)/2,0];
                        [0,1,0,0,dt,0,0,(dt^2)/2];
                        [0,0,1,0,0,dt,0,0];
                        [0,0,0,1,0,0,dt,0];
                        [0,0,0,0,1,0,0,dt];
                        [0,0,0,0,0,1,0,0];
                        [0,0,0,0,0,0,1,0];
                        [0,0,0,0,0,0,0,1];];
    %propagate the new state forward
    state_est = state_transition * state_est;
    if(state(3) > pi)
       state(3) = -pi + (state(3) - pi); 
    elseif(state(3) < -pi)
       state(3) = pi + (state(3) + pi);
    end
    
    %add control command
    omega = (command(1) * tan(command(2))) / dwb;%angular velocity
    
    %propagate state uncertainty
    state_transition_jacobian = [[1,0,0,dt,0,0,(dt^2)/2,0];
                                 [0,1,0,0,dt,0,0,(dt^2)/2];
                                 [0,0,1,0,0,dt,0,0];
                                 [0,0,0,1,0,0,dt,0];
                                 [0,0,0,0,1,0,0,dt];
                                 [0,0,0,0,0,1,0,0];
                                 [0,0,0,0,0,0,1,0];
                                 [0,0,0,0,0,0,0,1];];
    
    state_covariance = state_transition_jacobian * state_covariance * transpose(state_transition_jacobian) + motion_noise_covariance;
    %visualize results
    quiver(last_position(1),last_position(2),state_est(1) - last_position(1),state_est(2) - last_position(2),0,'MaxHeadSize',0.8,'color','red');
    hold on;
    grid

    %%%%%%%%%%%% model update %%%%%%%%%%%%%%%%%
    %cite http://web.cecs.pdx.edu/~mperkows/temp/KALMAN/KALMAN-PAPERS/EKF-mobile-iran-GOOD.pdf
    true_last_position = [state(1), state(2)];      % last Point
    command(1) = add_gauss_noise(command(1),vehicle_noise_vector(1,1));
    command(2) = add_gauss_noise(command(2),vehicle_noise_vector(2,2));
    
    state(8) = state(5);
    state(7) = state(4);
    state(6) = state(3);
    state(5) = state(2);
    state(4) = state(1);
    state(1) = state(1) + dt*command(1)*cos(state(3));
    state(2) = state(2) + dt*command(1)*sin(state(3));
    state(3) = state(3) + (dt*command(1)*tan(command(2)))/dwb;
    
    if(state(3) > pi)
       state(3) = -pi + (state(3) - pi); 
    elseif(state(3) < -pi)
       state(3) = pi + (state(3) + pi);
    end
    
    state(4) = (state(1) - state(4)) / dt;
    state(5) = (state(2) - state(5)) / dt;
    state(6) = (state(3) - state(6)) / dt;
    state(7) = (state(4) - state(7)) / dt;
    state(8) = (state(5) - state(8)) / dt;
    
    %visualize results
    quiver(true_last_position(1),true_last_position(2),state(1) - true_last_position(1),state(2) - true_last_position(2),0,'MaxHeadSize',0.8,'color','blue');
    hold on;
    grid
    
    %%%%%%%%%%%% prediction step %%%%%%%%%%%%%%%%%
    %generate measurements based on actual state  
    
    imu_list = [3,6,7,8];
    for jj = 1:size(imu_measurements(:,1))
       imu_measurements(jj) = add_gauss_noise(state(imu_list(jj)),imu_covariance(jj,jj));
    end
    
    if(imu_measurements(1) > pi)
       imu_measurements(1) = -pi + (imu_measurements(1) - pi); 
    elseif(imu_measurements(1) < -pi)
        imu_measurements(1) = pi + (imu_measurements(1) + pi);
    end
    measurement_error = imu_measurements - (imu_jacobian * state_est);
    s_matrix = imu_jacobian * state_covariance * transpose(imu_jacobian) + imu_covariance;
    K_imu = state_covariance * transpose(imu_jacobian) * inv(s_matrix); %#ok<*MINV>
    state_est = state_est + (K_imu * measurement_error);
    state_covariance = (I - K_imu * imu_jacobian) * state_covariance;
    
    
    %visualize results
    quiver(last_position(1),last_position(2),state_est(1) - last_position(1),state_est(2) - last_position(2),0,'MaxHeadSize',0.8,'color','green');
    hold on;
    grid
end
txt = 'Final estimate';
text(state_est(1),state_est(2),txt);
hold on;
txt = 'Final position';
text(state(1),state(2),txt);
hold on;
