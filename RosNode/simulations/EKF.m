figure(1);
%%%% Extended Kalman Filter implentation for an ackerrman vehicle

dt = 0.025; %40 hertz update frequency (timestep)
state_est = zeros(8,1); %estimate of x_position, y_position, heading,
                %x_linear velocity, y_linear velocity, angular velocity,
                %x_angular acceleration,y_angular acceleration
state = zeros(8,1); %actual x_position, y_position, heading
                    %x_linear_velocity, y_linear velocity, angular velocity,
                    %x_angular acceleration,y_angular acceleration
state(3) = pi;
state_est(3) = state(3);
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

%%%%%%%%%%%% sensors %%%%%%%%%%%%%%%%%
%%%imu%%%
imu_covariance = [[.0025,0,0,0];%imu measurement covariance matrix
                  [0,.02,0,0];
                  [0,0,.04,0];
                  [0,0,0,.04]];
imu_jacobian = [[0,0,1,0,0,0,0,0];
               [0,0,0,0,0,1,0,0];
               [0,0,0,0,0,0,1,0];
               [0,0,0,0,0,0,0,1]];
imu_measurements = zeros(4,1);%yaw, rotational velocity,
                              %x_linear_acceleration, y_linear_acceleration
K_imu = zeros(8,1);
imu_list = [3,6,7,8];

%%%%encoder%%%
encoder_covariance = [[.01,0];%imu measurement covariance matrix
                      [0,.01]];
encoder_jacobian = [[0,0,0,1,0,0,0,0];
                    [0,0,0,0,1,0,0,0]];
encoder_measurements = zeros(2,1);%x_linear_velocity, y_linear_velocity
K_encoder = zeros(8,1);
encoder_list = [4,5];

%%%%accelerometer%%%
accelerometer_covariance = [[.02,0];%imu measurement covariance matrix
                            [0,.02]];
accelerometer_jacobian = [[0,0,0,0,0,0,1,0];
                          [0,0,0,0,0,0,0,1]];
accelerometer_measurements = zeros(2,1);%x_linear_acceleration, y_linear_acceleration
K_accelerometer = zeros(8,1);
accelerometer_list = [7,8];

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
<<<<<<< HEAD
for ii = 1:50
    [command(1), command(2)] = generate_command(ii);
=======

for ii = 1:100
   [command(1), command(2)] = generate_command(ii);
>>>>>>> d68981c59f05486c55adcd99f083837bae7d67ca

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
    state(4) = (state(1) - state(4)) / dt;
    state(5) = (state(2) - state(5)) / dt;
    state(6) = (state(3) - state(6)) / dt;
    state(7) = (state(4) - state(7)) / dt;
    state(8) = (state(5) - state(8)) / dt;
    
    %visualize results
    quiver(true_last_position(1),true_last_position(2),state(1) - true_last_position(1),state(2) - true_last_position(2),0,'MaxHeadSize',0.8,'color','blue');
    hold on;
    grid
    
    %%%%%%%%%%%% update step %%%%%%%%%%%%%%%%%
    %generate measurements based on actual state and transform the measurements 
    %transform the IMUs readings from the car's reference frame to the
    %global frame
    imu_measurements(1) = add_gauss_noise(state(3),imu_covariance(1,1));
    imu_measurements(2) = add_gauss_noise(state(6),imu_covariance(2,2));
    acc_x = add_gauss_noise(state(7) * cos(state(3)-pi/2) + state(8) * cos(state(3)),imu_covariance(3,3));
    acc_y = add_gauss_noise(state(7) * sin(state(3)-pi/2) - state(8) * sin(state(3)),imu_covariance(4,4));
    imu_measurements(3) = -acc_x * cos(state_est(3)-pi/2) - acc_y*cos(state_est(3));
    imu_measurements(4) = -acc_x * sin(state_est(3)-pi/2) + acc_y*sin(state_est(3));
    
    %decompose the encoder's speed measurement to velocities using the car's heading 
    speed = add_gauss_noise(sqrt((state(1) - true_last_position(1))^2+(state(2) - true_last_position(2))^2)/dt,sqrt(encoder_covariance(1,1)^2 + encoder_covariance(2,2)^2));
    %incorporate the throttle direction
    if (command(1) < 0)
        speed = speed * -1;
    end
    encoder_measurements(1) = speed * cos(state_est(3));
    encoder_measurements(2) = speed * sin(state_est(3));
    
    %transform the accelerometer readings from the car's reference frame to the
    %global frame
    acc_x = add_gauss_noise(state(7)* cos(state(3)-pi/2) + state(8)*cos(state(3)),accelerometer_covariance(1,1));
    acc_y = add_gauss_noise(state(7)* sin(state(3)-pi/2) - state(8)*sin(state(3)),accelerometer_covariance(2,2));
    accelerometer_measurements(1) = -acc_x * cos(state_est(3)-pi/2) - acc_y*cos(state_est(3));
    accelerometer_measurements(2) = -acc_x * sin(state_est(3)-pi/2) + acc_y*sin(state_est(3));
    
    %imu update
    measurement_error = imu_measurements - (imu_jacobian * state_est);
    s_matrix = imu_jacobian * state_covariance * transpose(imu_jacobian) + imu_covariance;
    K_imu = state_covariance * transpose(imu_jacobian) * inv(s_matrix); %#ok<*MINV>
    state_est = state_est + (K_imu * measurement_error);
    state_covariance = (I - K_imu * imu_jacobian) * state_covariance;

    %encoder update
    measurement_error = encoder_measurements - (encoder_jacobian * state_est);
    s_matrix = encoder_jacobian * state_covariance * transpose(encoder_jacobian) + encoder_covariance;
    K_encoder = state_covariance * transpose(encoder_jacobian) * inv(s_matrix); %#ok<*MINV>
    state_est = state_est + (K_encoder * measurement_error);
    state_covariance = (I - K_encoder * encoder_jacobian) * state_covariance;
    
%     %accelerometer update
%     measurement_error = accelerometer_measurements - (accelerometer_jacobian * state_est);
%     s_matrix = accelerometer_jacobian * state_covariance * transpose(accelerometer_jacobian) + accelerometer_covariance;
%     K_accelerometer = state_covariance * transpose(accelerometer_jacobian) * inv(s_matrix); %#ok<*MINV>
%     state_est = state_est + (K_accelerometer * measurement_error);
%     state_covariance = (I - K_accelerometer * accelerometer_jacobian) * state_covariance;
    
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
