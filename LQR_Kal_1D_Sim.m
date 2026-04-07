function [t, altitude, velocity, acceleration, thrust, mass, fmass, altitude_est, velocity_est, altitude_meas] = LQR_Kal_1D_Sim(dmass, fm, gravity, int_alt, int_velo, ...
    max_time, maxthrust, max_alt_error, max_velo_error, dt, Cd, Pden, sigma_alt, sigma_acc, model_uncertainty)

    t = 0:dt:max_time;
    numSteps = length(t);

    % True state arrays
    altitude = zeros(numSteps, 1);
    velocity = zeros(numSteps, 1);
    acceleration = zeros(numSteps, 1);
    thrust = zeros(numSteps, 1);
    mass = zeros(numSteps, 1);
    fmass = zeros(numSteps, 1);

    % Kalman filter arrays
    altitude_est = zeros(numSteps, 1);
    velocity_est = zeros(numSteps, 1);
    altitude_meas = zeros(numSteps, 1);

    % Constants
    max_Massflow = 0.0625;
    fm0 = fm;
    Area = (pi * 0.11^2) / 4;

    % LQR matrices
    Q_lqr = [1/(max_alt_error^2) 0; 0 1/(max_velo_error^2)];
    R_lqr = 1/maxthrust^2;
    x_target = [0; 0];

    % True state initialization
    h_true = int_alt;
    v_true = int_velo;

    % Kalman filter initialization
    x_est = [int_alt; int_velo];  % Initial state estimate
    P = [10 0; 0 1];              % Initial estimate uncertainty

    % Kalman filter matrices
    F = [1 dt; 0 1];                          % State transition
    B_kf = [0.5*dt^2; dt];                    % Control input (accelerometer)
    H = [1 0];                                % Measurement matrix (altimeter)
    R_kf = sigma_alt^2;                       % Measurement noise covariance
    Q_kf = model_uncertainty * sigma_acc^2 ...
         * [dt^4/4, dt^3/2; dt^3/2, dt^2];   % Process noise covariance

    % Initial accelerometer reading
    a_meas = 0;

    for i = 1:numSteps
        % RECORD TRUE STATE
        altitude(i) = h_true;
        velocity(i) = v_true;
        m = dmass + fm;

        % KALMAN PREDICT
        x_pred = F * x_est + B_kf * a_meas;
        P_pred = F * P * F' + Q_kf;

        % SIMULATE ALTIMETER
        h_meas = h_true + sigma_alt * randn();
        altitude_meas(i) = h_meas;

        % KALMAN UPDATE
        y_innov = h_meas - H * x_pred;
        S = H * P_pred * H' + R_kf;
        K_kf = P_pred * H' / S;
        x_est = x_pred + K_kf * y_innov;
        P = (eye(2) - K_kf * H) * P_pred;

        h_est = x_est(1);
        v_est = x_est(2);
        altitude_est(i) = h_est;
        velocity_est(i) = v_est;

        % LQR CONTROLLER
        A_lqr = [0 1; 0 -Cd * Pden * Area * abs(v_est) / m];
        B_lqr = [0; 1/m];
        [K, ~, ~] = lqr(A_lqr, B_lqr, Q_lqr, R_lqr);

        error_alt  = h_est - x_target(1);
        error_velo = v_est - x_target(2);
        u = -(K(1) * error_alt + K(2) * error_velo);
        u_ff = m * gravity;
        controlSignal = u + u_ff;
        throttle = max(0, min(1, controlSignal / maxthrust));

        if fm <= 0
            thrst = 0;
        else
            thrst = max(0, min(maxthrust, controlSignal));
            fm = max(0, fm - throttle * max_Massflow * dt);
        end

        % Uses true state
        DForce = -sign(v_true) * 0.5 * Cd * Pden * v_true^2 * Area;
        a_true = ((thrst + DForce) / m) - gravity;

        v_true = v_true + a_true * dt;
        h_true = h_true + v_true * dt;

        % SIMULATE ACCELEROMETER
        a_meas = a_true + sigma_acc * randn();

        % Record remaining outputs
        thrust(i) = thrst;
        acceleration(i) = a_true;
        mass(i) = m;
        fmass(i) = (fm / fm0) * 100;

        % LANDING CHECK
        if h_true <= 0.5
            h_true = 0;
            fprintf('Landed at t = %.2f s | Final velocity = %.3f m/s\n', t(i), v_true);
            altitude(i:end)      = h_true;
            velocity(i:end)      = v_true;
            acceleration(i:end)  = a_true;
            thrust(i:end)        = 0;
            mass(i:end)          = m;
            fmass(i:end)         = (fm / fm0) * 100;
            altitude_est(i:end)  = h_est;
            velocity_est(i:end)  = v_est;
            altitude_meas(i:end) = h_true;
            return;
        end
    end
end
