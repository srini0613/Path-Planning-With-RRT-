% 
% state = [x, y, yaw, delta]
% input = [v_des, delta_des]
% ref = [x_ref, y_ref, yaw_ref, v_ref]
% 
% 
% 

clear variables;
close all;


set(0, 'defaultAxesFontSize', 12);
set(0, 'defaultTextFontSize', 20);
set(0, 'DefaultAxesLineWidth', 1.0, 'DefaultLineLineWidth', 1.0);


% addpath ../path_design

control_mode_option = ["pure_pursuit", "pid", "mpc", "mpc_no_constraints"];
control_mode = control_mode_option(1);
    
save_video = 1; %1:save, 0:no

%% preliminaries
rad2deg = 180 / pi;
deg2rad = pi / 180;
kmh2ms = 1000 / 3600;

simulation_time = 26;
simulation_rk4_time_step = 0.02; % simulation time step

vel_ref = 7 * kmh2ms;
% vel_ref = 5 * kmh2ms;
%perpendicular 5 vel
%% if there is lot of laterall error ,as look up distance depends on the speed of the vehicle hence the speed has to been defined properly


% for dynamics model
param.tau = 0.27; % steering dynamics: 1d-approximated time constant
param.wheelbase = 4;
param.wheelbaseT=1.5;
param.steer_lim = 30 * deg2rad;
param.vel_max = 10;
param.vel_min = -5;

param.input_delay = 0; % [s]
param.control_dt = 0.03; % [s]
param.measurement_noise_stddev = [0.1, 0.1, 1.0*deg2rad, 0.5*deg2rad,1.0*deg2rad]; % measurement noise
% param.measurement_noise_stddev = [0,0,0,0]; % measurement noise
param.steering_steady_state_error_deg = 1;

% for pure pursuit only
param.pure_pursuit_lookahead = 5; % [m]
% param.pure_pursuit_lookahead = 03;
%%perpendicular = 2

% for mpc only
param.mpc_dt = 0.1;
param.mpc_n = 30;
param.mpc_constraint_steering_deg = 30;
param.mpc_constraint_steer_rate_deg = 280;
param.mpc_model_dim = 3;
param.mpc_Q = diag([1,2]);
param.mpc_R = 0.5;
param.mpc_delay_comp_step = round(param.input_delay / param.control_dt);
% param.mpc_delay_comp_step = 0.0;

% use the input ahead of the delay time
param.mpc_sensor_delay = param.input_delay; 

% for mpc2
param.mpc2_dt = 0.2;
param.mpc2_n = 10;
param.mpc2_steering_lim_deg = 40;
param.mpc2_model_dim = 4;
param.mpc2_Q = diag([1,1,0]);
param.mpc2_R = 0.05;

%% simulation parameters

% initial position (x, y, yaw, delta)
load path; % x, y, yaw
% x0 = [path(1,1),path(1,2),270];

ts = 0;
dt = simulation_rk4_time_step;
tf = simulation_time;
t = ts:dt:tf;

%% reference trajectory design

% path_design; % using spline


ref = zeros(length(path), 6);
IDX_X = 1;
IDX_Y = 2;
IDX_XY = 1:2;
IDX_XYYAW = 1:3;
IDX_YAW = 3;
IDX_YAWt = 3;
IDX_VEL = 4;
IDX_CURVATURE = 5;
IDX_TIME = 6;

IDX_STEER = 4;


path_size_scale = 1;
path(:,IDX_XY) = path(:,IDX_XY) * path_size_scale;
ref(:,IDX_XYYAW) = path(:,IDX_XYYAW);

% x0 = [path(1,1), path(1,2), path(3,2), 0,pi-path(3,2)];
x0 = [path(1,1), path(1,2), 0, 0,pi-path(3,2)];

%%reverse 135 angle

ref(:,IDX_VEL) = ones(length(path),1)*vel_ref;

% insert curvature into path
for i = 2:length(ref)-1
    p1_ = ref(i-1,IDX_XY);
    p2_ = ref(i, IDX_XY);
    p3_ = ref(i+1, IDX_XY);
    A_ = ((p2_(1)-p1_(1))*(p3_(2)-p1_(2)) - (p2_(2)-p1_(2))*(p3_(1)-p1_(1))) / 2;
    ref(i, IDX_CURVATURE) = 4 * A_ / (norm(p1_-p2_) * norm(p2_-p3_) * norm(p3_-p1_));
end

% insert relative time into path
for i = 2:length(ref)
    v_ = ref(i,IDX_VEL);
    d_ = norm(ref(i,IDX_XY)-ref(i-1,IDX_XY));
    dt_ = d_ / v_;
    ref(i, IDX_TIME) = ref(i-1, IDX_TIME) + dt_;
end


%% simulation

if control_mode == "pure_pursuit"
    [X, U, debug] = simulate_rk4(@kinematics_model, @pure_pursuit, x0, ref, ts, dt, tf, param);
%     
    lat_error_vec = debug(:,end);
elseif control_mode == "pid"
   [X, U, debug] = simulate_rk4(@kinematics_model, @pid_controller, x0, ref, ts, dt, tf, param);
   lat_error_vec = debug(:,end);
elseif control_mode == "mpc"
    param.mpc_solve_without_constraint = false;
    [X, U, debug] = simulate_rk4(@kinematics_model, @model_predictive_controller, x0, ref, ts, dt, tf, param);
    lat_error_vec = debug(:,end);
elseif control_mode == "mpc_no_constraints"
    param.mpc_solve_without_constraint = true;
    [X, U, debug] = simulate_rk4(@kinematics_model, @model_predictive_controller, x0, ref, ts, dt, tf, param);
    lat_error_vec = debug(:,end);  
elseif control_mode == "mpc2"
    [X, U, debug] = simulate_rk4(@kinematics_model, @model_predictive_controller2, x0, ref, ts, dt, tf, param);
    lat_error_vec = debug(:,end);
end
fprintf("lattitude error: mean square = %f, max = %f", norm(lat_error_vec)/simulation_time, max(lat_error_vec));


%% movie plot

sp_num = 18;
subpl1 = 'subplot(sp_num,sp_num, sp_num+1:sp_num*12);';
subpl2 = 'subplot(sp_num,sp_num, sp_num*13+1:sp_num*15);';
subpl3 = 'subplot(sp_num,sp_num, sp_num*16+1:sp_num*18);';

% tire2steer = 12.5;

fig_trajectory_result = figure(1);
axis([0 50 0 50])

set(fig_trajectory_result, 'Position', [716 735 1026 1146]);
eval(subpl1);

plot(ref(:,1), ref(:,2),'k-.'); hold on; grid on;
 
    p1=[40 25];
    p2=[40 31];
    p3=[47.5 31];
    p4=[47.5 25];
    p5=[47.5 28];
    p6=[40 28];
    plot(p1(1),p1(2),'*')
    plot(p2(1),p2(2),'*')
    plot(p3(1),p3(2),'*')
    plot(p4(1),p4(2),'*')
    plot([p1(1) p4(1)],[p1(2) p4(2)],'Color', 'b', 'LineWidth', 1)
    plot([p2(1) p3(1)],[p2(2) p3(2)],'Color', 'b', 'LineWidth', 1)
    plot([p3(1) p4(1)],[p3(2) p4(2)],'Color', 'b', 'LineWidth', 1)
    plot([p6(1) p5(1)],[p6(2) p5(2)],'Color', 'b', 'LineWidth', 1)
% plot(X(:,1), X(:,2));
% legend('ref','tracked');
xlabel('x [m]'); ylabel('y [m]');
% img_orig = imread('handle.jpg');
% img = imrotate(img_orig, 10);
% handle_plt = image([150, 170],[110, 90], img);
% handle_plt_point = [160, 100, 0];


eval(subpl2);
plot(t, lat_error_vec, 'b'); grid on; hold on; 
xlabel('t [s]'); ylabel('latitude error [m]');
ulim = ceil(2*max(lat_error_vec))/2;
dlim = floor(2*min(lat_error_vec))/2;
ylim([dlim, ulim]);

eval(subpl3);
p1 = plot(t, X(:,IDX_STEER)*rad2deg, 'b'); grid on; hold on; 
p2 = plot(t, U(:,2)*rad2deg, 'Color', [0.7 0. 1]); hold on; 
writematrix(U(:,2),'angle.txt');
legend([p1,p2], {'measured','command'})
xlabel('t [s]'); ylabel('steering angle [deg]');
ulim = round(2*max(X(:,IDX_STEER)*rad2deg))/2;
dlim = round(2*min(X(:,IDX_STEER)*rad2deg))/2;
ylim([dlim, ulim]);

z_axis = [0 0 1];
setpoint = []; rear_tire = []; front_tire = []; body = []; tracked = []; rear_tire1=[];front_tire1 = [];trailer_hitch=[];
setpoint_ideal = []; error_point = []; steer_point = []; time_bar_laterror = []; time_bar_steer = []; trailer_tireleft=[];trailer_tire=[];
rear_tiret1=[];rear_tiret=[];body_c = [];
L = param.wheelbase;
Lt=param.wheelbaseT;
rear_length = 1;
front_length = 1;
side_width = 0.9;
fig_draw_i = 1:round(1/dt/20):length(t);

% for movie
clear frame_vec;
frame_vec(length(fig_draw_i)) = struct('cdata', [], 'colormap',[]);
plot(15,15,'*');

j = 1;
fig_trajectory_result; hold on;
for i = fig_draw_i
    eval(subpl1);
    disp(t(i))
    rear_x = X(i,1);
    rear_yt = X(i,2);
    rear_y = X(i,2);
    yaw = X(i,3);
    yaw_t = X(i,5);
    
    delta = X(i,IDX_STEER)
%     writematrix(X(i,IDX_STEER),'angle.txt')
    y_t=rear_y+3*sin(yaw-yaw_t);
    x_t =rear_x+3*cos(yaw-yaw_t);
    
    f_x_t = x_t+L;
    f_y_t = y_t;
    front_x = rear_x + L;
    front_y = rear_y;
    delete([setpoint,trailer_tireleft,trailer_tire, rear_tire,trailer_hitch,rear_tire1, front_tire,front_tire1, body, tracked, setpoint_ideal, error_point, steer_point, time_bar_laterror, time_bar_steer]);

    tracked = plot(X(1:i,1), X(1:i,2),'r');
    
    title_draw =  "[s], steer = " + num2str(delta*rad2deg,'%+3.1f') + "[deg], v = " + ...
        num2str(vel_ref*3600/1000,'%3.1f') + "[km/h], lat error = "+num2str(lat_error_vec(i),'%+2.2f') + "[m]";
%     title_draw = [title_draw; "Simulation: solver = rk4, sensor-delay = "  + num2str(param.input_delay*1000, '%d') + "[ms], control freq=" + ...
%         num2str(1/param.control_dt, '%d') + "[hz]"];
%     title_draw = [title_draw; "noise-sigma = " + num2str(param.measurement_noise_stddev(1),'%2.2f')+"(pos), "+ ...
%         num2str(param.measurement_noise_stddev(3),'%2.2f')+"(yaw), "+num2str(param.measurement_noise_stddev(4),'%2.2f')+"(steer)"];
    if control_mode == "mpc" || control_mode == "mpc_no_constraints"
        title_draw = [title_draw; "MPC: dt = " + num2str(param.mpc_dt, '%3.3f') + "[s], horizon step = " + num2str(param.mpc_n, '%d')];
%         pred_states = debug(i, param.mpc_n+1:param.mpc_n*(4+1));
%         pred_states = reshape(pred_states, param.mpc_n, length(pred_states)/param.mpc_n);
%         setpoint = plot(pred_states(:,1), pred_states(:,2), 'bo'); % include linealize error
        pred_error = debug(i, param.mpc_n*(4+1)+1:param.mpc_n*(2+4+1));
        pred_error = reshape(pred_error, param.mpc_n, length(pred_error)/param.mpc_n);
        setpoint_ideal = plot(pred_error(:,1), pred_error(:,2), 'mx'); % without linealize error
    elseif control_mode == "mpc2"
        title_draw = [title_draw; "MPC2: dt = " + num2str(param.mpc_dt, '%3.3f') + "[s], horizon step = " + num2str(param.mpc_n, '%d')];
        pred_states = debug(i, param.mpc_n+1:param.mpc_n*(4+1));
        pred_states = transpose(reshape(pred_states, length(pred_states)/param.mpc_n, param.mpc_n));
        setpoint = plot(pred_states(:,1), pred_states(:,2), 'bo'); % include linealize error
        pred_error = debug(i, param.mpc_n*(4+1)+1:param.mpc_n*(4+4+1));
        pred_error = transpose(reshape(pred_error, length(pred_error)/param.mpc_n, param.mpc_n));
        setpoint_ideal = plot(pred_error(:,1), pred_error(:,2), 'mx'); % without linealize error
    elseif control_mode == "pure_pursuit"
        title_draw = [title_draw; "Parallel parking Furbot, pure-pursuit: lookahead dist="+num2str(param.pure_pursuit_lookahead, '%1.1f')+"[m]"];
        sp = debug(i,:);
        setpoint = plot(sp(1), sp(2), 'ro');
    elseif control_mode == "pid"
        title_draw = [title_draw; "PID: kp = " + num2str(0.3, '%3.3f') + ", ki = " + num2str(0, '%3.3f') + ", kd = " + num2str(1.5, '%3.3f')];
        sp = debug(i,:);
        setpoint = plot(sp(1), sp(2), 'ro');
    end

    rear_tire = plot([rear_x-0.3, rear_x+0.3],[rear_y-0.5, rear_y-0.5], 'r', 'LineWidth', 2.0);
    rear_tire1 = plot([rear_x-0.3, rear_x+0.3],[rear_y+0.5, rear_y+0.5], 'r', 'LineWidth', 2.0);
    front_tire = plot([front_x-0.3, front_x+0.3],[front_y-0.5, front_y-0.5], 'r', 'LineWidth', 2.0);
    front_tire1 = plot([front_x-0.3, front_x+0.3],[front_y+0.5, front_y+0.5], 'r', 'LineWidth', 2.0);

    body = plot([rear_x-rear_length, front_x+front_length, front_x+front_length, rear_x-rear_length, rear_x-rear_length], ...
        [rear_y-side_width, front_y-side_width, front_y+side_width, rear_y+side_width, rear_y-side_width],'k');
    rear_origin = [rear_x, rear_y,180];
    trailer_origin = [rear_x, rear_y, 180];

    front_origin = [rear_x + L*cos(yaw), rear_y + L*sin(yaw), 180];
    rotate(body, z_axis, yaw * rad2deg, rear_origin);
    rotate(rear_tire, z_axis, yaw * rad2deg, rear_origin);
    rotate(trailer_hitch, z_axis, (yaw_t) * rad2deg, trailer_origin);
    rotate(rear_tire1, z_axis, yaw * rad2deg, rear_origin);
    rotate(front_tire, z_axis, yaw * rad2deg, rear_origin);
    rotate(front_tire, z_axis, delta * rad2deg, front_origin);
    rotate(front_tire1, z_axis, yaw * rad2deg, rear_origin);
    rotate(front_tire1, z_axis, delta * rad2deg, front_origin);
% %     rotate(handle_plt, [0,0,1], delta*tire2steer*rad2deg, handle_plt_point)
    title(title_draw);
    xlim([0 50]);
    ylim([0 50]); 
    % lat error
    eval(subpl2);
    error_point = plot(t(i), lat_error_vec(i), 'ko');
    time_bar_laterror = plot([t(i), t(i)], [100, -100], 'k');
    
    % steering
    eval(subpl3);
    steer_point = plot(t(i), X(i, IDX_STEER)*rad2deg, 'ko');
    time_bar_steer = plot([t(i), t(i)], [100, -100], 'k');
    legend([p1,p2], {'measured','command'})
    ylim([-40 40]);
  
    
    drawnow;
    frame_vec(j) = getframe(fig_trajectory_result);
    
    j = j + 1;
end



% for video save
if (save_video == 1)
    cd ./
    frame_vec(1) = [];
    vidObj = VideoWriter('Parallel parking PID.avi');
    vidObj.FrameRate = 22;
    open(vidObj);
    writeVideo(vidObj, frame_vec);
    close(vidObj);
    cd ../
end



% plot_pid;