close all; clear; clc;

kc = 10; % spring constant
bc = 0.6; % damping coefficient
f_des  = 1.0; % desired force
DELTA2 = 0.3; % hysteresis upper bound
DELTA1 = 0.1; % hysteresis lower bound
gamma1 = f_des - DELTA2;
gamma2 = f_des + DELTA2;

Kp_p = 10;
Kd_p = 0.1;

dt = 1e-3;
T = 250;  
N = round(T/dt);  
t = (0:N-1)'*dt;

x1_0 = 0.4;  
x2_0 = 0.0;
x1_meas_log = zeros(N,1);
x2_meas_log = zeros(N,1);

NOISE_STD_X1 = 0.01;
[x1_A, x2_A, x1m_A, x2m_A, ctrl_A, grip_mode_log_A, hyst_log_A, force_error_A, pos_error_A] = simulate_case('position', x1_0, x2_0, kc, bc, f_des, gamma1, gamma2, DELTA1, DELTA2, Kp_p, Kd_p, dt, N, NOISE_STD_X1);
%%
cut_idx = 250000;
x1_cut = x1_A(1:cut_idx);
x2_cut = x2_A(1:cut_idx);
ctrl_cut = ctrl_A(1:cut_idx);

figure('Color','w'); 
hold on; 
grid on; 
box on; 
set(gca,'FontName','Times New Roman');
plot_mode_segments(x2_cut, x1_cut, ctrl_cut, [1 0 0], [0 0 1], t, ctrl_A); 
yl = ylim; 
plot([0 0], yl, '--', 'Color',[0.6 0.6 0.6], 'LineWidth',1.2, 'DisplayName','S'); 
ylim(yl);
plot(x2_cut(1), x1_cut(1), 'k.', 'MarkerSize',16); text(x2_A(1), x1_A(1), '  z^0', 'FontSize', 18);
xlabel('$z_2$', 'Interpreter', 'latex'); 
ylabel('$e_p$', 'Interpreter', 'latex');
set(gca, 'FontSize', 18, 'FontName', 'Times New Roman');
annotation("ellipse", [0.4969 0.8146 0.01391 0.01345])
annotation("ellipse", [0.4969 0.145 0.01391 0.01495])
annotation("doublearrow", [0.4472 0.5583], [0.8807 0.8802])
annotation("textbox", [0.5105 0.1257 0.09656 0.03593], "String", "B", "FontSize", 18, "EdgeColor", "none", 'Interpreter', 'latex')
annotation("textbox", [0.5051 0.8323 0.09656 0.03593], "String", "A", "FontSize", 18, "EdgeColor", "none", 'Interpreter', 'latex')
annotation("textbox", [0.3316 0.8952 0.09656 0.03593], "String", ["force","control"], "FontSize", 18, "EdgeColor", "none")
xlim([-3.17 3.42])
ylim([-3.48 4.04])
%%
NOISE_STD_X1 = 0.00;
[x1_B, x2_B, x1m_B, x2m_B, ctrl_B, grip_mode_log_B, hyst_log_B, force_error_B, pos_error_B] = simulate_case('gamma', x1_0, x2_0, kc, bc, f_des, gamma1, gamma2, DELTA1, DELTA2, Kp_p, Kd_p, dt, N, NOISE_STD_X1);

figure('Color','w'); 
hold on; 
grid on; 
box on; 
set(gca,'FontName','Times New Roman');
plot_mode_segments(pos_error_B, x2m_B, ctrl_B, [1 0 0], [0 0 1], t, ctrl_B); % Pass t and ctrl_B or grip_mode_log if available

plot(x1_B(1), x2_B(1), 'k.', 'MarkerSize',16); 
text(x1_B(1) + .03, x2_B(1), 'z^0', "FontSize", 18);

xline(0.05, '--', 'Color',[0.6 0.6 0.6], 'LineWidth',1.4, 'HandleVisibility','off');
xline(0.2, '--', 'Color',[0.6 0.6 0.6], 'LineWidth',1.4, 'HandleVisibility','off');
xline(-0.05, '--', 'Color',[0.6 0.6 0.6], 'LineWidth',1.4, 'HandleVisibility','off');
xline(-0.2, '--', 'Color',[0.6 0.6 0.6], 'LineWidth',1.4, 'HandleVisibility','off');

xlabel('$e_p$', 'Interpreter', 'latex'); 
ylabel('$z_2$', 'Interpreter', 'latex');
set(gca, 'FontSize', 18, 'FontName', 'Times New Roman');
annotation("textbox", [0.45 0.97 0.0298 0.03082], "String", "$$-\varphi_1$$", "FontSize", 18, "EdgeColor", "none", 'Interpreter', 'latex')
annotation("textbox", [0.33 0.97 0.0298 0.03082], "String", "$$-\varphi_2$$", "FontSize", 18, "EdgeColor", "none", 'Interpreter', 'latex')
annotation("textbox", [0.64 0.97 0.0298 0.03082], "String", "$$\varphi_2$$", "FontSize", 18, "EdgeColor", "none", 'Interpreter', 'latex')
annotation("textbox", [0.52 0.97 0.0298 0.03082], "String", "$$\varphi_1$$", "FontSize", 18, "EdgeColor", "none", 'Interpreter', 'latex')
xlim([-0.5 0.5])
ylim([-4 0.3])

function [x1, x2, x1_meas_log, x2_meas_log, ctrl_log, grip_mode_log, hyst_log, force_error_log, pos_error] = simulate_case(kind, x1_0, x2_0, kc, bc, f_des, gamma1, gamma2, DELTA1, DELTA2, Kp_p, Kd_p, dt, N, NOISE_STD_X1)
    x1 = zeros(N,1); 
    x2 = zeros(N,1); 
    ctrl_log = zeros(N,1); 
    force_error_log = zeros(N,1); 
    pos_error = zeros(N, 1);
    grip_mode_log = zeros(N,1); 
    hyst_log = zeros(N,1);
    x1(1) = x1_0; 
    x2(1) = x2_0;
    ctrl = 0;
    LOOSEN = -1; 
    HOLD = 0; 
    TIGHTEN = 1;
    q = HOLD;
    KV_VEL = 5;
    V_STEP = 0.001;
    hyst = 1;
    for k = 1:N-1
        x1_meas = x1(k) + NOISE_STD_X1*randn();
        x2_meas = x2(k);
        
        x1_meas_log(k) = x1_meas;
        x2_meas_log(k) = x2_meas;

        fc = (kc*x1_meas + bc*x2_meas);
        if ctrl == 1
            
            force_error = f_des - fc;
        else
            force_error = 0;  
        end

        force_error_log(k) = force_error;

        if kind == "position"
            hyst = 0;
            if ctrl == 0 && (x1_meas <= 0)
                ctrl = 1;
            elseif ctrl == 1 && (x1_meas >= 0)
                ctrl = 0;
            end
        else
            phi1 = 0.05;
            phi2 = 0.2;
            pos_error(k) = x1_meas;
            ep = abs(x1_meas);
            fprintf("%d, ep\n", ep)
            
            if ctrl == 0
                if ep <= phi1
                    ctrl = 1;
                end
            elseif ctrl == 1
                if ep >= phi2
                    ctrl = 0;
                end
            end
            
        end
        if ctrl == 1 && hyst ~= 0
            f_target = f_des;
            force_error = f_target - fc;
            if (q == TIGHTEN) && (force_error <= DELTA1)
                q = HOLD;
            elseif (q == HOLD) && (force_error >= DELTA2)
                q = TIGHTEN;
            elseif (q == HOLD) && (force_error <= -DELTA2)
                q = LOOSEN;
            elseif (q == LOOSEN) && (force_error >= -DELTA1)
                q = HOLD;
            end

            if q == TIGHTEN
                a_cmd = (V_STEP - x2_meas) * KV_VEL;
            elseif q == LOOSEN
                a_cmd = (-V_STEP - x2_meas) * KV_VEL;
            else
                a_cmd = -x2_meas * KV_VEL;
            end
            if abs(force_error) <= 1e-3
                fprintf('Target force reached: %f\n', fc);
            end
            
        elseif ctrl == 1 && hyst == 0
            f_target = f_des;
            if (q == TIGHTEN) && (fc == f_target)
                q = HOLD;
            elseif (q == HOLD) && (fc < f_target)
                q = TIGHTEN;
            elseif (q == HOLD) && (fc > f_target)
                q = LOOSEN;
            elseif (q == LOOSEN) && (fc == f_target)
                q = HOLD;
            end
        elseif ctrl == 0
            a_cmd = Kp_p * (0 - x1_meas) - Kd_p * x2_meas;
        end
        x2(k+1) = x2(k) + a_cmd*dt;
        x1(k+1) = x1(k) + 0.1*x2(k+1)*dt; 
        ctrl_log(k) = ctrl;
        grip_mode_log(k) = q;
        hyst_log(k) = hyst;
    end
    ctrl_log(N) = ctrl;
    grip_mode_log(N) = q;
    hyst_log(N) = hyst;
    x1_meas_log(N) = x1_meas_log(N-1);
    x2_meas_log(N) = x2_meas_log(N-1);
    force_error_log(N) = force_error_log(N-1);
end

function plot_mode_segments(x1, x2, ctrl_log, pos_color, force_color, t, grip_mode_log)
    set(gca,'xticklabel',[]);
    set(gca, 'yticklabel', []);
    set(gcf, "Theme", "light");
    pos_mask = (ctrl_log==0); 
    force_mask = (ctrl_log==1);
    edges_pos = diff([false; pos_mask; false]);
    starts_pos = find(edges_pos==1); 
    stops_pos = find(edges_pos==-1)-1;
    for i=1:numel(starts_pos)
        idx = starts_pos(i):stops_pos(i);
        plot(x1(idx), x2(idx), 'Color', pos_color, 'LineWidth', 1.5, 'DisplayName', ternary(i==1,'position control',''));
    end
    edges_force = diff([false; force_mask; false]);
    starts_force = find(edges_force==1); 
    stops_force = find(edges_force==-1)-1;
    for i=1:numel(starts_force)
        idx = starts_force(i):stops_force(i);
        plot(x1(idx), x2(idx), 'Color', force_color, 'LineWidth', 1.8, 'DisplayName', ternary(i==1,'force control',''));
    end
end

function s = ternary(cond, a, b)
    if cond, s = a; else, s = b; end
end