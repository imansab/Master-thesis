%% ================== Decentralized ρ-partial + Hard Thermostat + Tap + Temperature ==================
% - Filtered frequency, FCR decisions every 200 ms (ρ-partial, lockouts).
% - Hard thermostat: immediate OFF if T>=Tmax, immediate ON if T<=Tmin (EVERY step).
% - Per-boiler thermal dynamics (wall losses + tap draw).
% - Tap: random Poisson openings per unit, random durations, fixed flow.
% - Two DISTINCT locks: FREQUENCY lock (short) and THERMOSTAT lock (long).
% - Detailed logs for one boiler i0.
% - Adds a constraint-aware OFF target (temperature/tap/locks).

clear; clc; close all; rng(0);  % reproducibility

%% ---------- Frequency data ----------
data     = readmatrix('data_instable.csv');   % col1 = f(Hz), col2 = t(s)
time_raw = data(:,2);
f_raw    = data(:,1);

% High-resolution interpolation (20 ms)
dt_high = 0.02;                         
TmaxSim = time_raw(end);
time    = (0:dt_high:TmaxSim)';         
steps   = numel(time);
f       = interp1(time_raw, f_raw, time, 'linear', 'extrap');

% Zero-lag moving-average filter
Tfilt = 0.20;                           
W     = max(1, round(Tfilt/dt_high));
b     = (1/W) * ones(1, W);
f_f   = filtfilt(b, 1, f);

%% ---------- Local FCR law ----------
f0        = 50;
deadband  = 0.02;                       
delta_max = 0.20;                       
fractionOFF = @(df) max(0, min((df - deadband)/delta_max, 1));  % df = f0 - f

%% ---------- Fleet & decentralized cadence ----------
N     = 1000;                           
Tdec  = 0.2;                            
Mdec  = round(Tdec/dt_high);            

% Local ρ_i (decentralized)
rho_min  = 0.08;
rho_base = 0.12;
rho_max  = 0.50;
rho_gain = 1.40;

% Locks (in 200 ms ticks) -- FREQUENCY (short)
Tlock_on_freq  = 1;                     
Tlock_off_freq = 1;
L_on_freq      = round(Tlock_on_freq  / Tdec);
L_off_freq     = round(Tlock_off_freq / Tdec);

% Locks (in 200 ms ticks) -- THERMOSTAT (long)
Tlock_on_th  = 600;                     % 120 s
Tlock_off_th = 600;
L_on_th      = round(Tlock_on_th  / Tdec);
L_off_th     = round(Tlock_off_th / Tdec);

% Separate locks per unit
lock_freq = zeros(N,1);                 
lock_th   = zeros(N,1);                 

%% ---------- Thermal parameters ----------
P_unit = 2000;           % W per boiler
V     = 0.010;           % m^3 (10 L)
rho_w = 1000;            % kg/m^3
c_w   = 4186;            % J/(kg.K)
Cth   = rho_w*V*c_w;     % J/K
Rth   = 2.5;             % K/W
T_env = 20;              % °C
T_in  = 15;              % °C

%% ---------- Thermostat ----------
Tset   = 65;             
dHyst  = 5;              
Tmin   = Tset - dHyst;
Tmax   = Tset + dHyst;

%% ---------- Individual states (0=OFF, 1=ON) ----------
etat = zeros(N,1);       

%% ---------- Thermal states ----------
T = 60 + 1*randn(N,1);   % around 60°C

%% ---------- Tap stochastic model ----------
TapTick      = 1.0;                          
Mtap         = round(TapTick/dt_high);       
lambda_min   = 0.10;                         % openings per minute (example)
lambda_sec   = lambda_min/60;                
dur_mean     = 60;                           
Q_tap        = 0.05;                         % kg/s when open (~3 L/min)

tap_open_steps_left = zeros(N,1);            
m_dot               = zeros(N,1);            

% Log i0 tap
m_dot_i0 = NaN(steps,1);

%% ---------- Logging boiler i0 ----------
i0 = 500;                               
etat_i0       = NaN(steps,1);           
p_off_i0      = NaN(steps,1);           
u1_i0         = NaN(steps,1);           
reconsider_i0 = false(steps,1);         
u2_i0         = NaN(steps,1);           
rho_i0        = NaN(steps,1);           
T_i0          = NaN(steps,1);           

%% ---------- Aggregates ----------
N_off_target         = zeros(steps,1);  % theoretical target
N_off_real           = zeros(steps,1);  % realized OFF

% ---- NEW: constraint-aware target series ----
N_off_target_constr  = zeros(steps,1);  % constrained OFF target
N_off_forced         = zeros(steps,1);  % forced OFF (thermostat/locks)
N_ctrl               = zeros(steps,1);  % controllable units
deficit_theo         = zeros(steps,1);
deficit_constr       = zeros(steps,1);

%% ---------- Temp sample logs ----------
K   = min(20, N);
sel = unique([i0, randperm(N, K)]);
Ksel = numel(sel);
T_hist_sel = NaN(steps, Ksel);

%% ---------- Initialization at t=0 ----------
df0        = f0 - f_f(1);
p_off_0    = fractionOFF(df0);
etat       = rand(N,1) >= p_off_0;      
lock_freq(:) = 0;
lock_th(:)   = 0;

% Initial target
p_off_cmd_cur   = fractionOFF(f0 - f_f(1));
N_off_target(1) = round(p_off_cmd_cur * N);

% Initial constrained target (consistency)
forced_off0 = (T >= Tmax) | (lock_th > 0 & etat == 0);
forced_on0  = (T <= Tmin) | (m_dot > 0) | (lock_th > 0 & etat == 1);
under_lock0 = (lock_freq > 0) | (lock_th > 0);
ctrl_mask0  = ~(forced_off0 | forced_on0 | under_lock0);
N_off_forced(1)        = sum(forced_off0);
N_ctrl(1)              = sum(ctrl_mask0);
N_off_target_constr(1) = min(max(N_off_forced(1) + round(p_off_cmd_cur * N_ctrl(1)), 0), N);

% Logs i0
etat_i0(1)  = etat(i0);
p_off_i0(1) = p_off_cmd_cur;
T_i0(1)     = T(i0);
m_dot_i0(1) = 0;

%% ================== Simulation ==================
for k = 1:steps

    % ====== FCR decision tick (200 ms) ======
    if mod(k, Mdec) == 0
        % Update OFF command
        df              = f0 - f_f(k);
        p_off_cmd_cur   = fractionOFF(df);
        N_off_target(k) = round(p_off_cmd_cur * N);

        % Local ρ (optionally thermo-aware)
        rho_i = rho_base + rho_gain * abs(p_off_cmd_cur - (1 - etat));
        rho_i = min(rho_max, max(rho_min, rho_i));

        % Participation draws
        u1_all   = rand(N,1);
        eligible = (u1_all < rho_i) & (lock_freq <= 0) & (lock_th <= 0);

        % Log i0 at tick
        u1_i0(k)         = u1_all(i0);
        reconsider_i0(k) = eligible(i0);
        p_off_i0(k)      = p_off_cmd_cur;
        rho_i0(k)        = rho_i(i0);

        % i0 first
        if eligible(i0)
            u2_i0(k)  = rand;
            new_off0  = (u2_i0(k) < p_off_cmd_cur);
            if new_off0 && etat(i0)==1
                etat(i0)      = 0;
                lock_freq(i0) = L_off_freq;
            elseif (~new_off0) && etat(i0)==0
                etat(i0)      = 1;
                lock_freq(i0) = L_on_freq;
            end
            eligible(i0) = false;
        end

        % Vectorized for the rest
        idx = find(eligible);
        if ~isempty(idx)
            r = rand(size(idx));
            new_off = r < p_off_cmd_cur;

            to_off = idx( new_off & (etat(idx)==1) );
            etat(to_off)      = 0;
            lock_freq(to_off) = L_off_freq;

            to_on  = idx( (~new_off) & (etat(idx)==0) );
            etat(to_on)       = 1;
            lock_freq(to_on)  = L_on_freq;
        end

        % -------- AVAILABILITY (for constrained target) --------
        % Forced OFF: high temperature OR thermostat-lock currently OFF
        forced_off = (T >= Tmax) | (lock_th > 0 & etat == 0);
        % Forced ON: low temperature OR tap open OR thermostat-lock currently ON
        forced_on  = (T <= Tmin) | (m_dot > 0) | (lock_th > 0 & etat == 1);
        % Any lock makes it non-controllable at this tick
        under_lock = (lock_freq > 0) | (lock_th > 0);
        ctrl_mask  = ~(forced_off | forced_on | under_lock);

        % Counts
        N_off_forced(k)        = sum(forced_off);
        N_ctrl(k)              = sum(ctrl_mask);

        % Constrained OFF target = forced OFF + requested OFF among controllables
        N_off_target_constr(k) = N_off_forced(k) + round(p_off_cmd_cur * N_ctrl(k));
        N_off_target_constr(k) = min(max(N_off_target_constr(k), 0), N);

        % Decrement locks (only on decision ticks)
        if any(lock_freq>0), lock_freq = max(lock_freq - 1, 0); end
        if any(lock_th>0),   lock_th   = max(lock_th   - 1, 0); end

    else
        % Hold last values for smooth logs
        if k>1 && isnan(p_off_i0(k)), p_off_i0(k) = p_off_i0(k-1); end
        if k>1 && isnan(rho_i0(k)),   rho_i0(k)   = rho_i0(k-1);   end
        if k>1 && N_off_target(k)==0,          N_off_target(k)          = N_off_target(k-1);          end
        if k>1 && N_off_target_constr(k)==0,   N_off_target_constr(k)   = N_off_target_constr(k-1);   end
        if k>1 && N_off_forced(k)==0,          N_off_forced(k)          = N_off_forced(k-1);          end
        if k>1 && N_ctrl(k)==0,                N_ctrl(k)                = N_ctrl(k-1);                end
    end

    % ====== Tap tick (1 s): Poisson openings ======
    if mod(k, Mtap) == 0
        closed = (tap_open_steps_left <= 0);
        if any(closed)
            p_start = 1 - exp(-lambda_sec * TapTick);
            start_now = closed & (rand(N,1) < p_start);
            dur_sec = exprnd(dur_mean, sum(start_now), 1);
            tap_open_steps_left(start_now) = max(1, round(dur_sec/dt_high));
        end
    end
    open_now = tap_open_steps_left > 0;
    m_dot(:) = 0;
    m_dot(open_now) = Q_tap;
    tap_open_steps_left(open_now) = tap_open_steps_left(open_now) - 1;

    % ====== Hard thermostat override (EVERY step) ======
    too_cold = (T <= Tmin);
    too_hot  = (T >= Tmax);

    % OFF immediately if too hot + thermostat lock
    idx_hot = find(too_hot & (etat==1));
    if ~isempty(idx_hot)
        etat(idx_hot)    = 0;
        lock_th(idx_hot) = max(lock_th(idx_hot), L_off_th);
    end

    % ON immediately if too cold + thermostat lock
    idx_cold = find(too_cold & (etat==0));
    if ~isempty(idx_cold)
        etat(idx_cold)   = 1;
        lock_th(idx_cold)= max(lock_th(idx_cold), L_on_th);
    end

    % ====== Thermal integration (Euler) ======
    Pin = double(etat) * P_unit;
    dT  = ( Pin ...
          - (T - T_env)./Rth ...
          - m_dot .* c_w .* (T - T_in) ) / Cth;
    T   = T + dt_high * dT;

    % Aggregates
    N_off_real(k) = sum(etat==0);

    % Logs i0 + sample
    etat_i0(k)      = etat(i0);
    T_i0(k)         = T(i0);
    m_dot_i0(k)     = m_dot(i0);
    T_hist_sel(k,:) = T(sel);
end

% Deficits
deficit_theo   = N_off_target        - N_off_real;
deficit_constr = N_off_target_constr - N_off_real;

%% ================== Aggregated Graphs ==================
figure;
plot(time, f, 'Color', [0.8 0.8 0.8]); hold on;
plot(time, f_f, 'LineWidth', 1.2);
yline(50,'--k');
xlabel('Time (s)'); ylabel('Frequency (Hz)');
title('Frequency (raw vs filtered)'); 
xlim([0 3600]);
grid on; legend('Raw','Filtered','Location','best');

figure;
plot(time, N_off_target, 'b-', 'LineWidth', 1.2); hold on;
plot(time, N_off_target_constr, 'Color',[0 0.5 0], 'LineWidth', 1.6); % dark green
plot(time, N_off_real,   'r-', 'LineWidth', 1.0);
xlabel('Time (s)'); ylabel('Number of OFF units');
title('OFF: theoretical vs constrained target vs aggregated');
xlim([0 3600]);
grid on; legend('Theoretical','Constrained','Aggregated','Location','best');

figure;
plot(time, deficit_theo,   'm-', 'LineWidth', 1.0); hold on; 
plot(time, deficit_constr, 'Color',[0.49 0.18 0.55], 'LineWidth', 1.2); % dark violet
yline(0,'--k');
xlabel('Time (s)'); ylabel('Deficit (OFF units)');
title('Deficit vs theoretical target and vs constrained target');
xlim([0 3600]);
grid on; legend('vs Theoretical','vs Constrained','Location','best');

%% ================== Graphs for boiler i0 ==================
figure;
t_tick = (mod((1:steps)',Mdec)==0);

subplot(7,1,1);
plot(time, p_off_i0, 'LineWidth', 1.1); hold on;
stem(time(t_tick), p_off_i0(t_tick), '.', 'HandleVisibility','off');
ylabel('p_{off}(i0)'); grid on;
title(sprintf('Boiler i0 = %d: p_{off}, u1, u2, State, \\rho(i0), T(i0), \\dot m(i0)', i0));
xlim([0 3600]);

subplot(7,1,2);
stem(time(t_tick), u1_i0(t_tick), '.', 'MarkerSize', 8, 'LineWidth', 1.1);
ylim([0 1]); yline(0,':k'); yline(1,':k');
ylabel('u1 (participation)'); grid on; xlim([0 3600]);

subplot(7,1,3);
u2_plot = u2_i0; u2_plot(~reconsider_i0) = NaN;
plot(time, u2_plot, '.', 'MarkerSize', 8);
ylim([0 1]); ylabel('u2 (decision)'); grid on; xlim([0 3600]);

subplot(7,1,4);
stairs(time, etat_i0, 'LineWidth', 1.2); ylim([-0.2 1.2]);
ylabel('State (1=ON,0=OFF)'); grid on; xlim([0 3600]);

subplot(7,1,5);
plot(time, rho_i0, 'LineWidth', 1.2); hold on;
stem(time(t_tick), rho_i0(t_tick), '.', 'HandleVisibility','off');
yline(rho_min,':k'); yline(rho_max,':k');
ylim([0 1]); ylabel('\rho(i0)'); grid on; xlim([0 3600]);

subplot(7,1,6);
plot(time, T_i0, 'LineWidth', 1.2); grid on;
ylabel('T_{i0} (°C)'); xlim([0 3600]);

subplot(7,1,7);
stairs(time, m_dot_i0, 'LineWidth', 1.2); grid on;
ylabel('\dot m_{i0} (kg/s)'); xlabel('Time (s)'); xlim([0 3600]);

%% ================== Temperature curves (sample) ==================
figure; hold on;
colors = lines(Ksel);
for j = 1:Ksel
    if sel(j) == i0
        plot(time, T_hist_sel(:,j), 'LineWidth', 1.8, 'Color', colors(j,:));
    else
        plot(time, T_hist_sel(:,j), 'LineWidth', 0.9, 'Color', colors(j,:));
    end
end
yline(Tmin,':r'); yline(Tmax,':r'); yline(Tset,'--b');
xlabel('Time (s)'); ylabel('Temperature (°C)');
title(sprintf('Temperatures – i0=%d (thick) + %d others', i0, Ksel-1));
xlim([0 3600]);
grid on; box on;
legend_labels = arrayfun(@(x) sprintf('B%d', x), sel, 'UniformOutput', false);
legend([legend_labels, ...
        {sprintf('T_{min}=%g°C',Tmin)}, ...
        {sprintf('T_{max}=%g°C',Tmax)}, ...
        {sprintf('T_{set}=%g°C',Tset)}], ...
       'Location','bestoutside');
