%% ================== Decentralized ρ-partial + HARD Thermostat & Temperature ==================
% - Each load reads ONLY the filtered frequency.
% - Decision tick: 200 ms; a fraction ρ_i (local) of loads reconsider their state.
% - OFF ~ Bernoulli(p_off_cmd) with p_off_cmd = fraction requested by frequency.
% - No communication between loads.
% - Additions:
%     * HARD thermostat (override at every step: OFF if T>=Tmax, ON if T<=Tmin)
%     * Thermal dynamics (energy balance, Euler integration dt_high)
%     * Thermo-aware ρ_i
%     * Two distinct locks: FREQUENCY lock (short) and THERMOSTAT lock (longer)
%     * Detailed logs for one boiler i0 + sample of temperatures
%     * Constrained FCR target (temperature/tap/locks)

clear; clc; close all; rng(0);  % rng for reproducibility

%% ---------- Frequency data ----------
data     = readmatrix('data_instable.csv');   % col1 = f(Hz), col2 = t(s)
time_raw = data(:,2);
f_raw    = data(:,1);

% High-resolution interpolation (20 ms)
dt_high = 0.02;                         % 20 ms
TmaxSim = time_raw(end);
time    = (0:dt_high:TmaxSim)';         % time axis
steps   = numel(time);
f       = interp1(time_raw, f_raw, time, 'linear', 'extrap');

% Noise filtering (zero-delay moving average)
Tfilt = 0.20;                           % 0.2–0.7 s typical
W     = max(1, round(Tfilt/dt_high));
b     = (1/W) * ones(1, W);
f_f   = filtfilt(b, 1, f);

%% ---------- Local FCR law ----------
f0        = 50;
deadband  = 0.02;                       % ±20 mHz deadband
delta_max = 0.20;                       % 200 mHz -> full OFF
fractionOFF = @(df) max(0, min((df - deadband)/delta_max, 1));  % df = f0 - f

%% ---------- Fleet & decentralized cadence ----------
N     = 1000;                           % number of loads
Tdec  = 0.2;                            % decision every 200 ms
Mdec  = round(Tdec/dt_high);            % number of 20 ms steps per tick

% Local ρ_i parameters (decentralized)
rho_min  = 0.08;
rho_base = 0.12;
rho_max  = 0.50;
rho_gain = 1.40;   % weight of the state/command mismatch

% Locks (in 200 ms ticks) -- FREQUENCY (short)
Tlock_on_freq  = 1;                     % 1 tick => 0.2 s
Tlock_off_freq = 1;
L_on_freq      = round(Tlock_on_freq  / Tdec);
L_off_freq     = round(Tlock_off_freq / Tdec);

% Locks (in 200 ms ticks) -- THERMOSTAT (longer)
Tlock_on_th  = 600;                     % 600 ticks => 120 s
Tlock_off_th = 600;
L_on_th      = round(Tlock_on_th  / Tdec);
L_off_th     = round(Tlock_off_th / Tdec);

%% ---------- Thermal parameters ----------
P_nom = 2000;              % W
V     = 0.010;             % m^3 (10 L)
rho_w = 1000;              % kg/m^3
c_w   = 4186;              % J/(kg.K)
Cth   = rho_w*V*c_w;       % J/K (thermal capacity)
Rth   = 2.5;               % K/W (losses to ambient)
T_env = 20;                % °C
T_in  = 15;                % °C (cold inlet water)
m_dot = zeros(N,1);        % kg/s (draw-off per unit, 0 here) — can be nonzero in Scenario C

%% ---------- Thermostat ----------
Tset   = 65;               % °C
dHyst  = 5;                % °C  (hysteresis ±5°C)
Tmin   = Tset - dHyst;
Tmax   = Tset + dHyst;

%% ---------- Individual states (0=OFF, 1=ON) ----------
etat = zeros(N,1);                      % will be re-initialized below

% Two separate locks (in TICKS of 200 ms)
lock_freq = zeros(N,1);                 % lock imposed by FREQUENCY decisions
lock_th   = zeros(N,1);                 % lock imposed by THERMOSTAT decisions

%% ---------- Thermal states ----------
T = 60 + 1*randn(N,1);                  % init around 60°C (small noise)

%% ---------- Logging of one boiler i0 ----------
i0 = 500;                               % index of boiler to trace (1..N)
etat_i0       = NaN(steps,1);           % 0=OFF, 1=ON
p_off_i0      = NaN(steps,1);           % OFF probability seen by i0 (held between ticks)
u1_i0         = NaN(steps,1);           % participation draw (at ticks)
reconsider_i0 = false(steps,1);         % did i0 reconsider at this tick?
u2_i0         = NaN(steps,1);           % OFF/ON draw if reconsidered (at ticks)
rho_i0        = NaN(steps,1);           % local ρ of i0 (propagated between ticks)
T_i0          = NaN(steps,1);           % temperature of boiler i0

%% ---------- Aggregates ----------
N_off_target         = zeros(steps,1);  % theoretical target (for plotting)
N_off_real           = zeros(steps,1);  % observed aggregate
deficit              = zeros(steps,1);  % deficit vs theoretical target

% ---- Series for constrained target ----
N_off_target_constr  = zeros(steps,1);  % constrained OFF target (temp/locks/tap)
N_off_forced         = zeros(steps,1);  % OFF forced by thermostat
N_ctrl               = zeros(steps,1);  % number of controllable units
deficit_constr       = zeros(steps,1);  % deficit vs constrained target

%% ---------- Temperature sample logging ----------
K   = min(20, N);                       % number of boilers to trace (besides i0)
sel = unique([i0, randperm(N, K)]);     % include i0
Ksel = numel(sel);
T_hist_sel = NaN(steps, Ksel);          % temperature history for 'sel'

%% ---------- Random initialization at time 0 ----------
df0        = f0 - f_f(1);
p_off_0    = fractionOFF(df0);
% Initialize states: OFF ~ p_off_0 ; convention: 1=ON, 0=OFF
etat       = rand(N,1) >= p_off_0;      % 1=ON, 0=OFF
lock_freq(:) = 0;
lock_th(:)   = 0;

% Initial ρ_i0 (optional)
rho_i0(1)  = min(rho_max, max(rho_min, rho_base + rho_gain * abs(p_off_0 - (1 - etat(i0)))));

% Initial commands for graphs
p_off_cmd_cur   = fractionOFF(f0 - f_f(1));
N_off_target(1) = round(p_off_cmd_cur * N);

% Init logs for i0
etat_i0(1) = etat(i0);
p_off_i0(1) = p_off_cmd_cur;
T_i0(1) = T(i0);

% Initial constrained target (consistency at t=0)
forced_off0 = (T >= Tmax);
forced_on0  = (T <= Tmin) | (m_dot > 0);
under_lock0 = (lock_freq > 0) | (lock_th > 0);
ctrl_mask0  = ~(forced_off0 | forced_on0 | under_lock0);
N_off_forced(1)        = sum(forced_off0);
N_ctrl(1)              = sum(ctrl_mask0);
N_off_target_constr(1) = N_off_forced(1) + round(p_off_cmd_cur * N_ctrl(1));
N_off_target_constr(1) = min(max(N_off_target_constr(1), 0), N);

%% ================== Simulation ==================
for k = 1:steps

    % ====== Decision tick every 200 ms ======
    if mod(k, Mdec) == 0
        % Update OFF command (held constant between ticks)
        df              = f0 - f_f(k);
        p_off_cmd_cur   = fractionOFF(df);
        N_off_target(k) = round(p_off_cmd_cur * N);

        % Local decentralized ρ + thermo-aware
        rho_i = rho_base + rho_gain * abs(p_off_cmd_cur - (1 - etat)) ...
              + 0.15 * tanh( (T - Tset)/dHyst );       % + if too hot, - if too cold
        rho_i = min(rho_max, max(rho_min, rho_i));

        % Participation draws (local) at tick
        u1_all   = rand(N,1);
        eligible = (u1_all < rho_i) & (lock_freq <= 0) & (lock_th <= 0);

        % Logs for i0 at tick
        u1_i0(k)         = u1_all(i0);
        reconsider_i0(k) = eligible(i0);
        p_off_i0(k)      = p_off_cmd_cur;
        rho_i0(k)        = rho_i(i0);

        % Process i0 individually if eligible: Bernoulli OFF/ON
        if eligible(i0)
            u2_i0(k)  = rand;
            new_off0  = (u2_i0(k) < p_off_cmd_cur);     % true = OFF requested
            if new_off0 && etat(i0)==1                  % ON -> OFF
                etat(i0)      = 0;
                lock_freq(i0) = L_off_freq;             % FREQUENCY lock
            elseif (~new_off0) && etat(i0)==0           % OFF -> ON
                etat(i0)      = 1;
                lock_freq(i0) = L_on_freq;              % FREQUENCY lock
            end
            eligible(i0) = false;  % i0 already processed
        end

        % Process the rest in vector (Bernoulli OFF/ON)
        idx = find(eligible);
        if ~isempty(idx)
            r = rand(size(idx));
            new_off = r < p_off_cmd_cur;               % OFF/ON logic for participants

            % ON -> OFF
            to_off = idx( new_off & (etat(idx)==1) );
            etat(to_off)       = 0;
            lock_freq(to_off)  = L_off_freq;           % FREQUENCY lock

            % OFF -> ON
            to_on  = idx( (~new_off) & (etat(idx)==0) );
            etat(to_on)        = 1;
            lock_freq(to_on)   = L_on_freq;            % FREQUENCY lock
        end

        % -------- Availability/constraints at tick (for constrained target) --------
        % Forced OFF = high temperature OR thermostat lock currently OFF
        forced_off = (T >= Tmax) | (lock_th > 0 & etat == 0);
        
        % Forced ON = low temperature OR tap OR thermostat lock currently ON
        forced_on  = (T <= Tmin) | (m_dot > 0) | (lock_th > 0 & etat == 1);
        under_lock = (lock_freq > 0) | (lock_th > 0);      % non-controllable
        ctrl_mask  = ~(forced_off | forced_on | under_lock);

        % Counts
        N_off_forced(k)        = sum(forced_off);
        N_ctrl(k)              = sum(ctrl_mask);

        % Constrained target = forced OFF + requested OFF among controllables
        N_off_target_constr(k) = N_off_forced(k) + round(p_off_cmd_cur * N_ctrl(k));
        N_off_target_constr(k) = min(max(N_off_target_constr(k), 0), N);

        % decrement locks (in TICKS) — only at decision ticks
        if any(lock_freq>0), lock_freq = max(lock_freq - 1, 0); end
        if any(lock_th>0),   lock_th   = max(lock_th   - 1, 0); end

    else
        % Between ticks: hold curves smooth for logs
        if k>1 && isnan(p_off_i0(k)), p_off_i0(k) = p_off_i0(k-1); end
        if k>1 && isnan(rho_i0(k)),   rho_i0(k)   = rho_i0(k-1);   end
        if k>1 && N_off_target(k)==0, N_off_target(k) = N_off_target(k-1); end
        if k>1 && N_off_target_constr(k)==0, N_off_target_constr(k) = N_off_target_constr(k-1); end
        if k>1 && N_off_forced(k)==0,        N_off_forced(k)        = N_off_forced(k-1);        end
        if k>1 && N_ctrl(k)==0,              N_ctrl(k)              = N_ctrl(k-1);              end
    end

    % ====== HARD THERMOSTAT OVERRIDE (EVERY step) ======
    too_cold = (T <= Tmin);
    too_hot  = (T >= Tmax);

    % OFF immediately if too hot + THERMOSTAT lock (longer)
    idx_hot = find(too_hot & (etat==1));    % switch only if state changes
    if ~isempty(idx_hot)
        etat(idx_hot)    = 0;
        lock_th(idx_hot) = max(lock_th(idx_hot), L_off_th);
    end

    % ON immediately if too cold + THERMOSTAT lock (longer)
    idx_cold = find(too_cold & (etat==0));
    if ~isempty(idx_cold)
        etat(idx_cold)   = 1;
        lock_th(idx_cold)= max(lock_th(idx_cold), L_on_th);
    end

    % ====== Thermal integration at EVERY dt_high step (Euler) ======
    % Power injected per unit (W)
    Pin = double(etat) * P_nom;

    % dT/dt = (Pin - (T - T_env)/Rth - m_dot*c_w*(T - T_in)) / Cth
    dT  = ( Pin ...
          - (T - T_env)./Rth ...
          - m_dot .* c_w .* (T - T_in) ) / Cth;

    T   = T + dt_high * dT;

    % Aggregate
    N_off_real(k) = sum(etat==0);

    % Logs i0 + sample
    etat_i0(k)      = etat(i0);
    T_i0(k)         = T(i0);
    T_hist_sel(k,:) = T(sel);

    % safety propagation (logs)
    if k>1 && isnan(p_off_i0(k)), p_off_i0(k) = p_off_i0(k-1); end
    if k>1 && isnan(rho_i0(k)),   rho_i0(k)   = rho_i0(k-1);   end
end

% Deficits
deficit        = N_off_target        - N_off_real;
deficit_constr = N_off_target_constr - N_off_real;

%% ================== Aggregated graphs ==================
figure;
plot(time, f, 'Color', [0.8 0.8 0.8]); hold on;
plot(time, f_f, 'LineWidth', 1.2);
yline(50,'--k');
xlabel('Time (s)'); ylabel('Frequency (Hz)');
title('Frequency (raw vs filtered)'); grid on; legend('Raw','Filtered');

figure;
plot(time, N_off_target, 'b-', 'LineWidth', 1.2); hold on;
plot(time, N_off_target_constr, 'c--', 'LineWidth', 1.6);
plot(time, N_off_real,   'r-', 'LineWidth', 1.0);
xlabel('Time (s)'); ylabel('Number of OFF units');
title('OFF: theoretical vs constrained target vs aggregate');
grid on; legend('Theoretical','Constrained','Aggregate','Location','best');

figure;
plot(time, deficit, 'm-', 'LineWidth', 1.0); hold on; 
plot(time, deficit_constr, 'k--', 'LineWidth', 1.2); 
yline(0,'--k');
xlabel('Time (s)'); ylabel('Deficit (OFF units)');
title('Deficit vs theoretical target and vs constrained target');
grid on; legend('vs Theoretical','vs Constrained','Location','best');

% Quick stats
fprintf('--- Deficit stats vs theoretical target ---\n');
fprintf('Mean deficit   : %.2f\n', mean(deficit));
fprintf('Median deficit : %.2f\n', median(deficit));
fprintf('Max|abs| def.  : %.0f\n', max(abs(deficit)));
fprintf('STD deficit    : %.2f\n\n', std(deficit));

fprintf('--- Deficit stats vs constrained target ---\n');
fprintf('Mean deficit   : %.2f\n', mean(deficit_constr));
fprintf('Median deficit : %.2f\n', median(deficit_constr));
fprintf('Max|abs| def.  : %.0f\n', max(abs(deficit_constr)));
fprintf('STD deficit    : %.2f\n', std(deficit_constr));

%% ================== Graphs for boiler i0 ==================
figure;
t_tick = (mod((1:steps)',Mdec)==0);            % decision instants

subplot(6,1,1);
plot(time, p_off_i0, 'LineWidth', 1.1); hold on;
stem(time(t_tick), p_off_i0(t_tick), '.', 'HandleVisibility','off');
ylabel('p_{off}(i0)'); grid on;
title(sprintf('Boiler i0 = %d : p_{off}, u1, u2, State, \\rho(i0), T(i0)', i0));

subplot(6,1,2);
stem(time(t_tick), u1_i0(t_tick), '.', 'MarkerSize', 8, 'LineWidth', 1.1);
ylim([0 1]); yline(0,':k'); yline(1,':k');
ylabel('u1 (particip)'); grid on;

subplot(6,1,3);
u2_plot = u2_i0; u2_plot(~reconsider_i0) = NaN;
plot(time, u2_plot, '.', 'MarkerSize', 8);
ylim([0 1]); ylabel('u2 (decision)'); grid on;

subplot(6,1,4);
stairs(time, etat_i0, 'LineWidth', 1.2); ylim([-0.2 1.2]);
ylabel('State (1=ON,0=OFF)'); grid on;

subplot(6,1,5);
plot(time, rho_i0, 'LineWidth', 1.2); hold on;
stem(time(t_tick), rho_i0(t_tick), '.', 'HandleVisibility','off');
yline(rho_min,':k'); yline(rho_max,':k');
ylim([0 1]); ylabel('\rho(i0)'); grid on;

subplot(6,1,6);
plot(time, T_i0, 'LineWidth', 1.2); grid on;
ylabel('T_{i0} (°C)'); xlabel('Time (s)');

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
grid on; box on;
legend_labels = arrayfun(@(x) sprintf('B%d', x), sel, 'UniformOutput', false);
legend([legend_labels, {sprintf('T_{min}=%g°C',Tmin)}, {sprintf('T_{max}=%g°C',Tmax)}, {sprintf('T_{set}=%g°C',Tset)}], ...
       'Location','bestoutside');
