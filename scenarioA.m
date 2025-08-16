%% ================== Decentralized partial-ρ (with logs for one boiler) ==================
% - Each load reads ONLY the filtered frequency.
% - Decision tick: every 200 ms; a local fraction ρ_i of loads reconsider their state.
% - OFF ~ Bernoulli(p_off_cmd) with p_off_cmd = fraction requested by frequency.
% - No communication between loads.
% - Additions:
%     * Random init according to p_off(t0)
%     * LOCAL ρ_i (function of state and p_off_cmd) -> purely decentralized
%     * Detailed logs for one boiler i0 (state, p_off, u1, u2, ρ_i0)
%
% >>> Pure FCR modifications:
%     - Read/update command and decide ONLY at the 200 ms ticks
%     - Lockouts in "tick" units (200 ms)
%     - p_off_cmd held constant between ticks for plotting

clear; clc; close all; rng(0);  % rng for reproducibility

%% ---------- Frequency data ----------
data     = readmatrix('data_instable.csv');   % col1 = f(Hz), col2 = t(s)
time_raw = data(:,2);
f_raw    = data(:,1);

% High-resolution interpolation (20 ms)
dt_high = 0.02;                         % 20 ms
Tmax    = time_raw(end);
time    = (0:dt_high:Tmax)';            % time axis
steps   = numel(time);
f       = interp1(time_raw, f_raw, time, 'linear', 'extrap');

% Noise filtering (zero-delay moving average)
Tfilt = 0.20;                           % 0.3–0.7 s typical
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
rho_gain = 1.40;   % weight of the local state/command mismatch

% Lockout (slight, asymmetric) — in "tick" units (200 ms)
Tlock_on  = 1;                          % stay ON a bit (avoid rapid toggling)
Tlock_off = 1;                          % allow returning ON quickly if needed
L_on      = round(Tlock_on / Tdec);     % NOTE: in TICKS (200 ms)
L_off     = round(Tlock_off / Tdec);    % NOTE: in TICKS (200 ms)

%% ---------- Individual states (0=OFF, 1=ON) ----------
etat = zeros(N,1);                      % will be re-initialized below
lock = zeros(N,1);                      % lockout counters (in TICKS)

%% ---------- Logging for one boiler i0 ----------
i0 = 500;                               % index of boiler to trace (1..N)
etat_i0       = NaN(steps,1);           % 0=OFF, 1=ON
p_off_i0      = NaN(steps,1);           % OFF prob seen by i0 (held between ticks)
u1_i0         = NaN(steps,1);           % participation draw (at ticks)
reconsider_i0 = false(steps,1);         % did i0 reconsider at this tick?
u2_i0         = NaN(steps,1);           % OFF/ON draw if reconsidered (at ticks)
rho_i0        = NaN(steps,1);           % local ρ of i0 (propagated between ticks)

%% ---------- Aggregate preallocs ----------
N_off_target = zeros(steps,1);          % theoretical target (for plotting)
N_off_real   = zeros(steps,1);          % observed aggregate

%% ---------- Random initialization at time 0 ----------
df0        = f0 - f_f(1);
p_off_0    = fractionOFF(df0);
% Initialize states: OFF ~ p_off_0 ; convention etat: 1=ON, 0=OFF
etat       = rand(N,1) >= p_off_0;      % 1=ON, 0=OFF
lock(:)    = 0;
etat_i0(1) = etat(i0);

% Initial ρ_i0 (optional)
rho_i0(1)  = min(rho_max, max(rho_min, rho_base + rho_gain * abs(p_off_0 - (1 - etat(i0)))));

% --- Current OFF command (held constant between two 200 ms ticks)
p_off_cmd_cur = fractionOFF(f0 - f_f(1));
p_off_i0(1)   = p_off_cmd_cur;          % log the initial command

% Initial aggregate target (for smoother plot)
N_off_target(1) = round(p_off_cmd_cur * N);

%% ================== Simulation ==================
for k = 1:steps
    % Decision tick every 200 ms
    if mod(k, Mdec) == 0
        % --- Read filtered frequency at tick and update OFF command
        df              = f0 - f_f(k);
        p_off_cmd_cur   = fractionOFF(df);             % new OFF command (held between ticks)
        N_off_target(k) = round(p_off_cmd_cur * N);    % aggregate target (updated at tick)

        % --- Local ρ, purely decentralized (vectorized) ---
        % etat(i) = 1 if ON, 0 if OFF ; (1 - etat) = 1 if OFF
        rho_i = rho_base + rho_gain * abs(p_off_cmd_cur - (1 - etat));
        rho_i = min(rho_max, max(rho_min, rho_i));

        % Participation draws (local) at the tick
        u1_all     = rand(N,1);
        reconsider = (u1_all < rho_i) & (lock <= 0);

        % --- Logs for i0 at tick ---
        u1_i0(k)         = u1_all(i0);
        reconsider_i0(k) = reconsider(i0);
        p_off_i0(k)      = p_off_cmd_cur;
        rho_i0(k)        = rho_i(i0);

        % Process i0 first to log u2 clearly
        if reconsider(i0)
            u2_i0(k)  = rand;
            new_off0  = (u2_i0(k) < p_off_cmd_cur);    % true = OFF requested
            if new_off0 && etat(i0)==1                 % ON -> OFF
                etat(i0) = 0; lock(i0) = L_off;
            elseif (~new_off0) && etat(i0)==0          % OFF -> ON
                etat(i0) = 1; lock(i0) = L_on;
            end
            reconsider(i0) = false;  % i0 already processed
        end

        % Process the other loads vectorially
        idx = find(reconsider);
        if ~isempty(idx)
            r = rand(size(idx));
            new_off = r < p_off_cmd_cur;               % OFF/ON logic for participants

            % ON -> OFF
            to_off = idx( new_off & (etat(idx)==1) );
            etat(to_off) = 0;  lock(to_off) = L_off;

            % OFF -> ON
            to_on  = idx( (~new_off) & (etat(idx)==0) );
            etat(to_on)  = 1;  lock(to_on)  = L_on;
        end

        % Decrement lockouts (in TICKS, only at decision ticks)
        if any(lock>0), lock = max(lock - 1, 0); end

    else
        % Between ticks: keep p_off_i0 constant and propagate rho_i0
        if k>1 && isnan(p_off_i0(k))
            p_off_i0(k) = p_off_i0(k-1);
        end
        if k>1 && isnan(rho_i0(k))
            rho_i0(k) = rho_i0(k-1);
        end
        % keep the aggregate target constant between ticks (for readable plots)
        if k>1 && N_off_target(k)==0
            N_off_target(k) = N_off_target(k-1);
        end
    end

    % Aggregate (offline) at every step for fine plotting
    N_off_real(k) = sum(etat==0);

    % Log i0 at every step
    etat_i0(k) = etat(i0);

    % Safety propagation (logs)
    if k>1 && isnan(p_off_i0(k)), p_off_i0(k) = p_off_i0(k-1); end
    if k>1 && isnan(rho_i0(k)),   rho_i0(k)   = rho_i0(k-1);   end
end

deficit = N_off_target - N_off_real;

%% ================== Aggregated graphs ==================
figure;
plot(time, f, 'Color', [0.8 0.8 0.8]); hold on;
plot(time, f_f, 'LineWidth', 1.2);
yline(50,'--k');
xlabel('Time (s)'); ylabel('Frequency (Hz)');
title('Frequency (raw vs filtered)'); grid on; legend('Raw','Filtered');

figure;
plot(time, N_off_target, 'b-', 'LineWidth', 1.2); hold on;
plot(time, N_off_real,   'r-', 'LineWidth', 1.0);
xlabel('Time (s)'); ylabel('Number of loads OFF');
title('Theoretical vs aggregated (decentralized, tick=200 ms, partial-\rho)'); grid on;
legend('Theoretical','Aggregated','Location','best');

figure;
plot(time, deficit, 'm', 'LineWidth', 1.1); hold on; yline(0,'--k');
xlabel('Time (s)'); ylabel('Deficit (loads OFF)');
title('Deficit between target and response (decentralized)'); grid on;

% Quick stats
fprintf('Mean deficit   : %.2f\n', mean(deficit));
fprintf('Median deficit : %.2f\n', median(deficit));
fprintf('Max|abs| def.  : %.0f\n', max(abs(deficit)));
fprintf('STD deficit    : %.2f\n', std(deficit));

%% ================== Graphs for boiler i0 ==================
figure;
t_tick = (mod((1:steps)',Mdec)==0);            % decision instants

subplot(5,1,1);
plot(time, p_off_i0, 'LineWidth', 1.1); hold on;
stem(time(t_tick), p_off_i0(t_tick), '.', 'HandleVisibility','off');
ylabel('p_{off}(i0)'); grid on;
title(sprintf('Boiler i0 = %d: p_{off}, u1, u2, State, \\rho(i0)', i0));

subplot(5,1,2);
stem(time(t_tick), u1_i0(t_tick), '.', 'MarkerSize', 8, 'LineWidth', 1.1);
ylim([0 1]); yline(0,':k'); yline(1,':k');
ylabel('u1 (participation)'); grid on;

subplot(5,1,3);
u2_plot = u2_i0; u2_plot(~reconsider_i0) = NaN;
plot(time, u2_plot, '.', 'MarkerSize', 8); hold on;
ylim([0 1]); ylabel('u2 (decision)'); grid on;

subplot(5,1,4);
stairs(time, etat_i0, 'LineWidth', 1.2); ylim([-0.2 1.2]);
ylabel('State (1=ON,0=OFF)'); grid on;

subplot(5,1,5);
plot(time, rho_i0, 'LineWidth', 1.2); hold on;
stem(time(t_tick), rho_i0(t_tick), '.', 'HandleVisibility','off');
yline(rho_min,':k'); yline(rho_max,':k');
ylim([0 1]); xlabel('Time (s)'); ylabel('\rho(i0)'); grid on;

%% ================== Text summary (last i0 decisions) ==================
idx_dec = find(reconsider_i0 & ~isnan(u2_i0));
K = min(10, numel(idx_dec));
if K > 0
    last = idx_dec(end-K+1:end);
    Ttbl = table( ...
        time(last), ...
        p_off_i0(last), ...
        u1_i0(last), ...
        u2_i0(last), ...
        etat_i0(last), ...
        'VariableNames', {'t_s','p_off','u1_particip','u2_decision','state(1=ON,0=OFF)'} );
    disp(Ttbl);
end

%% ================== Aggregated Graphs (English labels, 1-hour window) ==================
x_end = 3600;  % end of X-axis in seconds

figure;
plot(time, f, 'Color', [0.8 0.8 0.8]); hold on;
plot(time, f_f, 'LineWidth', 1.2);
yline(50,'--k');
xlabel('Time (s)'); ylabel('Frequency (Hz)');
title('Frequency (raw vs filtered)'); grid on; legend('Raw','Filtered');
xlim([0 x_end]);

figure;
plot(time, N_off_target, 'b-', 'LineWidth', 1.2); hold on;
plot(time, N_off_real,   'r-', 'LineWidth', 1.0);
xlabel('Time (s)'); ylabel('Number of loads OFF');
title('Theoretical vs aggregated (decentralized, tick=200 ms, partial-\rho)'); grid on;
legend('Theoretical','Aggregated','Location','best');
xlim([0 x_end]);

figure;
plot(time, deficit, 'm', 'LineWidth', 1.1); hold on; yline(0,'--k');
xlabel('Time (s)'); ylabel('Deficit (loads OFF)');
title('Deficit between target and response (decentralized)'); grid on;
xlim([0 x_end]);

%% ================== Aggregate power vs. continuous (linear) FCR target ==================
% Power assumptions (adjust as needed)
P_on_W      = 2000;   % W when ON
P_off_W     = 0;      % W when OFF (set e.g., 5 W if standby)

% Command p_off series (common to all loads, held between ticks)
p_off_cmd_vec = p_off_i0;  % same value for the whole fleet

% Measured aggregate power (with discrete ON/OFF)
P_meas_W = (N - N_off_real)  * P_on_W + N_off_real  * P_off_W;

% Continuous (linear) target
P_cont_W = N * ((1 - p_off_cmd_vec) .* P_on_W + p_off_cmd_vec .* P_off_W);

%% ----------- 1. Comparison (MW)
figure; hold on; grid on;
plot(time, P_cont_W/1e6, 'LineWidth', 1.6);
plot(time, P_meas_W/1e6, 'LineWidth', 1.0);
xlim([0 x_end]);
xlabel('Time (s)'); ylabel('Power (MW)');
title('Aggregate power: discrete ON/OFF vs continuous FCR target');
legend('Continuous FCR target', 'Measured aggregate (ON/OFF)', 'Location','best');

%% ----------- 2. Difference (MW)
P_diff_W = P_meas_W - P_cont_W;
figure; hold on; grid on;
plot(time, P_diff_W/1e6, 'LineWidth', 1.2);
yline(0,'--k');
xlim([0 x_end]);
xlabel('Time (s)'); ylabel('Difference (MW)');
title('Difference between measured and continuous target');
legend('Measured - Continuous target', 'Location','best');

%% ----------- 3. Difference stats
mean_diff_MW = mean(P_diff_W)/1e6;
max_diff_MW  = max(P_diff_W)/1e6;
min_diff_MW  = min(P_diff_W)/1e6;
std_diff_MW  = std(P_diff_W)/1e6;

fprintf('\n=== Power Difference Stats ===\n');
fprintf('Mean difference     : %.4f MW\n', mean_diff_MW);
fprintf('Max positive diff   : %.4f MW\n', max_diff_MW);
fprintf('Max negative diff   : %.4f MW\n', min_diff_MW);
fprintf('Standard deviation  : %.4f MW\n', std_diff_MW);
