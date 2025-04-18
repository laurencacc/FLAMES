% MotorDetection.m — Simulates stepper motor scanning and triggers fire detection

clc; clear; close all;

% --- Motor Scanning Setup ---
positions = 0:10:360;                 % Scan angles in degrees
num_positions = length(positions);
scan_lux_threshold = 110;            % Lux value to trigger motor stop

% --- Simulate Lux at Each Position ---
fire_position_index = 7;             % Fire exists at position 60°
lux_data_all = cell(1, num_positions);

for p = 1:num_positions
    if p == fire_position_index
        % Fire scenario
        lux = [100 + randn(1, 119)*3, 180 + randn(1,11)*10, 105 + randn(1,70)*3];
    else
        % No fire
        lux = 100 + randn(1, 200) * 3;
    end
    lux_data_all{p} = lux;
end

% --- Step 1: Motor Scan ---
locked_position = -1;
for p = 1:num_positions
    lux_max = max(lux_data_all{p});
    fprintf("Scanning position %d°: Max Lux = %.2f\n", positions(p), lux_max);
    if lux_max > scan_lux_threshold
        fprintf("🔥 High lux detected at %d°, stopping motor.\n", positions(p));
        locked_position = p;
        break;
    end
end

if locked_position == -1
    error("No high-lux region found. Motor scanned full range.");
end

% --- Step 2: Run Fire Detection on Locked Region ---
lux_input = lux_data_all{locked_position};
[ema_lux, baseline, z_fire_detected] = EnvBaseline(lux_input);
MainAnalysis(ema_lux, baseline, z_fire_detected, lux_input);
