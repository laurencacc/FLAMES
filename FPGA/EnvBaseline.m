function [ema_lux, baseline, z_fire_detected] = EnvBaseline(lux_input)
% EnvBaseline.m — EMA, adaptive thresholding, and Z-score detection

    alpha = 0.2;
    z_score_threshold = 3;
    threshold_delta = 50;
    window_size = 20;
    num_samples = length(lux_input);

    ema_lux = zeros(1, num_samples);
    baseline = zeros(1, num_samples);
    z_fire_detected = false(1, num_samples);
    rolling_mean = zeros(1, num_samples);
    rolling_std = zeros(1, num_samples);

    ema_lux(1) = lux_input(1);
    baseline(1) = lux_input(1);

    for i = 2:num_samples
        ema_lux(i) = alpha * lux_input(i) + (1 - alpha) * ema_lux(i - 1);
        baseline(i) = alpha * ema_lux(i) + (1 - alpha) * baseline(i - 1);

        if i > window_size
            window = ema_lux(i - window_size:i);
            rolling_mean(i) = mean(window);
            rolling_std(i) = std(window);
            if rolling_std(i) > 0
                z_score = (ema_lux(i) - rolling_mean(i)) / rolling_std(i);
                z_fire_detected(i) = abs(z_score) > z_score_threshold;
            end
        end
    end
end
