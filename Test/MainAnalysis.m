function MainAnalysis(ema_lux, baseline, z_fire_detected, lux_input)
% MainAnalysis.m — Fire detection using FFT and YCbCr

    num_samples = length(ema_lux);
    sampling_interval = 0.5;
    fft_window_size = 32;
    flicker_threshold = 1e3;
    pixel_threshold = 100;
    fire_flicker = false(1, num_samples);
    fire_color = false(1, num_samples);
    final_fire_detected = false(1, num_samples);

    % Load placeholder image
    frame = imread('peppers.jpg');
    frame = imresize(frame, [240 320]);

    for i = fft_window_size+1:num_samples
        if z_fire_detected(i)
            % --- FFT Flicker Detection ---
            lux_window = ema_lux(i - fft_window_size + 1:i);
            Y = fft(lux_window);
            f = (0:fft_window_size - 1) * (1 / sampling_interval / fft_window_size);
            power_spectrum = abs(Y).^2;
            fire_band = f >= 1 & f <= 10;
            flicker_energy = sum(power_spectrum(fire_band));
            fire_flicker(i) = flicker_energy > flicker_threshold;

            % --- YCbCr Color Feature Extraction ---
            ycbcr_img = rgb2ycbcr(frame);
            Cb = ycbcr_img(:,:,2);
            Cr = ycbcr_img(:,:,3);
            fire_mask_ycbcr = (Cr > 140) & (Cb < 120);
            pixel_count = sum(fire_mask_ycbcr(:));
            fire_color(i) = pixel_count > pixel_threshold;

            % --- Final Fire Confirmation ---
            if fire_flicker(i) && fire_color(i)
                final_fire_detected(i) = true;

                figure;
                imshow(frame);
                hold on;
                fire_mask_rgb = uint8(cat(3, fire_mask_ycbcr * 255, zeros(size(fire_mask_ycbcr)), zeros(size(fire_mask_ycbcr))));
                h = imshow(fire_mask_rgb);
                set(h, 'AlphaData', fire_mask_ycbcr * 0.4);
                title(['🔥 Fire Confirmed at Sample ', num2str(i), ' | Pixels: ', num2str(pixel_count)]);
            end
        end
    end

    % --- Summary Plot ---
    figure;
    subplot(4,1,1); plot(lux_input, 'Color', [0.4 0.4 0.4]); title('Raw Lux Input'); ylabel('Lux');
    subplot(4,1,2); plot(z_fire_detected, 'b'); title('Z-Score Anomaly Flag'); ylabel('Z-Score Spike');
    subplot(4,1,3); stem(fire_flicker, 'filled'); title('Fire Flicker (FFT)'); ylabel('Flicker Detected');
    subplot(4,1,4); stem(final_fire_detected, 'filled'); title('🔥 Confirmed Fire Detection'); ylabel('Fire'); xlabel('Sample Index');
end
