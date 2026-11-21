data = readtable('servo_log.csv');

% Pencere ve eksen arka planlarını tamamen beyaz yap
fig = figure('Color', 'w');

% 1. ÜST GRAFİK: Hedef Açı vs. Gerçekleşen Açı
ax1 = subplot(2,1,1);
set(ax1, 'Color', 'w', 'XColor', 'k', 'YColor', 'k'); % Eksen içi beyaz, yazılar siyah
hold on;
plot(data.Time, data.Setpoint, 'r--', 'LineWidth', 1.5);
plot(data.Time, data.Angle, 'b-', 'LineWidth', 2);
title('DC Servo Konum Kontrolü (C-Tabanlı Ayrık PID)', 'Color', 'k');
xlabel('Zaman (s)', 'Color', 'k'); 
ylabel('Açı (Derece)', 'Color', 'k');
lgd = legend('Hedef Açı (45°)', 'Gerçekleşen Açı', 'Location', 'southeast');
set(lgd, 'Color', 'w', 'TextColor', 'k');
grid on;

% 2. ALT GRAFİK: Kontrolcü Çıkışı (Voltaj)
ax2 = subplot(2,1,2);
set(ax2, 'Color', 'w', 'XColor', 'k', 'YColor', 'k');
hold on;
plot(data.Time, data.Voltage, 'k-', 'LineWidth', 1.5);
title('Kontrolcü Çıkışı (Motor Voltajı / Saturasyon)', 'Color', 'k');
xlabel('Zaman (s)', 'Color', 'k'); 
ylabel('Voltaj (V)', 'Color', 'k');
grid on;

% --- TOOLBOX OLMADAN PERFORMANS METRİKLERİ HESABI ---
target = 45.0;

% 1. Yüzde Aşım (Overshoot)
peak_angle = max(data.Angle);
overshoot = max(0, ((peak_angle - target) / target) * 100);

% 2. Yükselme Süresi (Rise Time: %10 ile %90 arası)
t10 = data.Time(find(data.Angle >= 0.1 * target, 1));
t90 = data.Time(find(data.Angle >= 0.9 * target, 1));
rise_time = t90 - t10;

% 3. Yerleşme Süresi (Settling Time: %2 hata bandı içinde kaldığı ilk an)
out_of_band = find(abs(data.Angle - target) > 0.02 * target, 1, 'last');
if isempty(out_of_band)
    settling_time = 0;
elseif out_of_band >= length(data.Time)
    settling_time = NaN; % 2 saniye içinde henüz yerleşmedi
else
    settling_time = data.Time(out_of_band + 1);
end