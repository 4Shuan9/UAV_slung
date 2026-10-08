% ==========================================================
% 1. 读取数据
% ==========================================================
% 设置读取选项，'VariableNamingRule', 'preserve' 是为了保留原本带斜杠的列名
opts = detectImportOptions('20260408_test1_Data.csv');
opts.VariableNamingRule = 'preserve'; 
data = readtable('20260408_test1_Data.csv', opts);

% ==========================================================
% 2. 提取我们需要的时间戳和 X/Y 轴摆角数据
% ==========================================================
% 注意：列名必须和你 CSV 文件中的表头完全一致
time_sec = data.("__time"); 

% 视觉测量值 (CV)
cv_x = data.("/uav/cv/swing_angle/cartesian/vector/x");
cv_y = data.("/uav/cv/swing_angle/cartesian/vector/y");

% 真实值 (Truth)
truth_x = data.("/uav/truth/swing_angle/cartesian/vector/x");
truth_y = data.("/uav/truth/swing_angle/cartesian/vector/y");

% ==========================================================
% 3. 数据对齐 (处理时间戳不同步的问题)
% ==========================================================
% 将时间秒数转换为 MATLAB 的 datetime 格式，并创建时间表 (timetable)
% 时间表是 MATLAB 处理时间序列数据的神器
tt = timetable(datetime(time_sec, 'ConvertFrom', 'posixtime'), cv_x, truth_x, cv_y, truth_y);

% 因为真实值和视觉值的发布频率可能不同，很多行会出现 NaN (空值)
% 我们使用 'linear' (线性插值) 来填补这些空缺，让它们在时间上对齐
tt_filled = fillmissing(tt, 'linear');

% ==========================================================
% 4. 计算均方根误差 (RMSE)
% ==========================================================
% RMSE 公式： 误差的平方 -> 求均值 -> 开根号
% omitnan 代表如果在计算中遇到无法插值的空值，则忽略它们
rmse_x = sqrt(mean((tt_filled.cv_x - tt_filled.truth_x).^2, 'omitnan'));
rmse_y = sqrt(mean((tt_filled.cv_y - tt_filled.truth_y).^2, 'omitnan'));

% 在命令行窗口打印出结果
fprintf('笛卡尔坐标系 X 轴的 RMSE: %.4f\n', rmse_x);
fprintf('笛卡尔坐标系 Y 轴的 RMSE: %.4f\n', rmse_y);

% ==========================================================
% 5. 绘制对比图
% ==========================================================
% 创建一个新的图形窗口
figure('Name', 'Gazebo 真实值与视觉值对比', 'Color', 'w', 'Position', [100, 100, 800, 600]);

% --- 画 X 轴的对比图 (占据上半部分) ---
subplot(2, 1, 1); % 将画板分为 2 行 1 列，现在画第 1 块
plot(tt_filled.Time, tt_filled.truth_x, 'b-', 'LineWidth', 1.5, 'DisplayName', '真实值 (Truth X)');
hold on; % 保持当前图像，把另一条线叠加上去
plot(tt_filled.Time, tt_filled.cv_x, 'r--', 'LineWidth', 1.5, 'DisplayName', '视觉值 (CV X)');
title(sprintf('摆角 X 方向对比 (RMSE: %.4f)', rmse_x), 'FontSize', 12);
xlabel('时间');
ylabel('X 轴摆角');
legend('Location', 'best'); % 自动把图例放在遮挡最少的地方
grid on; % 打开网格线

% --- 画 Y 轴的对比图 (占据下半部分) ---
subplot(2, 1, 2); % 切换到第 2 块画板
plot(tt_filled.Time, tt_filled.truth_y, 'b-', 'LineWidth', 1.5, 'DisplayName', '真实值 (Truth Y)');
hold on;
plot(tt_filled.Time, tt_filled.cv_y, 'r--', 'LineWidth', 1.5, 'DisplayName', '视觉值 (CV Y)');
title(sprintf('摆角 Y 方向对比 (RMSE: %.4f)', rmse_y), 'FontSize', 12);
xlabel('时间');
ylabel('Y 轴摆角');
legend('Location', 'best');
grid on;