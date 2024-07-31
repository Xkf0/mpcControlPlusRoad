% 打开文件
fileID = fopen('data.txt', 'r');

% 初始化存储位置数据的数组
poses = [];
pose_indices = [];

% 逐行读取文件
tline = fgets(fileID);
while ischar(tline)
    if startsWith(tline, 'pose')
        % 细致地处理字符串分割
        parts = strsplit(strrep(strrep(strrep(tline, 'pose', ''), '[', ''), ']', ''), ':');
        
        % 提取pose序号
        pose_index = str2double(strtrim(parts{1}));
        
        % 对parts{3}和parts{4}进行分割
        splitX = split(parts{3}, ',');
        splitY = split(parts{4}, ',');
        
        % 然后提取第一个元素
        x_val = str2double(strtrim(splitX{1}));  % 正确提取x值
        y_val = str2double(strtrim(splitY{1}));  % 正确提取y值
        
        poses = [poses; x_val y_val];
        pose_indices = [pose_indices; pose_index];
    end
    tline = fgets(fileID);
end

% 关闭文件
fclose(fileID);

% 绘图
figure('Position', [100, 100, 1000, 600]);

% 初始化绘图数组
x_data = [];
y_data = [];

for i = 1:size(poses, 1)
    if i > 1 && pose_indices(i) < pose_indices(i-1)
        % 如果当前pose序号小于前一个pose序号，绘制之前的数据并重置绘图数组
        plot(x_data, y_data, 'o-', 'DisplayName', 'Trajectory'); % 绘制轨迹
        hold on;
        x_data = [];
        y_data = [];
    end
    % 将当前点添加到绘图数组
    x_data = [x_data; poses(i, 1)];
    y_data = [y_data; poses(i, 2)];
end

% 绘制最后一段轨迹
if ~isempty(x_data) && ~isempty(y_data)
    plot(x_data, y_data, 'o-', 'DisplayName', 'Trajectory'); % 绘制轨迹
end

% 保持当前图形以绘制目标路线
hold on;

% 画圆弧1
theta = linspace(pi, pi/2, 100); % 圆弧的角度范围

r = 10; % 半径
xc = 10; % 圆心x坐标
yc = 0; % 圆心y坐标
x = xc + r*cos(theta);
y = yc + r*sin(theta);
plot(x, y, 'r--', 'DisplayName', 'Target Route'); % 绘制圆弧

r = 10; % 半径
xc = 10; % 圆心x坐标
yc = 20; % 圆心y坐标
x = xc - r*cos(theta);
y = yc - r*sin(theta);
h1 = plot(x, y, 'r--'); % 绘制圆弧
set(get(get(h1,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');

% 绘制障碍物
plot(0, 2, 'p', 'MarkerSize', 10, 'MarkerEdgeColor', 'm', 'MarkerFaceColor', 'none', 'DisplayName', 'Obstacle1'); % 空心星星
plot(10, 10, 'p', 'MarkerSize', 10, 'MarkerEdgeColor', 'g', 'MarkerFaceColor', 'none', 'DisplayName', 'Obstacle2'); % 空心星星

% 设置图例
legend;

% 设置图表标题和坐标轴标签
title('Trajectory and Target Route Comparison Plot');
xlabel('x');
ylabel('y');
grid on;

% 保存图像
saveas(gcf, 'trajectory_plot.png');
