function gear_gui
    % 创建图形界面
    fig = uifigure('Name', '齿轮设计工具', 'Position', [100 100 800 600]);

    % 创建三个按钮，分别用于识别齿廓、计算收缩率和反演设计
    btn1 = uibutton(fig, 'push', 'Text', '识别齿廓', 'Position', [50, 500, 100, 50], 'ButtonPushedFcn', @(btn,event) identifyContour(fig, 'D:\桌面\gears.STL'));
    btn2 = uibutton(fig, 'push', 'Text', '收缩情况', 'Position', [250, 500, 100, 50], 'ButtonPushedFcn', @(btn,event) calculateShrinkage(fig, 'D:\桌面\gears.STL'));
    btn3 = uibutton(fig, 'push', 'Text', '反演设计', 'Position', [450, 500, 100, 50], 'ButtonPushedFcn', @(btn,event) inverseDesign(fig, 'D:\桌面\gearre.stl'));

    % 创建用于显示图像的轴
    global ax3D ax2D;
    ax3D = uiaxes(fig, 'Position', [50, 50, 350, 400]);
    ax2D = uiaxes(fig, 'Position', [400, 50, 350, 400]);
end

function identifyContour(fig, filepath)
    global ax3D;
    gearModel = stlread(filepath); % 读取 STL 文件
    disp(['导入识别齿廓模型: ', filepath]); % 显示导入的模型路径

    % 提取齿轮外形点
    vertices = gearModel.Points; % 获取顶点
    faces = gearModel.ConnectivityList; % 获取面连接列表

    % 检查 faces 和 vertices 的维度
    if size(faces, 2) ~= 3
        error('faces 数组的维度不正确，必须是 Nx3 的数组');
    end
    if size(vertices, 2) ~= 3
        error('vertices 数组的维度不正确，必须是 Nx3 的数组');
    end

    % 计算几何中心
    center = mean(vertices, 1); % 计算顶点的几何中心

    % 计算边界
    k = boundary(vertices(:,1), vertices(:,2), vertices(:,3), 0.9); % 计算边界

    % 绘制齿廓点
    cla(ax3D); % 清空轴
    trisurf(faces, vertices(:,1), vertices(:,2), vertices(:,3), 'FaceColor', 'cyan', 'Parent', ax3D); % 绘制3D模型
    hold(ax3D, 'on');
    plot3(ax3D, vertices(k,1), vertices(k,2), vertices(k,3), 'r.', 'MarkerSize', 10); % 绘制齿廓点
    title(ax3D, '齿轮齿廓');
    xlabel(ax3D, 'X');
    ylabel(ax3D, 'Y');
    zlabel(ax3D, 'Z');
    hold(ax3D, 'off');
end

function calculateShrinkage(fig, filepath)
    global ax2D;
    gearModel = stlread(filepath); % 读取 STL 文件
    disp(['导入计算收缩率模型: ', filepath]); % 显示导入的模型路径

    vertices = gearModel.Points; % 获取顶点
    faces = gearModel.ConnectivityList; % 获取面连接列表

    % 计算几何中心
    center = mean(vertices, 1); % 计算顶点的几何中心

    % 计算边界点
    k = boundary(vertices(:,1), vertices(:,2), 0.9); % 计算边界

    % 计算每个点到中心的距离
    distances = sqrt(sum((vertices(k,:) - center).^2, 2)); % 计算边界点到中心的距离

    % 假设齿的数目通过边界点间的角度变化确定
    angles = atan2(vertices(k,2) - center(2), vertices(k,1) - center(1)); % 计算边界点的角度
    angles = mod(angles, 2*pi); % 将角度转换为0到2*pi之间
    [sortedAngles, sortedIdx] = sort(angles); % 对角度进行排序
    angleDiffs = diff([sortedAngles; sortedAngles(1) + 2*pi]); % 计算角度差

    % 判断角度变化的突变点作为齿数
    threshold = mean(angleDiffs) + std(angleDiffs); % 设置阈值
    toothBoundaries = find(angleDiffs > threshold); % 找到突变点
    numTeeth = length(toothBoundaries); % 齿数
    shrinkageRates = zeros(numTeeth, 1); % 初始化收缩率

    % 计算每个齿的收缩率
    for i = 1:numTeeth
        if i == numTeeth
            toothPoints = sortedIdx(toothBoundaries(i):end); % 获取最后一个齿的点
        else
            toothPoints = sortedIdx(toothBoundaries(i):toothBoundaries(i+1)-1); % 获取当前齿的点
        end
        toothDistances = distances(toothPoints); % 获取当前齿的距离
        maxDist = max(toothDistances); % 最大距离
        minDist = min(toothDistances); % 最小距离
        if maxDist ~= 0 % 防止除以零
            shrinkageRates(i) = (maxDist - minDist) / maxDist; % 计算收缩率
        else
            shrinkageRates(i) = 0;
        end
    end

    % 显示收缩率
    cla(ax2D); % 清空轴
    plot(ax2D, 1:numTeeth, shrinkageRates, '-o'); % 绘制收缩率曲线
    title(ax2D, '收缩后半径');
    xlabel(ax2D, '轮齿编号');
    ylabel(ax2D, '收缩率');
end

function inverseDesign(fig, filepath)
    global ax2D;
    gearModel = stlread(filepath); % 读取 STL 文件
    disp(['导入反演设计模型: ', filepath]); % 显示导入的模型路径

    vertices = gearModel.Points; % 获取顶点
    faces = gearModel.ConnectivityList; % 获取面连接列表

    % 计算几何中心
    center = mean(vertices, 1); % 计算顶点的几何中心

    % 计算边界点
    k = boundary(vertices(:,1), vertices(:,2), 0.9); % 计算边界

    % 计算每个点到中心的距离
    distances = sqrt(sum((vertices(k,:) - center).^2, 2)); % 计算边界点到中心的距离

    % 设定放大比例
    scaleFactor = 1.005; % 假设放大比例为 10%

    % 反演设计放大齿廓
    newVertices = vertices; % 初始化新顶点
    for i = 1:length(k)
        newVertices(k(i), :) = center + scaleFactor * (vertices(k(i), :) - center); % 按比例放大
    end

    % 投影到平面并绘制齿廓
    cla(ax2D); % 清空轴
    plot(ax2D, vertices(k,1), vertices(k,2), 'r.', 'MarkerSize', 10); % 绘制原始边界点
    hold(ax2D, 'on');
    plot(ax2D, newVertices(k,1), newVertices(k,2), 'g.', 'MarkerSize', 10); % 绘制放大边界点
    plot(ax2D, [newVertices(k,1); newVertices(k(1),1)], [newVertices(k,2); newVertices(k(1),2)], 'g-'); % 连接放大边界点
    title(ax2D, '齿轮齿廓反演设计');
    xlabel(ax2D, 'X');
    ylabel(ax2D, 'Y');
    legend(ax2D, '原始边界点', '放大边界点');
    hold(ax2D, 'off');
end
