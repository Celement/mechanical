function gear_gui
    % ����ͼ�ν���
    fig = uifigure('Name', '������ƹ���', 'Position', [100 100 800 600]);

    % ����������ť���ֱ�����ʶ����������������ʺͷ������
    btn1 = uibutton(fig, 'push', 'Text', 'ʶ�����', 'Position', [50, 500, 100, 50], 'ButtonPushedFcn', @(btn,event) identifyContour(fig, 'D:\����\gears.STL'));
    btn2 = uibutton(fig, 'push', 'Text', '�������', 'Position', [250, 500, 100, 50], 'ButtonPushedFcn', @(btn,event) calculateShrinkage(fig, 'D:\����\gears.STL'));
    btn3 = uibutton(fig, 'push', 'Text', '�������', 'Position', [450, 500, 100, 50], 'ButtonPushedFcn', @(btn,event) inverseDesign(fig, 'D:\����\gearre.stl'));

    % ����������ʾͼ�����
    global ax3D ax2D;
    ax3D = uiaxes(fig, 'Position', [50, 50, 350, 400]);
    ax2D = uiaxes(fig, 'Position', [400, 50, 350, 400]);
end

function identifyContour(fig, filepath)
    global ax3D;
    gearModel = stlread(filepath); % ��ȡ STL �ļ�
    disp(['����ʶ�����ģ��: ', filepath]); % ��ʾ�����ģ��·��

    % ��ȡ�������ε�
    vertices = gearModel.Points; % ��ȡ����
    faces = gearModel.ConnectivityList; % ��ȡ�������б�

    % ��� faces �� vertices ��ά��
    if size(faces, 2) ~= 3
        error('faces �����ά�Ȳ���ȷ�������� Nx3 ������');
    end
    if size(vertices, 2) ~= 3
        error('vertices �����ά�Ȳ���ȷ�������� Nx3 ������');
    end

    % ���㼸������
    center = mean(vertices, 1); % ���㶥��ļ�������

    % ����߽�
    k = boundary(vertices(:,1), vertices(:,2), vertices(:,3), 0.9); % ����߽�

    % ���Ƴ�����
    cla(ax3D); % �����
    trisurf(faces, vertices(:,1), vertices(:,2), vertices(:,3), 'FaceColor', 'cyan', 'Parent', ax3D); % ����3Dģ��
    hold(ax3D, 'on');
    plot3(ax3D, vertices(k,1), vertices(k,2), vertices(k,3), 'r.', 'MarkerSize', 10); % ���Ƴ�����
    title(ax3D, '���ֳ���');
    xlabel(ax3D, 'X');
    ylabel(ax3D, 'Y');
    zlabel(ax3D, 'Z');
    hold(ax3D, 'off');
end

function calculateShrinkage(fig, filepath)
    global ax2D;
    gearModel = stlread(filepath); % ��ȡ STL �ļ�
    disp(['�������������ģ��: ', filepath]); % ��ʾ�����ģ��·��

    vertices = gearModel.Points; % ��ȡ����
    faces = gearModel.ConnectivityList; % ��ȡ�������б�

    % ���㼸������
    center = mean(vertices, 1); % ���㶥��ļ�������

    % ����߽��
    k = boundary(vertices(:,1), vertices(:,2), 0.9); % ����߽�

    % ����ÿ���㵽���ĵľ���
    distances = sqrt(sum((vertices(k,:) - center).^2, 2)); % ����߽�㵽���ĵľ���

    % ����ݵ���Ŀͨ���߽���ĽǶȱ仯ȷ��
    angles = atan2(vertices(k,2) - center(2), vertices(k,1) - center(1)); % ����߽��ĽǶ�
    angles = mod(angles, 2*pi); % ���Ƕ�ת��Ϊ0��2*pi֮��
    [sortedAngles, sortedIdx] = sort(angles); % �ԽǶȽ�������
    angleDiffs = diff([sortedAngles; sortedAngles(1) + 2*pi]); % ����ǶȲ�

    % �жϽǶȱ仯��ͻ�����Ϊ����
    threshold = mean(angleDiffs) + std(angleDiffs); % ������ֵ
    toothBoundaries = find(angleDiffs > threshold); % �ҵ�ͻ���
    numTeeth = length(toothBoundaries); % ����
    shrinkageRates = zeros(numTeeth, 1); % ��ʼ��������

    % ����ÿ���ݵ�������
    for i = 1:numTeeth
        if i == numTeeth
            toothPoints = sortedIdx(toothBoundaries(i):end); % ��ȡ���һ���ݵĵ�
        else
            toothPoints = sortedIdx(toothBoundaries(i):toothBoundaries(i+1)-1); % ��ȡ��ǰ�ݵĵ�
        end
        toothDistances = distances(toothPoints); % ��ȡ��ǰ�ݵľ���
        maxDist = max(toothDistances); % ������
        minDist = min(toothDistances); % ��С����
        if maxDist ~= 0 % ��ֹ������
            shrinkageRates(i) = (maxDist - minDist) / maxDist; % ����������
        else
            shrinkageRates(i) = 0;
        end
    end

    % ��ʾ������
    cla(ax2D); % �����
    plot(ax2D, 1:numTeeth, shrinkageRates, '-o'); % ��������������
    title(ax2D, '������뾶');
    xlabel(ax2D, '�ֳݱ��');
    ylabel(ax2D, '������');
end

function inverseDesign(fig, filepath)
    global ax2D;
    gearModel = stlread(filepath); % ��ȡ STL �ļ�
    disp(['���뷴�����ģ��: ', filepath]); % ��ʾ�����ģ��·��

    vertices = gearModel.Points; % ��ȡ����
    faces = gearModel.ConnectivityList; % ��ȡ�������б�

    % ���㼸������
    center = mean(vertices, 1); % ���㶥��ļ�������

    % ����߽��
    k = boundary(vertices(:,1), vertices(:,2), 0.9); % ����߽�

    % ����ÿ���㵽���ĵľ���
    distances = sqrt(sum((vertices(k,:) - center).^2, 2)); % ����߽�㵽���ĵľ���

    % �趨�Ŵ����
    scaleFactor = 1.005; % ����Ŵ����Ϊ 10%

    % ������ƷŴ����
    newVertices = vertices; % ��ʼ���¶���
    for i = 1:length(k)
        newVertices(k(i), :) = center + scaleFactor * (vertices(k(i), :) - center); % �������Ŵ�
    end

    % ͶӰ��ƽ�沢���Ƴ���
    cla(ax2D); % �����
    plot(ax2D, vertices(k,1), vertices(k,2), 'r.', 'MarkerSize', 10); % ����ԭʼ�߽��
    hold(ax2D, 'on');
    plot(ax2D, newVertices(k,1), newVertices(k,2), 'g.', 'MarkerSize', 10); % ���ƷŴ�߽��
    plot(ax2D, [newVertices(k,1); newVertices(k(1),1)], [newVertices(k,2); newVertices(k(1),2)], 'g-'); % ���ӷŴ�߽��
    title(ax2D, '���ֳ����������');
    xlabel(ax2D, 'X');
    ylabel(ax2D, 'Y');
    legend(ax2D, 'ԭʼ�߽��', '�Ŵ�߽��');
    hold(ax2D, 'off');
end
