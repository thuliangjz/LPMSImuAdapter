classdef Visualizer < handle
    properties (SetAccess = private)
        accWindowTimespan (1, 1) double
        imuCnt (1, 1) double

        poseBoxSizeX (1, 1) double
        poseBoxSizeY (1, 1) double
        poseBoxSizeZ (1, 1) double

        adapter
        content
    end

    properties (Access = private)
        cTimestamp
        cAcc
        cQuat

        mpName2Function
        idxColSubplot

        cTimestampFull
        cAccFull
        cQuatFull
        cMachineTimestampFull

        accWorldCols
        figHandle
    end

    methods
        %content is a cell array containing keys of mpName2Function
        function obj = Visualizer(imuCnt, content)
            obj.adapter = clib.LPMSAdapterLib.LPMSAdapter(imuCnt);
            obj.imuCnt = imuCnt;
        
            obj.poseBoxSizeX = 2;
            obj.poseBoxSizeY = 3;
            obj.poseBoxSizeZ = 1;
            obj.accWindowTimespan = 30;

            obj.cTimestamp = cell(1, imuCnt);
            obj.cAcc = cell(1, imuCnt);
            obj.cQuat = cell(1, imuCnt);

            obj.cTimestampFull = cell(1, imuCnt);
            obj.cAccFull = cell(1, imuCnt);
            obj.cQuatFull = cell(1, imuCnt);
            obj.cMachineTimestampFull= cell(1, imuCnt);

            obj.mpName2Function = containers.Map({'acc', 'accw', 'pose', 'accn'}, {@obj.PlotAcc, @obj.PlotAccWorld, @obj.PlotPoseBox, @obj.PlotAccNorm});
            obj.content = content;

            obj.cTimestamp = cell(1, imuCnt);
            obj.cAcc = cell(1, imuCnt);
            obj.cQuat = cell(1, imuCnt);

            for i=1:length(content)
                if ~obj.mpName2Function.isKey(content{i})
                    error('content key error')
                end
            end

            obj.accWorldCols = [1, 2, 3];
        end

        function SetAccWorldColToPlot(obj, col)
            obj.accWorldCols = col;
        end

        function Save(obj, name)
            imuData = {obj.cTimestampFull, obj.cAccFull, obj.cQuatFull, obj.cMachineTimestampFull};
            save(name, 'imuData');
        end
        function SetPlotFigureHandle(obj, figHandle)
            obj.figHandle = figHandle;
        end

        function StartAdapter(obj)
            obj.adapter.start();
        end

        %time is a uint64 stamp from clib.LPMSAdapterLib.getMachineTimestamp
        function PlotBeforeTime(obj, time)
            timeoutCnt = 0;
            while true
                try 
                    imuDataList = obj.adapter.getImuBeforeTime(time);
                    break
                catch
                    if strcmp(obj.adapter.getLastError(), "QueueTimeout")
                        timeoutCnt = timeoutCnt + 1;
                        %if timeoutCnt > 3
                        %    error("too much reading timout")
                        %end
                        continue
                    end
                    error(obj.adapter.getLastError())
                end
            end
            colAcc = 3; colTimestamp = 1; colQuat = 4;
            for i = 1:obj.imuCnt
                imuIdx = i-1;
                row = imuDataList.getRecordCnt(imuIdx);
                vAcc = double(imuDataList.retrieveFloatData(imuIdx, 'a', row, colAcc));
                vTimestamp = imuDataList.retrieveDoubleData(imuIdx, 'timestamp', row, colTimestamp);
                vQuat = double(imuDataList.retrieveFloatData(imuIdx, 'q', row, colQuat));
                vMachineTimestamp = uint64(imuDataList.retrieveUint64Data(imuIdx, 'machine_timestamp', row, colTimestamp));
                
                obj.cTimestamp{i} = cat(1, obj.cTimestamp{i}, vTimestamp);
                obj.cAcc{i} = cat(1, obj.cAcc{i}, vAcc);
                obj.cQuat{i} = cat(1, obj.cQuat{i}, vQuat);
                
                obj.cTimestampFull{i} = cat(1, obj.cTimestampFull{i}, vTimestamp);
                obj.cAccFull{i} = cat(1, obj.cAccFull{i}, vAcc);
                obj.cQuatFull{i} = cat(1, obj.cQuatFull{i}, vQuat);
                obj.cMachineTimestampFull{i} = cat(1, obj.cMachineTimestampFull{i}, vMachineTimestamp);

                if ~isempty(obj.cTimestamp{i})
                    if obj.cTimestamp{i}(end) - obj.cTimestamp{i}(1) > obj.accWindowTimespan
                        idxFirst = findFirstIndexGreaterThan(obj.cTimestamp{i}(end) - obj.accWindowTimespan, obj.cTimestamp{i});
                        obj.cTimestamp{i} = obj.cTimestamp{i}(idxFirst:end);
                        obj.cAcc{i} = obj.cAcc{i}(idxFirst:end, :);
                        obj.cQuat{i} = obj.cQuat{i}(idxFirst:end, :);
                    end
                end
            end
            obj.idxColSubplot = 1;
            oldFig = gcf;
            figure(obj.figHandle)
            for i=1:length(obj.content)
                fcn = obj.mpName2Function(obj.content{i});
                fcn();
            end
            drawnow;
            figure(oldFig)
        end
    
        function PlotForever(obj)
            obj.adapter.start();
            while true
                currentMachineTime = clib.LPMSAdapterLib.getMachineTimestamp();
                obj.PlotBeforeTime(currentMachineTime);
            end
        end
    end

    methods (Access = private)
        function PlotAcc(obj)
            nPlot = length(obj.content);
            for i=1:obj.imuCnt
                if isempty(obj.cTimestamp{i})
                    continue
                end
                subplot(obj.imuCnt, nPlot, (i-1)*nPlot + obj.idxColSubplot);
                obj.idxColSubplot = obj.idxColSubplot + 1;
                if obj.idxColSubplot > nPlot
                    obj.idxColSubplot = 1;
                end
                plot(obj.cTimestamp{i}, obj.cAcc{i});
            end
        end

        function PlotAccNorm(obj)
            nPlot = length(obj.content);
            for i=1:obj.imuCnt
                if isempty(obj.cTimestamp{i})
                    continue
                end
                subplot(obj.imuCnt, nPlot, (i-1)*nPlot + obj.idxColSubplot);
                obj.idxColSubplot = obj.idxColSubplot + 1;
                if obj.idxColSubplot > nPlot
                    obj.idxColSubplot = 1;
                end
                plot(obj.cTimestamp{i}, vecnorm(obj.cAcc{i}, 2, 2));
            end
        end

        function PlotPoseBox(obj)
            nPlot = length(obj.content);
            for i=1:obj.imuCnt
                if isempty(obj.cTimestamp{i})
                    continue
                end
                subplot(obj.imuCnt, nPlot, (i-1)*nPlot + obj.idxColSubplot);
                obj.idxColSubplot = obj.idxColSubplot + 1;
                if obj.idxColSubplot > nPlot
                    obj.idxColSubplot = 1;
                end
                q = obj.cQuat{i}(end, :);
                [P, axisXYZ] = getRotatedCubeData(obj.poseBoxSizeX, obj.poseBoxSizeY, obj.poseBoxSizeZ, q);
                plot3(P(:, 1), P(:, 2), P(:, 3));
                hold on
                plot3([0, axisXYZ(1, 1)], [0, axisXYZ(1, 2)], [0, axisXYZ(1, 3)],'r-');
                plot3([0, axisXYZ(2, 1)], [0, axisXYZ(2, 2)], [0, axisXYZ(2, 3)], 'g-');
                plot3([0, axisXYZ(3, 1)], [0, axisXYZ(3, 2)], [0, axisXYZ(3, 3)], 'b-');
                hold off
                axis equal
                grid on
                xlabel('x'); ylabel('y'); zlabel('z')
                maxLen = norm([obj.poseBoxSizeX, obj.poseBoxSizeY, obj.poseBoxSizeZ]) / 2;
                xlim([-maxLen, maxLen]);ylim([-maxLen, maxLen]); zlim([-maxLen, maxLen]);
            end
        end

        function PlotAccWorld(obj)
            nPlot = length(obj.content);
            for i=1:obj.imuCnt
                if isempty(obj.cTimestamp{i})
                    continue
                end
                vAccWorld = zeros(size(obj.cAcc{i}, 1), 3);
                for j=1:size(obj.cAcc{i}, 1)
                    vAccWorld(j, :) = obj.cAcc{i}(j, :) * quat2rotm(obj.cQuat{i}(j, :)) + [0, 0, 1];
                end
                subplot(obj.imuCnt, nPlot, (i-1)*nPlot + obj.idxColSubplot);
                obj.idxColSubplot = obj.idxColSubplot + 1;
                if obj.idxColSubplot > nPlot
                    obj.idxColSubplot = 1;
                end
                plot(obj.cTimestamp{i}, vAccWorld(:, obj.accWorldCols));
            end
        end
    end
end


function idx = findFirstIndexGreaterThan(a, v)
    idx = 0;
    for i=1:length(v)
        if v(i) >= a
            idx = i;
            break
        end
    end
end

function [P, axisXYZ]= getRotatedCubeData(sizeX, sizeY, sizeZ, q)
    A = [0 0 0];
    B = [1 0 0];
    C = [0 1 0];
    D = [0 0 1];
    E = [0 1 1];
    F = [1 0 1];
    G = [1 1 0];
    H = [1 1 1];
    P = [A;B;F;H;G;C;A;D;E;H;F;D;E;C;G;B];
    vCenter = (A + B + C + D + E + F + G + H) / 8;
    P = P - vCenter;
    P(:, 1) = P(:, 1) * sizeX;
    P(:, 2) = P(:, 2) * sizeY;
    P(:, 3) = P(:, 3) * sizeZ;
    P = P * quat2rotm(q);
    axisXYZ = [sizeX, 0, 0; 0, sizeY, 0; 0, 0, sizeZ];
    axisXYZ = axisXYZ * quat2rotm(q);
end
