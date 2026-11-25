function WirelessTuneUI
%WIRELESSTUNEUI Wireless tuning dashboard for the CQU robot.
%
%   MATLAB R2025b syntax target.
%   BLE: MATLAB ble API, BT27 service FFE0 / notify-write FFE1 / write FFE2.
%   Serial: a USB BLE-Central virtual COM port; NOT the BT27 UART pins.
%   Demo: synthetic 86-byte telemetry for UI testing.
%
%   Telemetry: 86 B, AA 55 | len=80 | ver=2 | LCD_Data_t | CRC16
%   Tune:      64 B, A5 5A | ver=1 | type=1 | seq | mode | count |
%               target1 | target2 | 8 x {id,float32} | reserved | CRC16

fig = uifigure('Name', 'CQU Robot Wireless Tune', ...
    'Position', [80 40 1280 850], 'Color', [0.96 0.97 0.99]);

app = struct();
app.fig = fig;
app.transport = 'BLE';
app.connected = false;
app.bleObj = [];
app.bleNotify = [];
app.bleWrite = [];
app.bleTable = table();
app.serialObj = [];
app.rxBuf = uint8([]);
app.seq = uint16(0);
app.txCount = 0;
app.rxCount = 0;
app.demoTime = 0;
app.demoTimer = [];
app.plotTimer = [];
app.selectedRows = [];
app.paramStatus = 0;
app.paramVersion = 0;
app.paramSeq = 0;
app.paramId = 0;
app.imuAge = 0;
app.logData = zeros(0, 13);
app.autoSaveEnabled = true;
app.autoFile = '';
app.autoBuf = zeros(0, 13);
app.autoLastFlush = tic;
app.excFig = [];
app.excTimer = [];
app.excActive = false;
app.excStart = 0;
app.excTick = 0;
app.excSign = 1;
app.excPrbsHold = 0;

app.t = zeros(0, 1);
app.targetSpeed = zeros(0, 1);
app.actualSpeed = zeros(0, 1);
app.targetPitch = zeros(0, 1);
app.actualPitch = zeros(0, 1);
app.targetYaw = zeros(0, 1);
app.actualYaw = zeros(0, 1);
app.outputLeft = zeros(0, 1);
app.outputRight = zeros(0, 1);

% ---------------------------------------------------------------- connection
connPanel = uipanel(fig, 'Title', 'Connection', ...
    'Position', [10 700 400 140], 'FontWeight', 'bold');

uilabel(connPanel, 'Text', 'Transport', 'Position', [10 92 70 22]);
app.transportDrop = uidropdown(connPanel, ...
    'Items', {'BLE', 'Serial', 'Demo'}, 'Value', 'BLE', ...
    'Position', [85 92 90 26], ...
    'ValueChangedFcn', @(~, ~) onTransportChanged());

app.scanBtn = uibutton(connPanel, 'Text', 'Scan BLE', ...
    'Position', [185 92 85 26], ...
    'ButtonPushedFcn', @(~, ~) scanBle());

app.deviceDrop = uidropdown(connPanel, ...
    'Items', {'BT27'}, 'Value', 'BT27', ...
    'Position', [10 58 260 26]);

app.refreshBtn = uibutton(connPanel, 'Text', 'Refresh', ...
    'Position', [280 58 100 26], ...
    'ButtonPushedFcn', @(~, ~) refreshDeviceList());

app.connectBtn = uibutton(connPanel, 'Text', 'Connect', ...
    'Position', [10 24 100 28], ...
    'ButtonPushedFcn', @(~, ~) connectTransport());

app.disconnectBtn = uibutton(connPanel, 'Text', 'Disconnect', ...
    'Position', [120 24 110 28], 'Enable', 'off', ...
    'ButtonPushedFcn', @(~, ~) disconnectTransport());

app.saveBtn = uibutton(connPanel, 'Text', 'Save Log', ...
    'Position', [240 24 100 28], ...
    'ButtonPushedFcn', @(~, ~) saveLog());

app.linkLbl = uilabel(connPanel, 'Text', 'Disconnected', ...
    'Position', [10 4 370 22], 'FontColor', [0.55 0.2 0.2]);

% -------------------------------------------------------------- parameters
paramPanel = uipanel(fig, 'Title', 'PID Parameters', ...
    'Position', [10 300 400 390], 'FontWeight', 'bold');

app.paramData = paramDefs();
app.paramTable = uitable(paramPanel, ...
    'Data', app.paramData, ...
    'ColumnName', {'ID', 'Name', 'Value', 'Min', 'Max', 'Unit'}, ...
    'ColumnEditable', [false false true false false false], ...
    'ColumnWidth', {32, 108, 68, 55, 55, 48}, ...
    'Position', [8 58 384 305], ...
    'CellEditCallback', @(src, evt) onParamEdit(src, evt), ...
    'CellSelectionCallback', @(src, evt) onParamSelect(src, evt));

uibutton(paramPanel, 'Text', 'Send Selected', ...
    'Position', [8 20 120 30], ...
    'ButtonPushedFcn', @(~, ~) sendSelectedParams());

uilabel(paramPanel, ...
    'Text', 'Select up to 8 rows. Car returns status/version, not param values.', ...
    'Position', [135 20 255 30], 'FontSize', 10, ...
    'FontColor', [0.35 0.35 0.35]);

app.autoSaveCheck = uicheckbox(paramPanel, ...
    'Text', 'Auto-save attitude CSV', ...
    'Value', true, ...
    'Position', [8 366 190 20], ...
    'ValueChangedFcn', @(~, ~) onAutoSaveChanged());

% ------------------------------------------------------------------ target
targetPanel = uipanel(fig, 'Title', 'Target / Commands', ...
    'Position', [10 100 400 190], 'FontWeight', 'bold');

uilabel(targetPanel, 'Text', 'Mode', 'Position', [10 128 45 22]);
app.modeDrop = uidropdown(targetPanel, ...
    'Items', {'STOP', 'POSITION', 'SPEED', 'ATTITUDE', 'NO TARGET'}, ...
    'ItemsData', [0 1 2 3 255], 'Value', 3, ...
    'Position', [55 128 100 26]);

uilabel(targetPanel, 'Text', 'target1', 'Position', [165 128 50 22]);
app.target1 = uieditfield(targetPanel, 'numeric', ...
    'Value', 0, 'Position', [215 128 70 26], 'Limits', [-5000 5000]);
uilabel(targetPanel, 'Text', 'target2', 'Position', [290 128 50 22]);
app.target2 = uieditfield(targetPanel, 'numeric', ...
    'Value', 0, 'Position', [340 128 50 26], 'Limits', [-5000 5000]);

uibutton(targetPanel, 'Text', 'Send Target', ...
    'Position', [10 88 110 30], ...
    'ButtonPushedFcn', @(~, ~) sendTargetOnly());
uibutton(targetPanel, 'Text', 'Hold ATT 0/0', ...
    'Position', [130 88 110 30], ...
    'ButtonPushedFcn', @(~, ~) holdAttitude());
uibutton(targetPanel, 'Text', 'Excitation...', ...
    'Position', [250 88 120 30], ...
    'ButtonPushedFcn', @(~, ~) openExcitationWindow());

uilabel(targetPanel, ...
    'Text', 'POSITION: mm / 0.1deg   SPEED: mm/s / 1deg/s   ATTITUDE: 0.1deg / 1deg/s', ...
    'Position', [10 58 380 24], 'FontSize', 10, ...
    'FontColor', [0.35 0.35 0.35]);
uilabel(targetPanel, ...
    'Text', 'Queue is latest-only. Send one command and wait for status/version.', ...
    'Position', [10 30 380 24], 'FontSize', 10, ...
    'FontColor', [0.55 0.2 0.2]);

% ------------------------------------------------------------------ status
statusPanel = uipanel(fig, 'Title', 'Status', ...
    'Position', [10 10 400 80], 'FontWeight', 'bold');

app.statusLbl = uilabel(statusPanel, ...
    'Text', 'PARAM status=-- version=-- seq=-- id=--', ...
    'Position', [10 34 380 22], 'FontSize', 11);
app.rxLbl = uilabel(statusPanel, ...
    'Text', 'RX=0  TX=0  buffer=0', ...
    'Position', [10 8 380 22], 'FontSize', 10, ...
    'FontColor', [0.35 0.35 0.35]);

% ------------------------------------------------------------------- plots
app.axSpeed = uiaxes(fig, 'Position', [430 600 840 220]);
title(app.axSpeed, 'Speed (mm/s)');
xlabel(app.axSpeed, 'sample');
ylabel(app.axSpeed, 'mm/s');
grid(app.axSpeed, 'on');
hold(app.axSpeed, 'on');
app.hSpeedT = plot(app.axSpeed, nan, nan, '--', 'LineWidth', 1.0);
app.hSpeedA = plot(app.axSpeed, nan, nan, '-', 'LineWidth', 1.2);
legend(app.axSpeed, {'target', 'actual'}, 'Location', 'best');
hold(app.axSpeed, 'off');

app.axAtt = uiaxes(fig, 'Position', [430 350 840 220]);
title(app.axAtt, 'Attitude (deg)');
xlabel(app.axAtt, 'sample');
ylabel(app.axAtt, 'deg');
grid(app.axAtt, 'on');
hold(app.axAtt, 'on');
app.hPitchT = plot(app.axAtt, nan, nan, '--', 'LineWidth', 1.0);
app.hPitchA = plot(app.axAtt, nan, nan, '-', 'LineWidth', 1.2);
app.hYawA = plot(app.axAtt, nan, nan, '-', 'LineWidth', 1.0);
legend(app.axAtt, {'pitch target', 'pitch actual', 'yaw actual'}, ...
    'Location', 'best');
hold(app.axAtt, 'off');

app.axOut = uiaxes(fig, 'Position', [430 60 840 250]);
title(app.axOut, 'Motor PWM');
xlabel(app.axOut, 'sample');
ylabel(app.axOut, 'PWM');
grid(app.axOut, 'on');
hold(app.axOut, 'on');
app.hOutL = plot(app.axOut, nan, nan, '-', 'LineWidth', 1.2);
app.hOutR = plot(app.axOut, nan, nan, '-', 'LineWidth', 1.2);
legend(app.axOut, {'left', 'right'}, 'Location', 'best');
hold(app.axOut, 'off');

app.plotTimer = timer('ExecutionMode', 'fixedRate', ...
    'Period', 0.1, 'TimerFcn', @(~, ~) updatePlots());
start(app.plotTimer);
fig.CloseRequestFcn = @(~, ~) onClose();
refreshDeviceList();

% __NESTED_FUNCTIONS__

    function onTransportChanged()
        if app.connected
            app.transportDrop.Value = app.transport;
            uialert(app.fig, 'Disconnect before changing transport.', 'Busy');
            return;
        end
        app.transport = char(app.transportDrop.Value);
        if strcmp(app.transport, 'BLE')
            app.scanBtn.Enable = 'on';
            refreshDeviceList();
        elseif strcmp(app.transport, 'Serial')
            app.scanBtn.Enable = 'off';
            refreshDeviceList();
        else
            app.scanBtn.Enable = 'off';
            app.deviceDrop.Items = {'demo'};
            app.deviceDrop.Value = 'demo';
        end
    end

    function refreshDeviceList()
        if strcmp(app.transport, 'Serial')
            ports = serialportlist("available");
            if isempty(ports)
                ports = {'(none)'};
            end
            app.deviceDrop.Items = cellstr(ports);
            app.deviceDrop.Value = app.deviceDrop.Items{1};
        elseif strcmp(app.transport, 'BLE')
            if isempty(app.bleTable) || height(app.bleTable) == 0
                app.deviceDrop.Items = {'BT27'};
                app.deviceDrop.Value = 'BT27';
            else
                items = strings(height(app.bleTable), 1);
                for k = 1:height(app.bleTable)
                    nm = string(app.bleTable.Name(k));
                    ad = string(app.bleTable.Address(k));
                    if strlength(nm) > 0
                        items(k) = nm;
                    else
                        items(k) = ad;
                    end
                end
                app.deviceDrop.Items = cellstr(items);
                app.deviceDrop.Value = app.deviceDrop.Items{1};
            end
        else
            app.deviceDrop.Items = {'demo'};
            app.deviceDrop.Value = 'demo';
        end
    end

    function scanBle()
        if ~strcmp(app.transport, 'BLE') || app.connected
            return;
        end
        try
            app.linkLbl.Text = 'Scanning BLE...';
            drawnow;
            app.bleTable = blelist('Timeout', 3);
            refreshDeviceList();
            if height(app.bleTable) == 0
                uialert(app.fig, 'No BLE device found. Check adapter/power.', 'BLE scan');
            else
                app.linkLbl.Text = sprintf('Found %d BLE device(s)', height(app.bleTable));
            end
        catch err
            uialert(app.fig, err.message, 'BLE scan failed');
            app.linkLbl.Text = 'BLE scan failed';
        end
    end

    function connectTransport()
        if app.connected
            return;
        end
        app.connectBtn.Enable = 'off';
        app.transport = char(app.transportDrop.Value);
        try
            if strcmp(app.transport, 'BLE')
                name = char(app.deviceDrop.Value);
                app.bleObj = ble(name);
                app.bleNotify = characteristic(app.bleObj, "FFE0", "FFE1");
                try
                    app.bleWrite = characteristic(app.bleObj, "FFE0", "FFE2");
                catch
                    app.bleWrite = app.bleNotify;
                end
                app.bleNotify.DataAvailableFcn = @(src, evt) onBleData(src, evt);
                subscribe(app.bleNotify);
            elseif strcmp(app.transport, 'Serial')
                port = char(app.deviceDrop.Value);
                if strcmp(port, '(none)')
                    error('No serial port selected.');
                end
                app.serialObj = serialport(port, 115200);
                configureCallback(app.serialObj, "byte", 1, ...
                    @(src, ~) onSerialData(src));
            else
                app.demoTime = 0;
                app.demoTimer = timer('ExecutionMode', 'fixedRate', ...
                    'Period', 0.02, 'TimerFcn', @(~, ~) demoTick());
                start(app.demoTimer);
            end
            app.connected = true;
            app.disconnectBtn.Enable = 'on';
            updateLinkLabel();
            startAutoSave();
        catch err
            app.connectBtn.Enable = 'on';
            uialert(app.fig, err.message, 'Connect failed');
            updateLinkLabel();
        end
    end

    function disconnectTransport()
        app.connected = false;
        stopExcitation();
        stopAutoSave();
        try
            if ~isempty(app.demoTimer)
                stop(app.demoTimer);
                delete(app.demoTimer);
                app.demoTimer = [];
            end
        catch
        end
        try
            if ~isempty(app.bleNotify)
                app.bleNotify.DataAvailableFcn = [];
                unsubscribe(app.bleNotify);
            end
            if ~isempty(app.bleObj)
                delete(app.bleObj);
            end
        catch
        end
        try
            if ~isempty(app.serialObj)
                configureCallback(app.serialObj, "off");
                delete(app.serialObj);
            end
        catch
        end
        app.bleObj = [];
        app.bleNotify = [];
        app.bleWrite = [];
        app.serialObj = [];
        app.connectBtn.Enable = 'on';
        app.disconnectBtn.Enable = 'off';
        updateLinkLabel();
    end

    function updateLinkLabel()
        if app.connected
            app.linkLbl.Text = sprintf('Connected: %s / %s', ...
                app.transport, app.deviceDrop.Value);
            app.linkLbl.FontColor = [0.1 0.45 0.1];
        else
            app.linkLbl.Text = 'Disconnected';
            app.linkLbl.FontColor = [0.55 0.2 0.2];
        end
    end

    function onBleData(src, evt)
        d = [];
        try
            d = read(src);
        catch
            try
                if isprop(evt, 'Data')
                    d = evt.Data;
                end
            catch
            end
        end
        if isempty(d)
            return;
        end
        if iscell(d)
            d = cell2mat(d);
        end
        if isstruct(d) && isfield(d, 'Data')
            d = d.Data;
        end
        onBytes(uint8(d(:)));
    end

    function onSerialData(src)
        n = src.NumBytesAvailable;
        if n > 0
            d = read(src, n, "uint8");
            onBytes(d);
        end
    end

    function sendFrame(frame)
        if ~app.connected
            uialert(app.fig, 'Connect first.', 'Not connected');
            return;
        end
        try
            if strcmp(app.transport, 'BLE')
                try
                    write(app.bleWrite, uint8(frame), "uint8", "withoutresponse");
                catch
                    write(app.bleWrite, uint8(frame), "uint8");
                end
            elseif strcmp(app.transport, 'Serial')
                write(app.serialObj, uint8(frame), "uint8");
            end
            app.txCount = app.txCount + 1;
            app.rxLbl.Text = sprintf('RX=%d  TX=%d  buffer=%d', ...
                app.rxCount, app.txCount, numel(app.rxBuf));
        catch err
            uialert(app.fig, err.message, 'Send failed');
        end
    end

    function onClose()
        try
            if ~isempty(app.plotTimer)
                stop(app.plotTimer);
                delete(app.plotTimer);
            end
        catch
        end
        disconnectTransport();
        if ~isempty(app.excFig) && isvalid(app.excFig)
            delete(app.excFig);
        end
        delete(app.fig);
    end

    function onBytes(data)
        app.rxBuf = [app.rxBuf; uint8(data(:))];
        while numel(app.rxBuf) >= 86
            idx = find(app.rxBuf(1:end-1) == 170 & ...
                       app.rxBuf(2:end) == 85, 1);
            if isempty(idx)
                app.rxBuf = uint8([]);
                break;
            end
            if idx > 1
                app.rxBuf = app.rxBuf(idx:end);
            end
            if numel(app.rxBuf) < 86
                break;
            end
            frame = app.rxBuf(1:86);
            crcRx = double(frame(85)) + 256 * double(frame(86));
            if (frame(3) ~= 80) || (frame(4) ~= 2) || ...
                    (crc16_ccitt(frame(1:84)) ~= crcRx)
                app.rxBuf = app.rxBuf(2:end);
                continue;
            end
            app.rxBuf = app.rxBuf(87:end);
            app.rxCount = app.rxCount + 1;
            handleFrame(frame);
        end
        if numel(app.rxBuf) > 4096
            app.rxBuf = app.rxBuf(end-2047:end);
        end
        app.rxLbl.Text = sprintf('RX=%d  TX=%d  buffer=%d', ...
            app.rxCount, app.txCount, numel(app.rxBuf));
    end

    function handleFrame(frame)
        p = frame(5:84);
        v = struct();
        v.seq = u32le(p(1:4));
        v.timestamp = u32le(p(5:8));
        v.imuAge = u32le(p(9:12));
        v.targetDistance = i32le(p(21:24));
        v.currentDistance = i32le(p(25:28));
        v.targetSpeed = i32le(p(29:32));
        v.actualSpeed = i32le(p(33:36));
        v.targetPitch = i16le(p(45:46)) / 10;
        v.actualPitch = i16le(p(47:48)) / 10;
        v.targetYaw = i16le(p(49:50)) / 10;
        v.actualYaw = i16le(p(51:52)) / 10;
        v.targetYawRate = i16le(p(53:54));
        v.actualYawRate = i16le(p(55:56));
        v.pitchRate = i16le(p(57:58)) / 10;
        v.outputLeft = i16le(p(59:60));
        v.outputRight = i16le(p(61:62));
        v.mode = double(p(65));
        v.imuValid = double(p(68));
        v.imuTimestamp = u32le(p(69:72));
        v.paramStatus = u16le(p(73:74));
        v.paramVersion = u16le(p(75:76));
        v.paramSeq = u16le(p(77:78));
        v.paramId = u16le(p(79:80));
        onTelemetry(v);
    end

    function onTelemetry(v)
        app.paramStatus = v.paramStatus;
        app.paramVersion = v.paramVersion;
        app.paramSeq = v.paramSeq;
        app.paramId = v.paramId;
        app.imuAge = v.imuAge;

        n = numel(app.t) + 1;
        app.t(n, 1) = n;
        app.targetSpeed(n, 1) = v.targetSpeed;
        app.actualSpeed(n, 1) = v.actualSpeed;
        app.targetPitch(n, 1) = v.targetPitch;
        app.actualPitch(n, 1) = v.actualPitch;
        app.targetYaw(n, 1) = v.targetYaw;
        app.actualYaw(n, 1) = v.actualYaw;
        app.outputLeft(n, 1) = v.outputLeft;
        app.outputRight(n, 1) = v.outputRight;
        row = [v.timestamp, v.targetDistance, v.targetSpeed, v.actualSpeed, ...
            v.targetPitch, v.actualPitch, v.targetYaw, v.actualYaw, ...
            v.outputLeft, v.outputRight, v.paramStatus, v.paramVersion, v.mode];
        app.logData(end+1, :) = row;
        if app.autoSaveEnabled && ~isempty(app.autoFile)
            app.autoBuf(end+1, :) = row;
        end

        if numel(app.t) > 1200
            cut = numel(app.t) - 1200;
            app.t(1:cut) = [];
            app.targetSpeed(1:cut) = [];
            app.actualSpeed(1:cut) = [];
            app.targetPitch(1:cut) = [];
            app.actualPitch(1:cut) = [];
            app.targetYaw(1:cut) = [];
            app.actualYaw(1:cut) = [];
            app.outputLeft(1:cut) = [];
            app.outputRight(1:cut) = [];
        end
        updateStatus();
    end

    function updateStatus()
        app.statusLbl.Text = sprintf( ...
            'PARAM %s (0x%02X)  ver=%d  seq=%d  id=%d  IMU age=%d ms', ...
            statusName(app.paramStatus), app.paramStatus, ...
            app.paramVersion, app.paramSeq, app.paramId, app.imuAge);
        app.rxLbl.Text = sprintf('RX=%d  TX=%d  buffer=%d', ...
            app.rxCount, app.txCount, numel(app.rxBuf));
    end

    function updatePlots()
        flushAutoSave();
        if isempty(app.t)
            return;
        end
        set(app.hSpeedT, 'XData', app.t, 'YData', app.targetSpeed);
        set(app.hSpeedA, 'XData', app.t, 'YData', app.actualSpeed);
        set(app.hPitchT, 'XData', app.t, 'YData', app.targetPitch);
        set(app.hPitchA, 'XData', app.t, 'YData', app.actualPitch);
        set(app.hYawA, 'XData', app.t, 'YData', app.actualYaw);
        set(app.hOutL, 'XData', app.t, 'YData', app.outputLeft);
        set(app.hOutR, 'XData', app.t, 'YData', app.outputRight);
        drawnow limitrate;
    end

    function demoTick()
        app.demoTime = app.demoTime + 0.02;
        t = app.demoTime;
        targetSpeed = 180 * sin(2 * pi * 0.20 * t);
        actualSpeed = 0.85 * targetSpeed + 10 * sin(2 * pi * 0.6 * t);
        targetPitch = 4 * sin(2 * pi * 0.10 * t);
        actualPitch = 0.9 * targetPitch + 0.4 * sin(2 * pi * 1.1 * t);
        targetYaw = 15 * sin(2 * pi * 0.08 * t);
        actualYaw = 0.92 * targetYaw + 1.0 * sin(2 * pi * 0.7 * t);
        outL = 1200 * sin(2 * pi * 0.20 * t);
        outR = 0.9 * outL;
        onBytes(buildDemoFrame(t, targetSpeed, actualSpeed, ...
            targetPitch, actualPitch, targetYaw, actualYaw, outL, outR));
    end

    function frame = buildDemoFrame(t, ts, as, tp, ap, ty, ay, ol, or)
        p = zeros(1, 80, 'uint8');
        p = put_u32(p, 1, round(t * 1000));
        p = put_u32(p, 5, round(t * 1000));
        p = put_u32(p, 9, 1);
        p = put_i32(p, 21, 0);
        p = put_i32(p, 25, 0);
        p = put_i32(p, 29, round(ts));
        p = put_i32(p, 33, round(as));
        p = put_i16(p, 45, round(tp * 10));
        p = put_i16(p, 47, round(ap * 10));
        p = put_i16(p, 49, round(ty * 10));
        p = put_i16(p, 51, round(ay * 10));
        p = put_i16(p, 53, 0);
        p = put_i16(p, 55, 0);
        p = put_i16(p, 57, 0);
        p = put_i16(p, 59, round(ol));
        p = put_i16(p, 61, round(or));
        p = put_u16(p, 63, 120);
        p(65) = 3;
        p(66) = 0;
        p(67) = 1;
        p(68) = 1;
        p = put_u32(p, 69, round(t * 1000));
        p = put_u16(p, 73, 0);
        p = put_u16(p, 75, 1);
        p = put_u16(p, 77, 0);
        p = put_u16(p, 79, 0);
        frame = zeros(1, 86, 'uint8');
        frame(1:4) = uint8([170 85 80 2]);
        frame(5:84) = p;
        frame(85:86) = uint8([mod(crc16_ccitt(frame(1:84)), 256), ...
            floor(crc16_ccitt(frame(1:84)) / 256)]);
    end

    function onParamEdit(src, evt)
        if isempty(evt.Indices)
            return;
        end
        row = evt.Indices(1);
        col = evt.Indices(2);
        if col ~= 3
            return;
        end
        val = evt.NewData;
        lo = src.Data{row, 4};
        hi = src.Data{row, 5};
        if ~isnumeric(val) || ~isscalar(val) || ~isfinite(val) || ...
                val < lo || val > hi
            src.Data{row, 3} = evt.PreviousData;
            uialert(app.fig, sprintf('Value must be in [%g, %g].', lo, hi), ...
                'Invalid parameter');
        end
    end

    function onParamSelect(~, evt)
        if isempty(evt.Indices)
            app.selectedRows = [];
        else
            app.selectedRows = unique(evt.Indices(:, 1));
        end
    end

    function sendSelectedParams()
        rows = app.selectedRows;
        if isempty(rows)
            uialert(app.fig, 'Select at least one parameter row.', 'No selection');
            return;
        end
        if numel(rows) > 8
            uialert(app.fig, 'At most 8 parameters per frame. Select fewer rows.', ...
                'Too many parameters');
            return;
        end
        params = zeros(numel(rows), 2);
        for k = 1:numel(rows)
            params(k, 1) = app.paramTable.Data{rows(k), 1};
            params(k, 2) = app.paramTable.Data{rows(k), 3};
        end
        sendTuneFrame(app.modeDrop.Value, app.target1.Value, ...
            app.target2.Value, params);
    end

    function sendTargetOnly()
        sendTuneFrame(app.modeDrop.Value, app.target1.Value, ...
            app.target2.Value, zeros(0, 2));
    end

    function holdAttitude()
        app.modeDrop.Value = 3;
        app.target1.Value = 0;
        app.target2.Value = 0;
        sendTuneFrame(3, 0, 0, zeros(0, 2));
    end

    function sendTuneFrame(mode, target1, target2, params)
        if ~app.connected
            uialert(app.fig, 'Connect first.', 'Not connected');
            return;
        end
        app.seq = uint16(mod(double(app.seq) + 1, 65536));
        frame = buildTuneFrame(double(app.seq), mode, target1, target2, params);
        sendFrame(frame);
        app.statusLbl.Text = sprintf('TX seq=%d  mode=%d  params=%d', ...
            double(app.seq), mode, size(params, 1));
    end

    function saveLog()
        if isempty(app.logData)
            uialert(app.fig, 'No telemetry recorded.', 'Empty log');
            return;
        end
        [file, path] = uiputfile( ...
            {'*.mat', 'MAT file'; '*.csv', 'CSV file'}, ...
            'Save telemetry log', 'robot_log.mat');
        if isequal(file, 0)
            return;
        end
        data = app.logData;
        full = fullfile(path, file);
        try
            if endsWith(lower(file), '.csv')
                fid = fopen(full, 'w');
                if fid >= 0
                    fprintf(fid, '%s\n', strjoin(autoHeader(), ','));
                    fclose(fid);
                end
                writematrix(data, full, 'WriteMode', 'append');
            else
                save(full, 'data');
            end
        catch err
            uialert(app.fig, err.message, 'Save failed');
        end
    end

    function onAutoSaveChanged()
        app.autoSaveEnabled = app.autoSaveCheck.Value;
        if app.connected
            if app.autoSaveEnabled
                startAutoSave();
            else
                stopAutoSave();
            end
        end
    end

    function startAutoSave()
        if ~app.autoSaveEnabled || ~app.connected || ~isempty(app.autoFile)
            return;
        end
        folder = fullfile(fileparts(mfilename('fullpath')), 'logs');
        if ~exist(folder, 'dir')
            mkdir(folder);
        end
        stamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
        app.autoFile = fullfile(folder, ['attitude_' stamp '.csv']);
        fid = fopen(app.autoFile, 'w');
        if fid < 0
            app.autoFile = '';
            uialert(app.fig, 'Could not create auto-save file.', 'Auto-save');
            return;
        end
        fprintf(fid, '%s\n', strjoin(autoHeader(), ','));
        fclose(fid);
        app.autoBuf = zeros(0, 13);
        app.autoLastFlush = tic;
    end

    function stopAutoSave()
        flushAutoSave();
        app.autoFile = '';
        app.autoBuf = zeros(0, 13);
    end

    function flushAutoSave()
        if isempty(app.autoFile) || isempty(app.autoBuf)
            return;
        end
        if (toc(app.autoLastFlush) < 1.0) && (size(app.autoBuf, 1) < 20)
            return;
        end
        try
            writematrix(app.autoBuf, app.autoFile, 'WriteMode', 'append');
            app.autoBuf = zeros(0, 13);
            app.autoLastFlush = tic;
        catch err
            app.autoSaveCheck.Value = false;
            app.autoSaveEnabled = false;
            uialert(app.fig, err.message, 'Auto-save failed');
        end
    end

    function h = autoHeader()
        h = {'timestamp_ms', 'target_distance_mm', 'target_speed_mm_s', ...
            'actual_speed_mm_s', 'target_pitch_deg', 'actual_pitch_deg', ...
            'target_yaw_deg', 'actual_yaw_deg', 'output_left_pwm', ...
            'output_right_pwm', 'param_status', 'param_version', 'mode'};
    end

    function openExcitationWindow()
        if ~isempty(app.excFig) && isvalid(app.excFig)
            return;
        end
        app.excFig = uifigure('Name', 'Target Excitation', ...
            'Position', [360 220 460 370], ...
            'Color', [0.96 0.97 0.99]);

        uilabel(app.excFig, 'Text', 'Mode', 'Position', [10 305 50 22]);
        app.excMode = uidropdown(app.excFig, ...
            'Items', {'ATTITUDE', 'SPEED', 'POSITION'}, ...
            'ItemsData', [3 2 1], 'Value', 3, ...
            'Position', [60 305 110 26]);

        uilabel(app.excFig, 'Text', 'Signal', 'Position', [190 305 50 22]);
        app.excSignal = uidropdown(app.excFig, ...
            'Items', {'PRBS', 'Sine', 'Square', 'Step'}, ...
            'Value', 'PRBS', 'Position', [240 305 110 26]);

        uilabel(app.excFig, 'Text', 'Amplitude', 'Position', [10 265 65 22]);
        app.excAmp = uieditfield(app.excFig, 'numeric', ...
            'Value', 50, 'Limits', [-5000 5000], ...
            'Position', [80 265 70 26]);

        uilabel(app.excFig, 'Text', 'Offset', 'Position', [160 265 45 22]);
        app.excOffset = uieditfield(app.excFig, 'numeric', ...
            'Value', 0, 'Limits', [-5000 5000], ...
            'Position', [205 265 70 26]);

        uilabel(app.excFig, 'Text', 'Freq Hz', 'Position', [285 265 50 22]);
        app.excFreq = uieditfield(app.excFig, 'numeric', ...
            'Value', 1, 'Limits', [0.05 5], ...
            'Position', [335 265 70 26]);

        uilabel(app.excFig, 'Text', 'Duration s', 'Position', [10 225 65 22]);
        app.excDuration = uieditfield(app.excFig, 'numeric', ...
            'Value', 10, 'Limits', [0.5 30], ...
            'Position', [80 225 70 26]);

        app.excStartBtn = uibutton(app.excFig, 'Text', 'Start', ...
            'Position', [10 185 100 30], ...
            'ButtonPushedFcn', @(~, ~) startExcitation());
        app.excStopBtn = uibutton(app.excFig, 'Text', 'Stop', ...
            'Position', [120 185 100 30], 'Enable', 'off', ...
            'ButtonPushedFcn', @(~, ~) stopExcitation());

        app.excStatusLbl = uilabel(app.excFig, ...
            'Text', 'Idle', 'Position', [10 150 430 22], ...
            'FontColor', [0.35 0.35 0.35]);

        uilabel(app.excFig, ...
            'Text', 'Target input rate = 10 Hz. ATTITUDE amp: 0.1deg; SPEED amp: mm/s; POSITION amp: mm.', ...
            'Position', [10 120 430 22], 'FontSize', 10);
        uilabel(app.excFig, ...
            'Text', 'Only run in DEBUG, stationary/supported. Stop sends a zero target in the same mode.', ...
            'Position', [10 95 430 22], 'FontSize', 10, ...
            'FontColor', [0.55 0.2 0.2]);

        app.excFig.CloseRequestFcn = @(~, ~) closeExcitationWindow();
    end

    function startExcitation()
        if ~app.connected
            uialert(app.fig, 'Connect first.', 'Not connected');
            return;
        end
        if app.excActive
            return;
        end
        app.excActive = true;
        app.excStart = tic;
        app.excTick = 0;
        app.excSign = 1;
        app.excPrbsHold = 0;
        app.excStartBtn.Enable = 'off';
        app.excStopBtn.Enable = 'on';
        app.excTimer = timer('ExecutionMode', 'fixedRate', ...
            'Period', 0.1, 'TimerFcn', @(~, ~) excitationTick());
        start(app.excTimer);
        app.excStatusLbl.Text = 'Running...';
    end

    function excitationTick()
        if ~app.excActive
            return;
        end
        t = toc(app.excStart);
        if t >= app.excDuration.Value
            stopExcitation();
            return;
        end

        mode = app.excMode.Value;
        amp = app.excAmp.Value;
        offset = app.excOffset.Value;
        freq = app.excFreq.Value;
        signal = char(app.excSignal.Value);

        switch signal
            case 'PRBS'
                if app.excPrbsHold <= 0
                    app.excSign = 2 * randi([0 1]) - 1;
                    app.excPrbsHold = max(1, round(1 / freq / 0.1)) - 1;
                else
                    app.excPrbsHold = app.excPrbsHold - 1;
                end
                s = app.excSign;
            case 'Sine'
                s = sin(2 * pi * freq * t);
            case 'Square'
                s = sign(sin(2 * pi * freq * t));
            otherwise % Step
                s = 1;
        end

        target = offset + amp * s;
        if mode == 3
            target = min(max(target, -300), 300);
        elseif mode == 2
            target = min(max(target, -2000), 2000);
        else
            target = min(max(target, -5000), 5000);
        end

        sendTuneFrame(mode, round(target), 0, zeros(0, 2));
        app.excStatusLbl.Text = sprintf('t=%.2fs  mode=%d  target1=%d', ...
            t, mode, round(target));
    end

    function stopExcitation()
        if ~isempty(app.excTimer)
            try
                stop(app.excTimer);
                delete(app.excTimer);
            catch
            end
            app.excTimer = [];
        end
        app.excActive = false;
        if app.connected && ~isempty(app.excFig) && isvalid(app.excFig)
            sendTuneFrame(app.excMode.Value, 0, 0, zeros(0, 2));
            app.excStatusLbl.Text = 'Stopped; zero target sent.';
        end
        if ~isempty(app.excFig) && isvalid(app.excFig)
            app.excStartBtn.Enable = 'on';
            app.excStopBtn.Enable = 'off';
        end
    end

    function closeExcitationWindow()
        stopExcitation();
        if ~isempty(app.excFig) && isvalid(app.excFig)
            delete(app.excFig);
        end
        app.excFig = [];
    end

    function d = paramDefs()
        d = {
            1,  'kp_distance',      0.0,    0.0,   100.0,  '1/s';
            2,  'kd_distance',      0.0,    0.0,    50.0,  '-';
            3,  'max_speed',        0.5,    0.05,    1.0,  'm/s';
            4,  'kp_speed',         0.0,    0.0,   500.0,  'deg/(m/s)';
            5,  'kd_speed',         0.0,    0.0,   100.0,  '-';
            6,  'max_pitch',       10.0,    1.0,    30.0,  'deg';
            7,  'kp_angle',         0.0,    0.0,   100.0,  '1/s';
            8,  'kd_angle',         0.0,    0.0,    50.0,  's';
            9,  'max_rate',        180.0,   10.0,   180.0,  'deg/s';
            10, 'kp_pitch',         0.0,    0.0, 10000.0,  'PWM/deg';
            11, 'kd_pitch',         0.0,    0.0,  5000.0,  'PWM/(deg/s)';
            12, 'kp_yaw_rate',      0.0,    0.0, 10000.0,  'PWM/(deg/s)';
            13, 'ki_yaw_rate',      0.0,    0.0,  5000.0,  'PWM/(deg/s)';
            14, 'max_pwm',       4199.0,    0.0,  4199.0,  'PWM';
            15, 'max_diff_pwm',  4199.0,    0.0,  4199.0,  'PWM';
            16, 'max_yaw_integral', 4199.0, 0.0, 4199.0,  'PWM'
        };
    end

    function f = buildTuneFrame(seq, mode, target1, target2, params)
        f = zeros(1, 64, 'uint8');
        f(1) = 165;
        f(2) = 90;
        f(3) = 1;
        f(4) = 1;
        f = put_u16(f, 5, seq);
        f(7) = uint8(mode);
        f(8) = uint8(size(params, 1));
        f = put_i16(f, 9, round(target1));
        f = put_i16(f, 11, round(target2));
        for k = 1:size(params, 1)
            base = 13 + 5 * (k - 1);
            f(base) = uint8(params(k, 1));
            f = put_f32(f, base + 1, params(k, 2));
        end
        c = crc16_ccitt(f(1:62));
        f(63) = uint8(mod(c, 256));
        f(64) = uint8(floor(c / 256));
    end

    function f = put_u16(f, idx, val)
        b = typecast(uint16(val), 'uint8');
        f(idx:idx+1) = b;
    end

    function f = put_i16(f, idx, val)
        b = typecast(int16(val), 'uint8');
        f(idx:idx+1) = b;
    end

    function f = put_u32(f, idx, val)
        b = typecast(uint32(val), 'uint8');
        f(idx:idx+3) = b;
    end

    function f = put_i32(f, idx, val)
        b = typecast(int32(val), 'uint8');
        f(idx:idx+3) = b;
    end

    function f = put_f32(f, idx, val)
        b = typecast(single(val), 'uint8');
        f(idx:idx+3) = b;
    end

    function v = u16le(b)
        b = double(b);
        v = b(1) + 256 * b(2);
    end

    function v = i16le(b)
        b = double(b);
        v = b(1) + 256 * b(2);
        if v >= 32768
            v = v - 65536;
        end
    end

    function v = u32le(b)
        b = double(b);
        v = b(1) + 256 * b(2) + 65536 * b(3) + 16777216 * b(4);
    end

    function v = i32le(b)
        v = u32le(b);
        if v >= 2147483648
            v = v - 4294967296;
        end
    end

    function c = crc16_ccitt(data)
        c = uint16(65535);
        for byte = uint8(data(:))'
            c = bitxor(c, bitshift(uint16(byte), 8));
            for k = 1:8
                if bitand(c, uint16(32768)) ~= 0
                    c = bitxor(bitshift(c, 1), uint16(4129));
                else
                    c = bitshift(c, 1);
                end
            end
        end
        c = double(c);
    end

    function s = statusName(code)
        names = {'OK', 'BAD_SYNC', 'BAD_VERSION', 'BAD_TYPE', ...
            'BAD_COUNT', 'BAD_CRC', 'BAD_MODE', 'BAD_TARGET', ...
            'BAD_PARAM_ID', 'BAD_PARAM_RANGE', 'BAD_STATE', 'QUEUE_BUSY'};
        if code >= 0 && code < numel(names)
            s = names{code + 1};
        else
            s = 'UNKNOWN';
        end
    end

% __NESTED_FUNCTIONS__
end
