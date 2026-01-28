function [time, theta1x, theta1y, theta1z, theta2, torque_elbow, torque_wristy] = Animation_3D(pinput, param)
% ANIMATION_3D - run 3D sim and show animation + useful plots
% Robust extraction of Simulink outputs (handles timeseries, numeric, struct(signals.values), cell)
%
% pinput layout used here:
% pinput = [ tswing; lc1; lc2; theta1x_waypoints(1:n); theta1y_waypoints(1:n); theta2_waypoints(1:n) ]
%
% Outputs: time, theta1x, theta1y, theta1z, theta2, torque_elbow, torque_wristy

    %% --- unpack inputs ------------------------------------------------
    n       = param.ngrid;
    simTime = pinput(1);
    lc1     = pinput(2);
    lc2     = pinput(3);

    theta1x_in = pinput(4 : 3 + n);
    theta1y_in = pinput(4 + n : 3 + 2*n);
    theta2_in  = pinput(4 + 2*n : 3 + 3*n);

    timeVec = linspace(0, simTime, n)';

    %% --- push inputs/params to base for Simulink ----------------------
    assignin('base','timeVec', timeVec);
    assignin('base','theta1x_in', theta1x_in(:));
    assignin('base','theta1y_in', theta1y_in(:));
    assignin('base','theta2_in',  theta2_in(:));
    assignin('base','lc1', lc1); assignin('base','lc2', lc2);
    assignin('base','simTime', simTime);

    % push all param fields too (avoids missing-variable errors)
    fn = fieldnames(param);
    for ii = 1:numel(fn)
        assignin('base', fn{ii}, param.(fn{ii}));
    end

    %% --- run Simulink -------------------------------------------------
    mdl = 'PID_PFL_3D_winds';   % <-- change this if your model name differs
    set_param(mdl,'StopTime', num2str(simTime));
    simOut = sim(mdl, 'ReturnWorkspaceOutputs', 'on', 'SrcWorkspace', 'base');

    %% --- robust extractor helper -------------------------------------
    function v = extract_signal(s)
        % Converts many Simulink "to workspace" formats into a numeric column vector.
        % Returns empty [] if nothing sensible found.
        v = [];
        if isempty(s)
            return;
        end

        % If simOut.get returned a cell (sometimes happens), unwrap single cell
        if iscell(s)
            if numel(s) == 1
                s = s{1};
            else
                % multiple cells -> try concatenating numeric cells
                try
                    vcat = cellfun(@(c) double(c(:)), s, 'UniformOutput', false);
                    v = vertcat(vcat{:});
                    return;
                catch
                    v = [];
                    return;
                end
            end
        end

        % timeseries
        if isa(s, 'timeseries')
            try
                v = double(s.Data(:));
                return;
            catch
                v = [];
                return;
            end
        end

        % numeric array
        if isnumeric(s)
            try
                v = double(s(:));
                return;
            catch
                v = [];
                return;
            end
        end

        % common structure returned from "To Workspace" configured as Structure with time/signals
        if isstruct(s)
            % Case: s has fields 'signals' -> s.signals.values (older style)
            if isfield(s, 'signals') && isfield(s.signals, 'values')
                try
                    v = double(s.signals.values(:));
                    return;
                catch
                    v = [];
                    return;
                end
            end
            % Case: s has fields 'time' and 'signals' (Simulink logging struct)
            if isfield(s, 'time') && isfield(s,'signals') && isfield(s.signals,'values')
                try
                    v = double(s.signals.values(:));
                    return;
                catch
                    v = [];
                    return;
                end
            end
            % Case: timeseries inside struct
            if isfield(s, 'Data') && isa(s.Data, 'double')
                try
                    v = double(s.Data(:));
                    return;
                catch
                    v = [];
                    return;
                end
            end
        end

        % fallback: try to coerce to double
        try
            v_try = double(s(:));
            v = v_try(:);
            return;
        catch
            v = [];
            return;
        end
    end

    %% --- fetch and extract signals ----------------------------------
    theta1x = extract_signal( try_get_and_rethrow(simOut,'theta1x') );
    theta1y = extract_signal( try_get_and_rethrow(simOut,'theta1y') );
    theta1z = extract_signal( try_get_and_rethrow(simOut,'theta1z') );
    theta2  = extract_signal( try_get_and_rethrow(simOut,'theta2') );

    x2 = extract_signal( try_get_and_rethrow(simOut,'x2') );
    y2 = extract_signal( try_get_and_rethrow(simOut,'y2') );
    z2 = extract_signal( try_get_and_rethrow(simOut,'z2') );

    torque_elbow = extract_signal( try_get_and_rethrow(simOut,'torque_elbow') );
    torque_wristy = extract_signal( try_get_and_rethrow(simOut,'torque_wristy') );

    %% --- if any key signal is empty, throw helpful error ------------
    if isempty(theta1x) || isempty(theta1y) || isempty(theta2)
        error('Animation_3D: missing essential signals. Check that your model logs theta1x, theta1y, theta2 (to workspace or signal logging).');
    end

    %% --- time vector and interpolation --------------------------------
    simLength = max([numel(theta1x), numel(theta1y), numel(theta1z), numel(theta2), numel(x2), numel(y2), numel(z2)]);
    if simLength == 0
        error('Animation_3D: simulation produced zero-length signals.');
    end

    time = linspace(0, simTime, simLength)';

    % Interpolate/extrapolate each extracted signal onto `time`
    interp_safe = @(v) ( isempty(v) * zeros(simLength,1) + ~isempty(v) * interp1( linspace(0,simTime,numel(v)), double(v), time, 'linear', 'extrap') );
    theta1x = interp_safe(theta1x);
    theta1y = interp_safe(theta1y);
    theta1z = interp_safe(theta1z);
    theta2  = interp_safe(theta2);
    x2 = interp_safe(x2);
    y2 = interp_safe(y2);
    z2 = interp_safe(z2);
    torque_elbow = interp_safe(torque_elbow);
    torque_wristy = interp_safe(torque_wristy);

    %% --- recompute FK for animation consistency -----------------------
    L1 = param.l1; L2 = param.l2;
    x0_pis = -1.3; conductor_z = 0;
    gripper_attach = [x0_pis, 0, conductor_z];

    x1_rel = L1 .* sin(theta1x) .* cos(theta1y);
    y1_rel = L1 .* sin(theta1x) .* sin(theta1y);
    z1_rel = -L1 .* cos(theta1x);

    x2_rel = x1_rel + L2 .* sin(theta1x + theta2);
    y2_rel = y1_rel + L2 .* sin(theta1x + theta2) .* sin(theta1y);
    z2_rel = z1_rel - L2 .* cos(theta1x + theta2);

    x1g = gripper_attach(1) + x1_rel;
    y1g = gripper_attach(2) + y1_rel;
    z1g = gripper_attach(3) + z1_rel;

    x2g = gripper_attach(1) + x2_rel;
    y2g = gripper_attach(2) + y2_rel;
    z2g = gripper_attach(3) + z2_rel;

    %% --- setup figure & static geometry ------------------------------
    fig = figure(1); clf(fig);
    set(fig,'Color','w','Units','normalized','Position',[0.1 0.1 0.7 0.7]);
    ax = axes('Parent',fig); hold(ax,'on'); grid(ax,'on'); axis(ax,[-3 3 -3 3 -3 3]); axis equal;
    xlabel(ax,'X'); ylabel(ax,'Y'); zlabel(ax,'Z'); title(ax,'3D Brachiation animation');

    long_len = 3; jumper_len = 1.5; blue_len = 0.6;
    plot3(ax,[-long_len,long_len],[0,0],[conductor_z,conductor_z],'Color',[0.6 0.6 0.6],'LineWidth',1.5);
    plot3(ax,[0,0],[-long_len,long_len],[conductor_z,conductor_z],'Color',[0.6 0.6 0.6],'LineWidth',1.5);
    plot3(ax,[x0_pis, x0_pis + jumper_len],[0,0],[conductor_z,conductor_z],'r-','LineWidth',6);
    plot3(ax,[0,0],[-blue_len/2,blue_len/2],[conductor_z,conductor_z],'b-','LineWidth',6);
    plot3(ax,[0 0],[0 0],[-2.5 2.5],'r-','LineWidth',8);

    plot3(ax, gripper_attach(1),gripper_attach(2),gripper_attach(3),'gs','MarkerFaceColor','g','MarkerSize',8);

    % dynamic handles
    hLink1 = plot3(ax, [NaN NaN],[NaN NaN],[NaN NaN], 'k-','LineWidth',3);
    hLink2 = plot3(ax, [NaN NaN],[NaN NaN],[NaN NaN], 'k-','LineWidth',3);
    hJoint1= plot3(ax, NaN,NaN,NaN,'ko','MarkerFaceColor','k','MarkerSize',6);
    hEE   = plot3(ax, NaN,NaN,NaN,'mo','MarkerFaceColor','m','MarkerSize',6);
    hTraj = plot3(ax, NaN,NaN,NaN,'m--','LineWidth',1.5);
    hStart= plot3(ax, x2g(1), y2g(1), z2g(1), 'go','MarkerFaceColor','g','MarkerSize',8);
    hEnd  = plot3(ax, x2g(end), y2g(end), z2g(end), 'mo','MarkerFaceColor','m','MarkerSize',8);

    if isfield(param,'target_on_T')
        T = param.target_on_T(:)';
        plot3(ax,T(1),T(2),T(3),'kx','MarkerSize',12,'LineWidth',2);
    end
    view(ax,[40 20]);
    legend(ax, {'X conductor','Y conductor','red jumper','blue bar','pole','gripper attach','EE traj','start','end','target'}, 'Location','northeastoutside');

    %% --- animation loop ------------------------------------------------
    traj_x = nan(simLength,1); traj_y = nan(simLength,1); traj_z = nan(simLength,1);

    writerObj = VideoWriter('3DSwingmodel.avi'); writerObj.FrameRate = min(60, round(simLength/max(simTime,eps)));
    open(writerObj);

    for idx = 1:simLength
        p0 = gripper_attach;
        p1 = [x1g(idx), y1g(idx), z1g(idx)];
        p2 = [x2g(idx), y2g(idx), z2g(idx)];

        set(hLink1, 'XData',[p0(1) p1(1)], 'YData',[p0(2) p1(2)], 'ZData',[p0(3) p1(3)]);
        set(hLink2, 'XData',[p1(1) p2(1)], 'YData',[p1(2) p2(2)], 'ZData',[p1(3) p2(3)]);
        set(hJoint1,'XData',p1(1),'YData',p1(2),'ZData',p1(3));
        set(hEE,   'XData',p2(1),'YData',p2(2),'ZData',p2(3));

        traj_x(idx) = p2(1); traj_y(idx) = p2(2); traj_z(idx) = p2(3);
        set(hTraj,'XData',traj_x(1:idx),'YData',traj_y(1:idx),'ZData',traj_z(1:idx));

        title(ax, sprintf('3D Brachiation - t = %.2f s', time(idx)));
        drawnow limitrate;

        frame = getframe(fig);
        writeVideo(writerObj, frame);
    end
    try close(writerObj); end

    %% --- plots ---------------------------------------------------------
    figure(2); clf; grid on;
    plot(time, theta1x *180/pi, 'r-', time, theta1y*180/pi, 'g-', time, theta2*180/pi, 'k-','LineWidth',1.2);
    xlabel('Time (s)'); ylabel('Angle (deg)'); legend('\theta_{1x}','\theta_{1y}','\theta_2'); title('Joint angles');
    saveas(gca, fullfile('angles'), 'jpeg');

    figure(3); clf; grid on;
    plot(time, torque_elbow, 'b-', time, torque_wristy, 'r-','LineWidth',1.2);
    xlabel('Time (s)'); ylabel('Torque (N m)'); legend('Elbow','Wrist-y'); title('Torques');
    saveas(gca, fullfile('torques'), 'jpeg');

    figure(4); clf; grid on;
    plot(time, x2, 'b-', time, y2, 'r-', time, z2, 'k-','LineWidth',1.2);
    xlabel('Time (s)'); ylabel('Position (m)'); legend('x2','y2','z2'); title('End-effector positions');
    saveas(gca, fullfile('positions'), 'jpeg');

    % return outputs
    time = time(:);
end

%% small helper used above to throw consistent error if get fails
function out = try_get_and_rethrow(simOut, name)
    try
        out = simOut.get(name);
    catch ME
        % rethrow a clearer error message (helps debugging)
        error('Animation_3D: failed to get signal ''%s'' from simOut. Original error:\n%s', name, ME.message);
    end
end
