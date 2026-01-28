function J = Cost_3D_WCW(pinput, param)
    % Adapted cost: evaluate closed-loop sim for each wind, take worst-case cost.
    n = param.ngrid;
    tswing = pinput(1);
    lc1 = pinput(2);
    lc2 = pinput(3);
    theta1x_in = pinput(4 : 3+n);
    theta1y_in = pinput(4+n : 3+2*n);
    theta2_in  = pinput(4+2*n : 3+3*n);

    % push parameters & input spline to base workspace
    assignin('base','lc1',lc1); assignin('base','lc2',lc2);
    assignin('base','timeVec', linspace(0, tswing, n)');
    assignin('base','theta1x_in', theta1x_in(:));
    assignin('base','theta1y_in', theta1y_in(:));
    assignin('base','theta2_in',  theta2_in(:));
    assignin('base','simTime', tswing);

    fn = fieldnames(param);
    for ii = 1:numel(fn)
        assignin('base', fn{ii}, param.(fn{ii}));
    end

    winds = param.windSpeedList;
    Jvals = zeros(size(winds));

    % tunable weights (use param.* if you prefer)
    if isfield(param,'w_pos'), w_pos = param.w_pos; else w_pos = 1e6; end
    if isfield(param,'w_ang'), w_ang = param.w_ang; else w_ang = 1e3; end
    if isfield(param,'Torqhandmax'), Torqhandmax = param.Torqhandmax; else Torqhandmax = 100; end

    for k=1:numel(winds)
        ws = winds(k);
        assignin('base','windSpeed', ws);

        % run Simulink model
        simOut = sim(param.modelName, 'SimulationMode','normal', ...
                     'ReturnWorkspaceOutputs','on','SrcWorkspace','base');

        % extract signals robustly
        x2 = extract_ts(simOut.get('x2'));
        y2 = extract_ts(simOut.get('y2'));
        torque_elbow = safe_extract(simOut,'torque_elbow');
        torque_wristy = safe_extract(simOut,'torque_wristy');
        th1x = extract_ts(simOut.get('theta1x'));
        th1y = extract_ts(simOut.get('theta1y'));
        th2  = extract_ts(simOut.get('theta2'));

        % end-sample values
        x2_end = double(x2(end));
        y2_end = double(y2(end));
        theta1x_end = double(th1x(end));
        theta1y_end = double(th1y(end));
        theta2_end  = double(th2(end));

        % position error (final end-effector vs target)
        tx = param.x2_target;
        ty = param.y2_target;
        err_pos_sq = (x2_end - tx)^2 + (y2_end - ty)^2;        % squared Euclidean error
        err_theta = (theta1x_end - param.theta1x_target)^2 + ...
                    (theta1y_end - param.theta1y_target)^2 + ...
                    (theta2_end  - param.theta2_target)^2;

        % torque penalties (peak per actuator)
        peak_elbow = 0; peak_wrist = 0;
        if ~isempty(torque_elbow), peak_elbow = max(abs(torque_elbow)); end
        if ~isempty(torque_wristy), peak_wrist = max(abs(torque_wristy)); end
        pen_torque_elbow = 1e3 * max(0, peak_elbow - Torqhandmax).^2;
        pen_torque_wrist = 1e3 * max(0, peak_wrist - Torqhandmax).^2;

        %% add cost of the derivate of the torque


        %% Build cost: position error primary, angles secondary, torque penalty
        Jk = w_pos * err_pos_sq + w_ang * err_theta + pen_torque_elbow + pen_torque_wrist;

        Jvals(k) = Jk;

        fprintf('wind=%+g  cost=%g (pos_err=%g, ang_err=%g, peakT_elbow=%g, peakT_wrist=%g)\n', ...
                ws, Jk, sqrt(err_pos_sq), sqrt(err_theta), peak_elbow, peak_wrist);
    end

    % worst-case (maximize across winds)
    [J, idxmax] = max(Jvals);
    fprintf('*** worst-case wind = %+g  (J = %g)\n', winds(idxmax), J);
end


function vec = extract_ts(ts)
    % helper to turn timeseries / struct / numeric into column vector
    if isempty(ts)
        error('Expected signal missing from Simulink output.');
    end
    if isa(ts,'timeseries')
        vec = double(ts.Data(:));
    elseif isstruct(ts) && isfield(ts,'signals') && isfield(ts.signals,'values')
        vec = double(ts.signals.values(:));
    elseif isnumeric(ts)
        vec = double(ts(:));
    else
        error('Unsupported signal type returned from sim.');
    end
end

function vec = safe_extract(simOut, name)
    % tries to fetch named signal, returns [] if not present
    try
        s = simOut.get(name);
        vec = extract_ts(s);
    catch
        vec = [];
    end
end