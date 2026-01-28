function [c, ceq] = Constraint_3D_WCW(pinput, param)
    % Evaluate constraints over wind sweep and return worst-case violations.
    n = param.ngrid;
    tswing = pinput(1);
    lc1 = pinput(2); lc2 = pinput(3);
    theta1x_in = pinput(4 : 3+n);
    theta1y_in = pinput(4+n : 3+2*n);
    theta2_in  = pinput(4+2*n : 3+3*n);

    % push inputs to base
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
    numW = numel(winds);

    % allocate
    c_torque = zeros(numW,1);
    c_ypos   = zeros(numW,1);    % require y_peak >= 0
    c_vcross = zeros(numW,1);
    c_xfinal = zeros(numW,1);
    c_yfinal = zeros(numW,1);
    c_theta  = zeros(numW,1);

    % tolerances (tunable or put into param)
    tol_theta = 0.05; % rad
    tol_pos = 0.02;   % meters
    v_max = param.v_max;

    for k=1:numW
        ws = winds(k);
        assignin('base','windSpeed', ws);

        simOut = sim(param.modelName,'SimulationMode','normal','ReturnWorkspaceOutputs','on','SrcWorkspace','base');

        torque_elbow = safe_extract(simOut,'torque_elbow');
        torque_wristy = safe_extract(simOut,'torque_wristy');
        x2 = extract_ts(simOut.get('x2'));
        y2 = extract_ts(simOut.get('y2'));
        th1x = extract_ts(simOut.get('theta1x'));
        th1y = extract_ts(simOut.get('theta1y'));
        th2  = extract_ts(simOut.get('theta2'));

        theta1x_end = th1x(end);
        theta1y_end = th1y(end);
        theta2_end  = th2(end);
        x2_end = x2(end);
        y2_end = y2(end);

        % 1) peak torque constraint (<= Torqhandmax)
        if isempty(torque_elbow), peakT_elbow = 0; else peakT_elbow = max(abs(torque_elbow)); end
        if isempty(torque_wristy), peakT_wrist = 0; else peakT_wrist = max(abs(torque_wristy)); end
        c_torque(k) = max(peakT_elbow, peakT_wrist) - param.Torqhandmax;

        % 2) require positive peak height (y_peak >= 0 -> -y_peak <= 0)
        y_peak = max(y2);
        c_ypos(k) = -y_peak;

        % 3) speed at crossing <= v_max (compute v_y and crossing after peak)
        [~, idx_peak] = max(y2);
        tvec = linspace(0, tswing, numel(y2));
        v_y = gradient(y2, tvec);
        idx_cross_rel = find(y2(idx_peak:end) <= 0, 1, 'first');
        if isempty(idx_cross_rel)
            v_cross = v_y(end);
        else
            idx_cross = idx_peak -1 + idx_cross_rel;
            v_cross = v_y(idx_cross);
        end
        c_vcross(k) = v_cross - v_max;

        % 4) final position tolerances (inequalities: abs(err)-tol <= 0)
        c_xfinal(k) = abs(x2_end - param.x2_target) - tol_pos;
        c_yfinal(k) = abs(y2_end - param.y2_target) - tol_pos;

        % 5) final angle tolerances
        c_theta(k) = max([abs(theta1x_end - param.theta1x_target), ...
                          abs(theta1y_end - param.theta1y_target), ...
                          abs(theta2_end  - param.theta2_target)]) - tol_theta;
    end

    % worst-case (report maximum violation across winds)
    c = [ max(c_torque); max(c_ypos); max(c_vcross); max(c_xfinal); max(c_yfinal); max(c_theta) ];
    ceq = []; % keep as inequalities; if you want hard equality, move appropriate entries to ceq
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