function [t, x] = aceGetSignal(out, name)
% Returns time and data for a logged signal from a SimulationOutput,
% trying To Workspace variables, logsout, then the latest Data Inspector run
% (External mode / Monitor & Tune keeps streamed data there).
t = []; x = [];
try
    v = out.get(name);
    if isa(v, 'timeseries'), t = v.Time; x = squeeze(v.Data); end
catch
end
if isempty(t)
    try
        v = out.logsout.getElement(name).Values; t = v.Time; x = squeeze(v.Data);
    catch
    end
end
if isempty(t)
    try
        r = Simulink.sdi.Run.getLatest;
        s = r.getSignalsByName(name);
        if ~isempty(s), t = s(1).Values.Time; x = squeeze(s(1).Values.Data); end
    catch
    end
end
if isempty(t), error('Signal "%s" not found in the run output.', name); end
t = double(t(:)); x = double(x(:));
end
