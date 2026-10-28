%% test_str.m
% Desktop test of the self-tuning PI models (no hardware needed).
% ACE-Lab, ACE-CORE.
here = fileparts(mfilename('fullpath'));  cd(here);
try
    load_system('dc_motor_str');
    set_param('dc_motor_str', 'SimulationCommand', 'update');
    fprintf('dc_motor_str: updates OK for Arduino Mega 2560\n');
catch err
    fprintf(2, 'dc_motor_str: %s\n', err.getReport('basic'));
end
load_system('dc_motor_str_virtual');
mws = get_param('dc_motor_str_virtual', 'ModelWorkspace');
for adapt = [1 0]
    assignin(mws, 'adapt_on', adapt);
    out = sim('dc_motor_str_virtual');
    assignin('base', 'out', out);
    t = (0:numel(out.y_data)-1)'*0.01;
    e = out.r_data(:) - out.y_data(:);
    for seg = {[20 40], [60 80]}
        w = t >= seg{1}(1) & t < seg{1}(2);
        fprintf('adapt_on = %d, t = %2d-%2d s: RMS error %.2f RPM, Kp = %.3f, Ki = %.3f\n', ...
            adapt, seg{1}, sqrt(mean(e(w).^2)), out.Kp_data(find(w, 1, 'last')), ...
            out.Ki_data(find(w, 1, 'last')));
    end
end
mws.reload;                                   % restore adapt_on from the model code
assignin(mws, 'adapt_on', 1);
out = sim('dc_motor_str_virtual');  assignin('base', 'out', out);
plot_str_results;
