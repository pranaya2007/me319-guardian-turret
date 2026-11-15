%TUNE_STUDY  Sensitivity of tracking accuracy to the two free design parameters
%   (jerk PSD q of the estimator, loop gain K of the servo). This script only
%   documents WHY q = 1000 and K = 200 were chosen; the final design is fixed.
global GUARDIAN_OVERRIDE
cases = 2001:2025;
fprintf('param   value   mean RMS miss [m]   availability(miss<=2m)\n');
for q = [50 200 1000 5000 20000]
    GUARDIAN_OVERRIDE = struct('q', q);  clear guardian_params turret_controller turret_plant target_motion_generator
    [r, a] = study(cases);  fprintf('q     %7g      %6.2f              %5.3f\n', q, r, a);
end
for K = [48 64 90 120 200 400]
    GUARDIAN_OVERRIDE = struct('K', K);  clear guardian_params turret_controller turret_plant target_motion_generator
    [r, a] = study(cases);  fprintf('K     %7g      %6.2f              %5.3f\n', K, r, a);
end
GUARDIAN_OVERRIDE = [];  clear guardian_params turret_controller turret_plant target_motion_generator
