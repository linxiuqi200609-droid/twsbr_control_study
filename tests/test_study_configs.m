function tests = test_study_configs
%TEST_STUDY_CONFIGS Tests for frozen study configuration values.
tests = functiontests(localfunctions);
end

function setupOnce(test_case)
test_case.TestData.original_path = path;
project_root = string(fileparts(fileparts(mfilename("fullpath"))));
addpath(project_root, "-begin");
setup_project();
end

function teardownOnce(test_case)
path(test_case.TestData.original_path);
end

function test_objective_weights_scales_and_penalties_are_frozen(test_case)
config = objective_config();

verifyTrue(test_case, isscalar(config) && isstruct(config));
verifyEqual(test_case, config.metric_names, ["theta_itae"; ...
    "position_itae"; "control_energy"; "saturation_time"; ...
    "max_abs_theta_rad"; "disturbance_recovery_time"]);
verifyEqual(test_case, config.weights, ...
    [0.25; 0.25; 0.15; 0.15; 0.10; 0.10]);
verifyEqual(test_case, config.scales, ...
    [0.8; 8.0; 4.0; 1.0; deg2rad(10); 3.0], "AbsTol", 1e-15);
verifyEqual(test_case, sum(config.weights), 1.0, "AbsTol", 1e-15);
verifyEqual(test_case, config.failure_penalty, 500);
verifyEqual(test_case, config.invalid_penalty, 1e6);
verifyTrue(test_case, all(isfinite(config.weights)) && ...
    all(config.weights > 0));
verifyTrue(test_case, all(isfinite(config.scales)) && ...
    all(config.scales > 0));
verifyTrue(test_case, isfinite(config.failure_penalty) && ...
    config.failure_penalty > 0);
verifyTrue(test_case, isfinite(config.invalid_penalty) && ...
    config.invalid_penalty > 0);
end

function test_monte_carlo_ranges_are_frozen(test_case)
quick = monte_carlo_config("quick");
full = monte_carlo_config("full");

verifyTrue(test_case, isscalar(quick) && isstruct(quick));
verifyTrue(test_case, isscalar(full) && isstruct(full));
verifyEqual(test_case, quick.parameter_rho, 0.10);
verifyEqual(test_case, full.parameter_rho, 0.20);
verifyEqual(test_case, rmfield(quick, "parameter_rho"), ...
    rmfield(full, "parameter_rho"));
verifyEqual(test_case, quick.duration, 10.0);
verifyEqual(test_case, quick.initial_tilt_deg_range, [2.0, 8.0]);
verifyEqual(test_case, quick.reference_range, [0.25, 1.0]);
verifyEqual(test_case, quick.reference_start, 1.0);
verifyEqual(test_case, quick.force_range, [4.0, 9.0]);
verifyEqual(test_case, quick.force_start_range, [3.0, 5.0]);
verifyEqual(test_case, quick.force_duration_range, [0.10, 0.30]);
verifyEqual(test_case, quick.constant_force_range, [-0.5, 0.5]);
verifyEqual(test_case, quick.noise_std, ...
    [0.002; 0.01; deg2rad(0.15); deg2rad(0.5)], "AbsTol", 1e-15);
verifyTrue(test_case, all(isfinite(struct_numeric_values(quick))));
verifyGreaterThan(test_case, quick.duration, 0);
verifyGreaterThan(test_case, quick.parameter_rho, 0);
verifyTrue(test_case, all(quick.noise_std > 0));
end

function test_invalid_monte_carlo_modes_have_stable_error(test_case)
invalid_calls = { ...
    @() monte_carlo_config(), ...
    @() monte_carlo_config("unknown"), ...
    @() monte_carlo_config(["quick", "full"]), ...
    @() monte_carlo_config(1), ...
    @() monte_carlo_config(missing)};

for index = 1:numel(invalid_calls)
    verifyError(test_case, invalid_calls{index}, ...
        "twsbr:study:invalid_mode");
end
end

function test_configuration_calls_are_deterministic_and_preserve_rng(test_case)
original_rng = rng;
rng_cleanup = onCleanup(@() rng(original_rng));
rng(8642, "twister");
expected_draw = rand(1, 6);
rng(8642, "twister");

first_objective = objective_config();
first_quick = monte_carlo_config("quick");
first_full = monte_carlo_config("full");
actual_draw = rand(1, 6);
second_objective = objective_config();
second_quick = monte_carlo_config("quick");
second_full = monte_carlo_config("full");

verifyEqual(test_case, actual_draw, expected_draw, "AbsTol", 0);
verifyEqual(test_case, second_objective, first_objective);
verifyEqual(test_case, second_quick, first_quick);
verifyEqual(test_case, second_full, first_full);
end

function values = struct_numeric_values(config)
fields = fieldnames(config);
values = [];
for index = 1:numel(fields)
    field_value = config.(fields{index});
    if isnumeric(field_value)
        values = [values; field_value(:)]; %#ok<AGROW>
    end
end
end
