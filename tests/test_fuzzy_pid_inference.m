function tests = test_fuzzy_pid_inference
%TEST_FUZZY_PID_INFERENCE Tests for fixed fuzzy PID inference.
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

function test_rule_matrices_match_fixed_singletons(test_case)
rules = fuzzy_pid_rule_base();
expected_kp = [ ...
    3, 3, 2, 2, 2, 3, 3; ...
    3, 2, 2, 1, 2, 2, 3; ...
    2, 2, 1, 1, 1, 2, 2; ...
    2, 1, 1, 0, 1, 1, 2; ...
    2, 2, 1, 1, 1, 2, 2; ...
    3, 2, 2, 1, 2, 2, 3; ...
    3, 3, 2, 2, 2, 3, 3];
expected_ki = [ ...
    -3, -2, -1, 0, -1, -2, -3; ...
    -2, -1, 0, 1, 0, -1, -2; ...
    -1, 0, 1, 2, 1, 0, -1; ...
    0, 1, 2, 3, 2, 1, 0; ...
    -1, 0, 1, 2, 1, 0, -1; ...
    -2, -1, 0, 1, 0, -1, -2; ...
    -3, -2, -1, 0, -1, -2, -3];
expected_kd = [ ...
    0, -1, -2, -3, -2, -1, 0; ...
    1, 0, -1, -2, -1, 0, 1; ...
    2, 1, 0, -1, 0, 1, 2; ...
    3, 2, 1, 0, 1, 2, 3; ...
    2, 1, 0, -1, 0, 1, 2; ...
    1, 0, -1, -2, -1, 0, 1; ...
    0, -1, -2, -3, -2, -1, 0];

verifyEqual(test_case, fieldnames(rules), ...
    {'delta_kp'; 'delta_ki'; 'delta_kd'});
verifyEqual(test_case, rules.delta_kp, expected_kp);
verifyEqual(test_case, rules.delta_ki, expected_ki);
verifyEqual(test_case, rules.delta_kd, expected_kd);
end

function test_rule_matrices_are_rotationally_symmetric_and_bounded(test_case)
rules = fuzzy_pid_rule_base();
rule_names = fieldnames(rules);

for index = 1:numel(rule_names)
    matrix = rules.(rule_names{index});
    verifyEqual(test_case, size(matrix), [7, 7]);
    verifyEqual(test_case, matrix, rot90(matrix, 2));
    verifyLessThanOrEqual(test_case, max(abs(matrix), [], "all"), 3);
end
end

function test_inference_is_symmetric_deterministic_and_bounded(test_case)
[first, diagnostics] = fuzzy_pid_inference(0.35, -0.20);
second = fuzzy_pid_inference(0.35, -0.20);
opposite = fuzzy_pid_inference(-0.35, 0.20);

verifyEqual(test_case, second, first);
verifyEqual(test_case, opposite, first, "AbsTol", 1e-14);
values = cell2mat(struct2cell(first));
verifyGreaterThanOrEqual(test_case, min(values), -1);
verifyLessThanOrEqual(test_case, max(values), 1);
verifyEqual(test_case, numel(diagnostics.error_membership), 7);
verifyEqual(test_case, numel(diagnostics.rate_membership), 7);
verifyEqual(test_case, sum(diagnostics.error_membership), 1, ...
    "AbsTol", 1e-14);
verifyEqual(test_case, sum(diagnostics.rate_membership), 1, ...
    "AbsTol", 1e-14);
verifyGreaterThan(test_case, diagnostics.total_firing_strength, 0);
end

function test_endpoint_memberships_use_shoulder_functions(test_case)
[adjustments, diagnostics] = fuzzy_pid_inference(-1.0, 1.0);

verifyEqual(test_case, diagnostics.error_membership, ...
    [1, 0, 0, 0, 0, 0, 0], "AbsTol", 1e-15);
verifyEqual(test_case, diagnostics.rate_membership, ...
    [0, 0, 0, 0, 0, 0, 1], "AbsTol", 1e-15);
verifyEqual(test_case, diagnostics.total_firing_strength, 1, ...
    "AbsTol", 1e-15);
verifyEqual(test_case, adjustments, ...
    struct("delta_kp", 1.0, "delta_ki", -1.0, "delta_kd", 0.0), ...
    "AbsTol", 1e-15);
end

function test_inputs_are_clipped_before_membership_evaluation(test_case)
[clipped, clipped_diagnostics] = fuzzy_pid_inference(-1.0, 1.0);
[outside, outside_diagnostics] = fuzzy_pid_inference(-20.0, 30.0);

verifyEqual(test_case, outside, clipped, "AbsTol", 1e-15);
verifyEqual(test_case, outside_diagnostics, clipped_diagnostics, ...
    "AbsTol", 1e-15);
end

function test_min_tnorm_and_weighted_average_are_exact(test_case)
[adjustments, diagnostics] = fuzzy_pid_inference(-5 / 6, -1 / 2);

verifyEqual(test_case, diagnostics.error_membership, ...
    [0.5, 0.5, 0, 0, 0, 0, 0], "AbsTol", 1e-14);
verifyEqual(test_case, diagnostics.rate_membership, ...
    [0, 0.5, 0.5, 0, 0, 0, 0], "AbsTol", 1e-14);
verifyEqual(test_case, diagnostics.total_firing_strength, 2.0, ...
    "AbsTol", 1e-14);
verifyEqual(test_case, adjustments.delta_kp, 0.75, "AbsTol", 1e-14);
verifyEqual(test_case, adjustments.delta_ki, -1 / 3, "AbsTol", 1e-14);
verifyEqual(test_case, adjustments.delta_kd, -1 / 3, "AbsTol", 1e-14);
end

function test_inference_rejects_invalid_inputs_with_stable_identifier(test_case)
invalid_values = {NaN, Inf, -Inf, 1i, [0, 1], "invalid", struct(), true};

for index = 1:numel(invalid_values)
    verifyError(test_case, @() fuzzy_pid_inference( ...
        invalid_values{index}, 0.0), "twsbr:fuzzy:invalid_input");
    verifyError(test_case, @() fuzzy_pid_inference( ...
        0.0, invalid_values{index}), "twsbr:fuzzy:invalid_input");
end
end
