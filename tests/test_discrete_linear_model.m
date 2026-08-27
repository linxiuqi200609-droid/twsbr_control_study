function tests = test_discrete_linear_model
%TEST_DISCRETE_LINEAR_MODEL Tests for discrete and LQI plant models.
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

function test_zero_order_hold_matches_matrix_exponential(test_case)
plant = twsbr_params();
sample_time = 0.01;
[a_matrix, b_matrix] = twsbr_linear_model(plant);

[ad, bd, cd, dd] = twsbr_discrete_model(plant, sample_time);
zoh = expm([a_matrix, b_matrix; zeros(1, 5)] * sample_time);

verifyEqual(test_case, ad, zoh(1:4, 1:4), "AbsTol", 1e-14);
verifyEqual(test_case, bd, zoh(1:4, 5), "AbsTol", 1e-14);
verifyEqual(test_case, cd, eye(4));
verifyEqual(test_case, dd, zeros(4, 1));
verifyTrue(test_case, all(isfinite(ad), "all"));
verifyTrue(test_case, all(isfinite(bd), "all"));
end

function test_lqi_augmented_model_is_controllable(test_case)
sample_time = 0.01;
[ad, bd] = twsbr_discrete_model(twsbr_params(), sample_time);
[a_aug, b_aug] = twsbr_augmented_lqi_model(ad, bd, sample_time);
controllability = [b_aug, a_aug * b_aug, a_aug^2 * b_aug, ...
    a_aug^3 * b_aug, a_aug^4 * b_aug];

verifyEqual(test_case, size(a_aug), [5, 5]);
verifyEqual(test_case, size(b_aug), [5, 1]);
verifyEqual(test_case, rank(controllability), 5);
verifyEqual(test_case, a_aug(1:4, 1:4), ad);
verifyEqual(test_case, a_aug(1:4, 5), zeros(4, 1));
verifyEqual(test_case, a_aug(5, :), [0.01, 0, 0, 0, 1], ...
    "AbsTol", 1e-15);
verifyEqual(test_case, b_aug, [bd; 0]);
end

function test_discrete_model_rejects_invalid_sample_time(test_case)
invalid_values = {0, -0.01, NaN, Inf, 0.01 + 1i, [0.01, 0.02], ...
    "0.01", struct(), true};

for index = 1:numel(invalid_values)
    verifyError(test_case, ...
        @() twsbr_discrete_model(twsbr_params(), invalid_values{index}), ...
        "twsbr:linear:invalid_sample_time");
end
end

function test_discrete_model_rejects_nonfinite_zoh_result(test_case)
verifyError(test_case, ...
    @() twsbr_discrete_model(twsbr_params(), 1e6), ...
    "twsbr:linear:discretization_failed");
end

function test_lqi_model_rejects_invalid_discrete_matrices(test_case)
valid_ad = eye(4);
valid_bd = ones(4, 1);
invalid_models = { ...
    {eye(3), valid_bd}; ...
    {valid_ad, ones(1, 4)}; ...
    {replace_element(valid_ad, 1, NaN), valid_bd}; ...
    {valid_ad, replace_element(valid_bd, 1, Inf)}; ...
    {complex(valid_ad, eye(4)), valid_bd}; ...
    {valid_ad, complex(valid_bd, valid_bd)}; ...
    {"invalid", valid_bd}; ...
    {valid_ad, struct()}};

for index = 1:numel(invalid_models)
    model = invalid_models{index};
    verifyError(test_case, ...
        @() twsbr_augmented_lqi_model(model{1}, model{2}, 0.01), ...
        "twsbr:lqi:invalid_discrete_model");
end
end

function test_lqi_model_rejects_invalid_sample_time(test_case)
invalid_values = {0, -0.01, NaN, Inf, 0.01 + 1i, [0.01, 0.02], ...
    "0.01", struct(), true};

for index = 1:numel(invalid_values)
    verifyError(test_case, ...
        @() twsbr_augmented_lqi_model(eye(4), ones(4, 1), ...
        invalid_values{index}), "twsbr:lqi:invalid_sample_time");
end
end

function value = replace_element(value, index, replacement)
value(index) = replacement;
end
