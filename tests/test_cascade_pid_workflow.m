function tests = test_cascade_pid_workflow
%TEST_CASCADE_PID_WORKFLOW Verify cascade workflow outputs and documentation.
tests = functiontests(localfunctions);
end

function setupOnce(test_case)
test_case.TestData.original_path = path;
project_root = string(fileparts(fileparts(mfilename("fullpath"))));
addpath(project_root, "-begin");
setup_project();
end

function teardownOnce(test_case)
if bdIsLoaded("twsbr_cascade_pid")
    close_system("twsbr_cascade_pid", 0);
end
path(test_case.TestData.original_path);
end

function test_workflow_exports_accepted_summary_and_artifacts(test_case)
summary = run_cascade_pid(false);
expected_summary_fields = sort([ ...
    "acceptance"; "accepted"; "cascade_params"; "comparison"; ...
    "figure_path"; "matlab_simulations"; "model_path"; ...
    "plant_params"; "project_root"; "results_path"; "scenarios"; ...
    "simulink_simulation"; "test_results"]);

verifyEqual(test_case, sort(string(fieldnames(summary))), ...
    expected_summary_fields);
verifyTrue(test_case, islogical(summary.accepted) && isscalar(summary.accepted));
verifyTrue(test_case, summary.accepted);
assert(summary.accepted);
verifyTrue(test_case, all(structfun( ...
    @(result) result.accepted, summary.acceptance)));
verifyTrue(test_case, summary.comparison.accepted);
verifyLessThan(test_case, ...
    summary.comparison.maximum_tilt_difference_deg, 0.2);
verifyLessThan(test_case, ...
    summary.comparison.maximum_position_difference_m, 0.01);
verifyEqual(test_case, summary.simulink_simulation.scenario_name, ...
    "positive_position_step");
verifyTrue(test_case, isfile(summary.model_path));
verifyTrue(test_case, isfile(summary.results_path));
verifyTrue(test_case, isfile(summary.figure_path));

mat_variables = sort(string(who("-file", char(summary.results_path))));
expected_mat_variables = sort([ ...
    "acceptance"; "cascade_params"; "comparison"; ...
    "matlab_simulations"; "plant_params"; "scenarios"; ...
    "simulink_simulation"]);
verifyEqual(test_case, mat_variables, expected_mat_variables);

saved = load(summary.results_path);
verifyEqual(test_case, string(fieldnames(saved.scenarios)), [ ...
    "zero_state"; "positive_position_step"; ...
    "negative_position_step"; "initial_tilt"; "force_impulse"]);
verifyEqual(test_case, saved.comparison, summary.comparison);
verifyEqual(test_case, saved.acceptance, summary.acceptance);

image_information = imfinfo(summary.figure_path);
verifyEqual(test_case, string(image_information.Format), "png");
verifyGreaterThan(test_case, image_information.Width, 0);
verifyGreaterThan(test_case, image_information.Height, 0);
end

function test_test_gate_rejects_incomplete_results(test_case)
fixture_directory = string(tempname);
mkdir(fixture_directory);
cleanup = onCleanup(@() remove_directory_if_present(fixture_directory));
fixture_path = fullfile(fixture_directory, "test_incomplete_fixture.m");
write_incomplete_fixture(fixture_path);

test_results = runtests(fixture_path);

verifyEqual(test_case, numel(test_results), 1);
verifyFalse(test_case, any([test_results.Failed]));
verifyTrue(test_case, all([test_results.Incomplete]));
verifyError(test_case, @() assert_cascade_pid_test_success(test_results), ...
    "twsbr:cascade_pid:test_incomplete");
end

function test_workflow_rejects_nonlogical_test_flag(test_case)
verifyError(test_case, @() run_cascade_pid_workflow(1), ...
    "twsbr:cascade_pid:invalid_test_flag");
end

function test_readme_documents_final_cascade_contract(test_case)
project_root = string(fileparts(fileparts(mfilename("fullpath"))));
readme = string(fileread(fullfile(project_root, "README.md")));
required_text = [ ...
    "run_cascade_pid.m"; "run_cascade_pid(false)"; ...
    "run_cascade_pid_workflow.m"; ...
    "position_error = x_reference - x"; ...
    "theta_error = theta - theta_reference"; ...
    "positive_position_step"; "negative_position_step"; ...
    "initial_tilt"; "force_impulse"; ...
    "4.0 <= t < 4.2 s"; ...
    "0.05 m"; "5 s"; "3 s"; "0.2 deg"; "0.01 m"; ...
    "results/cascade_pid_results.mat"; ...
    "results/cascade_pid_response.png"; ...
    "setup_project.m uses an explicit allowlist"];

for index = 1:numel(required_text)
    verifyTrue(test_case, contains(readme, required_text(index)), ...
        required_text(index));
end
end

function write_incomplete_fixture(fixture_path)
fixture_text = strjoin([ ...
    "function tests = test_incomplete_fixture"; ...
    "tests = functiontests(localfunctions);"; ...
    "end"; ...
    "function test_assumption(test_case)"; ...
    "assumeTrue(test_case, false);"; ...
    "end"], newline);
file_identifier = fopen(fixture_path, "w");
file_cleanup = onCleanup(@() fclose(file_identifier));
fprintf(file_identifier, "%s", fixture_text);
end

function remove_directory_if_present(directory_path)
if isfolder(directory_path)
    rmdir(directory_path, "s");
end
end
