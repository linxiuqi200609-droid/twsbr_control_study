function summary = run_project_workflow(run_tests_flag)
%RUN_PROJECT_WORKFLOW Build, verify, simulate, and export the TWSBR plant project.

if nargin < 1
    run_tests_flag = true;
end
if ~islogical(run_tests_flag) || ~isscalar(run_tests_flag)
    error("twsbr:project:invalid_test_flag", ...
        "Test flag must be a logical scalar.");
end

project_root = fileparts(fileparts(mfilename("fullpath")));
addpath(project_root);
setup_project();

test_results = [];
if run_tests_flag
    test_results = runtests(fullfile(project_root, "tests"));
    if any([test_results.Failed])
        error("twsbr:project:test_failure", ...
            "At least one project test failed.");
    end
end

params = twsbr_params();
[a_matrix, b_matrix, c_matrix, d_matrix] = twsbr_linear_model(params);
controllability = [b_matrix, a_matrix * b_matrix, ...
    a_matrix^2 * b_matrix, a_matrix^3 * b_matrix];
controllability_rank = rank(controllability);

model_path = build_twsbr_simulink(params);
equilibrium_state = zeros(4, 1);
tilt_state = [0.0; 0.0; deg2rad(3.0); 0.0];
matlab_equilibrium = simulate_open_loop( ...
    params, equilibrium_state, 2.0, false);
matlab_tilt = simulate_open_loop(params, tilt_state, 2.0, true);
simulink_tilt = run_simulink_open_loop(params, tilt_state, 2.0);

results_directory = fullfile(project_root, "results");
if ~isfolder(results_directory)
    mkdir(results_directory);
end

figure_path = fullfile(results_directory, "open_loop_response.png");
exportgraphics(matlab_tilt.figure_handle, figure_path, "Resolution", 180);
close(matlab_tilt.figure_handle);

matlab_equilibrium.figure_handle = [];
matlab_tilt.figure_handle = [];
results_path = fullfile(results_directory, "open_loop_results.mat");
save(results_path, ...
    "params", ...
    "a_matrix", ...
    "b_matrix", ...
    "c_matrix", ...
    "d_matrix", ...
    "controllability", ...
    "controllability_rank", ...
    "matlab_equilibrium", ...
    "matlab_tilt", ...
    "simulink_tilt");

fprintf("Analytical A matrix:\n");
disp(a_matrix);
fprintf("Analytical B matrix:\n");
disp(b_matrix);
fprintf("Controllability rank: %d\n", controllability_rank);
fprintf("Simulink model: %s\n", model_path);
fprintf("Results data: %s\n", results_path);
fprintf("Response figure: %s\n", figure_path);

summary = struct();
summary.model_path = model_path;
summary.results_path = results_path;
summary.figure_path = figure_path;
summary.controllability_rank = controllability_rank;
summary.test_results = test_results;
end
