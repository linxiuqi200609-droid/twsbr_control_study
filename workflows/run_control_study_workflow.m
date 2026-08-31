function summary = run_control_study_workflow(mode, run_tests_flag, options)
%RUN_CONTROL_STUDY_WORKFLOW Execute the reproducible five-controller study.

if nargin < 2
    run_tests_flag = true;
end
if nargin < 3
    options = struct();
end
mode = validate_mode(mode);
validate_test_flag(run_tests_flag);
paths = setup_project();
source_dirty_before_run = source_is_dirty(paths.project_root);
options = study_options(options, mode, paths.project_root);
output_root = prepare_study_root(options.output_root, paths.project_root);
stage_status = initial_stage_status();

announce("environment check");
environment = validate_environment(paths.project_root, options.require_statistics);
config = experiment_config(mode);
objective = objective_config();
plant = twsbr_params();
train = training_scenarios(options.training_duration);
heldout = heldout_scenarios(options.test_duration);

if run_tests_flag
    announce("test suite");
    results = runtests(paths.test_directory);
    assertSuccess(results);
end

announce("tuning");
if isfield(options, "frozen_vectors")
    frozen = validate_frozen_vectors(options.frozen_vectors, config);
    tuning_runs = empty_tuning_runs();
    stage_status.tuning = stage("skipped", "frozen_vectors_bypass");
else
    tuning_runs = tune_all_controllers(plant, config, objective, train, ...
        options.starter_vectors);
    frozen = select_frozen_parameters(tuning_runs);
    stage_status.tuning = stage("completed", "");
end

announce("held-out deterministic batch");
[deterministic, raw] = run_deterministic_batch( ...
    frozen, plant, config, heldout, config.global_seed);
raw_index = save_raw_results(output_root, raw, deterministic);
stage_status.deterministic = stage("completed", "");

monte_carlo = deterministic([],:);
uncertainty = monte_carlo_config(mode);
if options.run_monte_carlo
    announce("paired Monte Carlo batch");
    monte_carlo = run_monte_carlo( ...
        frozen, plant, config, uncertainty, config.global_seed);
    stage_status.monte_carlo = stage("completed", "");
else
    stage_status.monte_carlo = stage("skipped", "disabled_by_option");
end

validation = empty_validation_table();
if options.run_simulink
    announce("Simulink equivalence");
    validation = run_simulink_validation_batch(frozen, plant, config);
    if equivalence_accepted_count(validation) == numel(config.controller_names)
        stage_status.simulink = stage("completed", "");
    else
        stage_status.simulink = stage("failed", "rejected_comparison");
    end
else
    stage_status.simulink = stage("skipped", "disabled_by_option");
end

announce("controller complexity");
complexity = benchmark_controllers(frozen, plant, config);
stage_status.complexity = stage("completed", "");

metric_names = ["theta_rms_deg", "position_itae", "control_energy", ...
    "saturation_ratio"];
deterministic_descriptive = summarize_deterministic_results( ...
    deterministic, metric_names);
descriptive = deterministic_descriptive;
omnibus = empty_omnibus_table();
pairwise = empty_pairwise_table();
if options.run_monte_carlo
    descriptive = summarize_monte_carlo_results(monte_carlo, metric_names, ...
        config.bootstrap_resamples, config.global_seed);
end
if options.require_statistics
    announce("nonparametric statistics");
    statistics_input = deterministic;
    if options.run_monte_carlo
        statistics_input = monte_carlo;
    end
    [omnibus, pairwise] = run_nonparametric_tests(statistics_input, metric_names);
    stage_status.statistics = stage("completed", "");
else
    stage_status.statistics = stage("skipped", "disabled_by_option");
end

announce("reporting tables");
payload = struct( ...
    "omnibus", omnibus, ...
    "pairwise", pairwise, ...
    "deterministic_descriptive", deterministic_descriptive, ...
    "frozen_vectors", frozen, ...
    "raw_index", raw_index, ...
    "training_scenarios", train, ...
    "heldout_scenarios", heldout);
report_paths = write_results_tables(output_root, tuning_runs, deterministic, ...
    monte_carlo, descriptive, complexity, validation, payload);
stage_status.reporting = stage("completed", "");

figure_paths = strings(0, 1);
clear_managed_figures(report_paths.figure_directory);
if options.generate_figures && options.run_monte_carlo
    announce("paper figures");
    figure_paths = generate_paper_figures(raw, monte_carlo, ...
        report_paths.figure_directory);
    stage_status.figures = stage("completed", "");
elseif ~options.generate_figures
    stage_status.figures = stage("skipped", "disabled_by_option");
else
    stage_status.figures = stage("skipped", "monte_carlo_disabled");
end

announce("run manifest");
context = struct( ...
    "effective_options", options_for_manifest(options), ...
    "stage_status", stage_status, ...
    "environment", environment, ...
    "uncertainty", uncertainty, ...
    "source_dirty", source_dirty_before_run, ...
    "equivalence_accepted_count", equivalence_accepted_count(validation));
manifest_path = build_run_manifest(output_root, config, objective, frozen, ...
    environment.git_commit, context);

if options.run_simulink && ...
        equivalence_accepted_count(validation) ~= numel(config.controller_names)
    error("twsbr:study:simulink_validation_failed", ...
        "Simulink validation requires five accepted controller comparisons.");
end

summary = struct();
summary.mode = mode;
summary.output_root = report_paths.output_root;
summary.manifest_path = manifest_path;
summary.paths = report_paths;
summary.figure_paths = figure_paths;
summary.tuning_runs = tuning_runs;
summary.frozen_vectors = frozen;
summary.deterministic = deterministic;
summary.raw = raw;
summary.raw_index = raw_index;
summary.monte_carlo = monte_carlo;
summary.validation = validation;
summary.complexity = complexity;
summary.descriptive = descriptive;
summary.omnibus = omnibus;
summary.pairwise = pairwise;
summary.environment = environment;
summary.stage_status = stage_status;
end

function mode = validate_mode(mode)
if ~((isstring(mode) && isscalar(mode) && ~ismissing(mode)) || ...
        (ischar(mode) && isrow(mode)))
    error("twsbr:study:invalid_mode", ...
        "mode must be either quick or full.");
end
mode = lower(string(mode));
if ~any(mode == ["quick", "full"])
    error("twsbr:study:invalid_mode", ...
        "mode must be either quick or full.");
end
end

function validate_test_flag(flag)
if ~islogical(flag) || ~isscalar(flag)
    error("twsbr:study:invalid_test_flag", ...
        "run_tests_flag must be a logical scalar.");
end
end

function options = study_options(provided, mode, project_root)
if ~isstruct(provided) || ~isscalar(provided)
    error("twsbr:study:invalid_options", ...
        "options must be a scalar structure.");
end
options = struct( ...
    "run_monte_carlo", true, ...
    "run_simulink", true, ...
    "require_statistics", true, ...
    "generate_figures", true, ...
    "output_root", fullfile(project_root, "results", "control_study_" + mode), ...
    "training_duration", 10.0, ...
    "test_duration", 10.0, ...
    "starter_vectors", default_starter_vectors());
allowed = [string(fieldnames(options)); "frozen_vectors"];
provided_names = string(fieldnames(provided));
if any(~ismember(provided_names, allowed))
    error("twsbr:study:invalid_options", ...
        "options contains an unknown field.");
end
for index = 1:numel(provided_names)
    options.(provided_names(index)) = provided.(provided_names(index));
end
logical_names = ["run_monte_carlo", "run_simulink", ...
    "require_statistics", "generate_figures"];
for field = logical_names
    if ~islogical(options.(field)) || ~isscalar(options.(field))
        error("twsbr:study:invalid_options", ...
            "Stage options must be logical scalars.");
    end
end
if ~is_duration(options.training_duration, 3.2) || ...
        ~is_duration(options.test_duration, 4.2)
    error("twsbr:study:invalid_options", ...
        "Training and test durations must meet their documented minimums.");
end
if ~((isstring(options.output_root) && isscalar(options.output_root) && ...
        ~ismissing(options.output_root)) || ...
        (ischar(options.output_root) && isrow(options.output_root))) || ...
        strlength(string(options.output_root)) == 0
    error("twsbr:study:invalid_options", ...
        "output_root must be a nonempty scalar path.");
end
if ~isfield(provided, "frozen_vectors")
    options.starter_vectors = validate_frozen_vectors( ...
        options.starter_vectors, experiment_config(mode));
end
end

function accepted = is_duration(value, minimum)
accepted = isnumeric(value) && isreal(value) && isscalar(value) && ...
    isfinite(value) && value >= minimum;
end

function output_root = prepare_study_root(output_root, project_root)
output_root = string(output_root);
if ~is_absolute_path(output_root)
    output_root = fullfile(project_root, output_root);
end
if ~isfolder(output_root)
    mkdir(output_root);
end
if ~isfolder(output_root)
    error("twsbr:study:invalid_output_root", ...
        "output_root could not be created.");
end
output_root = string(java.io.File(char(output_root)).getCanonicalPath());
end

function accepted = is_absolute_path(path_value)
accepted = ~isempty(regexp(char(path_value), "^[A-Za-z]:[\\/]", "once"));
end

function vectors = default_starter_vectors()
vectors = struct();
vectors.ATTITUDE_PID = log10([1.9, 0.2, 0.18]);
vectors.CASCADE_PID = log10([0.241, 0.000396, 0.193, 9.255, 1.011]);
vectors.FUZZY_PID = [log10([0.241, 0.000396, 0.193, ...
    9.255, 0.05, 1.011]), 0.2, 0.2, 0.2];
vectors.LQR = log10([10, 1, 200, 10, 0.1]);
vectors.LQI = log10([10, 1, 200, 10, 100, 0.1]);
end

function vectors = validate_frozen_vectors(vectors, config)
names = string(config.controller_names(:));
if ~isstruct(vectors) || ~isscalar(vectors) || ...
        numel(fieldnames(vectors)) ~= numel(names) || ...
        ~all(isfield(vectors, cellstr(names)))
    error("twsbr:study:invalid_vectors", ...
        "Frozen vectors must cover exactly the five configured controllers.");
end
for index = 1:numel(names)
    value = vectors.(names(index));
    space = controller_parameter_space(names(index));
    if ~isnumeric(value) || ~isreal(value) || ~isvector(value) || ...
            numel(value) ~= space.dimension || any(~isfinite(value)) || ...
            any(double(value(:).') < space.lower_bounds) || ...
            any(double(value(:).') > space.upper_bounds)
        error("twsbr:study:invalid_vectors", ...
            "Frozen vectors must be finite and inside controller bounds.");
    end
    vectors.(names(index)) = double(value(:).');
end
end

function runs = empty_tuning_runs()
runs = table(strings(0, 1), zeros(0, 1), zeros(0, 1), cell(0, 1), ...
    zeros(0, 1), zeros(0, 1), 'VariableNames', ...
    {'controller', 'seed', 'training_cost', 'vector', ...
    'evaluation_count', 'elapsed_seconds'});
end

function validation = empty_validation_table()
validation = table(strings(0, 1), strings(0, 1), nan(0, 1), ...
    nan(0, 1), nan(0, 1), nan(0, 1), nan(0, 1), nan(0, 1), ...
    nan(0, 1), false(0, 1), false(0, 1), 'VariableNames', ...
    {'controller', 'scenario', 'max_tilt_difference_deg', ...
    'max_position_difference_m', 'max_applied_input_difference', ...
    'tilt_tolerance_deg', 'position_tolerance_m', 'input_tolerance', ...
    'max_fuzzy_gain_relative_error', 'fuzzy_gain_accepted', 'accepted'});
end

function omnibus = empty_omnibus_table()
omnibus = table(strings(0, 1), zeros(0, 1), zeros(0, 1), nan(0, 1), ...
    zeros(0, 1), zeros(0, 1), 'VariableNames', ...
    {'metric', 'group_count', 'successful_n', 'p_value', ...
    'configured_group_count', 'analyzed_group_count'});
end

function pairwise = empty_pairwise_table()
pairwise = table(strings(0, 1), strings(0, 1), strings(0, 1), ...
    zeros(0, 1), zeros(0, 1), nan(0, 1), nan(0, 1), nan(0, 1), ...
    'VariableNames', {'metric', 'first_controller', 'second_controller', ...
    'first_n', 'second_n', 'p_value', 'p_value_holm', 'cliffs_delta'});
end

function raw_index = save_raw_results(output_root, raw, deterministic)
raw_index = table(strings(0, 1), strings(0, 1), strings(0, 1), ...
    'VariableNames', {'controller', 'scenario', 'relative_path'});
raw_directory = fullfile(output_root, "raw");
if ~isfolder(raw_directory)
    mkdir(raw_directory);
end
for row = 1:height(deterministic)
    controller = string(deterministic.controller(row));
    scenario = string(deterministic.scenario(row));
    key = matlab.lang.makeValidName(char(controller + "__" + scenario));
    raw_index = [raw_index; save_raw_simulation(output_root, raw_directory, ...
        controller, scenario, raw.(key))]; %#ok<AGROW>
end
end

function stage_status = initial_stage_status()
stage_status = struct( ...
    "tuning", stage("pending", ""), ...
    "deterministic", stage("pending", ""), ...
    "monte_carlo", stage("pending", ""), ...
    "simulink", stage("pending", ""), ...
    "complexity", stage("pending", ""), ...
    "statistics", stage("pending", ""), ...
    "reporting", stage("pending", ""), ...
    "figures", stage("pending", ""));
end

function value = stage(state, reason)
value = struct("state", string(state), "reason", string(reason));
end

function clear_managed_figures(directory)
names = ["F1_nominal_response", "F2_saturation_response", ...
    "F3_disturbance_recovery", "F4_monte_carlo_boxplots", ...
    "F5_performance_pareto", "F6_normalized_radar"];
for name = names
    for extension = [".png", ".pdf"]
        path_value = fullfile(directory, name + extension);
        if isfile(path_value)
            delete(path_value);
        end
    end
end
end

function context = options_for_manifest(options)
context = options;
if isfield(context, "starter_vectors") && isfield(context, "frozen_vectors")
    context = rmfield(context, "starter_vectors");
end
end

function count = equivalence_accepted_count(validation)
count = 0;
if ismember("accepted", string(validation.Properties.VariableNames))
    count = sum(validation.accepted);
end
end

function dirty = source_is_dirty(project_root)
[status, output] = system(sprintf( ...
    'git -C "%s" status --porcelain', project_root));
if status ~= 0
    dirty = "unavailable";
    return
end
dirty = strlength(strtrim(string(output))) > 0;
end

function announce(name)
fprintf("[control-study] %s\n", name);
end
