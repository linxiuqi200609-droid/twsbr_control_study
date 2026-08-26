function config = experiment_config(mode)
mode = lower(string(mode));
if ~isscalar(mode) || ~any(mode == ["quick", "full"])
    error("twsbr:study:invalid_mode", ...
        "mode must be either quick or full.");
end

config = struct();
config.mode = mode;
config.controller_names = ["ATTITUDE_PID"; "CASCADE_PID"; ...
    "FUZZY_PID"; "LQR"; "LQI"];
config.sample_time = 0.01;
config.plant_step = 0.001;
config.theta_reference_limit = deg2rad(12);
config.position_integral_limit = 1e6;
config.fuzzy_gain_adjustment_limit = 1.5;
config.global_seed = 2026;

if mode == "quick"
    config.population_size = 24;
    config.evaluation_budget = 240;
    config.tuning_seeds = 0;
    config.monte_carlo_runs = 10;
    config.bootstrap_resamples = 100;
    config.benchmark_repeats = 1000;
else
    config.population_size = 40;
    config.evaluation_budget = 3200;
    config.tuning_seeds = 0:9;
    config.monte_carlo_runs = 200;
    config.bootstrap_resamples = 2000;
    config.benchmark_repeats = 50000;
end
end
