function summary = run_control_study(mode, run_tests_flag, options)
%RUN_CONTROL_STUDY Run the unified five-controller study from any caller CWD.

setup_project();
if nargin < 1
    mode = "quick";
end
if nargin < 2
    run_tests_flag = true;
end
if nargin < 3
    options = struct();
end
summary = run_control_study_workflow(mode, run_tests_flag, options);
end
