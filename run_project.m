function summary = run_project(run_tests_flag)
%RUN_PROJECT Build, verify, simulate, and export the TWSBR plant project.

setup_project();
if nargin < 1
    summary = run_project_workflow();
else
    summary = run_project_workflow(run_tests_flag);
end
end
