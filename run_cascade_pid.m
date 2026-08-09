function summary = run_cascade_pid(run_tests_flag)
%RUN_CASCADE_PID Build, simulate, verify, and export cascade PID results.

setup_project();
if nargin < 1
    summary = run_cascade_pid_workflow();
else
    summary = run_cascade_pid_workflow(run_tests_flag);
end
end
