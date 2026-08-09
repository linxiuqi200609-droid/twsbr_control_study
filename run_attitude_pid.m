function summary = run_attitude_pid(run_tests_flag)
%RUN_ATTITUDE_PID Build, simulate, verify, and export attitude PID results.

setup_project();
if nargin < 1
    summary = run_attitude_pid_workflow();
else
    summary = run_attitude_pid_workflow(run_tests_flag);
end
end
