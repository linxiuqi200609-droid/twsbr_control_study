function assert_cascade_pid_test_success(test_results)
%ASSERT_CASCADE_PID_TEST_SUCCESS Reject failed or incomplete project tests.

assertSuccess(test_results);
if any([test_results.Incomplete])
    error("twsbr:cascade_pid:test_incomplete", ...
        "At least one project test was incomplete.");
end
end
