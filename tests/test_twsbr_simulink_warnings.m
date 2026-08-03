function tests = test_twsbr_simulink_warnings
%TEST_TWSBR_SIMULINK_WARNINGS Warning regression tests for model builds.
tests = functiontests(localfunctions);
end

function test_repeated_model_build_is_warning_free(test_case)
build_twsbr_simulink();

verifyWarningFree(test_case, @() build_twsbr_simulink());
end
