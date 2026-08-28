function tests = test_validate_environment
tests = functiontests(localfunctions);
end

function test_environment_report_has_required_fields(test_case)
root = string(fileparts(fileparts(mfilename("fullpath"))));
report = validate_environment(root, false);
verifyEqual(test_case, fieldnames(report), { ...
    'matlab_release'; 'platform'; 'simulink_available'; ...
    'control_available'; 'statistics_available'; ...
    'output_writable'; 'git_commit'; 'accepted'});
verifyTrue(test_case, report.simulink_available);
verifyTrue(test_case, report.control_available);
verifyTrue(test_case, report.output_writable);
verifyTrue(test_case, report.accepted);
end

function test_invalid_root_is_rejected(test_case)
verifyError(test_case, @() validate_environment("Z:/missing-srtp", false), ...
    "twsbr:environment:invalid_project_root");
end
