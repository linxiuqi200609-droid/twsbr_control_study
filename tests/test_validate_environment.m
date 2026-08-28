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

function test_unrelated_repository_is_rejected(test_case)
unrelated_root = string(tempname());
mkdir(unrelated_root);
cleanup = onCleanup(@() remove_directory(unrelated_root));
mkdir(fullfile(unrelated_root, "results"));
[status, ~] = system(sprintf('git init "%s"', unrelated_root));
assumeFalse(test_case, status ~= 0, "Git is required for this test.");
verifyError(test_case, @() validate_environment(unrelated_root, false), ...
    "twsbr:environment:invalid_project_root");
clear cleanup
end

function remove_directory(directory_path)
if isfolder(directory_path)
    rmdir(directory_path, "s");
end
end
