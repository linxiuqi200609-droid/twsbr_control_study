function manifest_path = build_run_manifest( ...
        output_root, config, objective, vectors, git_commit, varargin)
%BUILD_RUN_MANIFEST Record reproducibility metadata for one study run.

output_root = validate_output_root(output_root);
validate_inputs(config, objective, vectors, git_commit);
project_root = string(fileparts(fileparts(mfilename("fullpath"))));
toolboxes = ver;
toolbox_names = string({toolboxes.Name}).';
toolbox_versions = string({toolboxes.Version}).';
manifest = struct();
manifest.created_utc = string(datetime("now", "TimeZone", "UTC", ...
    "Format", "yyyy-MM-dd'T'HH:mm:ss.SSS'Z'"));
manifest.version = string(version);
manifest.computer = string(computer);
manifest.matlab_release = string(version("-release"));
manifest.toolboxes = table_to_records(toolbox_names, toolbox_versions);
manifest.git_commit = string(git_commit);
manifest.mode = string(config.mode);
manifest.seed = config.global_seed;
manifest.controller_names = string(config.controller_names(:));
manifest.frozen_vectors = vectors;
manifest.configuration_hashes = struct( ...
    "experiment_config", file_sha256(fullfile(project_root, "config", "experiment_config.m")), ...
    "objective_config", file_sha256(fullfile(project_root, "config", "objective_config.m")), ...
    "monte_carlo_config", file_sha256(fullfile(project_root, "config", "monte_carlo_config.m")));
if ~isempty(varargin)
    context = varargin{1};
    if ~isstruct(context) || ~isscalar(context)
        error("twsbr:reporting:InvalidManifestContext", ...
            "Optional manifest context must be a scalar structure.");
    end
    manifest.context = context;
end
manifest_path = fullfile(output_root, "run_manifest.json");
write_json_file(manifest_path, manifest);
manifest_path = canonical_path(manifest_path);
end

function output_root = validate_output_root(output_root)
if ~((isstring(output_root) && isscalar(output_root) && ~ismissing(output_root)) || ...
        (ischar(output_root) && isrow(output_root))) || ...
        strlength(string(output_root)) == 0 || ~isfolder(output_root)
    error("twsbr:reporting:InvalidPath", ...
        "Manifest output root must be an existing scalar directory.");
end
output_root = canonical_path(output_root);
end

function validate_inputs(config, objective, vectors, git_commit)
if ~isstruct(config) || ~isscalar(config) || ...
        ~all(isfield(config, ["mode", "global_seed", "controller_names"])) || ...
        ~isstruct(objective) || ~isscalar(objective) || ...
        ~isstruct(vectors) || ~isscalar(vectors) || ...
        ~((isstring(git_commit) && isscalar(git_commit)) || ...
        (ischar(git_commit) && isrow(git_commit)))
    error("twsbr:reporting:InvalidManifestInput", ...
        "Manifest inputs must use the study configuration contract.");
end
end

function digest = file_sha256(path_value)
if ~isfile(path_value)
    error("twsbr:reporting:MissingConfiguration", ...
        "Configuration file is unavailable: %s", path_value);
end
message_digest = java.security.MessageDigest.getInstance("SHA-256");
message_digest.update(uint8(fileread(path_value)));
bytes = typecast(message_digest.digest(), "uint8");
digest = lower(string(reshape(dec2hex(bytes, 2).', 1, [])));
end

function records = table_to_records(names, versions)
records = repmat(struct("name", "", "version", ""), numel(names), 1);
for index = 1:numel(names)
    records(index).name = names(index);
    records(index).version = versions(index);
end
end

function path_value = canonical_path(path_value)
path_value = string(java.io.File(char(string(path_value))).getCanonicalPath());
end
