function index = save_raw_simulation( ...
        output_root, raw_directory, controller, scenario, simulation)
%SAVE_RAW_SIMULATION Persist a safe raw MAT result and return one index row.

output_root = canonical_directory(output_root, "output root");
raw_directory = canonical_directory(raw_directory, "raw directory");
if ~is_descendant(raw_directory, output_root)
    error("twsbr:reporting:UnsafeRawPath", ...
        "Raw directory must be contained by the selected output root.");
end
controller = validate_name(controller, "controller");
scenario = validate_name(scenario, "scenario");
if ~isstruct(simulation) || ~isscalar(simulation)
    error("twsbr:reporting:InvalidRawSimulation", ...
        "Raw simulation must be a scalar structure.");
end

file_stem = sanitize_name(controller) + "__" + sanitize_name(scenario);
target = fullfile(raw_directory, file_stem + ".mat");
identity = struct("controller", controller, "scenario", scenario);
if isfile(target)
    existing = load(target, "raw_identity");
    if ~isfield(existing, "raw_identity") || ...
            ~isequal(existing.raw_identity, identity)
        error("twsbr:reporting:RawNameCollision", ...
            "Sanitized raw result names collide for different trials.");
    end
end
raw_identity = identity;
save(target, "simulation", "raw_identity");
relative_path = relative_to_root(canonical_file(target), output_root);
index = table(controller, scenario, relative_path, ...
    'VariableNames', {'controller', 'scenario', 'relative_path'});
end

function directory = canonical_directory(directory, label)
if ~((isstring(directory) && isscalar(directory) && ~ismissing(directory)) || ...
        (ischar(directory) && isrow(directory))) || ...
        strlength(string(directory)) == 0 || ~isfolder(directory)
    error("twsbr:reporting:InvalidPath", ...
        "%s must be an existing scalar directory path.", label);
end
directory = string(java.io.File(char(string(directory))).getCanonicalPath());
end

function name = validate_name(name, label)
if ~((isstring(name) && isscalar(name) && ~ismissing(name)) || ...
        (ischar(name) && isrow(name))) || strlength(string(name)) == 0
    error("twsbr:reporting:InvalidRawName", ...
        "%s must be a nonempty text scalar.", label);
end
name = string(name);
end

function value = sanitize_name(value)
value = regexprep(string(value), "[^A-Za-z0-9_]", "_");
value = regexprep(value, "_+", "_");
if strlength(value) == 0
    error("twsbr:reporting:InvalidRawName", ...
        "Raw names must contain at least one safe character.");
end
end

function accepted = is_descendant(path_value, root)
path_value = replace(string(path_value), "\", "/");
root = replace(string(root), "\", "/");
accepted = startsWith(lower(path_value), lower(root + "/"));
end

function file_path = canonical_file(file_path)
file_path = string(java.io.File(char(string(file_path))).getCanonicalPath());
end

function relative_path = relative_to_root(file_path, root)
file_path = replace(string(file_path), "\", "/");
root = replace(string(root), "\", "/");
if ~startsWith(lower(file_path), lower(root + "/"))
    error("twsbr:reporting:UnsafeRawPath", ...
        "Raw result escaped the selected output root.");
end
relative_path = extractAfter(file_path, strlength(root) + 1);
relative_path = replace(relative_path, "/", filesep);
end
