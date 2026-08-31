function write_json_file(path_value, data)
%WRITE_JSON_FILE Write pretty-printed JSON as verified UTF-8 bytes.

path_value = validate_path(path_value);
try
    encoded = jsonencode(data, "PrettyPrint", true);
    bytes = unicode2native(encoded, "UTF-8");
catch exception
    throwAsCaller(MException("twsbr:reporting:JsonEncodeFailed", ...
        "JSON encoding failed: %s", exception.message));
end

[file_identifier, message] = fopen(path_value, "w", "n", "UTF-8");
if file_identifier < 0
    error("twsbr:reporting:JsonOpenFailed", ...
        "Could not open JSON target: %s", message);
end
cleanup = onCleanup(@() fclose_if_open(file_identifier));
written = fwrite(file_identifier, bytes, "uint8");
if written ~= numel(bytes)
    error("twsbr:reporting:JsonWriteFailed", ...
        "JSON writer did not write every UTF-8 byte.");
end
clear cleanup
details = dir(path_value);
if isempty(details) || details.bytes ~= numel(bytes)
    error("twsbr:reporting:JsonWriteFailed", ...
        "JSON target byte count did not match the encoded value.");
end
end

function path_value = validate_path(path_value)
valid = (isstring(path_value) && isscalar(path_value) && ...
    ~ismissing(path_value)) || (ischar(path_value) && isrow(path_value));
if ~valid || strlength(string(path_value)) == 0
    error("twsbr:reporting:InvalidPath", ...
        "JSON target must be a nonempty scalar path.");
end
path_value = char(string(path_value));
parent = fileparts(path_value);
if isempty(parent) || ~isfolder(parent)
    error("twsbr:reporting:InvalidPath", ...
        "JSON target parent directory must already exist.");
end
end

function fclose_if_open(file_identifier)
if file_identifier >= 0
    fclose(file_identifier);
end
end
