function annotate_unavailable_controllers(figure_handle, controllers, prefix)
%ANNOTATE_UNAVAILABLE_CONTROLLERS Keep missing identities readable in exports.
lines = prefix;
for first = 1:3:numel(controllers)
    lines(end + 1, 1) = strjoin(controllers(first:min(first + 2, end)), ", "); %#ok<AGROW>
end
annotation(figure_handle, "textbox", [0.08, 0.015, 0.88, 0.14], ...
    "String", lines, "Interpreter", "none", "EdgeColor", "none", ...
    "VerticalAlignment", "bottom", "FontSize", 9);
end
