function paths = generate_paper_figures(raw, monte_carlo, output_directory)
%GENERATE_PAPER_FIGURES Export the six paper figures as PNG and vector PDF.
if ~((isstring(output_directory) && isscalar(output_directory)) || ...
        (ischar(output_directory) && isrow(output_directory))) || ...
        strlength(string(output_directory)) == 0
    error("twsbr:figures:InvalidOutputDirectory", ...
        "Output directory must be a nonempty text scalar.");
end
output_directory = string(output_directory);
if ~isfolder(output_directory)
    mkdir(output_directory);
    if ~isfolder(output_directory)
        error("twsbr:figures:InvalidOutputDirectory", ...
            "Output directory could not be created.");
    end
end
names = ["F1_nominal_response"; "F2_saturation_response"; ...
    "F3_disturbance_recovery"; "F4_monte_carlo_boxplots"; ...
    "F5_performance_pareto"; "F6_normalized_radar"];
plotters = {@() plot_nominal_response(raw), @() plot_saturation_response(raw), ...
    @() plot_disturbance_recovery(raw), @() plot_monte_carlo_boxplots(monte_carlo), ...
    @() plot_performance_pareto(monte_carlo), @() plot_normalized_radar(monte_carlo)};
paths = strings(12, 1);
path_index = 0;
for index = 1:numel(names)
    figure_handle = plotters{index}();
    cleanup = onCleanup(@() close_if_valid(figure_handle));
    png_path = string(fullfile(output_directory, names(index) + ".png"));
    pdf_path = string(fullfile(output_directory, names(index) + ".pdf"));
    exportgraphics(figure_handle, png_path, "Resolution", 300);
    exportgraphics(figure_handle, pdf_path, "ContentType", "vector");
    if ~isfile(png_path) || ~isfile(pdf_path) || ...
            dir(png_path).bytes == 0 || dir(pdf_path).bytes == 0
        error("twsbr:figures:ExportFailed", "Figure export did not create nonempty files.");
    end
    path_index = path_index + 1;
    paths(path_index) = string(java.io.File(char(png_path)).getCanonicalPath());
    path_index = path_index + 1;
    paths(path_index) = string(java.io.File(char(pdf_path)).getCanonicalPath());
    close_if_valid(figure_handle);
    delete(cleanup);
end
end

function close_if_valid(figure_handle)
if isgraphics(figure_handle)
    close(figure_handle);
end
end
