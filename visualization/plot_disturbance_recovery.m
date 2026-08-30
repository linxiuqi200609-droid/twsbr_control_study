function figure_handle = plot_disturbance_recovery(raw)
%PLOT_DISTURBANCE_RECOVERY Plot five-controller recovery from positive impulse.
results = validate_figure_raw(raw, "S3_positive_impulse");
figure_handle = plot_response_axes(results, "Disturbance recovery");
end
