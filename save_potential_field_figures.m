%% save_potential_field_figures.m
% Saves the generated potential field figures as high-resolution images
% for inclusion in technical reports.

% Run the visualization script first
visualize_potential_field;

% Get current directory and create figures subdirectory
current_dir = pwd;
fig_dir = fullfile(current_dir, 'figures');

if ~exist(fig_dir, 'dir')
    mkdir(fig_dir);
end

% Save Figure 1: 3D Surface Plots
figure(1);
saveas(gcf, fullfile(fig_dir, 'potential_field_3D_surfaces.png'));
saveas(gcf, fullfile(fig_dir, 'potential_field_3D_surfaces.fig'));

% Save Figure 2: 2D Contour Plots
figure(2);
saveas(gcf, fullfile(fig_dir, 'potential_field_2D_contours.png'));
saveas(gcf, fullfile(fig_dir, 'potential_field_2D_contours.fig'));

% Save Figure 3: Gradient Vector Field
figure(3);
saveas(gcf, fullfile(fig_dir, 'potential_field_gradient_vectors.png'));
saveas(gcf, fullfile(fig_dir, 'potential_field_gradient_vectors.fig'));

fprintf('\nFigures saved to %s:\n', fig_dir);
fprintf('  - potential_field_3D_surfaces.png (and .fig)\n');
fprintf('  - potential_field_2D_contours.png (and .fig)\n');
fprintf('  - potential_field_gradient_vectors.png (and .fig)\n');
fprintf('\nAll figures saved for publication quality.\n');
