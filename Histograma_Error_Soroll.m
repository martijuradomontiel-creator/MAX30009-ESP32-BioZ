close all;
clear;

%% User Options
plot_mode = 'fullscreen'; % Options: 'fullscreen' or 'fixed'
num_bins = 150;           % Cantidad de barras (más alto = extractos más pequeños y finos)

%% Data Loading
files = dir('*.csv');
if isempty(files)
    error('No CSV files found in the current directory.');
end

% Load the first CSV file found
complete_route = fullfile(files(1).folder, files(1).name);
fprintf('Loading data from: %s\n', files(1).name);
data_table = readtable(complete_route);

% Extraer el nombre base del archivo sin la extensión .csv para la exportación
[~, base_name, ~] = fileparts(files(1).name);

% Extract columns directly
Z = double(data_table.Magnitude);
I = double(data_table.Real);
Q = double(data_table.Imaginary);
Phase = atan2d(Q, I);

%% Define Fixed Dimensions Plots (Used only for 'fixed' mode)
axes_width_cm = 18;  
axes_height_cm = 12; 
margin_left_cm = 2.0; 
margin_bottom_cm = 1.5; 
margin_right_cm = 2.0; 
margin_top_cm = 1.5; 
total_width_cm = margin_left_cm + axes_width_cm + margin_right_cm;
total_height_cm = margin_bottom_cm + axes_height_cm + margin_top_cm;

%% Define Normalized Dimensions Plots (Used for 'fullscreen' mode)
norm_ax_pos = [0.10, 0.10, 0.85, 0.80]; 

%% Plotting Histograms with Gaussian Fits
data_cells = {Z, I, Q, Phase};
titles = {'Histogram of Impedance Magnitude (Z)', ...
          'Histogram of In-Phase Component (I)', ...
          'Histogram of Quadrature Component (Q)', ...
          'Histogram of Phase'};
      
x_labels = {'Z [\Omega]', 'In-Phase [\Omega]', 'Quadrature [\Omega]', 'Phase [deg]'};

% Sufijos para diferenciar los archivos exportados
export_suffixes = {'Z', 'InPhase', 'Quadrature', 'Phase'};

for i = 1:4
    current_data = data_cells{i};
    
    % 1. Calculate Gaussian parameters (Mean and Standard Deviation)
    mu = mean(current_data, 'omitnan');
    sigma = std(current_data, 'omitnan');
    
    % 2. Create the figure and configure the axes based on the selected plot_mode
    if strcmp(plot_mode, 'fullscreen')
        fig = figure('Units', 'normalized');
        fig.WindowState = 'maximized';
        ax = axes('Units', 'normalized', 'Position', norm_ax_pos);
        
        title_font_size = 18;
        label_font_size = 14;
        legend_font_size = 12;
    else
        fig = figure('Units', 'centimeters', ...
                    'Position', [2 + i*0.5, 2, total_width_cm, total_height_cm]);
        ax = axes('Units', 'centimeters', ...
                  'Position', [margin_left_cm, margin_bottom_cm, axes_width_cm, axes_height_cm]);
        
        title_font_size = 12;
        label_font_size = 11;
        legend_font_size = 10;
    end
    
    % 3. Aplicar estilo estricto tipo Paper Científico (Times New Roman + Bold + Ejes Fuertes)
    set(ax, 'FontName', 'Times New Roman', ...
            'FontSize', label_font_size - 1, ...
            'FontWeight', 'bold', ...      
            'LineWidth', 1.5, ...          
            'XColor', [0.15 0.15 0.15], ...
            'YColor', [0.15 0.15 0.15]);
    
    % 4. Plot the normalized histogram especificando un ancho de barra manual óptimo
    optimal_width = 0.35 * sigma; 
    
    h = histogram(ax, current_data, 'BinWidth', optimal_width, 'Normalization', 'pdf', ...
                  'FaceColor', [0.2, 0.6, 0.8], 'EdgeColor', 'w', 'FaceAlpha', 0.6);
    hold(ax, 'on');
    
    % 5. Generate X range to evaluate the theoretical Gaussian curve
    x_range = linspace(min(current_data), max(current_data), 300);
    y_gauss = (1 / (sigma * sqrt(2 * pi))) * exp(-(x_range - mu).^2 / (2 * sigma^2));
    
    % 6. Plot the Gaussian bell curve
    plot(ax, x_range, y_gauss, 'r-', 'LineWidth', 2.5);
    
    % 7. Labels y Títulos estructurados
    title(ax, titles{i}, 'FontSize', title_font_size, 'FontWeight', 'bold', 'FontName', 'Times New Roman');
    xlabel(ax, x_labels{i}, 'FontSize', label_font_size, 'FontWeight', 'bold', 'FontName', 'Times New Roman');
    ylabel(ax, 'Probability Density', 'FontSize', label_font_size, 'FontWeight', 'bold', 'FontName', 'Times New Roman');
    
    % 8. Add legend matching the font style
    legend_text = sprintf('Data (N = %d)\nGaussian Fit:\n\\mu = %.4g\n\\sigma = %.4g', ...
                          length(current_data), mu, sigma);
    lgd = legend(ax, {'Histogram', legend_text}, 'Location', 'best', ...
                 'FontSize', legend_font_size, 'FontName', 'Times New Roman');
    set(lgd, 'FontWeight', 'bold'); 
    
    grid(ax, 'on');
    hold(ax, 'off');
    
    % Asegurar que la ventana gráfica se procese por completo antes de exportar
    drawnow;
    
    %% Export Options
    % Definir nombre único uniendo el nombre del CSV con el componente
    actual_name = sprintf('%s_Histogram_%s', base_name, export_suffixes{i});
    
    % PDF export
    filepath_pdf = fullfile(files(1).folder, 'PDF');
    if ~exist(filepath_pdf, 'dir')
        mkdir(filepath_pdf);
    end
    filename_pdf = sprintf('%s.pdf', actual_name);
    fullPath_pdf = fullfile(filepath_pdf, filename_pdf);
    exportgraphics(fig, fullPath_pdf, 'ContentType', 'vector');
    
    % PNG export
    filepath_png = fullfile(files(1).folder, 'PNG');
    if ~exist(filepath_png, 'dir')
        mkdir(filepath_png);
    end
    filename_png = sprintf('%s.png', actual_name);
    fullPath_png = fullfile(filepath_png, filename_png);
    exportgraphics(fig, fullPath_png, 'Resolution', 300);
end