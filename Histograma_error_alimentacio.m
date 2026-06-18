close all;
clear;

%% 1. Data Loading and Sorting
% Buscar todos los archivos que coincidan con el patrón
files = dir('timeValues_*.csv');
if isempty(files)
    error('No se encontraron archivos timeValues_*.csv en el directorio actual.');
end

num_files = length(files);
volts = zeros(num_files, 1);

% Extraer el voltaje del nombre de cada archivo
for k = 1:num_files
    tokens = regexp(files(k).name, 'timeValues_([\d\.]+)\.csv', 'tokens');
    if ~isempty(tokens)
        volts(k) = str2double(tokens{1}{1});
    else
        error('El archivo %s no sigue el patrón timeValues_X.csv', files(k).name);
    end
end

% Ordenar los archivos de mayor a menor voltaje (3.3V primero)
[volts, sort_idx] = sort(volts, 'descend');
files = files(sort_idx);

%% 2. Data Processing
% Inicializar matrices para guardar medias (mu) y ruidos/desviaciones (sigma)
mu_Z = zeros(num_files, 1); sigma_Z = zeros(num_files, 1);
mu_I = zeros(num_files, 1); sigma_I = zeros(num_files, 1);
mu_Q = zeros(num_files, 1); sigma_Q = zeros(num_files, 1);
mu_Phase = zeros(num_files, 1); sigma_Phase = zeros(num_files, 1);

fprintf('Procesando datos para %d niveles de voltaje...\n', num_files);

for k = 1:num_files
    data_table = readtable(fullfile(files(k).folder, files(k).name));
    
    Z_temp = double(data_table.Magnitude);
    I_temp = double(data_table.Real);
    Q_temp = double(data_table.Imaginary);
    Phase_temp = atan2d(Q_temp, I_temp); % Calcular fase
    
    % Calcular parámetros estadísticos
    mu_Z(k) = mean(Z_temp, 'omitnan'); sigma_Z(k) = std(Z_temp, 'omitnan');
    mu_I(k) = mean(I_temp, 'omitnan'); sigma_I(k) = std(I_temp, 'omitnan');
    mu_Q(k) = mean(Q_temp, 'omitnan'); sigma_Q(k) = std(Q_temp, 'omitnan');
    mu_Phase(k) = mean(Phase_temp, 'omitnan'); sigma_Phase(k) = std(Phase_temp, 'omitnan');
end

% Identificar los valores de referencia (el primer elemento, asumiendo 3.3V)
ref_Z = mu_Z(1);
ref_I = mu_I(1);
ref_Q = mu_Q(1);
ref_Phase = mu_Phase(1);

% Calcular el error absoluto (Deriva) respecto a 3.3V
error_Z = mu_Z - ref_Z;
error_I = mu_I - ref_I;
error_Q = mu_Q - ref_Q;
error_Phase = mu_Phase - ref_Phase;

%% 3. Plotting Results
data_mu = {mu_Z, mu_I, mu_Q, mu_Phase};
data_sigma = {sigma_Z, sigma_I, sigma_Q, sigma_Phase};
data_error = {error_Z, error_I, error_Q, error_Phase};

titles = {'Impedance Magnitude (Z)', 'In-Phase Component (I)', ...
          'Quadrature Component (Q)', 'Phase'};
units = {'[\Omega]', '[\Omega]', '[\Omega]', '[deg]'};

% Nombres seguros para exportar los archivos
export_names = {'Z', 'I', 'Q', 'Phase'};

for i = 1:4
    fig = figure('Units', 'normalized');
    fig.WindowState = 'maximized';
    
    % --- TOP SUBPLOT: Mean Value with Noise (3-Sigma Error Bars) ---
    ax1 = subplot(2,1,1);
    
    errorbar(ax1, volts, data_mu{i}, 3*data_sigma{i}, '-o', 'LineWidth', 1, ...
             'MarkerSize', 6, 'MarkerFaceColor', 'w', 'Color', [0.2 0.4 0.8]);
    
    title(ax1, sprintf('%s vs Supply Voltage', titles{i}), ...
          'FontSize', 16, 'FontWeight', 'bold', 'FontName', 'Times New Roman');
    ylabel(ax1, sprintf('Measured Value %s', units{i}), ...
           'FontSize', 14, 'FontWeight', 'bold', 'FontName', 'Times New Roman');
    
    grid(ax1, 'on');
    
    set(ax1, 'FontName', 'Times New Roman', 'FontSize', 12, 'FontWeight', 'bold', ...
             'LineWidth', 1, 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], ...
             'XDir', 'reverse'); 
    
    lgd1 = legend(ax1, 'Mean \pm 3\sigma (Noise)', 'Location', 'best');
    set(lgd1, 'FontName', 'Times New Roman', 'FontSize', 12, 'FontWeight', 'bold');
         
    % --- BOTTOM SUBPLOT: Absolute Error vs 3.3V Baseline ---
    ax2 = subplot(2,1,2);
    
    plot(ax2, volts, data_error{i}, '-s', 'LineWidth', 1, ...
         'MarkerSize', 6, 'MarkerFaceColor', 'w', 'Color', [0.85 0.3 0.2]);
    hold(ax2, 'on');
    
    yline(ax2, 0, 'k--', 'LineWidth', 1.0);
    hold(ax2, 'off');
    
    title(ax2, 'Absolute Error relative to 3.3 V Baseline', ...
          'FontSize', 14, 'FontWeight', 'bold', 'FontName', 'Times New Roman');
    xlabel(ax2, 'Supply Voltage [V]', 'FontSize', 14, 'FontWeight', 'bold', 'FontName', 'Times New Roman');
    ylabel(ax2, sprintf('Absolute Error \\Delta %s', units{i}), ...
           'FontSize', 14, 'FontWeight', 'bold', 'FontName', 'Times New Roman');
    
    grid(ax2, 'on');
    
    set(ax2, 'FontName', 'Times New Roman', 'FontSize', 12, 'FontWeight', 'bold', ...
             'LineWidth', 1, 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], ...
             'XDir', 'reverse'); 
         
    lgd2 = legend(ax2, {'Measurement Drift', 'Ideal Baseline (0 Error)'}, 'Location', 'best');
    set(lgd2, 'FontName', 'Times New Roman', 'FontSize', 12, 'FontWeight', 'bold');
    
    % Dar tiempo a MATLAB para renderizar la figura maximizada antes de exportar
    drawnow;
    
    %% Export Options (Comment if just debuging)
    % Nombre base para los archivos de salida
    actual_name = sprintf('Voltage_Degradation_%s', export_names{i});
    
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