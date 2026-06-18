close all;
clear;

%% 1. Experiment Configuration
% Definimos los archivos, el tipo de análisis y los títulos para cada caso
experimentos = {
    'frequencyValues_força.csv',        'Temporal',    'Bioimpedance during Muscle Force (Forearm)';
    'frequencyValues_pulsacions2.csv',  'Temporal',    'Bioimpedance for Pulse Detection (Apnea)';
    'frequencyValues_respiració1.csv',  'Temporal',    'Bioimpedance during Respiration';
    'frequencyValues_right_side.csv',   'Frequencial', 'Right Side Body Composition (5 kHz - 200 kHz)'
};
num_files = size(experimentos, 1);

%% 2. Data Processing and Plotting
for k = 1:num_files
    filename = experimentos{k, 1};
    tipo = experimentos{k, 2};
    plot_title = experimentos{k, 3};
    
    % Comprobar si el archivo existe
    if ~isfile(filename)
        warning('No se encontró el archivo: %s. Saltando...', filename);
        continue;
    end
    
    fprintf('\n------------------------------------------\n');
    fprintf('Procesando: %s (%s)...\n', filename, tipo);
    data_table = readtable(filename);
    
    % Extraer variables (si hay NaN o celdas de texto, las forzamos a número)
    Z = double(data_table.Magnitude);
    I = double(data_table.Real);
    Q = double(data_table.Imaginary);
    Phase = atan2d(Q, I);
    
    varNames = data_table.Properties.VariableNames;
    
    %% 3. Asignación Inteligente del Eje X
    if strcmp(tipo, 'Frequencial')
        % Buscar columna de frecuencia
        idx_freq = find(contains(lower(varNames), 'freq'), 1);
        
        if ~isempty(idx_freq)
            x_data = double(data_table{:, idx_freq});
            
            % CASO A: Los valores están en kHz (ej. 5 a 200) en lugar de Hz
            if max(x_data) <= 500
                fprintf(' -> ALERTA: Valores en kHz detectados. Convirtiendo a Hz...\n');
                x_data = x_data * 1000;
            end
            
            % CASO B: Los valores son simples índices (1 a N)
            if max(x_data) < 5000
                fprintf(' -> ALERTA: Valores X no coinciden con la escala. Forzando vector de 5k a 200k...\n');
                x_data = logspace(log10(5000), log10(200000), length(Z))';
            end
            
            x_label = 'Frequency [Hz]';
        else
            % CASO C: No existe columna de frecuencia
            fprintf(' -> ALERTA: No hay columna de frecuencia. Forzando vector logarítmico de 5k a 200k...\n');
            x_data = logspace(log10(5000), log10(200000), length(Z))';
            x_label = 'Frequency [Hz]';
        end
        
    else
        % MODO TEMPORAL
        idx_time = find(contains(lower(varNames), 'time') | contains(lower(varNames), 'tiempo'), 1);
        if ~isempty(idx_time)
            x_data = double(data_table{:, idx_time});
            x_label = 'Time [s]';
        else
            x_data = (1:length(Z))';
            x_label = 'Sample Number';
        end
    end
    
    %% 4. Crear Figura
    fig = figure('Units', 'normalized');
    fig.WindowState = 'maximized';
    
    % --- TOP SUBPLOT: Impedance Magnitude (Z) ---
    ax1 = subplot(2,1,1);
    if strcmp(tipo, 'Frequencial')
        semilogx(ax1, x_data, Z, '-o', 'LineWidth', 1, 'MarkerSize', 4, 'Color', [0.2 0.4 0.8], 'MarkerFaceColor', 'w');
    else
        plot(ax1, x_data, Z, '-', 'LineWidth', 1, 'Color', [0.2 0.4 0.8]);
    end
    
    title(ax1, plot_title, 'FontSize', 16, 'FontWeight', 'bold', 'FontName', 'Times New Roman');
    ylabel(ax1, 'Magnitude Z [\Omega]', 'FontSize', 14, 'FontWeight', 'bold', 'FontName', 'Times New Roman');
    grid(ax1, 'on');
    set(ax1, 'FontName', 'Times New Roman', 'FontSize', 12, 'FontWeight', 'bold', ...
             'LineWidth', 1, 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15]);
         
    % --- BOTTOM SUBPLOT: Phase ---
    ax2 = subplot(2,1,2);
    if strcmp(tipo, 'Frequencial')
        semilogx(ax2, x_data, Phase, '-s', 'LineWidth', 1, 'MarkerSize', 4, 'Color', [0.85 0.3 0.2], 'MarkerFaceColor', 'w');
    else
        plot(ax2, x_data, Phase, '-', 'LineWidth', 1, 'Color', [0.85 0.3 0.2]);
    end
    
    xlabel(ax2, x_label, 'FontSize', 14, 'FontWeight', 'bold', 'FontName', 'Times New Roman');
    ylabel(ax2, 'Phase [deg]', 'FontSize', 14, 'FontWeight', 'bold', 'FontName', 'Times New Roman');
    grid(ax2, 'on');
    set(ax2, 'FontName', 'Times New Roman', 'FontSize', 12, 'FontWeight', 'bold', ...
             'LineWidth', 1, 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15]);
         
    %% 5. Ajustes Exclusivos de los Ejes X
    if strcmp(tipo, 'Frequencial')
        % Forzar límites
        xlim(ax1, [5000, 200000]);
        xlim(ax2, [5000, 200000]);
        
        % Aplicar Ticks
        freq_ticks = [5000:1000:9000, 10000:10000:90000, 100000, 200000];
        freq_labels = {'5k','6k','7k','8k','9k', '10k','20k','30k','40k','50k', ...
                       '60k','70k','80k','90k', '100k','200k'};
        
        set([ax1, ax2], 'XTick', freq_ticks, 'XTickLabel', freq_labels);
    else
        % Modo temporal
        t_min = min(x_data);
        t_max_entero = floor(max(x_data)); 
        
        if t_max_entero > t_min
            xlim(ax1, [t_min, t_max_entero]);
            xlim(ax2, [t_min, t_max_entero]);
        end
    end
    
    % Sincronizar ejes
    linkaxes([ax1, ax2], 'x');
    drawnow;
    
    %% 6. Export Options
    [~, base_name, ~] = fileparts(filename);
    
    filepath_pdf = fullfile(pwd, 'PDF');
    if ~exist(filepath_pdf, 'dir'), mkdir(filepath_pdf); end
    exportgraphics(fig, fullfile(filepath_pdf, sprintf('%s_Analysis.pdf', base_name)), 'ContentType', 'vector');
    
    filepath_png = fullfile(pwd, 'PNG');
    if ~exist(filepath_png, 'dir'), mkdir(filepath_png); end
    exportgraphics(fig, fullfile(filepath_png, sprintf('%s_Analysis.png', base_name)), 'Resolution', 300);
end
fprintf('\n¡Análisis y exportación completados!\n');