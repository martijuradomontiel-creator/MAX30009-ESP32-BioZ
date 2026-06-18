close all;
clear;

%% User Options
plot_mode = 'fullscreen'; 

%% Data Loading
files = dir('*.csv');
name = string({files.name});
complete_route = fullfile({files.folder}, {files.name});

sorted_name = natsortfiles(name);
sorted_route = natsortfiles(complete_route);

% Constants and Setup
const = 0.006621284;
num_values = height(readtable(string(complete_route(1))));
num_files = length(name);

% Pre-allocate arrays for memory efficiency
I_ohm = zeros(num_values,num_files);
Q_ohm = zeros(num_values,num_files);
Z = zeros(num_values,num_files);
Phase_meas = zeros(num_values,num_files);
error_Z_percent = zeros(1, num_files);
error_phase_deg = zeros(1, num_files);
capacitance_farad_list = zeros(1, num_files);

res = zeros(1, num_files);
cap = zeros(1, num_files);
cap_units = strings(1, num_files); 

%% Define Fixed Dimensions plots
% Plot area size (The box containing the curves)
axes_width = 18;  % cm
axes_height = 12; % cm

% Margins and Gaps
margin_left = 2.0; % cm
margin_bottom = 1.5; % cm
margin_right = 2.0; % cm
margin_top = 1.5; % cm
gap_x = 2.5; % Espacio horizontal entre gráficas
gap_y = 2.0; % Espacio vertical entre gráficas

% Calculate total figure size for 2 columns
total_width = margin_left + (axes_width * 2) + gap_x + margin_right;
total_height = margin_bottom + (axes_height * 2) + gap_y + margin_top;

%% Main loop
for i = 1:num_files
    % --- Read Data ---
    actual_name = sorted_name(i);
    
    % --- DINAMIC DETECTION OF R, C AND UNIT ---
    if contains(actual_name, '_uF.csv', 'IgnoreCase', true)
        read_values = sscanf(actual_name, '%d_Ohm_%d_uF.csv');
        unit_text = "\muF";
        mult = 1e-6;
    elseif contains(actual_name, '_nF.csv', 'IgnoreCase', true)
        read_values = sscanf(actual_name, '%d_Ohm_%d_nF.csv');
        if read_values(2)<1000
            unit_text = "nF";
            mult = 1e-9;
        else
            read_values(2) = read_values(2)/1000;
            unit_text = "uF";
            mult = 1e-6;
        end
    elseif contains(actual_name, '_pF.csv', 'IgnoreCase', true)
        read_values = sscanf(actual_name, '%d_Ohm_%d_pF.csv');
        if read_values(2)<1000
            unit_text = "pF";
            mult = 1e-12;
        else
            read_values(2) = read_values(2)/1000;
            unit_text = "nF";
            mult = 1e-9;
        end
    else
        read_values = []; 
        unit_text = "?F";
        mult = 1;
    end
    if length(read_values) == 2
        res(i) = read_values(1); 
        cap(i) = read_values(2); 
        cap_units(i) = unit_text;
    else
        warning("R and C couldn't be red in the file: " + actual_name);
    end
    
    % Frequency (Col A)
    f_kHz = double(readtable(string(sorted_route{i})).Frequency_kHz_);
    f = f_kHz * 1000;

    % Data
    Z(:,i) = double(readtable(string(sorted_route{i})).Magnitude);
    % Phase_meas(:,i) = double(readtable(string(sorted_route{i})).Angle);
    I_ohm(:,i) = double(readtable(string(sorted_route{i})).Real);
    Q_ohm(:,i) = double(readtable(string(sorted_route{i})).Imaginary);
    
    % --- Calculations ---
    % I_ohm(:,i) = I_counts(:,i) * const;
    % Q_ohm(:,i) = Q_counts(:,i) * const;
    % Z(:,i) = sqrt(Q_ohm(:,i).^2 + I_ohm(:,i).^2);
    Phase_meas(:,i) = atan2d(Q_ohm(:,i), I_ohm(:,i));
    
    % --- THEORETICAL RC PARALLEL IMPEDANCE ---
    C_farad = cap(i) * mult;
    w = 2 * pi * f; % Velocidad angular para simplificar las fórmulas
    denominador = 1 + (w .* res(i) .* C_farad).^2;
    
    I_theo = res(i) ./ denominador;
    Q_theo = -(w .* res(i)^2 .* C_farad) ./ denominador;
    Z_theo = res(i) ./ sqrt(denominador);
    Phase_theo = atan2d(Q_theo, I_theo);

    capacitance_farad_list(i) = C_farad;
    error_Z_percent(i) = mean(abs((Z(:,i) - Z_theo) ./ Z_theo)) * 100;
    error_phase_deg(i) = mean(abs(Phase_meas(:,i) - Phase_theo));
    
    % --- Plotting ---
    fig = figure(i);
    
    % APPLY WINDOW MODE
    if strcmp(plot_mode, 'fullscreen')
        set(fig, 'WindowState', 'maximized');
    elseif strcmp(plot_mode, 'custom')
        set(fig, 'Units', 'centimeters');
        set(fig, 'Position', [0, 0, total_width, total_height * 2.1]);
    end
    
    % =========================================================
    % 1. UPPER SUBFIGURE (I y Q)
    % =========================================================
    ax1 = subplot(2, 2, [1, 3]);
    
    % --- 1. Calcular los límites GLOBALES primero ---
    % Mínimo y máximo de la parte real (I)
    min_i = min([min(I_ohm(:,i)), min(I_theo)]);
    max_i = max([max(I_ohm(:,i)), max(I_theo)]);
    
    % Mínimo y máximo de la parte imaginaria (Q)
    min_q = min([min(abs(Q_ohm(:,i))), min(abs(Q_theo))]);
    max_q = max([max(abs(Q_ohm(:,i))), max(abs(Q_theo))]);
    
    % Límite global (el más pequeño de los mínimos y el más grande de los máximos)
    min_global = min(min_i, min_q);
    max_global = max(max_i, max_q);
    
    rango_global = max_global - min_global;
    if rango_global == 0
        rango_global = 1e-3; % Protección
    end
    
    % Aplicamos el 15% de margen sobre el rango global
    limite_inf = min_global - 0.15 * rango_global;
    limite_sup = max_global + 0.15 * rango_global;
    
    % Evitamos que el límite inferior baje de 0 si tus datos originales no son negativos
    if limite_inf < 0 && min_global >= 0
        limite_inf = 0;
    end

    % --- 2. Dibujar Left Axis (I -> Re(Z)) ---
    yyaxis(ax1, "left");
    semilogx(f, I_ohm(:,i), 'LineWidth', 1.5, 'color', 'b');
    hold on;
    semilogx(f, I_theo, 'b--', 'LineWidth', 1.5);
    hold off;
    
    ylim([limite_inf, limite_sup]); % Asignamos el límite global
    ylabel('Re(Z) [\Omega]', 'FontWeight', 'bold', 'FontSize', 14);
    
    % --- 3. Dibujar Right Axis (Q -> Im(Z)) ---
    yyaxis(ax1, "right");
    color_naranja = [0.8500, 0.3250, 0.0980]; 
    
    semilogx(f, abs(Q_ohm(:,i)), 'LineWidth', 1.5, 'Color', color_naranja);
    hold on;
    semilogx(f, abs(Q_theo), '--', 'LineWidth', 1.5, 'Color', color_naranja); 
    hold off;
    
    ylim([limite_inf, limite_sup]); % ¡Asignamos EXACTAMENTE el mismo límite global!
    ylabel('Im(Z) [\Omega]', 'FontWeight', 'bold', 'FontSize', 14);
    
    % --- 4. Configuración general de la gráfica ---
    xlabel('Frequency [Hz]', 'FontWeight', 'bold', 'FontSize', 16);
    grid on;
    xlim([min(f) max(f)]);
    legend('Measured R', 'Theoretical R', 'Measured |X|', 'Theoretical |X|', 'Location', 'northeast');
    title("MAX30009 2-Wire Parallel RC: " + res(i) + " \Omega || " + cap(i) + " " + cap_units(i), 'FontSize', 16); 
    set(ax1, 'FontName', 'Times New Roman', 'FontSize', 14);
    
    if strcmp(plot_mode, 'custom')
        set(ax1, 'Units', 'centimeters');
        set(ax1, 'Position', [margin_left, margin_bottom, axes_width, (axes_height * 2) + gap_y]);
    end
    
    % =========================================================
    % 2. TOP RIGHT SUBFIGURE (Impedance |Z|) - Posición 2
    % =========================================================
    ax2 = subplot(2, 2, 2);
    
    loglog(f, Z(:,i), 'k', 'LineWidth', 1.5);
    hold on;
    loglog(f, Z_theo, 'r--', 'LineWidth', 1.5); 
    hold off;
    
    min_z = min([min(Z(:,i)), min(Z_theo)]);
    max_z = max([max(Z(:,i)), max(Z_theo)]);
    ylim([min_z * 0.8, max_z * 1.2]);
    ylabel('|Z| [\Omega]', 'FontWeight', 'bold', 'FontSize', 14);
    legend('Measured |Z|', 'Theoretical |Z|', 'Location', 'southwest');
    
    grid on;
    xlim([min(f) max(f)]);
    title('Magnitude', 'FontSize', 14);
    set(ax2, 'FontName', 'Times New Roman', 'FontSize', 14);
    
    if strcmp(plot_mode, 'custom')
        set(ax2, 'Units', 'centimeters');
        set(ax2, 'Position', [margin_left + axes_width + gap_x, margin_bottom + axes_height + gap_y, axes_width, axes_height]);
    end

    % =========================================================
    % 3. BOTTOM RIGHT SUBFIGURE (Phase) - Posición 4
    % =========================================================
    ax3 = subplot(2, 2, 4);
    
    semilogx(f, Phase_meas(:,i), 'k', 'LineWidth', 1.5);
    hold on;
    semilogx(f, Phase_theo, 'r--', 'LineWidth', 1.5);
    hold off;
    
    % --- CÁLCULO DINÁMICO DE LOS LÍMITES DE FASE ---
    % 1. Encontramos la fase más pequeña (la más negativa) y la mayor
    min_phase = min([min(Phase_meas(:,i)), min(Phase_theo)]);
    max_phase = max([max(Phase_meas(:,i)), max(Phase_theo)]);
    
    % 2. Redondeamos hacia abajo al múltiplo de 15 más cercano. 
    % Usamos floor() para forzarlo a ir hacia el número más negativo.
    lim_inf_phase = floor(min_phase / 15) * 15;
    
    % 3. Ajustamos el límite superior
    lim_sup_phase = ceil(max_phase / 15) * 15;
    if lim_sup_phase <= 0
        lim_sup_phase = 5; % Damos un margen de +5º para que la línea del 0 no toque el techo
    end
    
    % Aplicamos los límites calculados
    ylim([lim_inf_phase, lim_sup_phase]);
    
    % Mantenemos tus marcas maestras. MATLAB solo dibujará las que entren en el nuevo ylim.
    yticks([-180 -165 -150 -135 -120 -105 -90 -75 -60 -45 -30 -15 0]); 
    
    ylabel('Phase (\theta) [deg]', 'FontWeight', 'bold', 'FontSize', 14);
    xlabel('Frequency [Hz]', 'FontWeight', 'bold', 'FontSize', 16);
    legend('Measured \theta', 'Theoretical \theta', 'Location', 'southwest');
    
    grid on;
    xlim([min(f) max(f)]);
    title('Phase', 'FontSize', 14);
    set(ax3, 'FontName', 'Times New Roman', 'FontSize', 14);
    
    if strcmp(plot_mode, 'custom')
        set(ax3, 'Units', 'centimeters');
        set(ax3, 'Position', [margin_left + axes_width + gap_x, margin_bottom, axes_width, axes_height]);
    end

    %% Export Options (Comment if just debuging)
    % % PDF export
    % filepath_pdf = fullfile(files(1).folder, 'PDF');
    % if ~exist(filepath_pdf, 'dir')
    %     mkdir(filepath_pdf);
    % end
    % filename_pdf = replace(string(actual_name), ".csv", ".pdf");
    % fullPath_pdf = fullfile(filepath_pdf, filename_pdf);
    % exportgraphics(fig, fullPath_pdf, 'ContentType', 'vector');
    % 
    % %PNG export
    % filepath_png = fullfile(files(1).folder, 'PNG');
    % if ~exist(filepath_png, 'dir')
    %     mkdir(filepath_png);
    % end
    % filename_png = replace(string(actual_name), ".csv", ".png");
    % fullPath_png = fullfile(filepath_png, filename_png);
    % exportgraphics(fig, fullPath_png, 'Resolution', 300);
    % 
    close(i);
end

%% =========================================================
% ESTUDIO DE ERROR GLOBAL
% =========================================================
% Ordenamos los arrays de menor a mayor capacitancia para que la línea se dibuje bien
[C_sorted, sort_idx] = sort(capacitance_farad_list);
Err_Z_sorted = error_Z_percent(sort_idx);
Err_Phase_sorted = error_phase_deg(sort_idx);

fig_err = figure(num_files + 1);

% APPLY WINDOW MODE FOR ERROR PLOT
if strcmp(plot_mode, 'fullscreen')
    set(fig_err, 'WindowState', 'maximized');
elseif strcmp(plot_mode, 'custom')
    set(fig_err, 'Units', 'centimeters');
    set(fig_err, 'Position', [0, 0, total_width, total_height]);
end

% --- DEFINICIÓN DE LAS REGIONES DE TOLERANCIA ---
% Valores de C y su correspondiente tolerancia máxima en %
C_tol_refs = [2.2e-9, 10e-9, 100e-9, 470e-9, 680e-9];
Err_tol_refs = [10, 15, 15, 25, 25];

% Creamos las coordenadas para el polígono del área sombreada
% Empezamos en 0, subimos a la tolerancia, recorremos los puntos, bajamos a 0 y cerramos.
C_poly = [C_tol_refs(1), C_tol_refs, C_tol_refs(end), C_tol_refs(1)];
Err_poly = [0, Err_tol_refs, 0, 0];

% =========================================================
% 1. Gráfica de error en Magnitud (con región de tolerancia)
% =========================================================
subplot(2, 1, 1);

% FORZAMOS EL EJE X A LOGARÍTMICO ANTES DE PINTAR NADA
set(gca, 'XScale', 'log'); 

% 1º Pintamos el área sombreada al fondo 
patch(C_poly, Err_poly, 'r', 'FaceAlpha', 0.1, 'EdgeColor', 'r', 'LineStyle', '--', 'LineWidth', 1.5);
hold on;

% 2º Pintamos nuestras medidas reales por encima
semilogx(C_sorted, Err_Z_sorted, '-o', 'LineWidth', 1.5, 'MarkerSize', 6, 'MarkerFaceColor', 'b', 'Color', 'b');
hold off;

grid on;
ylabel('Mean |Z| Error (%)', 'FontWeight', 'bold', 'FontSize', 14);
title('Measurement Error vs Capacitance Value', 'FontSize', 16);
set(gca, 'FontName', 'Times New Roman', 'FontSize', 14);

% Añadimos leyenda para que quede claro qué es la franja roja
legend('Expected Error Tolerance', 'Measured Error', 'Location', 'northwest');

% Ajustamos los límites de X para que la gráfica no quede flotando
xlim([min(C_tol_refs)*0.8, max(C_tol_refs)*1.2]); 

% =========================================================
% 2. Gráfica de error en Fase
% =========================================================
subplot(2, 1, 2);

% AUNQUE AQUÍ SOLO HAY SEMILOGX, LO PONEMOS POR SEGURIDAD Y SIMETRÍA
set(gca, 'XScale', 'log'); 

color_naranja = [0.8500, 0.3250, 0.0980];
semilogx(C_sorted, Err_Phase_sorted, '-o', 'LineWidth', 1.5, 'MarkerSize', 6, 'MarkerFaceColor', color_naranja, 'Color', color_naranja);

grid on;
xlabel('Capacitance (Farads)', 'FontWeight', 'bold', 'FontSize', 16);
ylabel('Mean Phase Error (^\circ)', 'FontWeight', 'bold', 'FontSize', 14);
set(gca, 'FontName', 'Times New Roman', 'FontSize', 14);

% Aplicamos el mismo límite X que arriba para simetría
xlim([min(C_tol_refs)*0.8, max(C_tol_refs)*1.2]); 

% %% Export Options (Comment if just debuging)
% % PDF export
% filepath_pdf = fullfile(files(1).folder, 'PDF');
% if ~exist(filepath_pdf, 'dir')
%     mkdir(filepath_pdf);
% end
% filename_pdf = "error.pdf";
% fullPath_pdf = fullfile(filepath_pdf, filename_pdf);
% exportgraphics(fig_err, fullPath_pdf, 'ContentType', 'vector');
% 
% %PNG export
% filepath_png = fullfile(files(1).folder, 'PNG');
% if ~exist(filepath_png, 'dir')
%     mkdir(filepath_png);
% end
% filename_png = "error.png";
% fullPath_png = fullfile(filepath_png, filename_png);
% exportgraphics(fig_err, fullPath_png, 'Resolution', 300);
% 
% close(num_files + 1);