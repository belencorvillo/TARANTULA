import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Configuración de tipografía y estilo académico (IEEE/Springer)
plt.rcParams.update({
    'font.family': 'serif',          # Fuente serif para tono de tesis/TFG
    'font.serif': ['Times New Roman', 'DejaVu Serif'],
    'font.size': 10,
    'axes.labelsize': 10,
    'axes.titlesize': 11,
    'xtick.labelsize': 9,
    'ytick.labelsize': 9,
    'figure.titlesize': 13,
    'legend.fontsize': 9,
    'savefig.dpi': 300
})

# Archivos de entrada para los 3 perfiles de rigidez
ENSAYOS = {
    'Rigidez 3': 'telemetria_rigidez_3.csv',
    'Rigidez 4': 'telemetria_rigidez_4.csv',
    'Rigidez 5': 'telemetria_rigidez_5.csv'
}

# Colores de publicación académica (IEEE de alto contraste)
COLORES = {
    'Rigidez 3': '#0f4c81',  # Azul oscuro
    'Rigidez 4': '#f58220',  # Naranja
    'Rigidez 5': '#375a23'   # Verde oscuro
}

def detect_push_index(df, threshold_deg=0.3):
    """
    Detecta automáticamente el índice donde comienza el empujón analizando el error de posición.
    """
    err_cols = [c for c in df.columns if '_error_deg' in c]
    if not err_cols:
        return 0
    
    abs_err_mean = df[err_cols].abs().mean(axis=1)
    baseline_samples = min(30, len(df))
    baseline = abs_err_mean.iloc[:baseline_samples].mean()
    
    for idx in range(baseline_samples, len(df)):
        if abs_err_mean.iloc[idx] > baseline + threshold_deg:
            return idx
            
    return abs_err_mean.idxmax()

def resolve_path(filename):
    """
    Busca un archivo en el directorio actual o recursivamente dentro de 'build'.
    """
    if os.path.exists(filename):
        return filename
    
    if os.path.exists('build'):
        for root, dirs, files in os.walk('build'):
            if filename in files:
                return os.path.join(root, filename)
                
    return None

def apply_academic_styling(ax):
    """
    Aplica bordes limpios (sin top/right spines), grilla suave y colores profesionales.
    """
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.spines['left'].set_color('#cccccc')
    ax.spines['bottom'].set_color('#cccccc')
    ax.spines['left'].set_linewidth(0.8)
    ax.spines['bottom'].set_linewidth(0.8)
    ax.grid(True, which='both', linestyle=':', linewidth=0.5, color='#dddddd')
    ax.set_facecolor('white')

def process_and_plot():
    # Resolver rutas de archivos
    resolved_paths = {}
    missing_files = []
    
    for label, filename in ENSAYOS.items():
        path = resolve_path(filename)
        if path:
            resolved_paths[label] = path
        else:
            missing_files.append(filename)
            
    if missing_files:
        print("[Error] No se encuentran los siguientes archivos de telemetria:")
        for mf in missing_files:
            print(f"   - {mf}")
        print("\n[Tip] Asegurate de realizar los ensayos en el robot. Si usas Qt Creator, los archivos")
        print("se guardan en la carpeta de compilacion 'build/'. Este script los buscara alli automaticamente.")
        return

    # Estructura de patas solicitada por el usuario (solo el nombre de la pata)
    patas = {
        1: 'Pata 1',
        2: 'Pata 2',
        3: 'Pata 3',
        4: 'Pata 4'
    }

    # Cargar y alinear datos
    datasets = {}
    for label in ENSAYOS.keys():
        file_path = resolved_paths[label]
        df = pd.read_csv(file_path)
        idx_push = detect_push_index(df)
        t_push_ms = df['timestamp_ms'].iloc[idx_push]
        
        # Alinear el empujón en t = 2.0s
        df['time_rel'] = 2.0 + (df['timestamp_ms'] - t_push_ms) / 1000.0
        
        err_cols = [c for c in df.columns if '_error_deg' in c]
        abs_err_mean = df[err_cols].abs().mean(axis=1)
        baseline = abs_err_mean.iloc[:min(30, len(df))].mean()
        
        # Calcular tiempo de estabilización
        t_settle_rel = 2.0
        for idx in range(idx_push, len(df)):
            if abs_err_mean.iloc[idx] > baseline + 0.15:
                t_settle_rel = df['time_rel'].iloc[idx]
        
        datasets[label] = {
            'df': df,
            't_push_idx': idx_push,
            't_settle_rel': t_settle_rel
        }
    
    # Ventana de graficación
    max_settle_time = max(data['t_settle_rel'] for data in datasets.values())
    t_end = max_settle_time + 3.0
    t_end = max(6.0, min(12.0, t_end))
    
    # Detectar formato de columnas (antiguo vs nuevo con joint j2)
    first_label = list(datasets.keys())[0]
    df_first = datasets[first_label]['df']
    j2_suffix = '_j2' if 'leg1_j2_actual_deg' in df_first.columns else ''

    # =========================================================================
    # FIGURA 1: TORQUE
    # =========================================================================
    fig_torque, axes_t = plt.subplots(2, 2, figsize=(10, 7.5), sharex=True, sharey=True)
    fig_torque.patch.set_facecolor('white')
    axes_t_flat = axes_t.flatten()

    for i, leg_id in enumerate(patas.keys()):
        ax = axes_t_flat[i]
        apply_academic_styling(ax)
        col_torque = f'leg{leg_id}{j2_suffix}_torque_nm'
        
        for label, data in datasets.items():
            df_plot = data['df'][(data['df']['time_rel'] >= 0.0) & (data['df']['time_rel'] <= t_end)]
            ax.plot(df_plot['time_rel'], df_plot[col_torque], 
                    color=COLORES[label], linewidth=1.5)

        ax.set_title(patas[leg_id], fontweight='bold')
        ax.axvline(x=2.0, color='#d62728', linestyle=':', linewidth=1.0)
        
        if i in [2, 3]:
            ax.set_xlabel('Tiempo (s)')
        if i in [0, 2]:
            ax.set_ylabel('Torque (Nm)')

    # Crear una leyenda única y limpia en la parte inferior externa para evitar solapamientos
    handles = [plt.Line2D([0], [0], color=COLORES[lbl], lw=1.5, label=lbl) for lbl in ENSAYOS.keys()]
    handles.append(plt.Line2D([0], [0], color='#d62728', linestyle=':', lw=1.0, label='Perturbación (t=2s)'))
    
    fig_torque.legend(handles=handles, loc='lower center', ncol=4, frameon=True, facecolor='white', edgecolor='#e5e7eb')
# fig_torque.suptitle('Respuesta en Torque de la Articulación 2 (Fémur) ante Perturbación Vertical', fontweight='bold', y=0.96)
    
    plt.tight_layout(rect=[0, 0.05, 1, 0.93])
    fig_torque.savefig('comparativa_torque.pdf', format='pdf', dpi=300)
    fig_torque.savefig('comparativa_torque.png', format='png', dpi=300)
    print("Grafica de Torque guardada en comparativa_torque.pdf / .png")

    # Encontrar límites globales de posición para asegurar que compartan el mismo eje Y invertido de forma consistente
    all_pos_vals = []
    for label, data in datasets.items():
        df_p = data['df'][(data['df']['time_rel'] >= 0.0) & (data['df']['time_rel'] <= t_end)]
        for leg_id in patas.keys():
            all_pos_vals.extend(df_p[f'leg{leg_id}{j2_suffix}_actual_deg'].dropna().tolist())
            all_pos_vals.extend(df_p[f'leg{leg_id}{j2_suffix}_target_deg'].dropna().tolist())
    
    y_min = min(all_pos_vals) - 1.5 if all_pos_vals else 24.0
    y_max = max(all_pos_vals) + 1.5 if all_pos_vals else 44.0

    # =========================================================================
    # FIGURA 2: POSICIÓN ABSOLUTA (Con eje Y invertido)
    # =========================================================================
    fig_pos, axes_p = plt.subplots(2, 2, figsize=(10, 7.5), sharex=True, sharey=True)
    fig_pos.patch.set_facecolor('white')
    axes_p_flat = axes_p.flatten()

    for i, leg_id in enumerate(patas.keys()):
        ax = axes_p_flat[i]
        apply_academic_styling(ax)
        
        # Fijar límites invertidos: el valor mayor abajo y el menor arriba
        ax.set_ylim(y_max, y_min)
        
        col_actual = f'leg{leg_id}{j2_suffix}_actual_deg'
        col_target = f'leg{leg_id}{j2_suffix}_target_deg'
        
        # Graficar Setpoint de consigna del primer dataset disponible
        first_label = list(datasets.keys())[0]
        df_first = datasets[first_label]['df']
        df_first_plot = df_first[(df_first['time_rel'] >= 0.0) & (df_first['time_rel'] <= t_end)]
        ax.plot(df_first_plot['time_rel'], df_first_plot[col_target], 
                color='#333333', linestyle='--', linewidth=1.2, alpha=0.9)

        # Graficar posición real para cada rigidez
        for label, data in datasets.items():
            df_plot = data['df'][(data['df']['time_rel'] >= 0.0) & (data['df']['time_rel'] <= t_end)]
            ax.plot(df_plot['time_rel'], df_plot[col_actual], 
                    color=COLORES[label], linewidth=1.5)

        ax.set_title(patas[leg_id], fontweight='bold')
        ax.axvline(x=2.0, color='#d62728', linestyle=':', linewidth=1.0)
        
        if i in [2, 3]:
            ax.set_xlabel('Tiempo (s)')
        if i in [0, 2]:
            ax.set_ylabel('Posición Absoluta (°)')

    # Leyenda única inferior
    handles_p = [plt.Line2D([0], [0], color='#333333', linestyle='--', lw=1.2, label='Consigna (Setpoint)')]
    for lbl in ENSAYOS.keys():
        handles_p.append(plt.Line2D([0], [0], color=COLORES[lbl], lw=1.5, label=f'Real ({lbl})'))
    handles_p.append(plt.Line2D([0], [0], color='#d62728', linestyle=':', lw=1.0, label='Perturbación (t=2s)'))

    fig_pos.legend(handles=handles_p, loc='lower center', ncol=5, frameon=True, facecolor='white', edgecolor='#e5e7eb')
  #  fig_pos.suptitle('Posición Absoluta de la Articulación 2 (Fémur) ante Perturbación Vertical', fontweight='bold', y=0.96)
    
    plt.tight_layout(rect=[0, 0.05, 1, 0.93])
    fig_pos.savefig('comparativa_posicion.pdf', format='pdf', dpi=300)
    fig_pos.savefig('comparativa_posicion.png', format='png', dpi=300)
    print("Grafica de Posicion guardada en comparativa_posicion.pdf / .png")

    # =========================================================================
    # FIGURA 3: ERROR DE SEGUIMIENTO (Por si lo necesitas en el TFG)
    # =========================================================================
    fig_error, axes_e = plt.subplots(2, 2, figsize=(10, 7.5), sharex=True, sharey=True)
    fig_error.patch.set_facecolor('white')
    axes_e_flat = axes_e.flatten()

    for i, leg_id in enumerate(patas.keys()):
        ax = axes_e_flat[i]
        apply_academic_styling(ax)
        col_error = f'leg{leg_id}{j2_suffix}_error_deg'
        
        for label, data in datasets.items():
            df_plot = data['df'][(data['df']['time_rel'] >= 0.0) & (data['df']['time_rel'] <= t_end)]
            ax.plot(df_plot['time_rel'], df_plot[col_error], 
                    color=COLORES[label], linewidth=1.5)

        ax.set_title(patas[leg_id], fontweight='bold')
        ax.axvline(x=2.0, color='#d62728', linestyle=':', linewidth=1.0)
        
        if i in [2, 3]:
            ax.set_xlabel('Tiempo (s)')
        if i in [0, 2]:
            ax.set_ylabel('Error de Posición (°)')

    fig_error.legend(handles=handles, loc='lower center', ncol=4, frameon=True, facecolor='white', edgecolor='#e5e7eb')
    fig_error.suptitle('Error de Posición (Consigna - Real) - Joint 2 (Fémur) ante Perturbación Vertical', fontweight='bold', y=0.96)
    
    plt.tight_layout(rect=[0, 0.05, 1, 0.93])
    fig_error.savefig('comparativa_error.pdf', format='pdf', dpi=300)
    fig_error.savefig('comparativa_error.png', format='png', dpi=300)
    print("Grafica de Error de Posicion guardada en comparativa_error.pdf / .png")

if __name__ == '__main__':
    process_and_plot()
