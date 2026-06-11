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

# Archivos de entrada para las 3 situaciones de carga
ENSAYOS = {
    'Carga: 0 kg': 'telemetria_carga_0kg.csv',
    'Carga: 1 kg': 'telemetria_carga_1kg.csv',
    'Carga: 2 kg': 'telemetria_carga_2kg.csv'
}

# Colores de publicación académica (IEEE de alto contraste)
COLORES = {
    'Carga: 0 kg': '#0f4c81',   # Azul oscuro
    'Carga: 1 kg': '#f58220',  # Naranja
    'Carga: 2 kg': '#375a23'   # Verde oscuro
}

def detect_lift_index(df):
    """
    Detecta automáticamente el índice donde comienza el levantamiento
    analizando cuándo cambia la consigna (target) de la articulación 2 de la Pata 1.
    """
    col_target = 'leg1_j2_target_deg'
    if col_target not in df.columns:
        # Si no existe la columna por defecto, buscar cualquier columna de consigna de joint 2
        cols = [c for c in df.columns if '_j2_target_deg' in c]
        if cols:
            col_target = cols[0]
        else:
            return 0
            
    target_series = df[col_target]
    initial_val = target_series.iloc[0]
    
    # Buscar el primer índice donde la consigna varíe significativamente (más de 0.5 grados)
    for idx in range(1, len(df)):
        if abs(target_series.iloc[idx] - initial_val) > 0.5:
            return idx
            
    return 0

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
    Aplica bordes limpios (sin spines superiores/derechos), grilla suave y fondo blanco.
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
        print("\n[Tip] Asegúrate de realizar los ensayos en el robot con cada carga y guardarlos")
        print("con los siguientes nombres en el panel de la UI:")
        for lbl, fn in ENSAYOS.items():
            print(f"   * Para {lbl} -> guardar como: {fn}")
        return

    patas = {
        1: 'Pata 1 (Delantera Derecha)',
        2: 'Pata 2 (Delantera Izquierda)',
        3: 'Pata 3 (Trasera Izquierda)',
        4: 'Pata 4 (Trasera Derecha)'
    }

    # Cargar y alinear datos
    datasets = {}
    for label in ENSAYOS.keys():
        file_path = resolved_paths[label]
        df = pd.read_csv(file_path)
        idx_lift = detect_lift_index(df)
        t_lift_ms = df['timestamp_ms'].iloc[idx_lift]
        
        # Alinear de forma que el segundo 1 sea el instante del comando Levantarse (t_lift)
        df['time_rel'] = 1.0 + (df['timestamp_ms'] - t_lift_ms) / 1000.0
        
        datasets[label] = {
            'df': df,
            't_lift_idx': idx_lift
        }
        print(f"Alineando {label}: Levantarse detectado en t = {t_lift_ms / 1000.0:.3f}s de telemetría original.")
    
    # Definir ventana de graficación: de 0.0s (1 segundo antes de presionar) hasta 7.0s
    t_start = 0.0
    t_end = 7.0

    # =========================================================================
    # FIGURA 1: POSICIÓN Y SETPOINT (Articulación 2 - Fémur) - Todas las Patas
    # =========================================================================
    fig_pos, axes_p = plt.subplots(2, 2, figsize=(10, 7.5), sharex=True, sharey=True)
    fig_pos.patch.set_facecolor('white')
    axes_p_flat = axes_p.flatten()

    # Calcular límites globales de posición de la articulación 2
    all_pos_vals = []
    for label, data in datasets.items():
        df_p = data['df'][(data['df']['time_rel'] >= t_start) & (data['df']['time_rel'] <= t_end)]
        for leg_id in patas.keys():
            all_pos_vals.extend(df_p[f'leg{leg_id}_j2_actual_deg'].dropna().tolist())
            all_pos_vals.extend(df_p[f'leg{leg_id}_j2_target_deg'].dropna().tolist())
    
    y_min = min(all_pos_vals) - 2.0 if all_pos_vals else 0.0
    y_max = max(all_pos_vals) + 2.0 if all_pos_vals else 25.0

    for i, leg_id in enumerate(patas.keys()):
        ax = axes_p_flat[i]
        apply_academic_styling(ax)
        
        # RESTAURADO: Eje Y invertido para mantener la coherencia con otros ensayos
        ax.set_ylim(y_max, y_min)
        
        col_actual = f'leg{leg_id}_j2_actual_deg'
        col_target = f'leg{leg_id}_j2_target_deg'
        
        # Graficar Consigna / Setpoint (usando el perfil del primer dataset)
        first_label = list(datasets.keys())[0]
        df_first = datasets[first_label]['df']
        df_first_plot = df_first[(df_first['time_rel'] >= t_start) & (df_first['time_rel'] <= t_end)]
        ax.plot(df_first_plot['time_rel'], df_first_plot[col_target], 
                color='#333333', linestyle='--', linewidth=1.2, alpha=0.9)

        # Graficar posición real para cada nivel de carga
        for label, data in datasets.items():
            df_plot = data['df'][(data['df']['time_rel'] >= t_start) & (data['df']['time_rel'] <= t_end)]
            ax.plot(df_plot['time_rel'], df_plot[col_actual], 
                    color=COLORES[label], linewidth=1.5)

        ax.set_title(patas[leg_id], fontweight='bold')
        # Línea roja discontinua a t = 1.0s indicando cuándo se pulsó "Levantarse"
        ax.axvline(x=1.0, color='#d62728', linestyle=':', linewidth=1.2)
        
        if i in [2, 3]:
            ax.set_xlabel('Tiempo (s)')
        if i in [0, 2]:
            ax.set_ylabel('Posición Articulación 2 (°)')

    # Leyenda única
    handles_p = [plt.Line2D([0], [0], color='#333333', linestyle='--', lw=1.2, label='Setpoint (Consigna)')]
    for lbl in ENSAYOS.keys():
        handles_p.append(plt.Line2D([0], [0], color=COLORES[lbl], lw=1.5, label=f'Real ({lbl})'))
    handles_p.append(plt.Line2D([0], [0], color='#d62728', linestyle=':', lw=1.2, label='Comienzo Levantamiento (t=1s)'))

    fig_pos.legend(handles=handles_p, loc='lower center', ncol=5, frameon=True, facecolor='white', edgecolor='#e5e7eb')
    fig_pos.suptitle('Posición y Setpoint de la Articulación 2 (Fémur) en Levantamiento con Cargas', fontweight='bold', y=0.96)
    
    plt.tight_layout(rect=[0, 0.05, 1, 0.93])
    fig_pos.savefig('comparativa_posicion_carga.pdf', format='pdf', dpi=300)
    fig_pos.savefig('comparativa_posicion_carga.png', format='png', dpi=300)
    print("Gráfica de Posiciones (todas las patas) guardada como comparativa_posicion_carga.pdf y .png")


    # =========================================================================
    # FIGURA 2: TORQUE (Articulación 2 - Fémur) - Todas las Patas
    # =========================================================================
    fig_torque, axes_t = plt.subplots(2, 2, figsize=(10, 7.5), sharex=True, sharey=True)
    fig_torque.patch.set_facecolor('white')
    axes_t_flat = axes_t.flatten()

    for i, leg_id in enumerate(patas.keys()):
        ax = axes_t_flat[i]
        apply_academic_styling(ax)
        col_torque = f'leg{leg_id}_j2_torque_nm'
        
        for label, data in datasets.items():
            df_plot = data['df'][(data['df']['time_rel'] >= t_start) & (data['df']['time_rel'] <= t_end)]
            ax.plot(df_plot['time_rel'], df_plot[col_torque], 
                    color=COLORES[label], linewidth=1.5)

        ax.set_title(patas[leg_id], fontweight='bold')
        ax.axvline(x=1.0, color='#d62728', linestyle=':', linewidth=1.2)
        
        if i in [2, 3]:
            ax.set_xlabel('Tiempo (s)')
        if i in [0, 2]:
            ax.set_ylabel('Torque Articulación 2 (Nm)')

    # Leyenda única
    handles_t = [plt.Line2D([0], [0], color=COLORES[lbl], lw=1.5, label=lbl) for lbl in ENSAYOS.keys()]
    handles_t.append(plt.Line2D([0], [0], color='#d62728', linestyle=':', lw=1.2, label='Comienzo Levantamiento (t=1s)'))
    
    fig_torque.legend(handles=handles_t, loc='lower center', ncol=4, frameon=True, facecolor='white', edgecolor='#e5e7eb')
    fig_torque.suptitle('Respuesta en Torque de la Articulación 2 (Fémur) en Levantamiento con Cargas', fontweight='bold', y=0.96)
    
    plt.tight_layout(rect=[0, 0.05, 1, 0.93])
    fig_torque.savefig('comparativa_torque_carga.pdf', format='pdf', dpi=300)
    fig_torque.savefig('comparativa_torque_carga.png', format='png', dpi=300)
    print("Gráfica de Torques (todas las patas) guardada como comparativa_torque_carga.pdf y .png")


    # =========================================================================
    # FIGURA 3: COMPARATIVA EXCLUSIVA DE LA PATA 1 (1x2 Layout, Posición vs Torque)
    # =========================================================================
    fig_pata1, (ax_pos, ax_torq) = plt.subplots(1, 2, figsize=(10, 4.8))
    fig_pata1.patch.set_facecolor('white')

    # --- Subplot Izquierda: Posición Pata 1 ---
    apply_academic_styling(ax_pos)
    # RESTAURADO: Eje Y invertido para mantener la coherencia con otros ensayos
    ax_pos.set_ylim(y_max, y_min)
    
    # Consigna (Setpoint)
    ax_pos.plot(df_first_plot['time_rel'], df_first_plot['leg1_j2_target_deg'], 
                color='#333333', linestyle='--', linewidth=1.2, alpha=0.9)
    
    # Real para cada carga
    for label, data in datasets.items():
        df_plot = data['df'][(data['df']['time_rel'] >= t_start) & (data['df']['time_rel'] <= t_end)]
        ax_pos.plot(df_plot['time_rel'], df_plot['leg1_j2_actual_deg'], 
                    color=COLORES[label], linewidth=1.5)
        
    ax_pos.set_title('Posición Articulación 2 (Fémur)', fontweight='bold', pad=10)
    ax_pos.axvline(x=1.0, color='#d62728', linestyle=':', linewidth=1.2)
    ax_pos.set_xlabel('Tiempo (s)')
    ax_pos.set_ylabel('Posición Absoluta J2 (°)')

    # --- Subplot Derecha: Torque Pata 1 ---
    apply_academic_styling(ax_torq)
    
    # Torque para cada carga
    for label, data in datasets.items():
        df_plot = data['df'][(data['df']['time_rel'] >= t_start) & (data['df']['time_rel'] <= t_end)]
        ax_torq.plot(df_plot['time_rel'], df_plot['leg1_j2_torque_nm'], 
                    color=COLORES[label], linewidth=1.5)
        
    ax_torq.set_title('Torque Articulación 2 (Fémur)', fontweight='bold', pad=10)
    ax_torq.axvline(x=1.0, color='#d62728', linestyle=':', linewidth=1.2)
    ax_torq.set_xlabel('Tiempo (s)')
    ax_torq.set_ylabel('Torque J2 (Nm)')

    # Leyenda única inferior para la Figura de la Pata 1
    handles_p1 = [
        plt.Line2D([0], [0], color='#333333', linestyle='--', lw=1.2, label='Setpoint (Consigna)'),
        plt.Line2D([0], [0], color=COLORES['Carga: 0 kg'], lw=1.5, label='Real (0 kg)'),
        plt.Line2D([0], [0], color=COLORES['Carga: 1 kg'], lw=1.5, label='Real (1 kg)'),
        plt.Line2D([0], [0], color=COLORES['Carga: 2 kg'], lw=1.5, label='Real (2 kg)'),
        plt.Line2D([0], [0], color='#d62728', linestyle=':', lw=1.2, label='Comienzo Levantamiento (t=1s)')
    ]
    
    fig_pata1.legend(handles=handles_p1, loc='lower center', ncol=5, frameon=True, facecolor='white', edgecolor='#e5e7eb')
    
    # Nota: Sin título global (no suptitle) por petición del usuario
    plt.tight_layout(rect=[0, 0.12, 1, 0.95])
    fig_pata1.savefig('comparativa_pata1_carga.pdf', format='pdf', dpi=300)
    fig_pata1.savefig('comparativa_pata1_carga.png', format='png', dpi=300)
    print("Gráfica combinada de la Pata 1 guardada como comparativa_pata1_carga.pdf y .png")

if __name__ == '__main__':
    process_and_plot()
