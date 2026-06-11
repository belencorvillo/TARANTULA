import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Configuración de tipografía y estilo académico
plt.rcParams.update({
    'font.family': 'serif',
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

# Archivos de entrada para las 3 alturas
ENSAYOS = {
    'Altura: 5 cm': 'telemetria_altura_5.csv',
    'Altura: 10 cm': 'telemetria_altura_10.csv',
    'Altura: 15 cm': 'telemetria_altura_15.csv'
}

# Colores académicos
COLORES = {
    'Altura: 5 cm': '#0f4c81',   # Azul oscuro
    'Altura: 10 cm': '#f58220',  # Naranja
    'Altura: 15 cm': '#375a23'   # Verde oscuro
}

def detect_impact_index(df, threshold_deg=0.3):
    """
    Detecta automáticamente el impacto analizando el cambio en el error de posición.
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
    if os.path.exists(filename):
        return filename
    if os.path.exists('build'):
        for root, dirs, files in os.walk('build'):
            if filename in files:
                return os.path.join(root, filename)
    return None

def apply_academic_styling(ax):
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.spines['left'].set_color('#cccccc')
    ax.spines['bottom'].set_color('#cccccc')
    ax.spines['left'].set_linewidth(0.8)
    ax.spines['bottom'].set_linewidth(0.8)
    ax.grid(True, which='both', linestyle=':', linewidth=0.5, color='#dddddd')
    ax.set_facecolor('white')

def process_and_plot():
    # Resolver rutas
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
        print("\n[Tip] Realiza los ensayos de caida en el robot y guardalos con esos nombres exactos.")
        return

    patas = {1: 'Pata 1', 2: 'Pata 2', 3: 'Pata 3', 4: 'Pata 4'}

    # Cargar y alinear
    datasets = {}
    for label in ENSAYOS.keys():
        df = pd.read_csv(resolved_paths[label])
        idx_push = detect_impact_index(df)
        t_push_ms = df['timestamp_ms'].iloc[idx_push]
        
        df['time_rel'] = 2.0 + (df['timestamp_ms'] - t_push_ms) / 1000.0
        
        err_cols = [c for c in df.columns if '_error_deg' in c]
        abs_err_mean = df[err_cols].abs().mean(axis=1)
        baseline = abs_err_mean.iloc[:min(30, len(df))].mean()
        
        t_settle_rel = 2.0
        for idx in range(idx_push, len(df)):
            if abs_err_mean.iloc[idx] > baseline + 0.15:
                t_settle_rel = df['time_rel'].iloc[idx]
        
        datasets[label] = {
            'df': df,
            't_push_idx': idx_push,
            't_settle_rel': t_settle_rel
        }
    
    max_settle_time = max(data['t_settle_rel'] for data in datasets.values())
    t_end = max_settle_time + 3.0
    t_end = max(6.0, min(12.0, t_end))
    
    print(f"Alineando impactos en t = 2.0s. Graficando ventana t = [0.0s, {t_end:.2f}s]")

    # Calcular limites globales de Y para consistencia en las graficas de posicion
    # Articulacion 2 (Femur)
    all_pos_j2 = []
    for data in datasets.values():
        df_p = data['df'][(data['df']['time_rel'] >= 0.0) & (data['df']['time_rel'] <= t_end)]
        for leg_id in patas.keys():
            all_pos_j2.extend(df_p[f'leg{leg_id}_j2_actual_deg'].dropna().tolist())
            all_pos_j2.extend(df_p[f'leg{leg_id}_j2_target_deg'].dropna().tolist())
    y_min_j2 = min(all_pos_j2) - 1.5 if all_pos_j2 else 24.0
    y_max_j2 = max(all_pos_j2) + 1.5 if all_pos_j2 else 44.0

    # Articulacion 3 (Tibia)
    all_pos_j3 = []
    for data in datasets.values():
        df_p = data['df'][(data['df']['time_rel'] >= 0.0) & (data['df']['time_rel'] <= t_end)]
        for leg_id in patas.keys():
            all_pos_j3.extend(df_p[f'leg{leg_id}_j3_actual_deg'].dropna().tolist())
            all_pos_j3.extend(df_p[f'leg{leg_id}_j3_target_deg'].dropna().tolist())
    y_min_j3 = min(all_pos_j3) - 3.0 if all_pos_j3 else -130.0
    y_max_j3 = max(all_pos_j3) + 3.0 if all_pos_j3 else -90.0

    # Generar leyendas
    handles = [plt.Line2D([0], [0], color=COLORES[lbl], lw=1.5, label=lbl) for lbl in ENSAYOS.keys()]
    handles.append(plt.Line2D([0], [0], color='#d62728', linestyle=':', lw=1.0, label='Impacto (t=2s)'))

    handles_p = [plt.Line2D([0], [0], color='#333333', linestyle='--', lw=1.2, label='Setpoint (Consigna)')]
    for lbl in ENSAYOS.keys():
        handles_p.append(plt.Line2D([0], [0], color=COLORES[lbl], lw=1.5, label=f'Real ({lbl})'))
    handles_p.append(plt.Line2D([0], [0], color='#d62728', linestyle=':', lw=1.0, label='Impacto (t=2s)'))

    # =========================================================================
    # PART 1: GRAFICAS INDIVIDUALES (2x2)
    # =========================================================================
    
    # 1.1 Torque J2
    fig_t_j2, axes = plt.subplots(2, 2, figsize=(10, 7.5), sharex=True, sharey=True)
    axes_flat = axes.flatten()
    for i, leg_id in enumerate(patas.keys()):
        ax = axes_flat[i]
        apply_academic_styling(ax)
        for label, data in datasets.items():
            df_plot = data['df'][(data['df']['time_rel'] >= 0.0) & (data['df']['time_rel'] <= t_end)]
            ax.plot(df_plot['time_rel'], df_plot[f'leg{leg_id}_j2_torque_nm'], color=COLORES[label], lw=1.5)
        ax.set_title(patas[leg_id], fontweight='bold')
        ax.axvline(x=2.0, color='#d62728', linestyle=':', lw=1.0)
        if i in [2, 3]: ax.set_xlabel('Tiempo (s)')
        if i in [0, 2]: ax.set_ylabel('Torque J2 (Nm)')
    fig_t_j2.legend(handles=handles, loc='lower center', ncol=4, frameon=True, facecolor='white', edgecolor='#e5e7eb')
    fig_t_j2.suptitle('Respuesta en Torque - Articulación 2 (Fémur) ante Impacto por Caída', fontweight='bold', y=0.96)
    plt.tight_layout(rect=[0, 0.05, 1, 0.93])
    fig_t_j2.savefig('comparativa_torque_j2.pdf', format='pdf')
    fig_t_j2.savefig('comparativa_torque_j2.png', format='png')

    # 1.2 Posicion J2 (Invertida)
    fig_p_j2, axes = plt.subplots(2, 2, figsize=(10, 7.5), sharex=True, sharey=True)
    axes_flat = axes.flatten()
    for i, leg_id in enumerate(patas.keys()):
        ax = axes_flat[i]
        apply_academic_styling(ax)
        ax.set_ylim(y_max_j2, y_min_j2) # Invertido
        # Setpoint
        first_label = list(datasets.keys())[0]
        df_first = datasets[first_label]['df']
        df_first_plot = df_first[(df_first['time_rel'] >= 0.0) & (df_first['time_rel'] <= t_end)]
        ax.plot(df_first_plot['time_rel'], df_first_plot[f'leg{leg_id}_j2_target_deg'], color='#333333', ls='--', lw=1.2)
        for label, data in datasets.items():
            df_plot = data['df'][(data['df']['time_rel'] >= 0.0) & (data['df']['time_rel'] <= t_end)]
            ax.plot(df_plot['time_rel'], df_plot[f'leg{leg_id}_j2_actual_deg'], color=COLORES[label], lw=1.5)
        ax.set_title(patas[leg_id], fontweight='bold')
        ax.axvline(x=2.0, color='#d62728', linestyle=':', lw=1.0)
        if i in [2, 3]: ax.set_xlabel('Tiempo (s)')
        if i in [0, 2]: ax.set_ylabel('Posición Absoluta J2 (°)')
    fig_p_j2.legend(handles=handles_p, loc='lower center', ncol=5, frameon=True, facecolor='white', edgecolor='#e5e7eb')
    fig_p_j2.suptitle('Posición Absoluta - Articulación 2 (Fémur) ante Impacto por Caída', fontweight='bold', y=0.96)
    plt.tight_layout(rect=[0, 0.05, 1, 0.93])
    fig_p_j2.savefig('comparativa_posicion_j2.pdf', format='pdf')
    fig_p_j2.savefig('comparativa_posicion_j2.png', format='png')

    # 1.3 Torque J3
    fig_t_j3, axes = plt.subplots(2, 2, figsize=(10, 7.5), sharex=True, sharey=True)
    axes_flat = axes.flatten()
    for i, leg_id in enumerate(patas.keys()):
        ax = axes_flat[i]
        apply_academic_styling(ax)
        for label, data in datasets.items():
            df_plot = data['df'][(data['df']['time_rel'] >= 0.0) & (data['df']['time_rel'] <= t_end)]
            ax.plot(df_plot['time_rel'], df_plot[f'leg{leg_id}_j3_torque_nm'], color=COLORES[label], lw=1.5)
        ax.set_title(patas[leg_id], fontweight='bold')
        ax.axvline(x=2.0, color='#d62728', linestyle=':', lw=1.0)
        if i in [2, 3]: ax.set_xlabel('Tiempo (s)')
        if i in [0, 2]: ax.set_ylabel('Torque J3 (Nm)')
    fig_t_j3.legend(handles=handles, loc='lower center', ncol=4, frameon=True, facecolor='white', edgecolor='#e5e7eb')
    fig_t_j3.suptitle('Respuesta en Torque - Articulación 3 (Tibia) ante Impacto por Caída', fontweight='bold', y=0.96)
    plt.tight_layout(rect=[0, 0.05, 1, 0.93])
    fig_t_j3.savefig('comparativa_torque_j3.pdf', format='pdf')
    fig_t_j3.savefig('comparativa_torque_j3.png', format='png')

    # 1.4 Posicion J3 (Estándar, ya que al doblarse va a más negativo y desciende de forma natural)
    fig_p_j3, axes = plt.subplots(2, 2, figsize=(10, 7.5), sharex=True, sharey=True)
    axes_flat = axes.flatten()
    for i, leg_id in enumerate(patas.keys()):
        ax = axes_flat[i]
        apply_academic_styling(ax)
        ax.set_ylim(y_min_j3, y_max_j3) # Estándar (menor abajo, mayor arriba)
        # Setpoint
        first_label = list(datasets.keys())[0]
        df_first = datasets[first_label]['df']
        df_first_plot = df_first[(df_first['time_rel'] >= 0.0) & (df_first['time_rel'] <= t_end)]
        ax.plot(df_first_plot['time_rel'], df_first_plot[f'leg{leg_id}_j3_target_deg'], color='#333333', ls='--', lw=1.2)
        for label, data in datasets.items():
            df_plot = data['df'][(data['df']['time_rel'] >= 0.0) & (data['df']['time_rel'] <= t_end)]
            ax.plot(df_plot['time_rel'], df_plot[f'leg{leg_id}_j3_actual_deg'], color=COLORES[label], lw=1.5)
        ax.set_title(patas[leg_id], fontweight='bold')
        ax.axvline(x=2.0, color='#d62728', linestyle=':', lw=1.0)
        if i in [2, 3]: ax.set_xlabel('Tiempo (s)')
        if i in [0, 2]: ax.set_ylabel('Posición Absoluta J3 (°)')
    fig_p_j3.legend(handles=handles_p, loc='lower center', ncol=5, frameon=True, facecolor='white', edgecolor='#e5e7eb')
    fig_p_j3.suptitle('Posición Absoluta - Articulación 3 (Tibia) ante Impacto por Caída', fontweight='bold', y=0.96)
    plt.tight_layout(rect=[0, 0.05, 1, 0.93])
    fig_p_j3.savefig('comparativa_posicion_j3.pdf', format='pdf')
    fig_p_j3.savefig('comparativa_posicion_j3.png', format='png')


    # =========================================================================
    # PART 2: GRAFICAS COMBINADAS (4x2)
    # =========================================================================
    
    # 2.1 Posicion Combinada J2 y J3
    fig_pos_comb, axes_c = plt.subplots(4, 2, figsize=(10, 11), sharex=True)
    fig_pos_comb.patch.set_facecolor('white')
    
    for r in range(4):
        leg_id = r + 1
        
        # Columna 0: Articulacion 2 (Femur)
        ax_j2 = axes_c[r, 0]
        apply_academic_styling(ax_j2)
        ax_j2.set_ylim(y_max_j2, y_min_j2) # Invertido
        
        # Setpoint
        first_label = list(datasets.keys())[0]
        df_first = datasets[first_label]['df']
        df_first_plot = df_first[(df_first['time_rel'] >= 0.0) & (df_first['time_rel'] <= t_end)]
        ax_j2.plot(df_first_plot['time_rel'], df_first_plot[f'leg{leg_id}_j2_target_deg'], color='#333333', ls='--', lw=1.2)
        
        for label, data in datasets.items():
            df_plot = data['df'][(data['df']['time_rel'] >= 0.0) & (data['df']['time_rel'] <= t_end)]
            ax_j2.plot(df_plot['time_rel'], df_plot[f'leg{leg_id}_j2_actual_deg'], color=COLORES[label], lw=1.3)
            
        ax_j2.axvline(x=2.0, color='#d62728', linestyle=':', lw=1.0)
        ax_j2.set_ylabel(f'Pata {leg_id}\nPos. J2 (°)')
        if r == 0: ax_j2.set_title('Articulación 2 (Fémur) - Eje Y Invertido', fontweight='bold', pad=10)
        if r == 3: ax_j2.set_xlabel('Tiempo (s)')

        # Columna 1: Articulacion 3 (Tibia)
        ax_j3 = axes_c[r, 1]
        apply_academic_styling(ax_j3)
        ax_j3.set_ylim(y_min_j3, y_max_j3) # Estándar
        
        # Setpoint
        ax_j3.plot(df_first_plot['time_rel'], df_first_plot[f'leg{leg_id}_j3_target_deg'], color='#333333', ls='--', lw=1.2)
        
        for label, data in datasets.items():
            df_plot = data['df'][(data['df']['time_rel'] >= 0.0) & (data['df']['time_rel'] <= t_end)]
            ax_j3.plot(df_plot['time_rel'], df_plot[f'leg{leg_id}_j3_actual_deg'], color=COLORES[label], lw=1.3)
            
        ax_j3.axvline(x=2.0, color='#d62728', linestyle=':', lw=1.0)
        ax_j3.set_ylabel('Pos. J3 (°)')
        if r == 0: ax_j3.set_title('Articulación 3 (Tibia) - Eje Y Estándar', fontweight='bold', pad=10)
        if r == 3: ax_j3.set_xlabel('Tiempo (s)')

    fig_pos_comb.legend(handles=handles_p, loc='lower center', ncol=5, frameon=True, facecolor='white', edgecolor='#e5e7eb')
    fig_pos_comb.suptitle('Comparativa de Posición Absoluta (Fémur y Tibia) ante Impacto por Caída', fontweight='bold', y=0.97)
    plt.tight_layout(rect=[0, 0.04, 1, 0.95])
    fig_pos_comb.savefig('comparativa_posicion_combinada.pdf', format='pdf')
    fig_pos_comb.savefig('comparativa_posicion_combinada.png', format='png')

    # 2.2 Torque Combinado J2 y J3
    fig_t_comb, axes_c = plt.subplots(4, 2, figsize=(10, 11), sharex=True, sharey='col')
    fig_t_comb.patch.set_facecolor('white')
    
    for r in range(4):
        leg_id = r + 1
        
        # Columna 0: Torque J2
        ax_j2 = axes_c[r, 0]
        apply_academic_styling(ax_j2)
        for label, data in datasets.items():
            df_plot = data['df'][(data['df']['time_rel'] >= 0.0) & (data['df']['time_rel'] <= t_end)]
            ax_j2.plot(df_plot['time_rel'], df_plot[f'leg{leg_id}_j2_torque_nm'], color=COLORES[label], lw=1.3)
        ax_j2.axvline(x=2.0, color='#d62728', linestyle=':', lw=1.0)
        ax_j2.set_ylabel(f'Pata {leg_id}\nTorque J2 (Nm)')
        if r == 0: ax_j2.set_title('Torque Articulación 2 (Fémur)', fontweight='bold', pad=10)
        if r == 3: ax_j2.set_xlabel('Tiempo (s)')

        # Columna 1: Torque J3
        ax_j3 = axes_c[r, 1]
        apply_academic_styling(ax_j3)
        for label, data in datasets.items():
            df_plot = data['df'][(data['df']['time_rel'] >= 0.0) & (data['df']['time_rel'] <= t_end)]
            ax_j3.plot(df_plot['time_rel'], df_plot[f'leg{leg_id}_j3_torque_nm'], color=COLORES[label], lw=1.3)
        ax_j3.axvline(x=2.0, color='#d62728', linestyle=':', lw=1.0)
        ax_j3.set_ylabel('Torque J3 (Nm)')
        if r == 0: ax_j3.set_title('Torque Articulación 3 (Tibia)', fontweight='bold', pad=10)
        if r == 3: ax_j3.set_xlabel('Tiempo (s)')

    fig_t_comb.legend(handles=handles, loc='lower center', ncol=4, frameon=True, facecolor='white', edgecolor='#e5e7eb')
    fig_t_comb.suptitle('Comparativa de Torque (Fémur y Tibia) ante Impacto por Caída', fontweight='bold', y=0.97)
    plt.tight_layout(rect=[0, 0.04, 1, 0.95])
    fig_t_comb.savefig('comparativa_torque_combinada.pdf', format='pdf')
    fig_t_comb.savefig('comparativa_torque_combinada.png', format='png')
    plt.close(fig_t_comb)

    # =========================================================================
    # PART 3: GRAFICAS COMBINADAS SOLO PATA 2 (1x2)
    # =========================================================================
    leg_id = 2
    
    # 3.1 Posición Combinada J2 y J3 - Pata 2
    fig_pos_pata2, axes_p2 = plt.subplots(1, 2, figsize=(10, 4.5))
    fig_pos_pata2.patch.set_facecolor('white')
    
    # Columna 0: Articulacion 2 (Femur)
    ax_j2 = axes_p2[0]
    apply_academic_styling(ax_j2)
    ax_j2.set_ylim(y_max_j2, y_min_j2) # Invertido
    
    # Setpoint
    first_label = list(datasets.keys())[0]
    df_first = datasets[first_label]['df']
    df_first_plot = df_first[(df_first['time_rel'] >= 0.0) & (df_first['time_rel'] <= t_end)]
    ax_j2.plot(df_first_plot['time_rel'], df_first_plot[f'leg{leg_id}_j2_target_deg'], color='#333333', ls='--', lw=1.2)
    
    for label, data in datasets.items():
        df_plot = data['df'][(data['df']['time_rel'] >= 0.0) & (data['df']['time_rel'] <= t_end)]
        ax_j2.plot(df_plot['time_rel'], df_plot[f'leg{leg_id}_j2_actual_deg'], color=COLORES[label], lw=1.3)
        
    ax_j2.axvline(x=2.0, color='#d62728', linestyle=':', lw=1.0)
    ax_j2.set_ylabel('Posición J2 (°)')
    ax_j2.set_title('Articulación 2 (Fémur) - Eje Y Invertido', fontweight='bold', pad=10)
    ax_j2.set_xlabel('Tiempo (s)')

    # Columna 1: Articulacion 3 (Tibia)
    ax_j3 = axes_p2[1]
    apply_academic_styling(ax_j3)
    ax_j3.set_ylim(y_min_j3, y_max_j3) # Estándar
    
    # Setpoint
    ax_j3.plot(df_first_plot['time_rel'], df_first_plot[f'leg{leg_id}_j3_target_deg'], color='#333333', ls='--', lw=1.2)
    
    for label, data in datasets.items():
        df_plot = data['df'][(data['df']['time_rel'] >= 0.0) & (data['df']['time_rel'] <= t_end)]
        ax_j3.plot(df_plot['time_rel'], df_plot[f'leg{leg_id}_j3_actual_deg'], color=COLORES[label], lw=1.3)
        
    ax_j3.axvline(x=2.0, color='#d62728', linestyle=':', lw=1.0)
    ax_j3.set_ylabel('Posición J3 (°)')
    ax_j3.set_title('Articulación 3 (Tibia) - Eje Y Estándar', fontweight='bold', pad=10)
    ax_j3.set_xlabel('Tiempo (s)')

    fig_pos_pata2.legend(handles=handles_p, loc='lower center', ncol=5, frameon=True, facecolor='white', edgecolor='#e5e7eb')
    #fig_pos_pata2.suptitle('Comparativa de Posición Absoluta (Pata 2 - Fémur y Tibia) ante Caída', fontweight='bold', y=0.96)
    plt.tight_layout(rect=[0, 0.12, 1, 0.88])
    fig_pos_pata2.savefig('comparativa_posicion_pata2.pdf', format='pdf')
    fig_pos_pata2.savefig('comparativa_posicion_pata2.png', format='png')
    plt.close(fig_pos_pata2)

    # 3.2 Torque Combinado J2 y J3 - Pata 2
    fig_t_pata2, axes_tp2 = plt.subplots(1, 2, figsize=(10, 4.5))
    fig_t_pata2.patch.set_facecolor('white')
    
    # Columna 0: Torque J2
    ax_tj2 = axes_tp2[0]
    apply_academic_styling(ax_tj2)
    for label, data in datasets.items():
        df_plot = data['df'][(data['df']['time_rel'] >= 0.0) & (data['df']['time_rel'] <= t_end)]
        ax_tj2.plot(df_plot['time_rel'], df_plot[f'leg{leg_id}_j2_torque_nm'], color=COLORES[label], lw=1.3)
    ax_tj2.axvline(x=2.0, color='#d62728', linestyle=':', lw=1.0)
    ax_tj2.set_ylabel('Torque J2 (Nm)')
    ax_tj2.set_title('Torque Articulación 2 (Fémur)', fontweight='bold', pad=10)
    ax_tj2.set_xlabel('Tiempo (s)')

    # Columna 1: Torque J3
    ax_tj3 = axes_tp2[1]
    apply_academic_styling(ax_tj3)
    for label, data in datasets.items():
        df_plot = data['df'][(data['df']['time_rel'] >= 0.0) & (data['df']['time_rel'] <= t_end)]
        ax_tj3.plot(df_plot['time_rel'], df_plot[f'leg{leg_id}_j3_torque_nm'], color=COLORES[label], lw=1.3)
    ax_tj3.axvline(x=2.0, color='#d62728', linestyle=':', lw=1.0)
    ax_tj3.set_ylabel('Torque J3 (Nm)')
    ax_tj3.set_title('Torque Articulación 3 (Tibia)', fontweight='bold', pad=10)
    ax_tj3.set_xlabel('Tiempo (s)')

    fig_t_pata2.legend(handles=handles, loc='lower center', ncol=4, frameon=True, facecolor='white', edgecolor='#e5e7eb')
    #fig_t_pata2.suptitle('Comparativa de Torque (Pata 2 - Fémur y Tibia) ante Caída', fontweight='bold', y=0.96)
    plt.tight_layout(rect=[0, 0.12, 1, 0.88])
    fig_t_pata2.savefig('comparativa_torque_pata2.pdf', format='pdf')
    fig_t_pata2.savefig('comparativa_torque_pata2.png', format='png')
    plt.close(fig_t_pata2)
    
    print("Todas las graficas del Drop Test guardadas con exito (individuales, combinadas y Pata 2).")

if __name__ == '__main__':
    process_and_plot()
