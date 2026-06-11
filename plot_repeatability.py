import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Configuracion de tipografia y estilo academico
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

FILENAME = "repetibilidad_iso9283.csv"

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

def leg_fk(q1_deg, q2_deg, q3_deg):
    L1 = 0.08    # Coxa (m)
    L2 = 0.188   # Femur (m)
    L3 = 0.211   # Tibia (m)
    
    q1_rad = np.radians(q1_deg)
    q2_rad = np.radians(q2_deg)
    q3_rad = np.radians(q3_deg)
    
    r = L1 + L2 * np.cos(q2_rad) + L3 * np.cos(q2_rad + q3_rad)
    x = r * np.cos(q1_rad)
    y = r * np.sin(q1_rad)
    z = L2 * np.sin(q2_rad) + L3 * np.sin(q2_rad + q3_rad)
    return x, y, z

def main():
    csv_path = resolve_path(FILENAME)
    if not csv_path:
        print(f"[Error] No se encuentra el archivo de telemetria: {FILENAME}")
        print("Asegurate de realizar el ensayo de repetibilidad en la interfaz y vuelve a intentarlo.")
        return
        
    print(f"Cargando datos desde: {csv_path}")
    df = pd.read_csv(csv_path)
    
    if len(df) < 5:
        print("[Error] El archivo de telemetria contiene muy pocos datos.")
        return
        
    # 1. Detectar pata activa
    leg_moving = 2
    max_std = 0.0
    for leg_id in [1, 2, 3, 4]:
        col_name = f'leg{leg_id}_j2_target_deg'
        if col_name in df.columns:
            std_val = df[col_name].std()
            if std_val > max_std:
                max_std = std_val
                leg_moving = leg_id
    print(f"Analizando repetibilidad para la Pata {leg_moving}...")

    # 2. Calcular FK para consigna y real
    q1_t = df[f'leg{leg_moving}_j1_target_deg']
    q2_t = df[f'leg{leg_moving}_j2_target_deg']
    q3_t = df[f'leg{leg_moving}_j3_target_deg']
    
    q1_a = df[f'leg{leg_moving}_j1_actual_deg']
    q2_a = df[f'leg{leg_moving}_j2_actual_deg']
    q3_a = df[f'leg{leg_moving}_j3_actual_deg']
    
    x_t, y_t, z_t = leg_fk(q1_t, q2_t, q3_t)
    x_a, y_a, z_a = leg_fk(q1_a, q2_a, q3_a)
    
    x_t_mm, y_t_mm, z_t_mm = x_t * 1000.0, y_t * 1000.0, z_t * 1000.0
    x_a_mm, y_a_mm, z_a_mm = x_a * 1000.0, y_a * 1000.0, z_a * 1000.0
    
    # 3. Detectar las ventanas de permanencia en el punto objetivo A (X ~ 350, Y ~ 0, Z ~ 80 mm)
    # Buscamos cuando el target esta cerca del punto A
    is_at_A = (x_t_mm > 340) & (y_t_mm.abs() < 10) & (z_t_mm > 70)
    
    # Identificar bloques continuos
    blocks = []
    current_block = []
    for idx in range(len(df)):
        if is_at_A.iloc[idx]:
            current_block.append(idx)
        else:
            if len(current_block) > 20: # Bloque de tamaño razonable (al menos 400 ms a 50Hz)
                blocks.append(current_block)
            current_block = []
    if len(current_block) > 20:
        blocks.append(current_block)
        
    print(f"Detectadas {len(blocks)} visitas estables al punto objetivo.")
    
    if len(blocks) < 2:
        print("[Error] No se detectaron suficientes visitas estables en la consigna.")
        return
        
    # 4. Extraer las coordenadas finales promedio de cada visita (usando el ultimo 50% de cada bloque para estabilidad)
    reached_points = []
    for b_idx, block in enumerate(blocks):
        stable_subsegment = block[int(len(block)*0.5):]
        x_mean = x_a_mm.iloc[stable_subsegment].mean()
        y_mean = y_a_mm.iloc[stable_subsegment].mean()
        z_mean = z_a_mm.iloc[stable_subsegment].mean()
        reached_points.append([x_mean, y_mean, z_mean])
        
    points = np.array(reached_points) # shape: (N, 3)
    N = len(points)
    
    # 5. Calculo de repetibilidad estatica segun la norma ISO 9283
    # A. Calcular el baricentro (coordenada media)
    baricentro = np.mean(points, axis=0)
    
    # B. Calcular distancias de cada punto al baricentro
    distancias = np.linalg.norm(points - baricentro, axis=1)
    
    # C. Distancia media
    d_mean = np.mean(distancias)
    
    # D. Desviacion estandar de las distancias
    S_d = np.std(distancias, ddof=1) if N > 1 else 0.0
    
    # E. Repetibilidad de pose oficial ISO 9283 (Rp = d_mean + 3*S_d)
    R_p = d_mean + 3.0 * S_d
    
    # Imprimir estadisticas
    print("\n--- Analisis de Repetibilidad (Norma ISO 9283) ---")
    print(f"Numero de ensayos analizados (N): {N}")
    print(f"Baricentro: X = {baricentro[0]:.4f} mm, Y = {baricentro[1]:.4f} mm, Z = {baricentro[2]:.4f} mm")
    print(f"Distancia media al baricentro (d_med): {d_mean:.4f} mm")
    print(f"Desviacion estandar (S_d): {S_d:.4f} mm")
    print(f"-> Repetibilidad de Posicion (Rp): {R_p:.4f} mm")
    print("--------------------------------------------------")

    # =========================================================================
    # FIGURA 1: DISPERSION 2D (Vistas de Reticulo)
    # =========================================================================
    fig_2d, axes = plt.subplots(1, 3, figsize=(12, 4))
    fig_2d.patch.set_facecolor('white')
    
    labels_ejes = [
        ('Y', 'X', 1, 0, 'Plano Horizontal (Y-X)'),
        ('Y', 'Z', 1, 2, 'Plano Frontal (Y-Z)'),
        ('X', 'Z', 0, 2, 'Plano Lateral (X-Z)')
    ]
    
    for ax_idx, (xlbl, ylbl, xi, yi, title) in enumerate(labels_ejes):
        ax = axes[ax_idx]
        apply_academic_styling(ax)
        
        # Puntos y Baricentro
        ax.scatter(points[:, xi], points[:, yi], color='#d62728', s=30, label='Puntos alcanzados', zorder=4)
        ax.scatter(baricentro[xi], baricentro[yi], color='#2ca02c', marker='*', s=150, label='Baricentro', zorder=5)
        
        # Dibujar la circunferencia ISO 9283 de radio Rp
        theta_c = np.linspace(0, 2*np.pi, 200)
        xc = baricentro[xi] + R_p * np.cos(theta_c)
        yc = baricentro[yi] + R_p * np.sin(theta_c)
        ax.plot(xc, yc, color='#0f4c81', linestyle='--', lw=1.2, label=f'R_p ISO ({R_p:.3f} mm)', zorder=3)
        
        ax.set_xlabel(f'{xlbl} (mm)')
        ax.set_ylabel(f'{ylbl} (mm)')
        ax.set_title(title, fontweight='bold')
        ax.axis('equal') # Escala identica para que el circulo sea circulo
        
    axes[0].legend(loc='lower left', frameon=True, facecolor='white', edgecolor='#e5e7eb')
    #fig_2d.suptitle(f'Precision de Posicionamiento (ISO 9283) - Pata {leg_moving}\nRp = {R_p:.4f} mm', fontweight='bold', y=0.98)
    plt.tight_layout(rect=[0, 0, 1, 0.90])
    fig_2d.savefig('repetibilidad_iso9283_2d.pdf', format='pdf')
    fig_2d.savefig('repetibilidad_iso9283_2d.png', format='png')
    plt.close(fig_2d)

    # =========================================================================
    # FIGURA 2: DISPERSION 3D (Espacial)
    # =========================================================================
    fig_3d = plt.figure(figsize=(9, 8))
    fig_3d.patch.set_facecolor('white')
    ax_3d = fig_3d.add_subplot(111, projection='3d')
    ax_3d.set_facecolor('white')
    
    # Graficar puntos
    ax_3d.scatter(points[:, 0], points[:, 1], points[:, 2], color='#d62728', s=40, label='Ensayos Reales', zorder=4)
    ax_3d.scatter(baricentro[0], baricentro[1], baricentro[2], color='#2ca02c', marker='*', s=200, label='Baricentro', zorder=5)
    
    # Dibujar esfera de confianza de repetibilidad Rp
    u = np.linspace(0, 2 * np.pi, 30)
    v = np.linspace(0, np.pi, 30)
    x_s = baricentro[0] + R_p * np.outer(np.cos(u), np.sin(v))
    y_s = baricentro[1] + R_p * np.outer(np.sin(u), np.sin(v))
    z_s = baricentro[2] + R_p * np.outer(np.ones(np.size(u)), np.cos(v))
    ax_3d.plot_wireframe(x_s, y_s, z_s, color='#0f4c81', alpha=0.15, label=f'Esfera Rp ({R_p:.3f} mm)')
    
    # Forzar limites proporcionales con margen holgado para ver la distribucion
    max_range = max(R_p * 1.5, 0.5) # al menos 0.5 mm de rango de visualizacion
    ax_3d.set_xlim(baricentro[0] - max_range, baricentro[0] - max_range + 2*max_range)
    ax_3d.set_ylim(baricentro[1] - max_range, baricentro[1] - max_range + 2*max_range)
    ax_3d.set_zlim(baricentro[2] - max_range, baricentro[2] - max_range + 2*max_range)
    
    ax_3d.set_xlabel('Eje X (mm)', labelpad=10)
    ax_3d.set_ylabel('Eje Y (mm)', labelpad=10)
    ax_3d.set_zlabel('Eje Z (mm)', labelpad=10)
   # ax_3d.set_title(f'Distribucion Tridimensional (3D) de Repetibilidad Estatica\nPata {leg_moving} (Rp = {R_p:.4f} mm)', fontweight='bold', pad=15)
    ax_3d.legend(loc='upper right', frameon=True, facecolor='white', edgecolor='#e5e7eb')
    
    ax_3d.view_init(elev=20, azim=45)
    plt.tight_layout()
    fig_3d.savefig('repetibilidad_iso9283_3d.pdf', format='pdf')
    fig_3d.savefig('repetibilidad_iso9283_3d.png', format='png')
    plt.close(fig_3d)
    
    print("Todas las graficas de repetibilidad ISO 9283 guardadas con exito (2D y 3D).")

if __name__ == '__main__':
    main()
