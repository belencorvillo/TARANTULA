import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Configuracion de tipografia y estilo academico (Times New Roman)
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

FILENAME = "validacion_ik_circulo.csv"

def resolve_path(filename):
    """
    Busca el archivo CSV en el directorio actual o recursivamente en la carpeta 'build'.
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
    Aplica lineas de cuadricula sutiles y remueve bordes innecesarios para formato de tesis.
    """
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.spines['left'].set_color('#cccccc')
    ax.spines['bottom'].set_color('#cccccc')
    ax.spines['left'].set_linewidth(0.8)
    ax.spines['bottom'].set_linewidth(0.8)
    ax.grid(True, which='both', linestyle=':', linewidth=0.5, color='#dddddd')
    ax.set_facecolor('white')

def forward_kinematics(q1_deg, q2_deg, q3_deg):
    """
    Calcula la posicion (x, y, z) en metros del extremo de la pata en coordenadas locales
    basado en las dimensiones fisicas de Tarantula.
    """
    L1 = 0.08    # Coxa/Hip offset (m)
    L2 = 0.188   # Femur length (m)
    L3 = 0.211   # Tibia length (m)
    
    # Convertir a radianes
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
        print("Asegurate de realizar el ensayo 'Dibujar Circulo' en la interfaz y vuelve a intentarlo.")
        return
        
    print(f"Cargando datos desde: {csv_path}")
    df = pd.read_csv(csv_path)
    
    if len(df) < 5:
        print("[Error] El archivo de telemetria contiene muy pocos datos.")
        return

    # 1. Detectar automaticamente la pata que se movio
    leg_moving = 2 # Por defecto Pata 2 (DI)
    max_std = 0.0
    for leg_id in [1, 2, 3, 4]:
        col_name = f'leg{leg_id}_j2_target_deg'
        if col_name in df.columns:
            std_val = df[col_name].std()
            if std_val > max_std:
                max_std = std_val
                leg_moving = leg_id
            
    print(f"Pata detectada para analisis: Pata {leg_moving}")
    
    # 2. Extraer angulos (consigna y reales) en grados
    q1_t = df[f'leg{leg_moving}_j1_target_deg']
    q2_t = df[f'leg{leg_moving}_j2_target_deg']
    q3_t = df[f'leg{leg_moving}_j3_target_deg']
    
    q1_a = df[f'leg{leg_moving}_j1_actual_deg']
    q2_a = df[f'leg{leg_moving}_j2_actual_deg']
    q3_a = df[f'leg{leg_moving}_j3_actual_deg']
    
    # Tiempo relativo en segundos (inicia en 0)
    t = (df['timestamp_ms'] - df['timestamp_ms'].iloc[0]) / 1000.0
    
    # 3. Calcular cinematica directa (FK) para consigna y real
    x_t, y_t, z_t = forward_kinematics(q1_t, q2_t, q3_t)
    x_a, y_a, z_a = forward_kinematics(q1_a, q2_a, q3_a)
    
    # Convertir a milimetros para mejor legibilidad en las graficas
    x_t_mm, y_t_mm, z_t_mm = x_t * 1000.0, y_t * 1000.0, z_t * 1000.0
    x_a_mm, y_a_mm, z_a_mm = x_a * 1000.0, y_a * 1000.0, z_a * 1000.0
    
    # 4. Calcular el error euclidiano cartesiano (en mm)
    error_euclid_mm = np.sqrt((x_t_mm - x_a_mm)**2 + (y_t_mm - y_a_mm)**2 + (z_t_mm - z_a_mm)**2)
    
    # Imprimir estadisticas por consola
    print("\n--- Estadisticas de Error Cartesiano (en mm) ---")
    print(f"Error medio: {error_euclid_mm.mean():.2f} mm")
    print(f"Error maximo: {error_euclid_mm.max():.2f} mm")
    print(f"Desviacion estandar: {error_euclid_mm.std():.2f} mm")
    print("------------------------------------------------")

    # =========================================================================
    # FIGURA 1: COMPARATIVA TEMPORAL (2D)
    # =========================================================================
    fig_2d, axes = plt.subplots(4, 1, figsize=(9, 10), sharex=True)
    fig_2d.patch.set_facecolor('white')
    
    colores_ejes = {'X': '#0f4c81', 'Y': '#f58220', 'Z': '#375a23'}
    
    # Subplot X
    apply_academic_styling(axes[0])
    axes[0].plot(t, x_t_mm, color='#333333', linestyle='--', lw=1.2, label='Consigna')
    axes[0].plot(t, x_a_mm, color=colores_ejes['X'], lw=1.5, label='Medida Real')
    axes[0].set_ylabel('Eje X (mm)')
    axes[0].legend(loc='upper right')
    
    # Subplot Y
    apply_academic_styling(axes[1])
    axes[1].plot(t, y_t_mm, color='#333333', linestyle='--', lw=1.2)
    axes[1].plot(t, y_a_mm, color=colores_ejes['Y'], lw=1.5)
    axes[1].set_ylabel('Eje Y (mm)')
    
    # Subplot Z
    apply_academic_styling(axes[2])
    axes[2].plot(t, z_t_mm, color='#333333', linestyle='--', lw=1.2)
    axes[2].plot(t, z_a_mm, color=colores_ejes['Z'], lw=1.5)
    axes[2].set_ylabel('Eje Z (mm)')
    
    # Subplot Error Euclidiano
    apply_academic_styling(axes[3])
    axes[3].plot(t, error_euclid_mm, color='#d62728', lw=1.5)
    axes[3].set_ylabel('Error (mm)')
    axes[3].set_xlabel('Tiempo (s)')
    axes[3].fill_between(t, 0, error_euclid_mm, color='#d62728', alpha=0.1)
    
    plt.tight_layout()
    fig_2d.savefig('validacion_ik_error.pdf', format='pdf')
    fig_2d.savefig('validacion_ik_error.png', format='png')
    plt.close(fig_2d)

    # =========================================================================
    # FIGURA 2: TRAYECTORIA ESPACIAL (3D)
    # =========================================================================
    fig_3d = plt.figure(figsize=(9, 8))
    fig_3d.patch.set_facecolor('white')
    
    ax_3d = fig_3d.add_subplot(111, projection='3d')
    ax_3d.set_facecolor('white')
    
    # Trazar lineas
    ax_3d.plot(x_t_mm, y_t_mm, z_t_mm, color='#333333', linestyle='--', lw=1.5, label='Trayectoria Deseada (Consigna)')
    ax_3d.plot(x_a_mm, y_a_mm, z_a_mm, color='#0f4c81', lw=2.0, label='Trayectoria Real (Seguimiento)')
    
    # Punto de inicio y fin
    ax_3d.scatter(x_t_mm.iloc[0], y_t_mm.iloc[0], z_t_mm.iloc[0], color='#2ca02c', s=50, label='Inicio / Fin', zorder=5)
    
    # Ajuste de relacion de aspecto proporcional (ejes iguales en 3D)
    max_range = np.array([x_t_mm.max()-x_t_mm.min(), y_t_mm.max()-y_t_mm.min(), z_t_mm.max()-z_t_mm.min()]).max() / 2.0
    mid_x = (x_t_mm.max()+x_t_mm.min()) * 0.5
    mid_y = (y_t_mm.max()+y_t_mm.min()) * 0.5
    mid_z = (z_t_mm.max()+z_t_mm.min()) * 0.5
    
    # Forzar un rango minimo para evitar divisiones o visualizacion plana
    max_range = max(max_range, 50.0) 
    
    ax_3d.set_xlim(mid_x - max_range - 10, mid_x + max_range + 10)
    ax_3d.set_ylim(mid_y - max_range - 10, mid_y + max_range + 10)
    ax_3d.set_zlim(mid_z - max_range - 10, mid_z + max_range + 10)
    
    # Etiquetas y Leyendas
    ax_3d.set_xlabel('Eje X (mm)', labelpad=10)
    ax_3d.set_ylabel('Eje Y (mm)', labelpad=10)
    ax_3d.set_zlabel('Eje Z (mm)', labelpad=10)
   # ax_3d.set_title(f'Trayectoria Tridimensional (3D) en el Espacio - Pata {leg_moving}', fontweight='bold', pad=15)
    ax_3d.legend(loc='upper right', frameon=True, facecolor='white', edgecolor='#e5e7eb')
    
    # Ajustar angulo inicial de visualizacion para ver el circulo de forma clara
    ax_3d.view_init(elev=20, azim=45)
    
    plt.tight_layout()
    fig_3d.savefig('validacion_ik_trayectoria_3d.pdf', format='pdf')
    fig_3d.savefig('validacion_ik_trayectoria_3d.png', format='png')
    plt.close(fig_3d)

    print("Todas las graficas de validacion IK guardadas con exito (2D y 3D).")

if __name__ == '__main__':
    main()
