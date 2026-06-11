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

FILENAME = "validacion_ik_cuerpo.csv"

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
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.spines['left'].set_color('#cccccc')
    ax.spines['bottom'].set_color('#cccccc')
    ax.spines['left'].set_linewidth(0.8)
    ax.spines['bottom'].set_linewidth(0.8)
    ax.grid(True, which='both', linestyle=':', linewidth=0.5, color='#dddddd')
    ax.set_facecolor('white')

def leg_fk(q1_deg, q2_deg, q3_deg):
    """
    Calcula la posicion (x, y, z) en metros del pie en coordenadas locales de la pata.
    """
    L1 = 0.08    # Coxa offset (m)
    L2 = 0.188   # Femur length (m)
    L3 = 0.211   # Tibia length (m)
    
    q1_rad = np.radians(q1_deg)
    q2_rad = np.radians(q2_deg)
    q3_rad = np.radians(q3_deg)
    
    r = L1 + L2 * np.cos(q2_rad) + L3 * np.cos(q2_rad + q3_rad)
    x = r * np.cos(q1_rad)
    y = r * np.sin(q1_rad)
    z = L2 * np.sin(q2_rad) + L3 * np.sin(q2_rad + q3_rad)
    return np.array([x, y, z])

def get_body_displacement(df, prefix='actual'):
    """
    Calcula el desplazamiento (dx, dy, dz) del chasis en metros a lo largo del tiempo
    estimandolo a partir de la variacion de la posicion de los pies en el frame del chasis.
    """
    # Angulos de instalacion de cada pata respecto al chasis
    thetas = [np.pi / 4.0, -np.pi / 4.0, -3.0 * np.pi / 4.0, 3.0 * np.pi / 4.0] # Pata 1, 2, 3, 4
    
    num_samples = len(df)
    dx = np.zeros(num_samples)
    dy = np.zeros(num_samples)
    dz = np.zeros(num_samples)
    
    # Calcular posiciones locales del pie para cada pata en t=0 (referencia nominal)
    p_local_0 = []
    for leg in range(1, 5):
        q1 = df[f'leg{leg}_j1_{prefix}_deg'].iloc[0]
        q2 = df[f'leg{leg}_j2_{prefix}_deg'].iloc[0]
        q3 = df[f'leg{leg}_j3_{prefix}_deg'].iloc[0]
        p_local_0.append(leg_fk(q1, q2, q3))
        
    for idx in range(num_samples):
        disps = []
        for leg in range(1, 5):
            th = thetas[leg - 1]
            # Angulos actuales en la muestra idx
            q1 = df[f'leg{leg}_j1_{prefix}_deg'].iloc[idx]
            q2 = df[f'leg{leg}_j2_{prefix}_deg'].iloc[idx]
            q3 = df[f'leg{leg}_j3_{prefix}_deg'].iloc[idx]
            p_local = leg_fk(q1, q2, q3)
            
            # Desplazamiento en el frame local de la pata
            dp_local = p_local_0[leg - 1] - p_local
            
            # Rotar al frame del chasis (cuerpo)
            cos_th = np.cos(th)
            sin_th = np.sin(th)
            dp_body_x = dp_local[0] * cos_th - dp_local[1] * sin_th
            dp_body_y = dp_local[0] * sin_th + dp_local[1] * cos_th
            dp_body_z = dp_local[2]
            
            disps.append([dp_body_x, dp_body_y, dp_body_z])
            
        # El desplazamiento del chasis es el promedio estimado de las 4 patas
        mean_disp = np.mean(disps, axis=0)
        dx[idx] = mean_disp[0]
        dy[idx] = mean_disp[1]
        dz[idx] = mean_disp[2]
        
    return dx, dy, dz

def main():
    csv_path = resolve_path(FILENAME)
    if not csv_path:
        print(f"[Error] No se encuentra el archivo de telemetria: {FILENAME}")
        print("Asegurate de realizar el ensayo 'Circulo Cuerpo' en la interfaz y vuelve a intentarlo.")
        return
        
    print(f"Cargando datos desde: {csv_path}")
    df = pd.read_csv(csv_path)
    
    if len(df) < 5:
        print("[Error] El archivo de telemetria contiene muy pocos datos.")
        return
        
    # Tiempo relativo en segundos
    t = (df['timestamp_ms'] - df['timestamp_ms'].iloc[0]) / 1000.0
    
    # Calcular desplazamiento de consigna (target) y real (actual) del chasis en mm
    dx_t, dy_t, dz_t = get_body_displacement(df, prefix='target')
    dx_a, dy_a, dz_a = get_body_displacement(df, prefix='actual')
    
    dx_t_mm, dy_t_mm, dz_t_mm = dx_t * 1000.0, dy_t * 1000.0, dz_t * 1000.0
    dx_a_mm, dy_a_mm, dz_a_mm = dx_a * 1000.0, dy_a * 1000.0, dz_a * 1000.0
    
    # Calcular error euclidiano cartesiano (en mm)
    error_euclid_mm = np.sqrt((dx_t_mm - dx_a_mm)**2 + (dy_t_mm - dy_a_mm)**2 + (dz_t_mm - dz_a_mm)**2)
    
    # Imprimir estadisticas
    print("\n--- Estadisticas de Error del Chasis (en mm) ---")
    print(f"Error medio: {error_euclid_mm.mean():.2f} mm")
    print(f"Error maximo: {error_euclid_mm.max():.2f} mm")
    print(f"Desviacion estandar: {error_euclid_mm.std():.2f} mm")
    print("------------------------------------------------")

    # =========================================================================
    # FIGURA 1: COMPARATIVA TEMPORAL (2D)
    # =========================================================================
    fig_2d, axes = plt.subplots(4, 1, figsize=(9, 10), sharex=True)
    fig_2d.patch.set_facecolor('white')
    
    colores = {'X': '#0f4c81', 'Y': '#f58220', 'Error': '#d62728'}
    
    # Subplot X
    apply_academic_styling(axes[0])
    axes[0].plot(t, dx_t_mm, color='#333333', linestyle='--', lw=1.2, label='Consigna')
    axes[0].plot(t, dx_a_mm, color=colores['X'], lw=1.5, label='Medida Real')
    axes[0].set_ylabel('Eje X (mm)')
    axes[0].legend(loc='upper right')
    
    # Subplot Y
    apply_academic_styling(axes[1])
    axes[1].plot(t, dy_t_mm, color='#333333', linestyle='--', lw=1.2)
    axes[1].plot(t, dy_a_mm, color=colores['Y'], lw=1.5)
    axes[1].set_ylabel('Eje Y (mm)')
    
    # Subplot Z
    apply_academic_styling(axes[2])
    axes[2].plot(t, dz_t_mm, color='#333333', linestyle='--', lw=1.2)
    axes[2].plot(t, dz_a_mm, color='#375a23', lw=1.5)
    axes[2].set_ylabel('Eje Z (mm)')
    
    # Subplot Error Euclidiano
    apply_academic_styling(axes[3])
    axes[3].plot(t, error_euclid_mm, color=colores['Error'], lw=1.5)
    axes[3].set_ylabel('Error (mm)')
    axes[3].set_xlabel('Tiempo (s)')
    axes[3].fill_between(t, 0, error_euclid_mm, color=colores['Error'], alpha=0.1)
    
    plt.tight_layout()
    fig_2d.savefig('validacion_cuerpo_error.pdf', format='pdf')
    fig_2d.savefig('validacion_cuerpo_error.png', format='png')
    plt.close(fig_2d)

    # =========================================================================
    # FIGURA 2: TRAYECTORIA ESPACIAL (3D)
    # =========================================================================
    fig_3d = plt.figure(figsize=(9, 8))
    fig_3d.patch.set_facecolor('white')
    
    ax_3d = fig_3d.add_subplot(111, projection='3d')
    ax_3d.set_facecolor('white')
    
    # Trazar lineas
    ax_3d.plot(dx_t_mm, dy_t_mm, dz_t_mm, color='#333333', linestyle='--', lw=1.5, label='Trayectoria Deseada (Consigna)')
    ax_3d.plot(dx_a_mm, dy_a_mm, dz_a_mm, color='#0f4c81', lw=2.0, label='Trayectoria Real (Seguimiento)')
    
    # Punto de inicio y fin
    ax_3d.scatter(dx_t_mm[0], dy_t_mm[0], dz_t_mm[0], color='#2ca02c', s=50, label='Inicio / Fin', zorder=5)
    
    # Ajuste de relacion de aspecto proporcional
    max_range = np.array([dx_t_mm.max()-dx_t_mm.min(), dy_t_mm.max()-dy_t_mm.min(), dz_t_mm.max()-dz_t_mm.min()]).max() / 2.0
    mid_x = (dx_t_mm.max()+dx_t_mm.min()) * 0.5
    mid_y = (dy_t_mm.max()+dy_t_mm.min()) * 0.5
    mid_z = (dz_t_mm.max()+dz_t_mm.min()) * 0.5
    
    max_range = max(max_range, 50.0) 
    
    ax_3d.set_xlim(mid_x - max_range - 10, mid_x + max_range + 10)
    ax_3d.set_ylim(mid_y - max_range - 10, mid_y + max_range + 10)
    ax_3d.set_zlim(mid_z - max_range - 10, mid_z + max_range + 10)
    
    ax_3d.set_xlabel('Desplazamiento X (mm)', labelpad=10)
    ax_3d.set_ylabel('Desplazamiento Y (mm)', labelpad=10)
    ax_3d.set_zlabel('Desplazamiento Z (mm)', labelpad=10)
    ax_3d.legend(loc='upper right', frameon=True, facecolor='white', edgecolor='#e5e7eb')
    
    ax_3d.view_init(elev=30, azim=45)
    
    plt.tight_layout()
    fig_3d.savefig('validacion_cuerpo_trayectoria_3d.pdf', format='pdf')
    fig_3d.savefig('validacion_cuerpo_trayectoria_3d.png', format='png')
    plt.close(fig_3d)

    print("Todas las graficas de validacion del cuerpo guardadas con exito (2D y 3D).")

if __name__ == '__main__':
    main()
