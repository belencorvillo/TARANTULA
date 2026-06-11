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
    'figure.titlesize': 12,
    'legend.fontsize': 9,
    'savefig.dpi': 300
})

def main():
    # Longitudes de eslabones en mm
    L1 = 80.0
    L2 = 188.0
    L3 = 211.0
    
    # Limites articulares en radianes
    q1_range = np.radians([-94.0, 105.0])
    q2_range = np.radians([-122.0, 117.0])
    q3_range = np.radians([-157.0, 154.0])
    
    # Generacion de muestras de Monte Carlo
    num_samples = 120000
    np.random.seed(42)
    q1 = np.random.uniform(q1_range[0], q1_range[1], num_samples)
    q2 = np.random.uniform(q2_range[0], q2_range[1], num_samples)
    q3 = np.random.uniform(q3_range[0], q3_range[1], num_samples)
    
    # Cinematica directa (FK)
    r = L1 + L2 * np.cos(q2) + L3 * np.cos(q2 + q3)
    x = r * np.cos(q1)
    y = r * np.sin(q1)
    z = L2 * np.sin(q2) + L3 * np.sin(q2 + q3)
    
    # Crear Figura
    fig = plt.figure(figsize=(10, 4.5))
    fig.patch.set_facecolor('white')
    
    # Subplot 1: Vista 3D del Espacio de Trabajo
    ax_3d = fig.add_subplot(121, projection='3d')
    ax_3d.set_facecolor('white')
    sc_3d = ax_3d.scatter(x, y, z, c=z, cmap='coolwarm', s=0.08, alpha=0.2)
    
    ax_3d.set_xlabel('Eje X (mm)', labelpad=6)
    ax_3d.set_ylabel('Eje Y (mm)', labelpad=6)
    ax_3d.set_zlabel('Eje Z (mm)', labelpad=6)
    ax_3d.set_title('(a) Representacion Tridimensional (3D)', fontweight='bold', pad=8)
    
    # Ajuste de perspectiva de camara
    ax_3d.view_init(elev=22, azim=-65)
    
    # Subplot 2: Proyeccion 2D Plano Sagital (R-Z)
    ax_2d = fig.add_subplot(122)
    ax_2d.set_facecolor('white')
    
    # Graficar puntos en plano R-Z
    sc_2d = ax_2d.scatter(r, z, c=z, cmap='coolwarm', s=0.08, alpha=0.2)
    
    # Dibujar limites teoricos (circunferencias desde la cadera L1, 0)
    theta_arc = np.linspace(-np.pi, np.pi, 200)
    # Alcance maximo L2 + L3
    r_max = L1 + (L2 + L3) * np.cos(theta_arc)
    z_max = (L2 + L3) * np.sin(theta_arc)
    ax_2d.plot(r_max, z_max, 'k--', lw=1.2, label=f'Alcance Max ({L2+L3:.0f} mm)')
    
    # Alcance minimo |L2 - L3|
    r_min = L1 + np.abs(L2 - L3) * np.cos(theta_arc)
    z_min = np.abs(L2 - L3) * np.sin(theta_arc)
    ax_2d.plot(r_min, z_min, 'r--', lw=1.2, label=f'Alcance Min ({np.abs(L2-L3):.0f} mm)')
    
    # Articulacion cadera (origen del femur)
    ax_2d.plot(L1, 0, 'go', ms=6, label='Articulacion J2 (Femur)')
    
    ax_2d.spines['top'].set_visible(False)
    ax_2d.spines['right'].set_visible(False)
    ax_2d.spines['left'].set_color('#cccccc')
    ax_2d.spines['bottom'].set_color('#cccccc')
    ax_2d.grid(True, which='both', linestyle=':', linewidth=0.5, color='#dddddd')
    
    ax_2d.set_xlabel('Alcance Radial R (mm)')
    ax_2d.set_ylabel('Altura Z (mm)')
    ax_2d.set_title('(b) Envolvente de Trabajo (Plano Sagital R-Z)', fontweight='bold', pad=8)
    ax_2d.legend(loc='lower left', frameon=True, facecolor='white', edgecolor='#e5e7eb')
    ax_2d.axis('equal')
    
    plt.tight_layout()
    
    # Guardar en PDF y PNG
    fig.savefig('espacio_trabajo_pata.pdf', format='pdf', bbox_inches='tight')
    fig.savefig('espacio_trabajo_pata.png', format='png', bbox_inches='tight')
    plt.close(fig)
    print("Graficas del espacio de trabajo generadas con exito (PDF y PNG).")

if __name__ == '__main__':
    main()
