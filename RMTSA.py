import open3d as o3d
import numpy as np
import os
from datetime import datetime
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull
from scipy.interpolate import Akima1DInterpolator
import tkinter as tk
from tkinter import filedialog
from textwrap import fill

# Set global font to solve Chinese display issue
plt.rcParams['font.sans-serif'] = ['SimHei']  # For Windows system
plt.rcParams['axes.unicode_minus'] = False

# —— Global Aesthetic Settings: Fonts & Line Widths ——
plt.rcParams.update({
    'font.size': 22,           # Base font size (affects ticks, labels, etc.)
    'axes.labelsize': 18,      # Axis label size
    'axes.titlesize': 20,      # Title size
    'xtick.labelsize': 16,     # X-axis tick labels
    'ytick.labelsize': 16,     # Y-axis tick labels
    'legend.fontsize': 16,     # Legend text size
    'lines.linewidth': 2.5,    # Default line thickness
})


# ================== Parameter Configuration Area ==================
VOXEL_SIZE = 0.05  # Downsampling granularity (0.05-0.3)
MIN_POINTS = 2000  # Minimum number of points for valid section
OUTPUT_DIR = "tunnel_sections"  # Output directory name

# Suggested thresholds (cm) for different construction stages (shown to user at input time)
# - Initial support: 0.05 m (5 cm)
# - Secondary lining: 0.15 m (15 cm)
# - For catching severe occlusion / missing-data outliers: 0.5 m (50 cm)
SUGGEST_THRESHOLD_INITIAL_SUPPORT_CM = 30
SUGGEST_THRESHOLD_SECONDARY_LINING_CM = 15



# ================== File 2 Function Definitions ==================
def load_point_cloud(file_path):
    """ Safely load point cloud file """
    try:
        pcd = o3d.io.read_point_cloud(file_path)
        if not pcd.has_points():
            raise ValueError("Point cloud file is empty or in incorrect format")
        print(f"Successfully loaded point cloud, original number of points: {len(pcd.points)}")
        return pcd
    except Exception as e:
        print(f"File loading failed: {str(e)}")
        exit(1)


def preprocess(pcd):
    """ Data preprocessing """
    # Downsample filtering
    down_pcd = pcd.voxel_down_sample(VOXEL_SIZE)
    print(f"Number of points after downsampling: {len(down_pcd.points)}")

    # Statistical outlier removal
    cl, _ = down_pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    print(f"Number of points after filtering: {len(cl.points)}")
    return cl


def compute_principal_axis(pcd, label="Verify principal direction"):
    """ Improved principal direction calculation (based on regional PCA) """
    points = np.asarray(pcd.points)

    # Take the middle 80% area to exclude interference
    x_coords = points[:, 0]
    q1, q3 = np.percentile(x_coords, [10, 90])
    mask = (x_coords > q1) & (x_coords < q3)
    core_points = pcd.select_by_index(np.where(mask)[0])

    # Covariance matrix calculation
    points = np.asarray(core_points.points)
    centroid = np.mean(points, axis=0)
    cov_matrix = np.cov((points - centroid).T)
    eigenvalues, eigenvectors = np.linalg.eigh(cov_matrix)

    # Select the direction with maximum variance and correct the direction
    principal_axis = eigenvectors[:, np.argmax(eigenvalues)]
    if principal_axis[0] < 0:  # Ensure direction consistency
        principal_axis *= -1

    print(f"{label}: {principal_axis} (Variance ratio: {np.max(eigenvalues) / np.sum(eigenvalues):.1%})")
    return principal_axis


def align_to_x_axis(pcd, principal_axis):
    """ Precise alignment to X-axis (Kabsch algorithm) """
    target_axis = np.array([1, 0, 0])

    # Calculate rotation matrix
    v = np.cross(principal_axis, target_axis)
    c = np.dot(principal_axis, target_axis)
    kmat = np.array([[0, -v[2], v[1]],
                     [v[2], 0, -v[0]],
                     [-v[1], v[0], 0]])
    R = np.eye(3) + kmat + kmat @ kmat * (1 / (1 + c))

    # Perform rotation
    aligned_pcd = pcd.rotate(R, center=(0, 0, 0))

    # Verify alignment result
    new_axis = compute_principal_axis(aligned_pcd)
    angle_dev = np.degrees(np.arccos(np.clip(new_axis.dot(target_axis), -1, 1)))
    print(f"Alignment verification: Deviation from X-axis {angle_dev:.2f}°")
    return aligned_pcd


def extract_sections(pcd, slice_interval, slice_thickness):
    """ Extract sections along the X-axis """
    points = np.asarray(pcd.points)
    x_coords = points[:, 0]
    min_x, max_x = np.min(x_coords), np.max(x_coords)

    sections = []
    for x_start in np.arange(min_x, max_x, slice_interval):
        x_end = x_start + slice_thickness
        mask = (x_coords >= x_start) & (x_coords < x_end)
        section = pcd.select_by_index(np.where(mask)[0])

        if len(section.points) >= MIN_POINTS:
            sections.append({
                "position": (x_start + x_end) / 2,
                "pcd": section
            })
            print(f"Section {x_start:.1f}-{x_end:.1f}m | Number of points: {len(section.points)}")
    return sections


def save_section_data(section, output_path, section_id):
    """ Save individual section data """
    pos = section["position"]
    # Save TXT file (Y/Z coordinates)
    txt_filename = f"section_{section_id:04d}_{pos:.1f}m.txt"
    txt_filepath = os.path.join(output_path, txt_filename)
    points = np.asarray(section["pcd"].points)
    np.savetxt(txt_filepath, points[:, 1:3], fmt="%.6f")  # Save only YZ coordinates

    print(f"Saved: {txt_filename}")
    return txt_filepath


# ================== File 1 Function Definitions ==================
class Curve:
    """ Curve data container class """

    def __init__(self, x_range, y_values):
        self.x = x_range
        self.y = y_values


def load_data(file_path):
    """ Enhanced data loading function """
    try:
        data = np.loadtxt(file_path)
        if data.shape[1] != 2:
            raise ValueError("Data format error: Each line should contain two numerical values")

        x, y = data[:, 0], data[:, 1]

        # Data cleaning and sorting
        sorted_idx = np.argsort(x)
        x, y = x[sorted_idx], y[sorted_idx]

        # Handle duplicate x values (take maximum y value)
        unique_x, unique_idx = np.unique(x, return_index=True)
        y_max = [np.max(y[x == ux]) for ux in unique_x]
        return unique_x, np.array(y_max)

    except Exception as e:
        raise RuntimeError(f"Data loading failed: {str(e)}")


def robust_convex_hull(points):
    """ Robust convex hull calculation function """
    try:
        hull = ConvexHull(points)
    except:
        # Add slight perturbation to solve collinearity issues
        points += np.random.normal(0, 1e-10, points.shape)
        hull = ConvexHull(points)

    hull_points = points[hull.vertices]

    # Multi-level sorting: First ascending x, then descending y
    sorted_indices = np.lexsort((-hull_points[:, 1], hull_points[:, 0]))
    sorted_hull = hull_points[sorted_indices]

    # Handle duplicate x values (retain maximum y value)
    unique_x, indices = np.unique(sorted_hull[:, 0], return_index=True)
    final_points = []
    for x_val in unique_x:
        mask = sorted_hull[:, 0] == x_val
        y_max = np.max(sorted_hull[mask, 1])
        final_points.append([x_val, y_max])

    sorted_hull = np.array(final_points)

    if len(sorted_hull) < 2:
        raise ValueError("Insufficient valid convex hull vertices")

    return sorted_hull


def process_file1(x, y):
    """ Process design curve file """
    points = np.column_stack([x, y])
    hull_points = robust_convex_hull(points)

    # Obtain boundary features
    cs = Akima1DInterpolator(hull_points[:, 0], hull_points[:, 1])
    x_min, x_max = np.min(x), np.max(x)
    y_min_x = cs(x_min)
    y_max_x = cs(x_max)

    # Filtering logic
    mid_x = (x_min + x_max) / 2
    valid_mask = ((x <= mid_x) & (y >= y_min_x)) | ((x > mid_x) & (y >= y_max_x))
    filtered_x, filtered_y = x[valid_mask], y[valid_mask]

    # Final convex hull fitting
    final_hull = robust_convex_hull(np.column_stack([filtered_x, filtered_y]))
    final_x = np.linspace(final_hull[:, 0].min(), final_hull[:, 0].max(), 1000)
    final_y = Akima1DInterpolator(final_hull[:, 0], final_hull[:, 1])(final_x)
    return Curve(final_x, final_y)


def process_file2(x, y, voxel_size=0.02):
    """ Process actual measurement data and return voxelized centroid points (no mirror flipping) """
    points = np.column_stack([x, y])

    # Voxelization processing - First perform voxelization on all data
    x_min, x_max = np.min(x), np.max(x)
    y_min, y_max = np.min(y), np.max(y)

    # Generate voxel grid boundaries
    x_edges = np.arange(x_min, x_max + voxel_size, voxel_size)
    y_edges = np.arange(y_min, y_max + voxel_size, voxel_size)

    # Assign data points to voxels
    x_bin_indices = np.digitize(x, x_edges)
    y_bin_indices = np.digitize(y, y_edges)

    # Calculate centroid of each voxel
    centroids = []
    for i in range(1, len(x_edges)):
        for j in range(1, len(y_edges)):
            mask = (x_bin_indices == i) & (y_bin_indices == j)
            if np.sum(mask) > 0:
                voxel_center_x = (x_edges[i - 1] + x_edges[i]) / 2
                voxel_center_y = (y_edges[j - 1] + y_edges[j]) / 2
                centroids.append([voxel_center_x, voxel_center_y])

    centroids = np.array(centroids)

    if len(centroids) == 0:
        return np.array([])

    # Strictly sort by x-coordinate (key step to solve interpolation issues)
    sort_idx = np.argsort(centroids[:, 0])
    centroids = centroids[sort_idx]

    # Duplicate removal (prevent identical x values)
    _, unique_idx = np.unique(centroids[:, 0], return_index=True)
    centroids = centroids[unique_idx]

    # Calculate convex hull to assist filtering
    hull_points = robust_convex_hull(centroids)

    # Filter valid data points (retain original logic)
    x1, y1 = hull_points[hull_points[:, 0].argmin()]
    x2, y2 = hull_points[hull_points[:, 0].argmax()]
    xm = (x1 + x2) / 2
    valid_mask = ~(((centroids[:, 0] < xm) & (centroids[:, 1] < y1)) |
                   ((centroids[:, 0] > xm) & (centroids[:, 1] < y2)) |
                   ((centroids[:, 0] < x1) | (centroids[:, 0] > x2)))
    filtered_centroids = centroids[valid_mask]

    return filtered_centroids


def fit_curve2_with_convex_hull(centroids):
    """ Fit curve 2 using convex hull """
    if len(centroids) < 2:
        raise ValueError("At least 2 centroid points required for fitting")

    # Calculate convex hull
    hull_points = robust_convex_hull(centroids)

    # Verify strictly increasing x-coordinates
    if not np.all(np.diff(hull_points[:, 0]) > 0):
        raise ValueError("X-coordinates are not strictly increasing")

    # Use Akima interpolation to enhance smoothness
    cs = Akima1DInterpolator(hull_points[:, 0], hull_points[:, 1])
    x_new = np.linspace(hull_points[0, 0], hull_points[-1, 0], 1000)
    return Curve(x_new, cs(x_new))


def compute_centroid(curve):
    """ Calculate curve centroid """
    centroid_x = np.mean(curve.x)
    centroid_y = np.mean(curve.y)
    return np.array([centroid_x, centroid_y])


def calculate_tangent_angles(curve):
    """ Calculate tangent angles (in degrees) at each point on the curve """
    dy = np.gradient(curve.y, curve.x)
    angles_rad = np.arctan(dy)  # Tangent slope corresponding to radians
    angles_deg = np.degrees(angles_rad)
    return angles_deg


def find_an_points(curve, angle_step=10):
    """ Find feature points with specified angle increments """
    angles_deg = calculate_tangent_angles(curve)
    target_angles = np.arange(-80, 90, angle_step)  # From -80 to 90 degrees, step 10
    an_points = []
    for target_angle in target_angles:
        # Find the point closest to the target angle
        angle_diff = np.abs(angles_deg - target_angle)
        closest_idx = np.argmin(angle_diff)

        # Verify angle difference is within acceptable range
        if angle_diff[closest_idx] <= angle_step / 2:
            x_an = curve.x[closest_idx]
            y_an = curve.y[closest_idx]
            an_points.append({
                'point': (x_an, y_an),
                'angle': target_angle,
                'index': closest_idx
            })
    return an_points


def calculate_normal_line(an_point, curve):
    """ Calculate normal line equation (A, B, C form: Ax + By + C = 0) """
    x_an, y_an = an_point['point']
    # Calculate tangent slope
    dy = np.gradient(curve.y, curve.x)
    tangent_slope = dy[an_point['index']]
    if tangent_slope == 0:
        # Horizontal tangent, normal line is vertical
        A, B, C = 0, 1, -y_an
    else:
        normal_slope = -1 / tangent_slope
        A = normal_slope
        B = -1
        C = y_an - normal_slope * x_an
    return (A, B, C)


def distance_to_line(point, line_coeffs):
    """ Calculate perpendicular distance from point to line """
    A, B, C = line_coeffs
    x, y = point
    return np.abs(A * x + B * y + C) / np.sqrt(A ** 2 + B ** 2)


def find_bn_point(an_point, line_coeffs, centroids, curve):
    """ Find the feature point closest to the normal line and calculate signed distance """
    # Calculate distances from all feature points to the normal line
    distances = np.array([distance_to_line(p, line_coeffs) for p in centroids])
    closest_idx = np.argmin(distances)
    bn_point = centroids[closest_idx]

    # Determine sign: Compare bn point's y value with curve's y value at that x
    curve_y_at_bn = np.interp(bn_point[0], curve.x, curve.y)
    sign = 1 if bn_point[1] > curve_y_at_bn else -1

    # Calculate actual distance
    dx = bn_point[0] - an_point['point'][0]
    dy = bn_point[1] - an_point['point'][1]
    distance = sign * np.sqrt(dx ** 2 + dy ** 2)

    return bn_point, distance


def visualize_an_points(translated_curve1, mirrored_centroids, results, output_path, section_id, abnormal_threshold):
    """
    Plot the design curve, measured points, and feature (AN) points with labels.
    - Labels are displaced along the curve normal to avoid covering geometry.
    - Angle text is the NEGATED value (per requirement); distance shown on next line.
    - X/Y labels use explicit coordinate names; only distance unit note is shown in-figure.
    - Z-order: design curve (bottom), measured points (above), feature stars/labels (top).
    """
    fig, ax = plt.subplots(figsize=(18, 10))

    # --- Base layers (set z-order explicitly) ---
    ax.plot(
        translated_curve1.x, translated_curve1.y,
        'b-', linewidth=4.5, label='Design curve', zorder=1  # bottom-most
    )
    ax.scatter(
        mirrored_centroids[:, 0], mirrored_centroids[:, 1],
        s=12, c='r', alpha=0.65, label='Measured data', zorder=3  # above the line
    )

    # --- Scale for label offset in data coordinates (adapts to scene size) ---
    x_all = np.concatenate([translated_curve1.x, mirrored_centroids[:, 0]])
    y_all = np.concatenate([translated_curve1.y, mirrored_centroids[:, 1]])
    x_rng = float(np.max(x_all) - np.min(x_all))
    y_rng = float(np.max(y_all) - np.min(y_all))
    offset = 0.03 * max(x_rng, y_rng)  # 3% of scene size

    # --- Feature points and labels (displace along normal) ---
    for idx, res in enumerate(results):
        x_an, y_an = res['an_point']
        ax.scatter(
            x_an, y_an,
            s=220, c='lime', edgecolor='k', linewidths=1.8,
            marker='*', zorder=6,  # on top of points and line
            label='Feature points' if idx == 0 else None
        )

        # Normal direction from tangent angle theta: n = (-sinθ, cosθ)
        theta = np.deg2rad(res['angle'])
        nx, ny = -np.sin(theta), np.cos(theta)
        x_lbl = x_an + nx * offset
        y_lbl = y_an + ny * offset

        # Flag potentially invalid points (e.g., occlusion/missing data).
        # Use absolute value so both extreme over-excavation (+) and under-excavation (-)
        # can be highlighted.
        is_abnormal = abs(res['distance']) > float(abnormal_threshold)
        txt_color = 'red' if is_abnormal else 'black'
        box_ec = 'red' if is_abnormal else 'black'

        # Stacked label: angle (negated) on first line, distance on second
        ax.annotate(
            f"{-res['angle']}°\n{res['distance']:.2f}",
            xy=(x_an, y_an), textcoords='data', xytext=(x_lbl, y_lbl),
            ha='center', va='bottom', fontsize=18, weight='bold', color=txt_color,
            bbox=dict(boxstyle='round,pad=0.25', fc='white', ec=box_ec, lw=1.2, alpha=0.9),
            zorder=7  # keep labels above all geometry
        )

    # --- Axes styling ---
    ax.grid(alpha=0.35, linewidth=1.6)
    ax.tick_params(axis='both', labelsize=22, width=1.6, length=8)

    # Units note inside the axes (distance only)
    msg = fill("The units of over/under excavation are centimeters (cm).", width=42)
    warn_msg = fill("Values in red color indicate potential occlusion/missing data.", width=44)
    ax.text(0.01, 0.02, warn_msg, transform=ax.transAxes, ha='left', va='bottom',
            fontsize=18, wrap=True, bbox=dict(boxstyle='round,pad=0.25', fc='white', ec='gray', lw=1.0, alpha=0.8))

    ax.text(0.65, 0.02, msg, transform=ax.transAxes, ha='left', va='bottom',
            fontsize=18, wrap=True, bbox=dict(boxstyle='round,pad=0.25', fc='white', ec='gray', lw=1.0, alpha=0.8))

    # Axis labels
    ax.set_xlabel('Horizontal coordinate (m)', fontsize=22)
    ax.set_ylabel('Vertical coordinate (m)', fontsize=22)

    # Preserve geometry aspect (1:1 in data space)
    ax.set_aspect('equal', adjustable='datalim')

    # Leave extra headroom at the bottom for the inside legend
    y_min, y_max = float(np.min(y_all)), float(np.max(y_all))
    y_range = max(y_max - y_min, 1e-9)
    ax.set_ylim(y_min - 0.18 * y_range, y_max)

    # Legend inside, bottom-center
    ax.legend(
        loc='lower center', frameon=True, fontsize=24,
        borderpad=0.6, handlelength=2.4, markerscale=1.8, labelspacing=1.0
    )

    # Save and close
    filename = f"over_under_excavation_diagram_section_{section_id:04d}.png"
    fig.savefig(os.path.join(output_path, filename), dpi=500, pad_inches=0.2)
    plt.close(fig)

    print(f"Results saved to {filename}")
    return filename



def output_results(results, output_path, section_id, abnormal_threshold):
    """ Output results to text file """
    filename = f"over_under_excavation_section_{section_id:04d}.txt"
    filepath = os.path.join(output_path, filename)
    with open(filepath, 'w') as f:
        # Keep the file ASCII/English-friendly for cross-platform readability.
        f.write("PointID\tAngle(deg)\tDistance(cm)\tAbnormal(1/0)\n")
        for idx, res in enumerate(results):
            abnormal = 1 if abs(res['distance']) > float(abnormal_threshold) else 0
            f.write(f"{idx + 1}\t{-res['angle']}\t{res['distance']:.2f}\t{abnormal}\n")
    print(f"Results saved to {filename}")
    return filename



# ================== Main Program ==================
if __name__ == "__main__":
    # User interaction input
    print("Welcome to the Tunnel Section Analysis System")

    # Create Tkinter root window
    root = tk.Tk()
    root.withdraw()  # Hide root window

    # Open file selection dialog for design curve file
    print("Please select design curve file")
    DESIGN_CURVE_FILE = filedialog.askopenfilename(
        title="Select Design Curve File",
        filetypes=[("Text Files", "*.txt"), ("All Files", "*.*")]
    )

    # Open file selection dialog for measurement data file
    print("Please select measurement data file")
    INPUT_FILE = filedialog.askopenfilename(
        title="Select Point Cloud File",
        filetypes=[("Point Cloud Data", "*.pcd"), ("All Files", "*.*")]
    )

    # Destroy Tkinter root window
    root.destroy()

    if not DESIGN_CURVE_FILE or not INPUT_FILE:
        print("No file selected, program exiting")
        exit(1)

    SLICE_INTERVAL = input("Please enter section spacing (meters) (Default: 10): ").strip()
    if not SLICE_INTERVAL:
        SLICE_INTERVAL = 10.0
    else:
        SLICE_INTERVAL = float(SLICE_INTERVAL)

    SLICE_THICKNESS = input("Please enter section thickness (meters) (Default: 0.3): ").strip()
    if not SLICE_THICKNESS:
        SLICE_THICKNESS = 0.3
    else:
        SLICE_THICKNESS = float(SLICE_THICKNESS)

    # Abnormal over/under excavation threshold (no default; user must input).
    # Practical suggestions (cm):
    #   - Initial support: 30 cm
    #   - Secondary lining: 15 cm
    #   - For catching severe occlusion/missing-data outliers: 50 cm
    print("\nAbnormal excavation threshold suggestions (cm):")
    print(f"  Initial support: {SUGGEST_THRESHOLD_INITIAL_SUPPORT_CM:.0f} (cm)")
    print(f"  Secondary lining: {SUGGEST_THRESHOLD_SECONDARY_LINING_CM:.0f} (cm)")
    print("  Note: Please set the threshold based on the construction stage and project requirements.")
    print("  Values exceeding the threshold may indicate missing data in that angular range (e.g., due to occlusions).")


    while True:
        _thr = input("Please enter abnormal excavation threshold (cm), e.g., 15 / 30: ").strip()
        try:
            ABNORMAL_THRESHOLD = float(_thr)
            if ABNORMAL_THRESHOLD <= 0:
                raise ValueError("Threshold must be > 0")
            break
        except Exception:
            print("Invalid input. Please enter a positive number in centimeters (e.g., 15).")

    raw_pcd = None
    curve1 = None
    centroid1 = None

    # 1. Load design curve
    try:
        x1, y1 = load_data(DESIGN_CURVE_FILE)
        curve1 = process_file1(x1, y1)
        centroid1 = compute_centroid(curve1)
    except Exception as e:
        print(f"Failed to load or process design curve file: {e}")
        exit(1)

    # 2. Load measurement data
    try:
        raw_pcd = load_point_cloud(INPUT_FILE)
    except Exception as e:
        print(f"Failed to load measurement data file: {e}")
        exit(1)

    if raw_pcd is None or curve1 is None or centroid1 is None:
        print("Critical variables not initialized, program exiting")
        exit(1)

    # 3. Preprocessing
    processed_pcd = preprocess(raw_pcd)

    # 4. Principal direction calculation and alignment
    principal_axis = compute_principal_axis(processed_pcd, label="Original principal direction")
    aligned_pcd = align_to_x_axis(processed_pcd, principal_axis)

    # 5. Section extraction
    sections = extract_sections(aligned_pcd, SLICE_INTERVAL, SLICE_THICKNESS)
    print(f"\nNumber of valid sections: {len(sections)}")

    if not sections:
        print("No valid sections found, please adjust parameters")
        exit(1)

    # 6. Prepare output directory
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_path = f"{OUTPUT_DIR}_{timestamp}"
    os.makedirs(output_path, exist_ok=True)

    # 7. Process each section
    metadata = []
    for section_id, section in enumerate(sections):
        try:
            # Save section data
            txt_filepath = save_section_data(section, output_path, section_id)
            print("\n" + "=" * 60)
            print(f"Processing section {section_id + 1}/{len(sections)}")

            # Load section data
            x2, y2 = load_data(txt_filepath)

            # Process actual measurement data
            centroids = process_file2(x2, y2)
            if len(centroids) < 2:
                print("Insufficient voxelized centroid points, skipping this section")
                continue

            # Fit curve 2 using convex hull
            curve2 = fit_curve2_with_convex_hull(centroids)

            # Calculate centroid of curve 2
            centroid2 = compute_centroid(curve2)
            print(f"Centroid of section {section_id}: {centroid2}")

            # Mirror flip centroid points
            mirrored_centroids = np.copy(centroids)
            mirrored_centroids[:, 0] = 2 * centroid2[0] - mirrored_centroids[:, 0]

            # Translate curve 1 to centroid position of curve 2
            translated_x = curve1.x - centroid1[0] + centroid2[0]
            translated_y = curve1.y - centroid1[1] + centroid2[1]
            translated_curve1 = Curve(translated_x, translated_y)

            # Find AN points and calculate BN point distances
            an_points = find_an_points(translated_curve1)
            results = []
            for an in an_points:
                line_coeffs = calculate_normal_line(an, translated_curve1)
                bn_point, distance = find_bn_point(an, line_coeffs, mirrored_centroids, translated_curve1)
                distance_cm = float(distance) * 100.0
                results.append({
                    'angle': an['angle'],
                    'distance': distance_cm,
                    'an_point': an['point'],
                    'bn_point': bn_point
                })

            # Sort by angle from 75 degrees to -75 degrees
            results.sort(key=lambda x: -x['angle'])

            # Output results file
            dist_filename = output_results(results, output_path, section_id, abnormal_threshold=ABNORMAL_THRESHOLD)

            # Annotate AN points
            an_filename = visualize_an_points(translated_curve1, mirrored_centroids, results, output_path, section_id, abnormal_threshold=ABNORMAL_THRESHOLD)

            # Record metadata
            metadata.append({
                "id": section_id,
                "position": section["position"],
                "points": len(section["pcd"].points),
                "pcd_file": os.path.basename(txt_filepath).replace("txt", "pcd"),
                "txt_file": os.path.basename(txt_filepath),
                "dist_file": os.path.basename(dist_filename),
                "an_file": os.path.basename(an_filename)
            })

        except Exception as e:
            print(f"Error processing section {section_id}: {str(e)}")
            continue

    # 8. Save metadata
    if metadata:
        with open(os.path.join(output_path, "metadata.csv"), "w") as f:
            f.write("ID,Center Position(m),Point Cloud File,Section File,Over/under Excavation File,Over/under Excavation Diagram File\n")
            for item in metadata:
                f.write(f"{item['id']},{item['position']:.2f},{item['pcd_file']},{item['txt_file']},{item['dist_file']},{item['an_file']}\n")
        print(f"\nMetadata saved to: {os.path.join(output_path, 'metadata.csv')}")

    print(f"\nAll results saved to: {os.path.abspath(output_path)}")