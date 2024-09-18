import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# Function to normalize a vector
def normalize(vector):
    return vector / np.linalg.norm(vector)

# Function to define the r and theta unit vectors
def create_polar_vectors(origin, point):
    r_vector = normalize(point - origin)  # r-axis points from the origin to the point P
    
    # Project r_vector onto the YZ-plane (ignore X-component)
    r_zy = np.array([0, r_vector[1], r_vector[2]])
    
    # Calculate theta vector as perpendicular to r_zy in the ZY-plane
    theta_vector = np.array([0, -r_zy[2], r_zy[1]])  # Rotate by 90 degrees in the ZY-plane
    
    # Normalize the theta_vector
    theta_vector = normalize(theta_vector)
    
    z_axis = np.array([0, 0, 1])  # Z-axis is aligned with the global Z-axis
    return r_vector, theta_vector, z_axis

# Function to plot a frame (axes) at a given origin
def plot_frame(ax, origin, x_axis, y_axis, z_axis, axis_length=15, linestyle='solid', label_prefix=''):
    ax.quiver(origin[0], origin[1], origin[2], 
              x_axis[0] * axis_length, x_axis[1] * axis_length, x_axis[2] * axis_length, 
              color='r', linestyle=linestyle, label=f'{label_prefix}X Axis')
    
    ax.quiver(origin[0], origin[1], origin[2], 
              y_axis[0] * axis_length, y_axis[1] * axis_length, y_axis[2] * axis_length, 
              color='g', linestyle=linestyle, label=f'{label_prefix}Y Axis')
    
    ax.quiver(origin[0], origin[1], origin[2], 
              z_axis[0] * axis_length, z_axis[1] * axis_length, z_axis[2] * axis_length, 
              color='b', linestyle=linestyle, label=f'{label_prefix}Z Axis')

# Function to plot vectors originating from a given point
def plot_vector(ax, origin, vector, color, label, linestyle='solid', length=10):
    ax.quiver(origin[0], origin[1], origin[2], 
              vector[0] * length, vector[1] * length, vector[2] * length, 
              color=color, linestyle=linestyle, label=label)

# Function to plot a triangular plane formed by three points
def plot_plane(ax, p1, p2, p3, alpha=0.5, color='blue'):
    vertices = [p1, p2, p3]
    poly = Poly3DCollection([vertices], color=color, alpha=alpha)
    ax.add_collection3d(poly)
    ax.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]], color=color)
    ax.plot([p2[0], p3[0]], [p2[1], p3[1]], [p2[2], p3[2]], color=color)
    ax.plot([p3[0], p1[0]], [p3[1], p1[1]], [p3[2], p1[2]], color=color)

# Main plotting function
def plot_coordinate_systems(point, origin1, origin2, axis_length=15, vector_length=10):
    fig = plt.figure(figsize=(12, 12))
    ax = fig.add_subplot(111, projection='3d')

    # Create the r and theta vectors for each frame
    r_axis1, theta_axis1, z_axis1 = create_polar_vectors(origin1, point)
    r_axis2, theta_axis2, z_axis2 = create_polar_vectors(origin2, point)

    # Reverse r_axis to point away from the origins, from the point P
    r_axis1 = -r_axis1
    r_axis2 = -r_axis2

    # First, plot the lines connecting the point to the origins
    ax.plot([origin1[0], point[0]], [origin1[1], point[1]], [origin1[2], point[2]], color='grey', label='Line to Frame 2 Origin')
    ax.plot([origin2[0], point[0]], [origin2[1], point[1]], [origin2[2], point[2]], color='grey', label='Line to Frame 2 Origin')

    # Plot frames after lines to avoid overlap
    plot_frame(ax, origin1, theta_axis1, normalize(np.cross(r_axis1, theta_axis1)), z_axis1, axis_length=axis_length, label_prefix='Frame 1 ')
    plot_frame(ax, origin2, theta_axis2, normalize(np.cross(r_axis2, theta_axis2)), z_axis2, axis_length=axis_length, linestyle='dashed', label_prefix='Frame 2 ')

    # Plot the point in space P
    ax.scatter(point[0], point[1], point[2], color='black', s=100, label='Point P')

    # Create and plot r1 and theta1 vectors
    plot_vector(ax, point, r_axis1, color='purple', label=r'$\vec{r}_1$', length=vector_length)
    plot_vector(ax, point, theta_axis1, color='orange', label=r'$\vec{\theta}_1$', length=vector_length)

    # Create and plot r2 and theta2 vectors
    plot_vector(ax, point, r_axis2, color='purple', label=r'$\vec{r}_2$', length=vector_length)
    plot_vector(ax, point, theta_axis2, color='orange', linestyle='dashed', label=r'$\vec{\theta}_2$', length=vector_length)

    # Create and plot h1 and h2 vectors
    h1_vector = r_axis1 + theta_axis1
    h2_vector = r_axis2 + theta_axis2
    
    plot_vector(ax, point, h1_vector, color='blue', label=r'$\vec{h}_1$', length=vector_length)
    plot_vector(ax, point, h2_vector, color='cyan', label=r'$\vec{h}_2$', length=vector_length)

    # Plot the plane containing h1 and h2
    plot_plane(ax, point, point + h1_vector * vector_length, point + h2_vector * vector_length)

    # Calculate and display the angle between h1 and h2
    dot_product = np.dot(h1_vector, h2_vector)
    angle_alpha = np.arccos(dot_product / (np.linalg.norm(h1_vector) * np.linalg.norm(h2_vector)))
    angle_alpha_deg = np.degrees(angle_alpha)
    print(f"The angle alpha between h1 and h2 is {angle_alpha_deg:.2f} degrees")

    # Set labels and limits
    ax.set_xlabel('X Axis')
    ax.set_ylabel('Y Axis')
    ax.set_zlabel('Z Axis')
    ax.set_xlim([-20, 20])
    ax.set_ylim([-20, 20])
    ax.set_zlim([-20, 20])

    # Show legend
    ax.legend()
    ax.legend(bbox_to_anchor=(1.1, 1), loc='upper left')
    plt.show()

# Example usage
random_point = np.array([2, 3, 4])
origin_1 = np.array([-10, -10, -10])
origin_2 = np.array([10, 10, 10])

plot_coordinate_systems(random_point, origin_1, origin_2)



