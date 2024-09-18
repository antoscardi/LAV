import numpy as np
from scipy.optimize import minimize
import sympy as sp
import matplotlib.pyplot as plt

# Define the variables and the function
theta = sp.symbols('theta')
a, b = sp.symbols('a b')
n_values = 100

# Original and approximate expressions
original_expr = (3 * sp.cos(theta)**2 + 1)
approx_expr = a * sp.cos(theta)**2 + b * sp.sin(theta)**2

# Define the error function (the absolute difference between the two expressions)
error_expr = sp.Abs(original_expr - approx_expr)

# Pretty print the expressions for verification
print("Original Expression:")
sp.pretty_print(original_expr)
print("\nApproximation Expression:")
sp.pretty_print(approx_expr)
print("\nError Expression:")
sp.pretty_print(error_expr)

# Define a function to calculate the mean squared error over a range of theta values
def calculate_error(params):
    a_val, b_val = params
    error_values = []
    theta_values = np.linspace(0, 2 * np.pi, n_values)
    for theta_val in theta_values:
        error_val = error_expr.evalf(subs={a: a_val, b: b_val, theta: theta_val})
        error_values.append(error_val**2)
    mean_error = np.mean(error_values)
    print(f"Iteration: a={a_val:.3f}, b={b_val:.3f}: Mean Error={mean_error*100:.3f}%")
    return mean_error

# Initial guess for a and b
initial_guess = [1, 1]
print("Initial Guess:", initial_guess)

# Perform the optimization to minimize the mean squared error
result = minimize(calculate_error, initial_guess, bounds=((0.0001, 10), (0.0001, 10)))
best_a, best_b = result.x

print("Optimization Result:")
print(f"Best a: {best_a:.3f}")
print(f"Best b: {best_b:.3f}")

# Plot the original and approximated functions on a polar plot
theta_values = np.linspace(0, 2 * np.pi, n_values)
original_values = [original_expr.evalf(subs={theta: val}) for val in theta_values]
approx_values = [approx_expr.evalf(subs={a: best_a, b: best_b, theta: val}) for val in theta_values]
unit_circle = [approx_expr.evalf(subs={a: 1, b: 1, theta: val}) for val in theta_values]

plt.figure(figsize=(15,5))
plt.subplot(1, 2, 1, projection='polar')
plt.plot(theta_values, original_values, label='actual', )
plt.plot(theta_values, approx_values, label='approximation')
#plt.plot(theta_values, unit_circle)

plt.legend(loc='upper center', bbox_to_anchor=(0.5, 0.5))

plt.savefig('/home/anto/Desktop/polar_plot.png', bbox_inches='tight')
plt.show()








import numpy as np
from scipy.optimize import minimize
import sympy as sp
import matplotlib.pyplot as plt

# Define the variables and the parameters
theta, phi = sp.symbols('theta phi')
a, b, c, d = sp.symbols('a b c d')
n_values = 100

# Define the original and approximate expressions
original_expr = (sp.cos(theta)**2 + 1) * (sp.cos(phi)**2 + 1)
approx_expr = a * sp.sin(theta)**2 + b * sp.cos(theta)**2 + c * sp.sin(phi)**2 + d * sp.cos(phi)**2

# Define the error function (the absolute difference between the two expressions)
error_expr = sp.Abs(original_expr - approx_expr)

# Pretty print the expressions for verification
print("Original Expression:")
sp.pretty_print(original_expr)
print("\nApproximation Expression:")
sp.pretty_print(approx_expr)
print("\nError Expression:")
sp.pretty_print(error_expr)

# Define a function to calculate the mean squared error over a range of theta and phi values
def calculate_error(params):
    a_val, b_val, c_val, d_val = params
    error_values = []
    theta_values = np.linspace(0, 2*np.pi, n_values)
    phi_values = np.linspace(0, 2 * np.pi, n_values)
    for theta_val in theta_values:
        for phi_val in phi_values:
            error_val = error_expr.evalf(subs={a: a_val, b: b_val, c: c_val, d: d_val, theta: theta_val, phi: phi_val})
            error_values.append(error_val**2)
    mean_error = np.mean(error_values)
    print(f"Iteration: a={a_val:.3f}, b={b_val:.3f}, c={c_val:.3f}, d={d_val:.3f}: Mean Error={mean_error:.3f}%")
    return mean_error

# Initial guess for a, b, c, and d
initial_guess = [1, 1, 1, 1]
print("Initial Guess:", initial_guess)

# Perform the optimization to minimize the mean squared error
result = minimize(calculate_error, initial_guess, bounds=((0.0001, 10), (0.0001, 10), (0.0001, 10), (0.0001, 10)))
best_a, best_b, best_c, best_d = result.x

# Print the optimization result
print("Optimization Result:")
print(f"Best a: {best_a:.3f}")
print(f"Best b: {best_b:.3f}")
print(f"Best c: {best_c:.3f}")
print(f"Best d: {best_d:.3f}")

# Plot the original function and the best-fit approximation
theta_values = np.linspace(0, np.pi, n_values)
phi_values = np.linspace(0, 2 * np.pi, n_values)
theta_grid, phi_grid = np.meshgrid(theta_values, phi_values)

# Evaluate the original and approximated expressions on the grid
original_values = np.array([[original_expr.evalf(subs={theta: t, phi: p}) for t in theta_values] for p in phi_values])
approx_values = np.array([[approx_expr.evalf(subs={a: best_a, b: best_b, c: best_c, d: best_d, theta: t, phi: p}) for t in theta_values] for p in phi_values])

# Plot the results
fig = plt.figure(figsize=(12, 6))

# Plot the original function
ax1 = fig.add_subplot(121, projection='3d')
ax1.plot_surface(theta_grid, phi_grid, original_values, cmap='viridis')
ax1.set_title('Original Function')
ax1.set_xlabel('Theta')
ax1.set_ylabel('Phi')

# Plot the approximation function
ax2 = fig.add_subplot(122, projection='3d')
ax2.plot_surface(theta_grid, phi_grid, approx_values, cmap='plasma')
ax2.set_title('Approximation Function')
ax2.set_xlabel('Theta')
ax2.set_ylabel('Phi')

plt.show()






import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize
from mpl_toolkits.mplot3d import Axes3D

# Define a grid of theta and phi values
theta = np.linspace(0, np.pi, 100)
phi = np.linspace(0, 2 * np.pi, 100)
theta, phi = np.meshgrid(theta, phi)

# Compute the original function values
r_original = (np.cos(theta)**2 + 1) * (np.cos(phi)**2 + 1)

# Function to calculate r_approx
def r_approx_func(params, theta, phi):
    a, b, c, d = params
    return a * np.sin(theta)**2 + b * np.cos(theta)**2 + c * np.sin(phi)**2 + d * np.cos(phi)**2

# Function to compute the error between the original and approximate functions
def approx_error(params):
    r_approx = r_approx_func(params, theta, phi)
    error = np.mean((r_original - r_approx)**2)
    return error

# Initial guess for a, b, c, d
initial_guess = [1, 1, 1, 1]

# Minimize the error function to find the best-fit parameters
result = minimize(approx_error, initial_guess, method='Nelder-Mead')

# Extract the best-fit parameters
a_best, b_best, c_best, d_best = result.x

# Calculate the minimum error
min_error = result.fun

# Compute the best-fit approximate surface
r_approx_best = r_approx_func(result.x, theta, phi)

# Convert both surfaces to Cartesian coordinates for plotting
def polar_to_cartesian(r, theta, phi):
    x = r * np.sin(theta) * np.cos(phi)
    y = r * np.sin(theta) * np.sin(phi)
    z = r * np.cos(theta)
    return x, y, z

x_original, y_original, z_original = polar_to_cartesian(r_original, theta, phi)
x_approx, y_approx, z_approx = polar_to_cartesian(r_approx_best, theta, phi)

# Plotting the original function and the best-fit approximation
fig = plt.figure(figsize=(10, 5))

# Subplot 1: The original surface plot of the function
ax1 = fig.add_subplot(121, projection='3d')
ax1.plot_surface(x_original, y_original, z_original, cmap='viridis', alpha=0.6)
ax1.set_title("Original Function")
ax1.set_xlim([-3, 3])
ax1.set_ylim([-3, 3])
ax1.set_zlim([-3, 3])
ax1.set_xlabel('X')
ax1.set_ylabel('Y')
ax1.set_zlabel('Z')

# Subplot 2: The best-fit approximation function
ax2 = fig.add_subplot(122, projection='3d')
ax2.plot_surface(x_approx, y_approx, z_approx, color='r', alpha=0.6)
ax2.set_title(f"Best-Fit Approximation\n(a={a_best:.2f}, b={b_best:.2f}, c={c_best:.2f}, d={d_best:.2f})")
ax2.set_xlim([-3, 3])
ax2.set_ylim([-3, 3])
ax2.set_zlim([-3, 3])
ax2.set_xlabel('X')
ax2.set_ylabel('Y')
ax2.set_zlabel('Z')

# Display the plot
plt.show()

# Output the best-fit parameters and the minimum error
print(a_best, b_best, c_best, d_best, min_error)












import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
from mpl_toolkits.mplot3d import Axes3D

# Define the new 2D function
def new_function(theta1, theta2):
    return 3 * np.cos(theta1)**2 + 3 * np.cos(theta2)**2 + 2 + 9 * np.cos(theta1)**2 * np.cos(theta2)**2

# Define a single 2D Gaussian function
def gaussian_2d(theta1, theta2, a, mu1, mu2, sigma):
    return a * np.exp(-((theta1 - mu1)**2 / (2 * sigma**2) + (theta2 - mu2)**2 / (2 * sigma**2)))

# Sum of multiple 2D Gaussians
def sum_of_gaussians_2d(X, *params):
    theta1, theta2 = X
    n_gaussians = len(params) // 4
    result = np.zeros_like(theta1)
    for i in range(n_gaussians):
        a = params[4 * i]
        mu1 = params[4 * i + 1]
        mu2 = params[4 * i + 2]
        sigma = params[4 * i + 3]
        result += gaussian_2d(theta1, theta2, a, mu1, mu2, sigma)
    return result

# Generate data
theta1 = np.linspace(0, 2 * np.pi, 50)
theta2 = np.linspace(0, 2 * np.pi, 50)
theta1, theta2 = np.meshgrid(theta1, theta2)
r = new_function(theta1, theta2)

# Flatten the meshgrid and function values
theta1_flat = theta1.ravel()
theta2_flat = theta2.ravel()
r_flat = r.ravel()

# Initial guess for Gaussian parameters [amplitude, mean1, mean2, std] * number of Gaussians
initial_guess = [
    9, 0, 0, 1, 
    9, np.pi, 0, 1,
    9, 2*np.pi, 0, 1,
    9, 0, np.pi, 1,
    9, np.pi, np.pi, 1,
    9, 2*np.pi, np.pi, 1,
    9, 0, 2*np.pi, 1,
    9, np.pi, 2*np.pi, 1,
    9, 2*np.pi, 2*np.pi, 1
]

# Fit the sum of Gaussians to the new function
params, covariance = curve_fit(sum_of_gaussians_2d, (theta1_flat, theta2_flat), r_flat, p0=initial_guess, maxfev=10000)

# Generate the approximated function
r_approx = sum_of_gaussians_2d((theta1, theta2), *params)

# Plot the original and approximated functions
fig = plt.figure(figsize=(14, 6))

# Original function surface plot
ax1 = fig.add_subplot(121, projection='3d')
ax1.plot_surface(theta1, theta2, new_function(theta1, theta2), cmap='viridis')
ax1.set_title('Original Function')

# Approximated function surface plot
ax2 = fig.add_subplot(122, projection='3d')
ax2.plot_surface(theta1, theta2, r_approx, cmap='viridis')
ax2.set_title('Sum of 2D Gaussians Approximation')

plt.show()

# Print the optimized parameters
params


#### ANOTHER APPROXIMATION : LINEAR REGRESSION
from sklearn.linear_model import LinearRegression
# Define the function f(theta1, theta2, alpha)
def f(theta1, theta2, alpha):
    return 2 + 3 * np.cos(theta1)**2 + 3 * np.cos(theta2)**2 + 2 * np.cos(alpha) * (3 * np.cos(theta1)**2 + 1) * (3 * np.cos(theta2)**2 + 1)

# Generate sample data
n_samples = 100
theta1 = np.random.uniform(0, 2 * np.pi, n_samples)
theta2 = np.random.uniform(0, 2 * np.pi, n_samples)
alpha = np.random.uniform(0, 2 * np.pi, n_samples)
y = f(theta1, theta2, alpha)

# Prepare data for linear regression
X = np.vstack([
    np.ones(n_samples),
    np.cos(theta1),
    np.sin(theta1),
    np.cos(theta2),
    np.sin(theta2),
    np.cos(alpha),
    np.sin(alpha),
    np.cos(theta1)**2,
    np.cos(theta2)**2
]).T

# Perform linear regression
model = LinearRegression()
model.fit(X, y)
coefficients = model.coef_
intercept = model.intercept_

# Output the linear function
print("Linear function approximation:")
print(f"{intercept:.0f} + {coefficients[1]:.0f}*cos(theta1) + {coefficients[2]:.0f}*sin(theta1) + {coefficients[3]:.0f}*cos(theta2) + {coefficients[4]:.0f}*sin(theta2) + {coefficients[5]:.0f}*cos(alpha) + {coefficients[6]:.0f}*sin(alpha) + {coefficients[7]:.0f}*cos^2(theta1) + {coefficients[8]:.0f}*cos^2(theta2)")

# Visualization (optional)
fig = plt.figure(figsize=(14, 6))

# Plot original function values
ax1 = fig.add_subplot(121, projection='3d')
sc = ax1.scatter(theta1, theta2, alpha, c=y, cmap='viridis', label='Original Function')
ax1.set_xlabel('Theta1')
ax1.set_ylabel('Theta2')
ax1.set_zlabel('Alpha')
ax1.set_title('Original Function')
plt.colorbar(sc, ax=ax1, label='Function Value')

# Plot linear approximation
y_approx = model.predict(X)
ax2 = fig.add_subplot(122, projection='3d')
sc2 = ax2.scatter(theta1, theta2, alpha, c=y_approx, cmap='viridis', label='Linear Approximation')
ax2.set_xlabel('Theta1')
ax2.set_ylabel('Theta2')
ax2.set_zlabel('Alpha')
ax2.set_title('Linear Approximation')
plt.colorbar(sc2, ax=ax2, label='Approx Function Value')

plt.show()
