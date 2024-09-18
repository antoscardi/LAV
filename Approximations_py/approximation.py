import numpy as np
from scipy.optimize import minimize
import sympy as sp
import matplotlib.pyplot as plt


# POWER MEAN APPROXIMATION
x = np.linspace(0, 1000, 1000)
y = np.linspace(0, 1000, 1000)
x, y = np.meshgrid(x, y)

# To avoid complex numbers for negative values when taking fractional powers,
# we take the absolute value first and then restore the sign
exact = x**(2/3) + y**(2/3)

# Power mean approximation: 2 * ((x + y) / 2)^(2/3)
power_mean_approx = 2 * ((x + y) / 2)**(2/3)

# Calculate relative error
relative_error = (exact - power_mean_approx)**2 / (exact**2 + 1e-10)  # Add small constant to avoid division by zero
mean_squared_relative_error = np.mean(relative_error)
print(f"Mean Squared Relative Error: {mean_squared_relative_error:.4f}")

# Plot the exact expression
fig = plt.figure(figsize=(12, 6))

ax1 = fig.add_subplot(121, projection='3d')
surf1 = ax1.plot_surface(x, y, exact, cmap='viridis')
ax1.set_title(r'Exact Expression: $x^{2/3} + y^{2/3}$')
ax1.set_xlabel('x')
ax1.set_ylabel('y')
ax1.set_zlabel('Value')
fig.colorbar(surf1, ax=ax1)

# Plot the power mean approximation
ax2 = fig.add_subplot(122, projection='3d')
surf2 = ax2.plot_surface(x, y, power_mean_approx, cmap='plasma')
ax2.set_title(r'Power Mean Approximation: $2 \left( \frac{x + y}{2} \right)^{2/3}$')
ax2.set_xlabel('x')
ax2.set_ylabel('y')
ax2.set_zlabel('Value')
fig.colorbar(surf2, ax=ax2)

plt.tight_layout()
plt.show()



### RANGE OF VALUES
theta_values = np.linspace(0, 2 * np.pi, 1000)
H_values = np.sqrt(3 * np.cos(theta_values)**2 + 1)

# Plot the graph of H vs theta
plt.figure(figsize=(8, 6))
plt.plot(theta_values, H_values, label=r'$|H| = \sqrt{3 \cos^2(\theta) + 1}$', color='b')
plt.title(r'Plot of $|H| = \sqrt{3 \cos^2(\theta) + 1}$')
plt.xlabel(r'$\theta$ (radians)')
plt.ylabel(r'$|H|$')
plt.grid(True)
plt.legend()
plt.show()



### PAPER APPROXIMATION
# Define the variables and the function
theta = sp.symbols('theta')
a, b = sp.symbols('a b')
n_values = 100

# Original and approximate expressions
original_expr = 1/((3 * sp.cos(theta)**2 + 1)**(1/3))
approx_expr = 1 / a**2 * sp.cos(theta)**2 + 1 / b**2 * sp.sin(theta)**2

# Relative Error
error_expr = sp.Abs((original_expr - approx_expr)/original_expr)

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
    print(f"Iteration: a={a_val:.3f}, b={b_val:.3f}: Mean Relative Error={mean_error*100:.3f}%")
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
plt.plot(theta_values, unit_circle)
plt.legend(loc='upper center', bbox_to_anchor=(0.5, 0.5))
plt.savefig('/home/anto/Desktop/polar_plot.png', bbox_inches='tight')
plt.show()


### MY APPROXIMATION
# Define variables
x, y, a, b, c, alpha = sp.symbols('x y a b c alpha')

# Original and approximate expressions
original_expr = x**2 + y**2 + 2 * sp.cos(alpha) * x * y
approx_expr = (a * x + b * y + c * sp.cos(alpha) * (x + y))**2

# Relative Error
error_expr = sp.Abs((original_expr - approx_expr)/original_expr)

# Number of sample points
n_values = 10

# Pretty print the expressions for verification
print("Original Expression:")
sp.pretty_print(original_expr)
print("\nApproximation Expression:")
sp.pretty_print(approx_expr)

# Function to calculate mean squared error for given parameters a and b
def calculate_error(params):
    a_val, b_val, c_val = params
    error_values = []
    
    # Define range of values for x, y, and alpha
    x_values = np.linspace(1, 2, n_values)
    y_values = np.linspace(1, 2, n_values)
    alpha_values = np.linspace(0, 2 * np.pi, n_values)
    
    # Iterate over combinations of x, y, and alpha
    for x_val in x_values:
        for y_val in y_values:
            for alpha_val in alpha_values:
                error_val = error_expr.evalf(subs={a: a_val, b: b_val,c: c_val, x: x_val, y: y_val, alpha: alpha_val})
                error_values.append(error_val**2)  # Mean squared error
    
    # Return the mean of the squared errors
    mean_error = np.mean(error_values)
    print(f"Iteration: a={a_val:.3f}, b={b_val:.3f}, c={c_val:.3f}: Mean Relative Error={mean_error*100:.3f}%")
    return mean_error

# Initial guess for a and b
initial_guess = [1, 1, 1]
print("Initial Guess:", initial_guess)

# Perform the optimization to minimize the mean squared error
result = minimize(calculate_error, initial_guess, bounds=((0.0001, 2), (0.0001, 2), (0.0001, 1)))
best_a, best_b, best_c = result.x

print("Optimization Result:")
print(f"Best a: {best_a:.3f}")
print(f"Best b: {best_b:.3f}")
print(f"Best c: {best_c:.3f}")

# Plotting the original and approximate functions for a particular set of x, y, and alpha values
x_values = np.linspace(1, 2, 100)
y_values = np.linspace(1, 2, 100)
alpha_values = np.linspace(0, 2 * np.pi, 100)

def evaluate_expr(expr, x_vals, y_vals, alpha_val):
    return np.array([[expr.evalf(subs={x: x_val, y: y_val, alpha: alpha_val}) for x_val in x_vals] for y_val in y_vals], dtype=float)


# 1) 3D SURFACE PLOT
alpha_value = alpha_values[0] # TAKE THE FIRST VALUE OF ALPHA
# Evaluate the original and approximate functions over the grid
original_values = evaluate_expr(original_expr, x_values, y_values, alpha_value)
approx_values = evaluate_expr(approx_expr.subs({a: best_a, b: best_b, c: best_c}), x_values, y_values, alpha_value)


fig = plt.figure(figsize=(12, 10))

# Original function plot
ax1 = fig.add_subplot(121, projection='3d')
x_grid, y_grid = np.meshgrid(x_values, y_values)
ax1.plot_surface(x_grid, y_grid, original_values, cmap='viridis')
ax1.set_xlabel('X')
ax1.set_ylabel('Y')
ax1.set_zlabel(r'$|H_{tot}|$')
ax1.set_title('Original: $x^2 + y^2 + 2 \cos(\\alpha)xy$')

# Approximate function plot
ax2 = fig.add_subplot(122, projection='3d')
ax2.plot_surface(x_grid, y_grid, approx_values, cmap='plasma')
ax2.set_xlabel('X')
ax2.set_ylabel('Y')
ax2.set_zlabel(r'$|H_{tot}| approx$')
ax2.set_title('Approx: $(a * x + b * y + c * \cos(\alpha) * (x + y))^2$')

plt.tight_layout()
plt.show()


# 2) POLAR PLOT
# Define fixed values for x and y
x_val = 1.5
y_val = 1.8

# Function to evaluate an expression given x, y, and a range of alpha values
def evaluate_polar(expr, x_val, y_val, alpha_values, best_a=1, best_b=1, best_c=1):
    return [expr.evalf(subs={x: x_val, y: y_val, alpha: alpha_val, a: best_a, b: best_b, c: best_c}) for alpha_val in alpha_values]

# Evaluate the original and approximate expressions over the range of alpha values
original_values = evaluate_polar(original_expr, x_val, y_val, alpha_values)
approx_values = evaluate_polar(approx_expr, x_val, y_val, alpha_values, best_a=best_a, best_b=best_b, best_c=best_c)

# Create the polar plot
fig, ax = plt.subplots(subplot_kw=dict(projection='polar'), figsize=(8, 6))

# Polar plot for both the original and approximate functions on the same plot
ax.plot(alpha_values, original_values, label="Original", color="blue")
ax.plot(alpha_values, approx_values, label="Approximate", color="red")

# Set the title and labels
ax.set_title('Original vs Approximate Functions')
ax.set_rlabel_position(-22.5)  # Move the radial labels away from the plot

# Add a legend to differentiate the plots
ax.legend(loc="upper right")

# Show the plot
plt.show()
