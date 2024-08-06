import numpy as np
from scipy.optimize import minimize
import sympy as sp
import matplotlib.pyplot as plt

# Define the variables and the function
theta = sp.symbols('theta')
a, b = sp.symbols('a b')
n_values = 100

# Original and approximate expressions
original_expr = 1/((3 * sp.cos(theta)**2 + 1)**(1/3))
approx_expr = 1 / a**2 * sp.cos(theta)**2 + 1 / b**2 * sp.sin(theta)**2

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
    print(f"Iteration: a={a_val:.3f}, b={b_val:.3f}: Mean Error={mean_error*100:.3f}")
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