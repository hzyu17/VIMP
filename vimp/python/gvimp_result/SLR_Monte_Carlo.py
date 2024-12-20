import numpy as np

def f(x):
    vx, vz, phi = x[3], x[4], x[2]
    g = 9.81
    return np.array([
        vx * np.cos(phi) - vz * np.sin(phi),
        vx * np.sin(phi) + vz * np.cos(phi),
        x[5],  # ϕ'
        vz * x[5] - g * np.sin(phi),
        -vx * x[5] - g * np.cos(phi),
        0
    ])

mu = np.array([12.7551,   15.3061,  0.267142, 2.38095, 2.85714, 0.0498666])
sigma = np.diag([0.01, 0.01, 0.01, 0.01, 0.01, 0.01]) / 100

# Monte Carlo
N = 1000000
samples = np.random.multivariate_normal(mu, sigma, N)
f_values = np.array([f(sample) for sample in samples])

expected_value_mc = np.mean(f_values, axis=0)

x_deviation = samples - mu  # x - \bar{x}
y_deviation = f_values - expected_value_mc  # y - \bar{y}
P_xy_mc = (x_deviation.T @ y_deviation) / (N - 1)  # Covariance matrix

expected_value_mc = np.mean(f_values, axis=0)
# np.set_printoptions(suppress=True, precision=8)
hA = P_xy_mc.T @ np.linalg.inv(sigma)

print("Expectation Using Monte-Carlo: \n", expected_value_mc)
print("P_xy:\n", P_xy_mc)

print("hA = ")
for row in hA:
    formatted_row = ' '.join(f"{num:.6f}" for num in row)
    print(formatted_row)

# print("f(\mu):\n", f(mu))
