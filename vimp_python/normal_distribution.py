from numpy.random import normal
import numpy as np
from numpy.random import multivariate_normal

if __name__ == '__main__':
    size_ = 5000
    mu = np.array([0, 0, 0, 0, 1.1, 0, 0, 0])
    covariance = np.eye(8) * 5.0
    samples_singlevariate = normal(0, 1.0, size=(8, size_))

    samples_multivariate = multivariate_normal(mu, covariance, size_)
    print(samples_multivariate.shape)
    mean_calculated = samples_multivariate.sum(axis=0) / size_
    print(mean_calculated)
    cov_calculated = np.zeros(covariance.shape)
    for row in samples_multivariate[:]:
        cov_calculated += row.T * row
    cov_calculated = cov_calculated / (size_-1)
    print(cov_calculated)
