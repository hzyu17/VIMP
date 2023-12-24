## A header-only implementation of Gaussian variational inference (GVI) algorithm that uses a natural gradient method and leverages the factorized structure of the problem.

The GVI algorithm aims to optimize a Gaussian distribution, $q$ parameterized by $\mathcal{N}(\mu_\theta, \Sigma_\theta)$, to approximate a given posterior distribution, $p(X|Z)$, by minimizing the KL divergence between the 2 distributions:

$$q^{\star} = \underset{q \in \mathcal{Q}}{\arg\min}\; {\rm KL} [q(X) || p(X|Z)]$$

If we denote the objective function $J(q) \triangleq {\rm KL} [q(X) || p(X|Z)] $, and define the negative-log-probability of the posterior distribution as $\psi(X) = -\log p(X|Z)$, then the update law of a natural gradient paradigm has closed-from [[1]](#1) as expectations of $\psi$ with respect to the proposal Gaussian. For generic distributions $\psi$, the expectations can be estimated using nonlinear quadratures; For Gaussian like posteriors, $p(X|Z) = \exp \left( \| \Lambda X - \Psi \mu \|_{K^{-1}}^2 \right)$, the expectations have closed-form, which is much more efficient.

## Example applications
Nonlinear state estimation [[2]](#2).

[Stochastic Motion Planning](https://github.com/hzyu17/VIMP) [[3]](#3).

## References
<a id="1">[1]</a> 
Opper, M. and Archambeau, C., 2009. The variational Gaussian approximation revisited. Neural computation, 21(3), pp.786-792.

<a id="2">[2]</a> 
Barfoot, T.D., Forbes, J.R. and Yoon, D.J., 2020. Exactly sparse Gaussian variational inference with application to derivative-free batch nonlinear state estimation. The International Journal of Robotics Research, 39(13), pp.1473-1502.

<a id="3">[3]</a> 
Yu, H., & Chen, Y. (2023). Stochastic Motion Planning as Gaussian Variational Inference: Theory and Algorithms. arXiv preprint arXiv:2308.14985.
