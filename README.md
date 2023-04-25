# Moment Control
Code for the paper, "Chance Constrained Stochastic Optimal Control for Arbitrarily Disturbed LTI Systems Via the One-Sided Vysochanskij-Petunin Inequality," Under Review, IEEE TAC.

arXiv: [https://arxiv.org/abs/2303.12295](https://arxiv.org/abs/2303.12295)

## Requirements
* CVX [http://cvxr.com/cvx/](http://cvxr.com/cvx/)

## Examples
### Exponential Disturbance
We use a 6d CWH system with additive Exponential noise to model the dynamics of a three satellite rendezvous operation. This example compares the efficacy of the proposed approach with with its predisessor based on Cantelli's inequality.

### Gaussian Disturbance
We use a 6d CWH system with additive Gaussian noise to model the dynamics of a single-satellite rendezvous operation. This example compares the efficiency of the proposed approach with more general chance constrainted approaches based on quantiles and sampling.
