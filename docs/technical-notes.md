# Implementation and reproduction notes

The project combines Koopman eigenfunction identification with energy-based control of a simulated Duffing oscillator. These notes distinguish the original 2020 material from the later numerical review.

## Code map

| File | Function |
| --- | --- |
| `SystemCreator_RunThisFirst.m` | Simulates 10,000 samples from `[0;-2.8]`, uses known derivatives, and identifies the conserved energy through SVD and least squares. |
| `ControllerDesign.m` | Simulates 5,000 samples from `[0;-2.3]`, estimates derivatives with a fourth-order stencil, identifies an energy coordinate, and compares analytical/data-driven feedback with conventional LQR. |
| `utils/buildTheta.m` | Builds polynomial observables through degree five; the project uses degree four and excludes a constant term. |
| `utils/buildGamma.m` | Builds the corresponding time derivatives using the product rule. |
| `utils/buildThetaGradient.m` | Computes feature gradients through degree four for constructing the input coefficient. |
| `utils/poolDataLIST.m` | Prints monomial labels and coefficients; retains its original Brunton/SINDy attribution. |

The 14 observables used in the project are

`x1, x2, x1^2, x1*x2, x2^2, x1^3, x1^2*x2, x1*x2^2, x2^3, x1^4, x1^3*x2, x1^2*x2^2, x1*x2^3, x2^4`.

`Theta` contains their sampled values and `Gamma` their sampled time derivatives. The smallest right singular vector of `Gamma` identifies a nearly conserved combination. The matrix `pinv(Theta)*Gamma` approximates the continuous-time Koopman generator. Although the report discusses a discrete-time formulation, the supplied code uses derivative data.

## Reproduction status

The identification equations and helper routines were checked in MATLAB R2025b with Control System Toolbox. The complete historical controller was not rerun end to end because the archive is missing two dependencies and contains the issues below. The audit script is independent of the historical scripts' workspace side effects.

| Numerical check | Result |
| --- | --- |
| Number of discovery samples / observables | 10,000 / 14 |
| Exact-derivative invariant coefficient error, after sign alignment | `5.52e-15` |
| Least-squares invariant coefficient error, after sign alignment | `1.66e-14` |
| Estimated-derivative invariant coefficient error, after sign alignment | `3.54e-8` |
| Gradient checks for two variables and degrees 1-4 | Relative errors below `8.1e-11` |
| Directional-derivative checks for degrees 1-5 | Relative errors below `9.6e-11` |

These values concern coefficient recovery from noiseless simulation, not experimental accuracy, noise robustness, or closed-loop performance. Detailed output is in [validation-results.json](validation-results.json).

## Known limitations

1. **Missing helpers.** `ControllerDesign.m` calls `color_line3` at line 33 and `evalCostFun` at line 136; neither was in the supplied archive. The first supports a colored trajectory plot and the second calculates the plotted cost.
2. **Workspace and coefficient coupling.** Line 39 reads `V` from the preceding identification script into `pKRONIC.phi`. The controller later computes `KRONIC.phi` from a new SVD, but its gradient and error calculations use the two vectors inconsistently. They aligned in the audit, but independent SVD sign choices make this fragile.
3. **Zero input effectiveness.** For the analytical energy coordinate, the input coefficient is `x2`. The scalar call `lqr(0,0,5,1)` has no stabilizing Riccati solution. An updated controller needs an explicit treatment of this case.
4. **Eigenfunction scaling.** The learned coordinate is approximately `(4/3)*H`. Applying the same numerical energy weight to both coordinates changes the effective physical penalty. Equivalent energy costs require consistent normalization or rescaled weights.
5. **Target-curve plotting.** The square root defining the separatrix is evaluated outside `abs(x1) <= sqrt(2)`, creating invalid values and green horizontal artifacts in the original phase portrait.
6. **Reversed legend.** In the original state-history figure, dashed curves are conventional LQR and solid curves are Koopman-based control. The legend assigns the names in reverse order.
7. **Cost comparison.** The controllers pursue different objectives. The visible code also uses inconsistent design/evaluation weights and cumulative sample sums rather than explicitly time-integrated costs. The missing cost helper adds uncertainty. The report itself recognizes the objective mismatch.
8. **Library scope.** `buildThetaGradient` stops at degree four, while the value and directional-derivative libraries support degree five. The unused `usesine` input does not enable sinusoidal features. A debugging `keyboard` statement remains in the gradient helper.

The transformed control input coefficient remains state-dependent. The implementation therefore does not establish global nonlinear optimality. The zero-energy separatrix consists of homoclinic trajectories, rather than a regular finite-period orbit. The forced data generated in the controller script are plotted but do not train the active identification branch; no EDMDc/MPC comparison is implemented in this version.

## Report and figure map

The [report](../report/Report.pdf) contains the theory, model, identification approach, and original results. Its scientific content is preserved. The public copy removes the title-page student number.

| Original export | Preview | Report figure |
| --- | --- | --- |
| `figures/source/TTTT.eps` | [Phase portrait](../figures/phase-portrait.png) | Figure 2 |
| `figures/source/lqq.eps` | [State histories](../figures/state-responses.png) | Figure 4 |
| `figures/source/sss.eps` | Same state-history plot; pixel-identical to `lqq.eps` when rendered | Figure 4 |
| `figures/source/jj.eps` | [Cumulative cost](../figures/cost-comparison.png) | Figure 5 |

The report's unforced phase-portrait family and actuation plot are present in the PDF but were not supplied as separate corresponding EPS files. The historical figure labels and artifacts remain visible in the previews so that they can be interpreted alongside the original report.
