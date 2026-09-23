# References and attribution

This repository presents Houman Asgari's 2020 Nonlinear Control course project at Sharif University of Technology, taught by Dr. Maryam Babazadeh. The report uses the spelling “Hooman Asgari.”

The project is based on a Brunton textbook example, as identified by the project author, and adapts published Koopman-control methods and MATLAB code. The contribution is the course-project implementation, investigation, and analysis of these methods.

## KRONIC

**Authors:** Eurika Kaiser, J. Nathan Kutz, and Steven L. Brunton.

- [Data-driven discovery of Koopman eigenfunctions for control](https://arxiv.org/abs/1707.01146).
- [KRONIC source repository](https://github.com/eurika-kaiser/KRONIC).
- [Upstream license](https://github.com/eurika-kaiser/KRONIC/blob/master/LICENSE), included locally as [licenses/KRONIC-LICENSE.txt](licenses/KRONIC-LICENSE.txt).

The identification script follows the discovery section of upstream `DiscoverDuffing.m`. The controller adapts substantial portions of `DiscoverDuffing_KRONICvsEDMDc.m`; the course-project version omits the upstream EDMDc/MPC branches and includes a conventional LQR comparison. The exact historical upstream revision is unknown.

The upstream license text is reproduced as published, including its abbreviated arXiv identifier. The paper link above uses the correct identifier, `1707.01146`.

## SINDy coefficient-display helper

`utils/poolDataLIST.m` retains its original 2015 copyright notice and credit to Steven L. Brunton, and names the paper *Discovering governing equations from data: Sparse identification of nonlinear dynamical systems* by Brunton, Proctor, and Kutz.

## Report and graphics

The six-page report is the original coursework report with the student number removed for public distribution. The four EPS files are unchanged original exports, including their graphics-procedure notices. The PNG previews are rendered from these EPS files and retain the historical plots and labels.

The README, technical notes, and numerical audit are later repository documentation and verification work. The included KRONIC license covers the upstream code under its terms; this repository does not assign a blanket license to the author's separate report and other material.
