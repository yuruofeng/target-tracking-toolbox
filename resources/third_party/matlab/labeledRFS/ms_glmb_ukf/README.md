IMPLEMENTATION
========================
- This Python implementation is ported from an implemented version of multi-sensor delta-GLMB, The Unscented Kalman Filter for Nonlinear Estimation.
- Sampling solutions (ranked assignments), `gibbs_multisensor_approx_cheap` is implemented in C++ based on __Algorithm 2: MM-Gibbs (Suboptimal)__ [0]. 
- A version of MS-GLMB C++ is also implemented (check out `/cpp_gibbs/src/ms_glmb_ukf`.

[0] Vo, B. N., Vo, B. T., & Beard, M. (2019). Multi-sensor multi-object tracking with the generalized labeled multi-Bernoulli filter. IEEE Transactions on Signal Processing, 67(23), 5952-5967.  

USAGE
=====
- Install packages.
    - `numpy`
    - `scipy`
    - `h5py` (read camera matrix from Matlab matrix)
    - `matplotlib(3.4.1)` and `opencv` draw result
    - C++ packages (`gibbs_multisensor_approx_cheap`, `ms_glmb_ukf`) in cpp_gibbs (`python setup.py build develop`).  
- Run: `python demo.py`.
- Uncomment the following statements to run `python demo.py` with C++ (Note: make sure to comment Python code, line#363-377).

            measZ = []
            for ik in range(model.N_sensors):
                measZ.append(meas.Z[k, ik])
            est = self.MSGLMB.run_msglmb_ukf(measZ, k)
            self.est.X[k], self.est.N[k], self.est.L[k] = est

LICENCE
=======
Linh Ma (`linh.mavan@gm.gist.ac.kr`), Machine Learning & Vision Laboratory, GIST, South Korea

