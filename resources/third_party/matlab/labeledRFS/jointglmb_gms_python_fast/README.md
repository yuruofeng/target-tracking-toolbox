IMPLEMENTATION IS BASED ON THE FOLLOWING PAPERS
========================
This Python implementation is ported from an implemented version of delta-GLMB in Matlab in [1] (it was done by Prof. Vo's research group).
In [1] the authors combine _prediction and update_ in a single step. GLMB is originally, theoretically proposed in [0].
* Original Matlab source: `jointglmb_gms_matlab`
* Original Python porting: `jointglmb_gms_python`
* Improved version (code optimized, adaptive birth): root folder

A detail of how to implement Delta-GLMB (with two separated prediction and update steps) is given [2]. 

Adaptive birth is implemented based on [3] (Section __Adaptive Birth Distribution__), mainly focused on equation (75).

* [0] Vo, Ba-Tuong, and Ba-Ngu Vo. "Labeled random finite sets and multi-object conjugate priors." IEEE Transactions on Signal Processing 61, no. 13 (2013): 3460-3475.
* [1] Vo, Ba-Ngu, Ba-Tuong Vo, and Hung Gia Hoang. "An efficient implementation of the generalized labeled multi-Bernoulli filter." IEEE Transactions on Signal Processing 65, no. 8 (2016): 1975-1987.*.
* [2] Vo, Ba-Ngu, Ba-Tuong Vo, and Dinh Phung. "Labeled random finite sets and the Bayes multi-target tracking filter." IEEE Transactions on Signal Processing 62, no. 24 (2014): 6554-6567.*
* [3] Reuter, Stephan, Ba-Tuong Vo, Ba-Ngu Vo, and Klaus Dietmayer. "The labeled multi-Bernoulli filter." IEEE Transactions on Signal Processing 62, no. 12 (2014): 3246-3260. 

USAGE
=====
Install packages: `numpy 1.19.2`, [`murty`](https://github.com/JohnPekl/murty)

Murty algorithm can be replaced by Gibb sampling `gibbswrap_jointpredupdt_custom` in the following [line](https://github.com/JohnPekl/joinglmb/blob/2c770cb88d266748f946be5d149baedd674240a3/run_filter.py#L175).
Murty algorithm was implemented in C++ making it faster than Python version.

Run: `python demo.py`

LICENCE
=======

