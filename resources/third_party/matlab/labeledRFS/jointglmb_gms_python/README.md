IMPLEMENTATION IS BASED ON THE FOLLOWING PAPERS
========================
*Vo, Ba-Ngu, Ba-Tuong Vo, and Hung Gia Hoang. "An efficient implementation of the generalized labeled multi-Bernoulli filter." IEEE Transactions on Signal Processing 65, no. 8 (2016): 1975-1987.*.

Detail of how to implement Delta-GLMB (with two seperated prediction and update steps) is given in the following paper.

*Vo, Ba-Ngu, Ba-Tuong Vo, and Dinh Phung. "Labeled random finite sets and the Bayes multi-target tracking filter." IEEE Transactions on Signal Processing 62, no. 24 (2014): 6554-6567.* 

USAGE
=====
Install packages: `numpy 1.19.2`, [`murty`](https://github.com/JohnPekl/murty)

Murty algorithm can be replaced by Gibb sampling `gibbswrap_jointpredupdt_custom` in the following [line](https://github.com/JohnPekl/joinglmb/blob/2c770cb88d266748f946be5d149baedd674240a3/run_filter.py#L175).
Murty algorithm was implemented in C++ making it faster than Python version.

Run: `python demo.py`

LICENCE
=======

