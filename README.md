# RRTpathFinder
run like this:

```
 python .\RRTsolver.py .\obst.txt .\finalgoal.txt
 python .\RRTsolverBiDirc.py .\obst.txt .\finalgoal.txt
```
And parameter in the code:

```
para_stepsize = 30 // the parameter step size
para_threshold = 20 // the parameter which decide whether it reached the goal
para_Dirthreshold = 0.5 // the parameter which decide how many percent of tree growth direction to be from start to goal
```
