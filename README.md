20 years ago or so when Half-Life 2 stunned the world with its physics (among other things)...

# firebolt

This is a 3D rigid body physics simulator I wrote as a high school graduate (which is an important distinction from a college student – if I were one at that time I might get some guidance on how to do this properly).

The challenge was to write everything from scratch, avoiding any external libraries. I was crazy enough to believe that doing the most difficult thing would land me a job as a game programmer even though I had no degree. This was before indie games and platforms like Unity.

I did manage to get a slow simulation running. Cuboid boxes only (as I didn’t have inertia tensor calculations for arbitrary meshes) and frictionless. The core algorithm was unforgiving and already dated. Ultimately the whole thing derailed on numerical instability. I tried to code a linear programming solver and all the sensitive polygon intersection routines, which in hindsight was pure insanity.

## Setup

I have checked that it compiles under Visual Studio 2022 after installing this ancient [DirectX Software Development Kit](https://www.microsoft.com/en-us/download/details.aspx?id=6812).

Right now this is a bad formatting, magical numbers and poor design galore.
Maybe one day I’ll port it to OpenGL and modern C++.

## References

- [Analytical Methods for Dynamic Simulation of Non-penetrating Rigid Bodies](https://www.cs.cmu.edu/~baraff/papers/sig89.pdf)
- [Fast Contact Force Computation for Nonpenetrating Rigid Bodies](https://www.cs.cmu.edu/~baraff/papers/sig94.pdf)
- [An Introduction to Physically Based Modeling](https://www.cs.cmu.edu/~baraff/pbm/pbm.html)

- [Game Physics 2nd Edition](https://www.amazon.com/Game-Physics-David-H-Eberly/dp/0123749034)
- [Collision Detection in Interactive 3D Environments](https://www.amazon.com/Collision-Detection-Interactive-Environments-Technology/dp/155860801X)
- [Real-Time Collision Detection](https://www.amazon.com/Real-Time-Collision-Detection-Interactive-Technology/dp/1558607323/)
