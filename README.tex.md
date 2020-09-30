# Geometry Processing – Registration

> **To get started:** Clone this repository then issue
> 
>     git clone --recursive http://github.com/[username]/geometry-processing-registration.git
>

## Installation, Layout, and Compilation

See
[introduction](http://github.com/alecjacobson/geometry-processing-introduction).

## Execution

Once built, you can execute the assignment from inside the `build/` using 

    ./registration [path to mesh1.obj] [path to mesh2.obj]

## Background

In this assignment, we will be implementing a version of the [iterative closest
point (ICP)](https://en.wikipedia.org/wiki/Iterative_closest_point), not to be
confused with [Insane Clown Posse](https://en.wikipedia.org/wiki/Insane_Clown_Posse).

Rather than [registering multiple point
clouds](https://en.wikipedia.org/wiki/Point_set_registration), we will register
multiple triangle mesh surfaces. 

This _algorithm_ and its many [variants](papers/Effcient_Variants_of_ICP.pdf) has been used for quite some time to
align discrete shapes. One of the first descriptions is given in ["A Method for
Registration of 3-D Shapes" by Besl & McKay 1992](papers/method-for-registration-3d-shapes.pdf). However, the award-winning
PhD thesis of Sofien Bouaziz [("Realtime Face Tracking and Animation" 2015,
section 3.2-3.3)](https://lgg.epfl.ch/publications/2015/Sofien_Thesis/thesis.pdf) contains a more modern view that unifies many of the variants
with respect to how they impact the same core optimization problem. 

For our assignment, we will assume that we have a triangle mesh representing a
complete scan of the surface $Y$ of some [rigid
object](https://en.wikipedia.org/wiki/Rigid_body) and a new partial scan of
that surface $X$.

![Example input: a partial scan mesh surface $X$ is misaligned with the 
mesh of the complete surface $Y$](images/max-inputs.jpg)

These meshes will not have the same number of vertices or the even the same
topology. We will first explore different ways to _measure_ how well aligned
two surfaces are and then how to optimize the _rigid_ alignment of the partial
surface $X$ to the complete surface $Y$.

## Hausdorff distance

We would like to compute a single scalar number that measures how poorly two
surfaces are matched. In other words, we would like to measure the _distance_
between two surfaces. Let's start by reviewing more familiar distances:

#### Point-to-point distance
The usually Euclidean distance between _two points_ $\mathbf{x}$ and $\mathbf{y}$ is the $L^2 $
norm of their difference :

$$
d(\mathbf{x},\mathbf{y}) = \| \mathbf{x} - \mathbf{y}\| .
$$

#### Point-to-projection distance

When we consider the distance between a point $\mathbf{x}$ and some _larger_ object $Y$ (a line,
a circle, a surface), the natural extension is to take the distance to the
closest point $\mathbf{y}$ on $Y$:

$$
d(\mathbf{x},Y) = \inf_{\mathbf{y} \in  Y} d(\mathbf{x},\mathbf{y}).
$$

written in this way the
[infimum](https://en.wikipedia.org/wiki/Infimum_and_supremum) considers all
possible points $\mathbf{y}$ and keeps the minimum distance. We may equivalently write
this distance instead as simply the point-to-point distance between $\mathbf{x}$ and
the _closest-point projection_ $P_Y(\mathbf{x})$:

$$
d(\mathbf{x},Y) = d((\mathbf{x},P_Y(\mathbf{x})) = \| \mathbf{x} - P_Y(\mathbf{x})\| .
$$


If $Y$ is a smooth surface, this projection will also be an [orthogonal
projection](https://en.wikipedia.org/wiki/Projection_(linear_algebra)#Orthogonal_projections).


![The distance between a surface $Y$ (light blue) and a point $\mathbf{x}$ (orange) is
determined by the closest point $P_Y(\mathbf{x})$ (blue)](images/max-point-mesh.gif)

### Directed Hausdorff Distance

We might be tempted to define the distance from surface $X$ to $Y$ as the
_infimum_ of _point-to-projection_ distances over all points $\mathbf{x}$ on $X$:

$$
D_\text{inf}(X,Y) = \inf_{\mathbf{x} \in  X} \| \mathbf{x} - P_Y(\mathbf{x})\| ,
$$


but this will not be useful for registering two surfaces: it will measure zero
if even just a single point of $\mathbf{x}$ happens to lie on $Y$. Imagine the noses of
two faces touching at their tips.

Instead, we should take the _supremum_ of _point-to-projection_ distances over
all points $\mathbf{x}$ on $X$:

$$
D_{\overrightarrow{H}}(X,Y) = \sup_{\mathbf{x} \in  X} \| \mathbf{x} - P_Y(\mathbf{x})\| .
$$

This surface-to-surface distance measure is called the _directed_ [Hausdorff
distance](https://en.wikipedia.org/wiki/Hausdorff_distance). We may interpret
this as taking the worst of the best: we 
let each point $\mathbf{x}$ on $X$ declare its shortest distance to $Y$ and then keep
the longest of those.

![The directed Hausdorff distance between from surface $X$ (light orange) to
another surface $Y$ (light blue) is determined by the point on $X$ (orange)
whose closest point on $Y$ (blue) is the farthest
away.](images/max-point-mesh-farthest.jpg)

It is easy to verify that $D_{\overrightarrow{H}}$ will only equal zero if all
points on $X$ also lie exactly on $Y$. 

The converse is not true: if $D_{\overrightarrow{H}}=0$ there may still be
points on $Y$ that do not lie on $X$. In other words, _in general_ the directed
Hausdorff distance from surface $X$ to surface $Y$ will not equal the Hausdorff
distance from surface $Y$ to surface $X$:

$$
D_{\overrightarrow{H}}(X,Y) \ne  D_{\overrightarrow{H}}(Y,X).
$$


#### directed Hausdorff distance between triangle meshes

We can approximate a _lower bound_ on the Hausdorff distance between two meshes
by densely sampling surfaces $X$ and $Y$. We will discuss sampling methods,
later. For now consider that we have chosen a set $\mathbf{P}_X$ of $k$ points on $X$
(each point might lie at a vertex, along an edge, or inside a triangle). The
directed Hausdorff distance from $X$ to another triangle mesh $Y$ must be
_greater_ than the directed Hausdorff distance from this [point
cloud](https://en.wikipedia.org/wiki/Point_cloud) $\mathbf{P}_X$ to $Y$:

$$
D_{\overrightarrow{H}}(X,Y) \ge  
D_{\overrightarrow{H}}(\mathbf{P}_X,Y) = \max_{i=1}^k \| \mathbf{p}_i - P_Y(\mathbf{p}_i)\| ,
$$


where we should be careful to ensure that the projection $P_Y(\mathbf{p}_i)$ of the
point $\mathbf{p}_i$ onto the triangle mesh $Y$ might lie at a vertex, along an edge or
inside a triangle. 

As our sampling $\mathbf{P}_X$ becomes denser and denser on $X$ this lower bound will
approach the true directed Hausdorff distance. Unfortunately, an efficient
_upper bound_ is significantly more difficult to design.

#### Hausdorff distance for alignment optimization

Even if it _were_ cheap to compute, Hausdorff distance is difficult to
_optimize_ when aligning two surfaces. If we treat the Hausdorff distance
between surfaces $X$ and $Y$ as an energy to be minimized, then only change to
the surfaces that will decrease the energy will be moving the (in general)
isolated point on $X$ and isolated point on $Y$ generating the maximum-minimum
distance. In effect, the rest of the surface does not even matter or effect the
Hausdorff distance. This, or any type of $L^\infty $ norm, will be much more
difficult to optimize.

Hausdorff distance can serve as a validation measure, while we turn to $L^2 $
norms for optimization.

## Integrated closest-point distance

We would like a distance measure between two surfaces that---like Hausdorff
distance---does not require a shared parameterization. Unlike Hausdorff
distance, we would like this distance to _diffuse_ the measurement over the
entire surfaces rather than generate it from the sole _worst offender_. We can
accomplish this by replacing the _supremum_ in the Hausdorff distance ($L^\infty $)
with a integral of squared distances ($L^2 $). Let us first define a directed
_closest-point distance_ from  a surface $X$ to another surface $Y$, as the
integral of the squared distance from every point $\mathbf{x}$ on $X$ to its
closest-point projection $P_Y(\mathbf{x})$ on the surfaces $Y$:

$$
D_{\overrightarrow{C}}(X,Y) = \sqrt{\ \int \limits_{\mathbf{x}\in X} \| \mathbf{x} - P_Y(\mathbf{x}) \| ^2  \;dA }.
$$


This distance will only be zero if all points on $X$ also lie on $Y$, but when
it is non-zero it is summing/averaging/diffusing the distance measures of all
of the points.

This distance is suitable to define a matching energy, but is not necessarily
welcoming for optimization: the function inside the square is non-linear. Let's
dig into it a bit. We'll define a directed _matching energy_
$E_{\overrightarrow{C}}(Z,Y)$ from $Z$ to $Y$ to be the squared directed
closest point distance from $X$ to $Y$:

$$
E_{\overrightarrow{C}}(Z,Y) = \int \limits_{\mathbf{z}\in Z} \| \mathbf{z} - P_Y(\mathbf{z}) \| ^2  \;dA =
\int \limits_{\mathbf{z}\in Z} \| f_Y(\mathbf{z}) \| ^2  \;dA
$$


where we introduce the proximity function $\mathbf{f}_Y:\mathbf{R}^3 \Rightarrow \mathbf{R}^3 $ defined simply as the
vector from a point $\mathbf{z}$ to its closest-point projection onto $Y$:

$$
\mathbf{f}(\mathbf{z}) = \mathbf{z} - P_Y(\mathbf{z}).
$$


Suppose $Y$ was not a surface, but just a single point $Y = \{\mathbf{y}\}$. In this
case, $\mathbf{f}(\mathbf{z}) = \mathbf{z} - \mathbf{y}$ is clearly linear in $\mathbf{z}$.

Similarly, suppose $Y$ was an [infinite
plane](https://en.wikipedia.org/wiki/Plane_(geometry)) $Y = \{\mathbf{y} | (\mathbf{y}-\mathbf{p})\cdot \mathbf{n} =
0\}$ defined by some point $\mathbf{p}$ on the plane and the plane's unit normal vector
$\mathbf{n}$. Then $\mathbf{f}(\mathbf{z}) = ((\mathbf{z}-\mathbf{p})\cdot \mathbf{n})\mathbf{n})$ is also linear in $\mathbf{z}$.

But in general, if $Y$ is an interesting surface $\mathbf{f}(\mathbf{z})$ will be non-linear; it
might not even be a continuous function.

![](images/closest-point-discontinuous.png)

In optimization, a common successful strategy to minimize energies composed of
squaring a non-linear functions $\mathbf{f}$ is to
[linearize](https://en.wikipedia.org/wiki/Linearization) the function about a
current input value (i.e., a current guess $\mathbf{z}₀$), minimize the energy built
from this linearization, then re-linearize around that solution, and then
repeat. 

This is the core idea behind [gradient
descent](https://en.wikipedia.org/wiki/Gradient_descent) and the
[Gauss-Newton](https://en.wikipedia.org/wiki/Gauss–Newton_algorithm) methods:

```
minimize f(z)^2 
  z₀ \Leftarrow  initial guess
  repeat until convergence
    f₀ \Leftarrow  linearize f(z) around z₀
    z₀ \Leftarrow  minimize f₀(z)^2 
```

Since our $\mathbf{f}$ is a geometric function, we can derive its linearizations
_geometrically_.

### Constant function approximation

If we make the convenient---however unrealistic---assumption that in the
neighborhood of the closest-point projection $P_Y(\mathbf{z}₀)$ of the current guess
$\mathbf{z}₀$ the surface $Y$ is simply the point $P_Y(\mathbf{z}₀)$ (perhaps imagine that $Y$
is makes a sharp needle-like point at $P_Y(\mathbf{z}₀)$ or that $Y$ is very far away
from $\mathbf{x}$), then we can approximate $\mathbf{f}(\mathbf{z})$ in the proximity of our current
guess $\mathbf{z}₀$ as the vector between the input point $\mathbf{z}$ and $P_Y(\mathbf{z}₀)$:

$$
\mathbf{f}(\mathbf{z}) \approx \mathbf{f}_\text{point}(\mathbf{z}) = \mathbf{z}-P_Y(\mathbf{z}₀)
$$


In effect, we are assuming that the surface $Y$ is _constant_ function of its
parameterization: $\mathbf{y}(u,v) = P_Y(\mathbf{z}₀)$.

Minimizing $E_{\overrightarrow{C}}$ iteratively using this linearization (or
rather _constantization_) of $\mathbf{f}$ is equivalent to the [gradient
descent](https://en.wikipedia.org/wiki/Gradient_descent). We have simply
derived our gradients geometrically.

### Linear function approximation

If we make make a slightly more appropriate assumption that in the neighborhood
of the  $P_Y(\mathbf{z}₀)$ the surface $Y$ is a plane, then we can improve this
approximation while keeping $\mathbf{f}$ linear in $\mathbf{z}$:

$$
\mathbf{f}(\mathbf{z}) \approx \mathbf{f}_\text{plane}(\mathbf{z}) = ((\mathbf{z}-P_Y(\mathbf{z}₀))\cdot \mathbf{n}) \mathbf{n}.
$$


where the plane that _best_ approximates $Y$ locally near $P_Y(\mathbf{z}₀)$ is the
[tangent plane](https://en.wikipedia.org/wiki/Tangent_space) defined by the
[normal vector](https://en.wikipedia.org/wiki/Normal_(geometry)) $\mathbf{n}$ at
$P_Y(\mathbf{z}₀)$.


Minimizing $E_{\overrightarrow{C}}$ iteratively using this linearization of
$\mathbf{f}$ is equivalent to the
[Gauss-Newton](https://en.wikipedia.org/wiki/Gauss–Newton_algorithm) method. We
have simply derived our linear approximation geometrically.

Equipped with these linearizations, we may now describe an [optimization
algorithm](https://en.wikipedia.org/wiki/Mathematical_optimization#Optimization_algorithms)
for minimizing the matching energy between a surface $Z$ and another surface
$Y$.

## Iterative closest point algorithm

So far we have derived distances between a surface $Z$ and another surface $Y$.
In our _rigid_ alignment and registration problem, we would like to
[transform](https://en.wikipedia.org/wiki/Transformation_(function)) one
surface $X$ into a new surface $T(X) = Z$ so that it best aligns with/matches
the other surface $Y$. Further we require that $T$ is a rigid transformation:
$T(\mathbf{x}) = \mathbf{R} \mathbf{x} + \mathbf{t}$ for some rotation matrix $\mathbf{R} \in  SO(3) ⊂ \mathbf{R}^{3\times 3}$
(i.e., an [orthogonal matrix with determinant
1](https://en.wikipedia.org/wiki/Rotation_group_SO(3))) and translation vector
$\mathbf{t}\in \mathbf{R}^3 $.

Our matching problem can be written as an optimization problem to find the best
possible rotation $\mathbf{R}$ and translation $\mathbf{t}$ that match surface $X$ to surface
$Y$:

$$
\mathop{\text{minimize}}_{\mathbf{t}\in \mathbf{R}^3 ,\ \mathbf{R} \in  SO(3)} 
  \int \limits_{\mathbf{x}\in X} \| \mathbf{R} \mathbf{x} + \mathbf{t} - P_Y(T(\mathbf{x})) \| ^2  \;dA
$$


Even if $X$ is a triangle mesh, it is difficult to _integrate_ over _all_
points on the surface of $X$. _At any point_, we can approximate this energy by
_summing_ over a point-sampling of $X$:

$$
\mathop{\text{minimize}}_{\mathbf{t}\in \mathbf{R}^3 ,\ \mathbf{R} \in  SO(3)} 
  \Sigma _{i=1}^k \| \mathbf{R} \mathbf{x}_i + \mathbf{t} - P_Y(T(\mathbf{x}_i)) \| ^2 ,
$$


where $\mathbf{X} \in  \mathbf{R}^{k\times 3}$ is a set of $k$ points on $X$ so that each point $\mathbf{x}_i$
might lie at a vertex, along an edge, or inside a triangle. We defer discussion
of _how_ to sample a triangle mesh surface.

### Pseudocode

As the name implies, the method proceeds by iteratively finding the closest
point on $Y$ to the current rigid transformation $\mathbf{R} \mathbf{x} + \mathbf{t}$ of each sample
point $\mathbf{x}$ in $\mathbf{X}$ and then minimizing the _linearized_ energy to update the
rotation $\mathbf{R}$ and translation $\mathbf{t}$. 

If $V_X$ and $F_X$ are the vertices and faces of a triangle mesh surface $X$
(and correspondingly for $Y$), then we can summarize a generic ICP algorithm in
pseudocode:

```
icp V_X, F_X, V_Y, F_Y
  R,t \Leftarrow  initialize (e.g., set to identity transformation)
  repeat until convergence
    X \Leftarrow  sample source mesh (V_X,F_X)
    P0 \Leftarrow  project all X onto target mesh (V_Y,F_Y)
    R,t \Leftarrow  update rigid transform to best match X and P0
    V_X \Leftarrow  rigidly transform original source mesh by R and t
```

### Updating the rigid transformation

We would like to find the rotation matrix $\mathbf{R} \in  SO(3) ⊂ \mathbf{R}^{3\times 3}$ and
translation vector $\mathbf{t}\in \mathbf{R}^3 $ that _best_ aligns a given a set of points $\mathbf{X} \in 
\mathbf{R}^{k\times 3}$ on the source mesh and their current closest points $\mathbf{P} \in  \mathbf{P}^{k\times 3}$
on the target mesh. We have two choices for _linearizing_ our matching energy:
point-to-point (gradient descent) and point-to-plane (Gauss-Newton).

![ICP using the point-to-point matching energy linearization is slow to
converge.](images/max-point-to-point.gif)

![ICP using the point-to-plane matching energy linearization is
faster.](images/max-point-to-plane.gif)

In either case, this is still a non-linear optimization problem. This time due
to the [constraints](https://en.wikipedia.org/wiki/Constrained_optimization)
rather than the energy term. 

We require that $\mathbf{R}$ is not just any 3\times 3 matrix, but a rotation matrix. We
can _linearize_ this constraint, by assuming that the rotation in $\mathbf{R}$ will
be very small and thus well approximated by the identity matrix $\mathbf{I}$ plus a
skew-symmetric matrix:

$$
\mathbf{R} \approx \mathbf{I} + 
  \left(\begin{array}{ccc}
   0 & -γ &  β \\
   γ &  0 & -α \\
  -β &  α &  0 \\
  \end{array}\right)
$$


where we can now work directly with the three scalar unknowns $α$, $β$ and $γ$.

### Approximate point-to-point minimizer

If we apply our linearization of $\mathbf{R}$ to the **point-to-point** distance
linearization of the matching energy, our minimization becomes:

$$
\mathop{\text{minimize}}_{\mathbf{t}\in \mathbf{R}^3 , α, β, γ} 
  \Sigma _{i=1}^k \left\|
  \left(\begin{array}{ccc}
   0 & -γ &  β \\
   γ &  0 & -α \\
  -β &  α &  0 \\
  \end{array}\right)
  \mathbf{x}_i + \mathbf{t} - \mathbf{p}_i \right\|^2.
$$


This energy is quadratic in the translation vector $\mathbf{t}$ and the linearized
rotation angles $α$, $β$ and $γ$. Let's gather these degrees of freedom into a
vector of unknowns: $\mathbf{u} = [α β γ \mathbf{t}^{\mathsf T}] \in  \mathbf{R}⁶$. Then we can write our
problem in summation form as:

$$
\mathop{\text{minimize}}_{\mathbf{u}\in \mathbf{R}⁶}
  \Sigma _{i=1}^k \left\| 
  \left(\begin{array}{cccccc}
         0 &  x_{i,3} & -x_{i,2} & 1 & 0 & 0 \\
  -x_{i,3} &        0 &  x_{i,1} & 0 & 1 & 0 \\
   x_{i,2} & -x_{i,1} &        0 & 0 & 0 & 1
  \end{array}\right) \mathbf{u} +
  \mathbf{x}_i - \mathbf{p}_i \right\|^2.
$$


This can be written compactly in matrix form as:

$$
\mathop{\text{minimize}}_{\mathbf{u}\in \mathbf{R}⁶}
  \left\|
  \underbrace{
  \left(\begin{array}{cccccc}
      0 &  \mathbf{X}_3 & -\mathbf{X}_2 & \mathbf{1} & 0    & 0 \\
  -\mathbf{X}_3 &     0 &  \mathbf{X}_1 & 0    & \mathbf{1} & 0 \\
   \mathbf{X}_2 & -\mathbf{X}_1 &     0 & 0    & 0    & \mathbf{1}
  \end{array}\right)
  }_{\mathbf{A}}
  \mathbf{u} +
\left[\begin{array}{c}
  \mathbf{X}_1-\mathbf{P}_1 \\
  \mathbf{X}_2-\mathbf{P}_2 \\
  \mathbf{X}_3-\mathbf{P}_3
\end{array}\right]
  \right\|_F^2,
$$

where we introduce the matrix $\mathbf{A} \in  \mathbf{R}^{3k \times  6}$ that gathers the columns
$\mathbf{X}_i$ of $\mathbf{X}$ and columns of ones $\mathbf{1} \in  \mathbf{R}^k$.

This quadratic energy is minimized with its partial derivatives with respect to
entries in $\mathbf{u}$ are all zero:

$$
\begin{align}
\mathbf{A}^{\mathsf T} \mathbf{A} \mathbf{u} & = -\mathbf{A}^{\mathsf T} 
\left[\begin{array}{c}
  \mathbf{X}_1-\mathbf{P}_1 \\
  \mathbf{X}_2-\mathbf{P}_2 \\
  \mathbf{X}_3-\mathbf{P}_3
\end{array}\right]
, \\
\mathbf{u} & = \left(\mathbf{A}^{\mathsf T} \mathbf{A}\right)^{-1} \left(-\mathbf{A}^\transpose
\left[\begin{array}{c}
  \mathbf{X}_1-\mathbf{P}_1 \\
  \mathbf{X}_2-\mathbf{P}_2 \\
  \mathbf{X}_3-\mathbf{P}_3
\end{array}\right]
\right),
\end{align}
$$


Solving this small 6\times 6 system gives us our translation vector $\mathbf{t}$ and the
linearized rotation angles $α$, $β$ and $γ$. If we simply assign 

$$
\mathbf{R} \Leftarrow   \mathbf{M} := \mathbf{I} + 
  \left(\begin{array}{ccc}
   0 & -γ &  β \\
   γ &  0 & -α \\
  -β &  α &  0 \\
  \end{array}\right)
$$


then our transformation will _not_ be rigid. Instead, we should project $\mathbf{M}$
onto the space of rotation matrices.

#### Recovering a pure rotation from its linearization

> In an effort to provide an alternative from "Least-Squares Rigid Motion Using
> SVD" [Sorkine 2009], this derivation purposefully _avoids_ the [trace
> operator](https://en.wikipedia.org/wiki/Trace_(linear_algebra)) and its
> various nice properties.

If $α$, $β$ and $γ$ are all small, then it may be safe to _interpret_ these
values as rotation angles about the $x$, $y$, and $z$ axes respectively.

In general, it is better to find the closest rotation matrix to $\mathbf{M}$. In other
words, we'd like to solve the small optimization problem:

$$
\mathbf{R}^* = \mathop{\text{argmin}}_{\mathbf{R} \in  SO(3)} \left\| \mathbf{R} - \mathbf{M} \right\|_F^2,
$$

where $\|\mathbf{X}\|_F^2$ computes the squared [Frobenius
norm](https://en.wikipedia.org/wiki/Matrix_norm#Frobenius_norm) of the matrix
$\mathbf{X}$ (i.e., the sum of all squared element values. In MATLAB syntax:
`sum(sum(A.^2))`). We can expand the norm by taking advantage of the [associativity
property](https://en.wikipedia.org/wiki/Associative_property) of the Frobenius
norm:
$$
\mathbf{R}^* = \mathop{\text{argmin}}_{\mathbf{R} \in  SO(3)} \left\| \mathbf{M} \right\|_F^2 + \left\| \Rot
\right\|_F^2 - 2 \left<\mathbf{R}, \mathbf{M} \right>_F,
$$

where $\left<\mathbf{A}, \mathbf{B} \right>_F$ is the
[Frobenius inner
product](https://en.wikipedia.org/wiki/Frobenius_inner_product) of  $\mathbf{A}$ and
$\mathbf{B}$ (i.e., the sum of all per-element products. In MATLAB syntax:
`sum(sum(A.*B))`). We can drop the Frobenius norm
of $\mathbf{M}$ term ($\left\| \mathbf{M}
\right\|_F^2$) because it is constant with respect to the unknown rotation
matrix $\mathbf{R}$. We can also drop the Frobenius norm of $\mathbf{R}$ term because it
must equal one ($\left\|
\mathbf{R}\right\|_F^2 = 1$) since $\mathbf{R}$ is required to be a orthonormal matrix
($\mathbf{R} \in  SO(3)$). We can drop the factor of $2$ and flip the minus sign to
change our _minimization_ problem into a _maximization_ problem:
$$
\mathbf{R}^* = \mathop{\text{argmax}}_{\mathbf{R} \in  SO(3)} \left<\mathbf{R}, \mathbf{M} \right>_F
$$


We now take advantage of the [singular value
decomposition](https://en.wikipedia.org/wiki/Singular_value_decomposition) of
$\mathbf{M} = \mathbf{U} \sigma  \mathbf{V}^{\mathsf T}$, where $\mathbf{U}, \mathbf{V} \in  \mathbf{R}^{3\times 3}$ are orthonormal matrices
and $\sigma \in \mathbf{R}^{3\times 3}$ is a non-negative diagonal matrix:

$$
\mathbf{R}^* = \mathop{\text{argmax}}_{\mathbf{R} \in  SO(3)} \left<\mathbf{R},\mathbf{U} \sigma  \mathbf{V}^{\mathsf T} \right>_F.
$$


The Frobenius inner product will let us move the products by $\mathbf{V}$ and $\mathbf{U}$ from
the right argument to the left argument:

> Recall some linear algebra properties:
> 
>  1. Matrix multiplication (on the left) can be understood as _acting_ on each
>    column: $\mathbf{A} \mathbf{B} = \mathbf{A} [\mathbf{B}_1 \  \mathbf{B}_2 \ \ldots  \ \mathbf{B}_n] = [\mathbf{A} \mathbf{B}_1 \  \mathbf{A} \mathbf{B}_2 \  \ldots  \
>    \mathbf{A} \mathbf{B}_n]$,
>  4. The [Kronecker product](https://en.wikipedia.org/wiki/Kronecker_product)
>    $\mathbf{I} ꕕ \mathbf{A}$ of the identity matrix $\mathbf{I}$ of size $k$ and a matrix $\mathbf{A}$ simply
>    repeats $\mathbf{A}$ along the diagonal k times. In MATLAB, `repdiag(A,k)`,
>  3. Properties 1. and 2. imply that the vectorization of a matrix product
>    $\mathbf{B}\mathbf{C}$ can be written as the Kronecker product of the #-columns-in-$\mathbf{C}$
>    identity matrix and $\mathbf{B}$ times the vectorization of $\mathbf{C}$:
>    $\text{vec}(\mathbf{B}\mathbf{C}) = (\mathbf{I} ꕕ \mathbf{B})\text{vec}(\mathbf{C})$,
>  4. The transpose of a Kronecker product is the Kronecker product of
>    transposes: $(\mathbf{A} ꕕ \mathbf{B})^{\mathsf T} = \mathbf{A}^{\mathsf T} ꕕ \mathbf{B}^{\mathsf T}$,
>  5. The Frobenius inner product can be written as a [dot
>    product](://en.wikipedia.org/wiki/Dot_product) of
>    [vectorized](https://en.wikipedia.org/wiki/Vectorization_(mathematics))
>    matrices: $<\mathbf{A},\mathbf{B}>_F = \text{vec}(\mathbf{A}) \cdot  \text{vec}(\mathbf{B}) =
>    \text{vec}(\mathbf{A})^{\mathsf T} \text{vec}(\mathbf{B})$,
>  6. Properties 3., 4., and 5. imply that Frobenius inner product of a matrix
>    $\mathbf{A}$ and the matrix product of matrix $\mathbf{B}$ and $\mathbf{C}$ is equal to the
>    Frobenius inner product of the matrix product of the transpose of $\mathbf{B}$ and
>    $\mathbf{A}$  and the matrix $\mathbf{C}$:
>    $<\mathbf{A},\mathbf{B}\mathbf{C}>_F = \text{vec}(\mathbf{A})^{\mathsf T} \text{vec}(\mathbf{B}\mathbf{C}) =
>    \text{vec}(\mathbf{A})^{\mathsf T} (\mathbf{I} ꕕ \mathbf{B})\text{vec}(\mathbf{C}) = 
>    \text{vec}(\mathbf{A})^{\mathsf T} (\mathbf{I} ꕕ \mathbf{B}^{\mathsf T})^{\mathsf T} \text{vec}(\mathbf{C}) = 
>    \text{vec}(\mathbf{B}^{\mathsf T}\mathbf{A})^{\mathsf T} \text{vec}(\mathbf{C}) = 
>    <\mathbf{B}^{\mathsf T} \mathbf{A},\mathbf{C}>_F$.
>  

$$
\mathbf{R}^* = \mathop{\text{argmax}}_{\mathbf{R} \in  SO(3)} \left<\mathbf{U}^{\mathsf T} \mathbf{R} \mathbf{V}, \sigma  \right>_F.
$$


Now, $\mathbf{U}$ and $\mathbf{V}$ are both
[orthonormal](https://en.wikipedia.org/wiki/Orthogonal_matrix), so multiplying
them against a rotation matrix $\mathbf{R}$ does not change its orthonormality. We can
pull them out of the maximization if we account for the reflection they _might_
incur: introduce ${\Omega} = \mathbf{U}^T\mathbf{R}\mathbf{V} \in  O(3)$ with $\det{{\Omega}} = \det{\mathbf{U}\mathbf{V}^{\mathsf T}}$.
This implies that the optimal rotation for the original probklem is recovered
via $\mathbf{R}^* = \mathbf{U} {\Omega}^* \mathbf{V}^{\mathsf T}$.  When we move the $\mathop{\text{argmax}}$ inside, we now
look for an orthonormal matrix ${\Omega} \in  O(3)$ that is a reflection (if
$\det{\mathbf{U}\mathbf{V}^{\mathsf T}} = -1$) or a rotation (if $\det{\mathbf{U}\mathbf{V}^{\mathsf T}} = 1$):

$$
  \mathbf{R}^* = \mathbf{U} \left( \mathop{\text{argmax}}_{{\Omega} \in  O(3),\ \det{{\Omega}} = \det{\mathbf{U}\mathbf{V}^{\mathsf T}}} \left<{\Omega}, \sigma  \right>_F \right) \mathbf{V}^{\mathsf T}.
$$


This ensures that as a result $\mathbf{R}^*$ will be a rotation: $\det{\mathbf{R}^*} = 1$.

> Recall that $\sigma \in \mathbf{R}^{3\times 3}$ is a non-negative diagonal matrix of singular values
> sorted so that the smallest value is in the bottom right corner.

Because ${\Omega}$ is orthonormal, each column (or row) of ${\Omega}$ must have unit norm.
Placing a non-zero on the off-diagonal will get "killed" when multiplied by the
corresponding zero in $\sigma $. So the optimal choice of ${\Omega}$ is to set all values to
zero except on the diagonal. If $\det{\mathbf{U}\mathbf{V}^{\mathsf T}} = -1$, then we should set
one (and only one) of these values to $-1$. The best choice is the bottom right
corner since that will multiply against the smallest singular value in $\Sigma $ (add
negatively affect the maximization the least):

$$
{\Omega}^*_{ij} = \begin{cases}
1 & \text{ if $i=j\lt3$} \\\\
\det{\mathbf{U}\mathbf{V}^{\mathsf T}} & \text{ if $i=j=3$} \\\\
0 & \text{ otherwise.}
\end{cases}
$$


Finally, we have a formula for our optimal rotation:

$$
\mathbf{R} = \mathbf{U} {\Omega}^* \mathbf{V}^{\mathsf T}.
$$


> ### Closed-form point-to-point minimizer
>
> 
> _Interestingly_, despite the non-linear constraint on $\mathbf{R}$ there is actually
> a closed-form solution to the point-to-point matching problem:
> 
> $$
> \mathop{\text{minimize}}_{\mathbf{t}\in \mathbf{R}^3 ,\ \mathbf{R} \in  SO(3)} \Sigma _{i=1}^k \| \mathbf{R} \mathbf{x}_i + \mathbf{t} - \mathbf{p}_i\| ^2 ,
> $$

> 
> This is a variant of what's known as a [Procrustes
> problem](https://en.wikipedia.org/wiki/Orthogonal_Procrustes_problem), named
> after a [mythical psychopath](https://en.wikipedia.org/wiki/Procrustes) who
> would kidnap people and force them to fit in his bed by stretching them or
> cutting off their legs. In our case, we are forcing $\mathbf{R}$ to be perfectly
> orthogonal (no "longer", no "shorter).
> 
> #### Substituting out the translation terms
> 
> This energy is _quadratic_ in $\mathbf{t}$ and there are no other constraints on
> $\mathbf{t}$. We can immediately solve for the optimal $\mathbf{t}^*$---leaving $\mathbf{R}$ as an unknown---by
> setting all derivatives with respect to unknowns in $\mathbf{t}$ to zero:
> 
> $$
> \begin{align}
> \mathbf{t}^*
>   &= \mathop{\text{argmin}}_{\mathbf{t}} \Sigma _{i=1}^k \| \mathbf{R} \mathbf{x}_i + \mathbf{t} - \mathbf{p}_i\| ^2   \\\\
>   &= \mathop{\text{argmin}}_\mathbf{t} \left\|\mathbf{R} \mathbf{X}^{\mathsf T} + \mathbf{t} \mathbf{1}^{\mathsf T} - \mathbf{P}^{\mathsf T}\right\|^2_F,
> \end{align}
> $$

> where $\mathbf{1} \in  \mathbf{R}^{k}$ is a vector ones. Setting the partial derivative with
> respect to $\mathbf{t}$ of this
> quadratic energy to zero finds the minimum:
> $$
> \begin{align}
> 0 
>   &= \frac{\partial }{\partial \mathbf{t}} \left\|\mathbf{R} \mathbf{X}^{\mathsf T} + \mathbf{t} \mathbf{1}^{\mathsf T} - \mathbf{P}^{\mathsf T}\right\|^2_F \\\\
>   &= \mathbf{1}^{\mathsf T} \mathbf{1} \mathbf{t} + \mathbf{R} \mathbf{X}^{\mathsf T} \mathbf{1} - \mathbf{P}^{\mathsf T} \mathbf{1},
> \end{align}
> $$

> 
> Rearranging terms above reveals that the optimal $\mathbf{t}$ is the vector aligning
> the [centroids](https://en.wikipedia.org/wiki/Centroid) of the points in $\mathbf{P}$
> and the points in $\mathbf{X}$ rotated by the---yet-unknown---$\mathbf{R}$. Introducing
> variables for the respective centroids $\widehat{\mathbf{p}} = \tfrac{1}{k} \Sigma _{i=1}^k
> \mathbf{p}_i$ and $\widehat{\mathbf{x}} = \tfrac{1}{k} \Sigma _{i=1}^k \mathbf{x}_i$, we can write the
> formula for the optimal  $\mathbf{t}$:
> 
> $$
> \begin{align}
> \mathbf{t} 
>   &= \frac{\mathbf{P}^{\mathsf T} \mathbf{1} - \mathbf{R} \mathbf{X}^{\mathsf T} \mathbf{1}}{ \mathbf{1}^{\mathsf T} \mathbf{1}} \\\\
>   &= \widehat{\mathbf{p}} - \mathbf{R} \widehat{\mathbf{x}}.
> \end{align}
> $$

> 
> Now we have a formula for the optimal translation vector $\mathbf{t}$ in terms of the
> unknown rotation $\mathbf{R}$. Let us
> [substitute](https://en.wikipedia.org/wiki/Substitution_(algebra)) this formula
> for all occurrences of $\mathbf{t}$ in our energy written in its original summation
> form:
> 
> $$
> \mathop{\text{minimize}}_{\mathbf{R} \in  SO(3)}  \Sigma \limits_{i=1}^k \left\| \mathbf{R} \mathbf{x}_i + ( \widehat{\mathbf{p}} - \mathbf{R}\widehat{\mathbf{x}}) - \mathbf{p}_i \right\|^2 \\\
> \mathop{\text{minimize}}_{\mathbf{R} \in  SO(3)}  \Sigma \limits_{i=1}^k \left\| \mathbf{R} (\mathbf{x}_i - \widehat{\mathbf{x}}) - (\mathbf{p}_i - \widehat{\mathbf{p}}) \right\|^2 \\\\
> \mathop{\text{minimize}}_{\mathbf{R} \in  SO(3)}  \Sigma \limits_{i=1}^k \left\| \mathbf{R} \overline{\mathbf{x}}_i - \overline{\mathbf{p}}_i \right\|^2 \\\\
> \mathop{\text{minimize}}_{\mathbf{R} \in  SO(3)}  \left\| \mathbf{R} \overline{\mathbf{X}}^{\mathsf T} - \overline{\mathbf{P}}^{\mathsf T} \right\|_F^2,
> $$

> 
> where we introduce $\overline{\mathbf{X}} \in  \mathbf{R}^{k \times  3}$ where the ith row contains the
> _relative position_ of the ith point to the centroid $\widehat{\mathbf{x}}$: i.e.,
> $\overline{\mathbf{x}}_i = (\mathbf{x}_i - \widehat{\mathbf{x}})$ (and analagously for $\overline{\mathbf{P}}$).
> 
> Now we have the canonical form of the [orthogonal procrustes
> problem](https://en.wikipedia.org/wiki/Orthogonal_Procrustes_problem). To
> find the optimal rotation matrix $\mathbf{R}^*$ we will massage the terms in the
> _minimization_ until we have a _maximization_ problem involving the [Frobenius
> inner-product](https://en.wikipedia.org/wiki/Frobenius_inner_product) of the
> unknown rotation $\mathbf{R}$ and [covariance
> matrix](https://en.wikipedia.org/wiki/Covariance_matrix) of $\mathbf{X}$ and $\mathbf{P}$:
> 
> $$
> \begin{align}
> \mathbf{R}^* 
> &= \mathop{\text{argmin}}_{\mathbf{R} \in  SO(3)} \left\| \mathbf{R} \overline{\mathbf{X}}^{\mathsf T} - \overline{\mathbf{P}}^{\mathsf T} \right\|_F^2 \\\\
> &= \mathop{\text{argmin}}_{\mathbf{R} \in  SO(3)} \left<\mathbf{R} \overline{\mathbf{X}}^{\mathsf T} - \overline{\mathbf{P}}^{\mathsf T} , \mathbf{R} \overline{\mathbf{X}}^{\mathsf T} - \overline{\mathbf{P}}^{\mathsf T} \right>_F\\\\
> &= \mathop{\text{argmin}}_{\mathbf{R} \in  SO(3)} \left\| \overline{\mathbf{X}} \right\|_F^2 + \left\| \overline{\mathbf{P}} \right\|_F^2 - 2 \left<\mathbf{R} \overline{\mathbf{X}}^{\mathsf T} , \overline{\mathbf{P}}^{\mathsf T} \right>_F\\\\
> &= \mathop{\text{argmax}}_{\mathbf{R} \in  SO(3)} \left<\mathbf{R},\overline{\mathbf{P}}^{\mathsf T}\,\overline{\mathbf{X}}\right>_F\\\\
> &= \mathop{\text{argmax}}_{\mathbf{R} \in  SO(3)} \left<\mathbf{R},\mathbf{M}\right>_F\\
> \end{align}
> $$

> 
> Letting $\mathbf{M} = \overline{\mathbf{P}}^{\mathsf T}\,\overline{\mathbf{X}}$ we can now follow the
> steps above using [singular value
> decomposition](https://en.wikipedia.org/wiki/Singular_value_decomposition) to
> find the optimal $\mathbf{R}$.

### Approximate point-to-plane minimizer

If we apply our linearization of $\mathbf{R}$ to the **point-to-plane** distance
linearization of the matching energy, our minimization is:

$$
\mathop{\text{minimize}}_{\mathbf{t}\in \mathbf{R}^3 , α, β, γ} 
  \Sigma _{i=1}^k 
  \left( 
  \left(
  \left(\begin{array}{ccc}
   0 & -γ &  β \\
   γ &  0 & -α \\
  -β &  α &  0 \\
  \end{array}\right)\mathbf{x}_i +
  \mathbf{x}_i + \mathbf{t} - \mathbf{p}_i 
  \right)\cdot \mathbf{n}_i
  \right)^2.
$$


We can follow similar steps as above. Let's gather a vector of unknowns: $\mathbf{u} =
[α β γ \mathbf{t}^{\mathsf T}] \in  \mathbf{R}⁶$. Then we can write our problem in summation form
as:

$$
\mathop{\text{minimize}}_{\mathbf{u}\in \mathbf{R}⁶}
  \Sigma _{i=1}^k \left(\mathbf{n}_i^{\mathsf T} 
  \left(\begin{array}{cccccc}
         0 &  x_{i,3} & -x_{i,2} & 1 & 0 & 0 \\
  -x_{i,3} &        0 &  x_{i,1} & 0 & 1 & 0 \\
   x_{i,2} & -x_{i,1} &        0 & 0 & 0 & 1
  \end{array}\right) \mathbf{u} +
  \mathbf{n}_i^{\mathsf T}(\mathbf{x}_i - \mathbf{p}_i) \right)^2.
$$


This can be written compactly in matrix form as:

$$
\mathop{\text{minimize}}_{\mathbf{u}\in \mathbf{R}⁶}
  \left(
  \left[\begin{array}{ccc} \text{diag}(\mathbf{N}_1) & \text{diag}(\mathbf{N}_2) & \text{diag}(\mathbf{N}_2)\end{array}\right]
  \left( 
  \mathbf{A}
  \mathbf{u} +
\left[\begin{array}{c}
  \mathbf{X}_1-\mathbf{P}_1 \\
  \mathbf{X}_2-\mathbf{P}_2 \\
  \mathbf{X}_3-\mathbf{P}_3
\end{array}\right]\right)
  \right)^2,
$$


where $\mathbf{N}_i$ is the ith column from the matrix of normals $\mathbf{N} \in  \mathbf{R}^{k \times  3}$,
$\text{diag}(\mathbf{v})$ [creates a diagonal
matrix](https://en.wikipedia.org/wiki/Diagonal_matrix#Matrix_operations) from a
vector, and $\mathbf{A} \in  \mathbf{R}^{3k \times  6}$ is the same as above.

This energy is quadratic in $\mathbf{u}$ and can be solve by setting all partial
derivatives with respect to $\mathbf{u}$ to zero.

> ### Closed-form point-to-point minimizer
>
> To the best of my knowledge, no known closed-form solution exists. I am not
> sure whether it **_can not_** exist or just whether no one has figured it out
> (or they did and I just do not know about it).

## Uniform random sampling of a triangle mesh

Our last missing piece is to sample the surface of a triangle mesh $X$ with $m$
faces uniformly randomly. This allows us to approximate _continuous_ integrals
over the surface $X$ with a summation of the integrand evaluated at a finite
number of randomly selected points. This type of [numerical
integration](https://en.wikipedia.org/wiki/Numerical_integration) is called the
[Monte Carlo method](https://en.wikipedia.org/wiki/Monte_Carlo_method).

We would like our [random
variable](https://en.wikipedia.org/wiki/Random_variable) $\mathbf{x} \in  X$ to have a
uniform [probability density
function](https://en.wikipedia.org/wiki/Probability_density_function) $f(\mathbf{x}) =
1/A_X$, where $A_X$ is the [surface
area](https://en.wikipedia.org/wiki/Surface_area) of the triangle mesh $X$. We
can achieve this by breaking the problem into two steps: uniformly sampling in
a single triangle and sampling triangles non-uniformly according to their
area.

Suppose we have a way to evaluate a continuous random point $\mathbf{x}$ in a triangle
$T$ with uniform probability density function $g_T(\mathbf{x}) = 1/A_T$ _and_ we have a
away to evaluate a discrete random triangle index $T \in  \{1,2,‥,m\}$ with [discrete
probability
distribution](https://en.wikipedia.org/wiki/Probability_distribution#Discrete_probability_distribution)
$h(T) = A_T/A_X$, then the joint probability of evaluating a certain triangle
index $T$ and then uniformly random point in that triangle $\mathbf{x}$ is indeed
uniform over the surface:

$$
h(T) g_T(\mathbf{x}) = \frac{A_T}{A_X} \frac{1}{A_T} = \frac{1}{A_X} = f(\mathbf{x}).
$$


### Uniform random sampling of a single triangle

In order to pick a point uniformly randomly in a triangle with corners $\mathbf{v}_1,
\mathbf{v}_2, \mathbf{v}_3 \in  \mathbf{R}^3$ we will _first_ pick a point uniformly randomly in the
[parallelogram](https://en.wikipedia.org/wiki/Parallelogram) formed by
reflecting $\mathbf{v}_1$ across the line $\overline{\mathbf{v}_2\mathbf{v}_3}$:

$$
\mathbf{x} = \mathbf{v}_1 + α (\mathbf{v}_2-\mathbf{v}_1) + β (\mathbf{v}_3 - \mathbf{v}_1)
$$


where $α,β$ are uniformly sampled from the unit interval $[0,1]$. If $α+β > 1$
then the point $\mathbf{x}$ above will lie in the reflected triangle rather than the
original one. In this case, preprocess $α$ and $β$ by setting $α\Leftarrow 1-α$ and
$β\Leftarrow 1-β$ to reflect the point $\mathbf{x}$ back into the original triangle.

### Area-weighted random sampling of triangles

Assuming we know how to draw a _continuous_ uniform random variable $γ$ from
the unit interval $[0,1]$, we would now like to draw a _discrete_ random
triangle index $T$ from the sequence ${1,‥,m}$ with likelihood proportional to
the relative area of each triangle in the mesh.

We can achieve this by first computing the [cumulative
sum](https://en.wikipedia.org/wiki/Running_total) $\mathbf{C} \in  \mathbf{R}^{m}$ of the relative
areas:

$$
C_i = \Sigma _{j=1}^i \frac{A_j}{A_X},
$$


Then our random index is found by identifying the first entry in $\mathbf{C}$ whose
value is greater than a uniform random variable $γ$. Since $\mathbf{C}$ is sorted,
locating this entry can be done in $O(\log m)$
[time](https://en.wikipedia.org/wiki/Big_O_notation).

### Why is my code so slow?

Try profiling your code. Where is most of the computation time spent?

If you have done things right, the majority of time is spent computing
point-to-mesh distances. For each query point, the [computational
complexity](https://en.wikipedia.org/wiki/Computational_complexity_theory) of
computing its distance to a mesh with $m$ faces is $O(m)$.

This can be _dramatically_ improved (e.g., to $O(\log m)$ on average) using an
[space partitioning](https://en.wikipedia.org/wiki/Space_partitioning) data
structure such as a [kd tree](https://en.wikipedia.org/wiki/K-d_tree), a
[bounding volume
hierarchy](https://en.wikipedia.org/wiki/Bounding_volume_hierarchy), or
[spatial hash](https://en.wikipedia.org/wiki/Bin_(computational_geometry)).

## Tasks

### Read \[Bouaziz 2015\]

This reading task is not directly graded, but it's expected that you read and
understand sections 3.2-3.3 of Sofien Bouaziz's PhD thesis "Realtime Face
Tracking and Animation" 2015. _Understanding_ this may require digging into
wikipedia, other online resources or other papers.

### Blacklist

You may not use the following libigl functions:

- `igl::AABB`
- `igl::fit_rotations`
- `igl::hausdorff`
- `igl::point_mesh_squared_distance`
- `igl::point_simplex_squared_distance`
- `igl::polar_dec`
- `igl::polar_svd3x3`
- `igl::polar_svd`
- `igl::random_points_on_mesh`

### Whitelist

You are encouraged to use the following libigl functions:

- `igl::cumsum` computes cumulative sum
- `igl::doublearea` computes triangle areas
- `igl::per_face_normals` computes normal vectors for each triangle face

### `src/random_points_on_mesh.cpp`

Generate `n` random points uniformly sampled _on_ a given triangle mesh with
vertex positions `VX` and face indices `FX`. 

### `src/point_triangle_distance.cpp`
Compute the distance `d` between a given point `x` and the closest point `p` on
a given triangle with corners `a`, `b`, and `c`.

### `src/point_mesh_distance.cpp`
Compute the distances `D` between a set of given points `X` and their closest
points `P` on a given mesh with vertex positions `VY` and face indices `FY`.
For each point in `P` also output a corresponding normal in `N`.

> It is OK to assume that all points in `P` lie inside (rather than exactly at
> vertices or exactly along edges) for the purposes of normal computation in
> `N`.

### `src/hausdorff_lower_bound.cpp`
Compute a lower bound on the _directed_ Hausdorff distance from a given mesh
(`VX`,`FX`) to another mesh (`VY`,`FY`). This function should be implemented by
randomly sampling the $X$ mesh.

### `src/closest_rotation.cpp`
Given a 3\times 3 matrix `M`, find the closest rotation matrix `R`.

### `src/point_to_point_rigid_matching.cpp`
Given a set of source points X and corresponding target points P, find the
optimal rigid transformation (R,t) that aligns X to P, minimizing the
point-to-point matching energy.

You may implement either that "Approximate" solution via linearizing the
rotation matrix or the "closed form" solution

### `src/point_to_plane_rigid_matching.cpp`
Given a set of source points `X` and corresponding target points `P` and their
normals `N`, find the optimal rigid transformation (`R`,`t`) that aligns `X` to
planes passing through `P` orthogonal to `N`, minimizing the point-to-point
matching energy.

### `src/icp_single_iteration.cpp`
Conduct a _single iteration_ of the iterative closest point method align
(`VX`,`FX`) to (`VY`,`FY`) by finding the rigid transformation (`R`,`t`)
minimizing the matching energy.

The caller can specify the number of samples `num_samples` used to approximate
the integral over $X$ and specify the `method` (point-to-point or
point-to-plane).
