# Geometry Processing – Registration

> **To get started:** Fork this repository then issue
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

This _algorithm_ and its many variants has been use for quite some time to
align discrete shapes. One of the first descriptions is given in "A Method for
Registration of 3-D Shapes" by Besl & McKay 1992. However, the award-winning
PhD thesis of Sofien Bouaziz ("Realtime Face Tracking and Animation" 2015,
section 3.2-3.3) contains a more modern view that unifies many of the variants
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
The usually Euclidean distance between _two points_ $\x$ and $\y$ is the $L²$
norm of their difference :

\\[
d(\x,\y) = ‖\x - \y‖.
\\]

#### Point-to-projection distance

When we consider the distance between a point $\x$ and some _larger_ object $Y$ (a line,
a circle, a surface), the natural extension is to take the distance to the
closest point $\y$ on $Y$:

\\[
d(\x,Y) = \inf_{\y ∈ Y} d(\x,\y).
\\]

written in this way the
[infimum](https://en.wikipedia.org/wiki/Infimum_and_supremum) considers all
possible points $\y$ and keeps the minimum distance. We may equivalently write
this distance instead as simply the point-to-point distance between $\x$ and
the _closest-point projection_ $P_Y(\x)$:

\\[
d(\x,Y) = d((\x,P_Y(\x)) = ‖\x - P_Y(\x)‖.
\\]

If $Y$ is a smooth surface, this projection will also be an [orthogonal
projection](https://en.wikipedia.org/wiki/Projection_(linear_algebra)#Orthogonal_projections).


![The distance between a surface $Y$ (light blue) and a point $\x$ (orange) is
determined by the closest point $P_Y(\x)$ (blue)](images/max-point-mesh.gif)

### Directed Hausdorff distance

We might be tempted to define the distance from surface $X$ to $Y$ as the
_infimum_ of _point-to-projection_ distances over all points $\x$ on $X$:

\\[
D_\text{inf}(X,Y) = \inf_{\x ∈ X} ‖\x - P_Y(\x)‖,
\\]

but this will not be useful for registering two surfaces: it will measure zero
if even just a single point of $\x$ happens to lie on $Y$. Imagine the noses of
two faces touching at their tips.

Instead, we should take the _supremum_ of _point-to-projection_ distances over
all points $\x$ on $X$:

\\[
D_{\overrightarrow{H}}(X,Y) = \sup_{\x ∈ X} ‖\x - P_Y(\x)‖.
\\]

This surface-to-surface distance measure is called the _directed_ [Hausdorff
distance](https://en.wikipedia.org/wiki/Hausdorff_distance). We may interpret
this as taking the worst of the best: we 
let each point $\x$ on $X$ declare its shortest distance to $Y$ and then keep
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

\\[
D_{\overrightarrow{H}}(X,Y) ≠ D_{\overrightarrow{H}}(Y,X).
\\]

#### directed Hausdorff distance between triangle meshes

We can approximate a _lower bound_ on the Hausdorff distance between two meshes
by densely sampling surfaces $X$ and $Y$. We will discuss sampling methods,
later. For now consider that we have chosen a set $\P_X$ of $k$ points on $X$
(each point might lie at a vertex, along an edge, or inside a triangle). The
directed Hausdorff distance from $X$ to another triangle mesh $Y$ must be
_greater_ than the directed Hausdorff distance from this [point
cloud](https://en.wikipedia.org/wiki/Point_cloud) $\P_X$ to $Y$:

\\[
D_{\overrightarrow{H}}(X,Y) ≥ 
D_{\overrightarrow{H}}(\P_X,Y) = \max_{i=1}^k ‖\p_i - P_Y(\p_i)‖,
\\]

where we should be careful to ensure that the projection $P_Y(\p_i)$ of the
point $\p_i$ onto the triangle mesh $Y$ might lie at a vertex, along an edge or
inside a triangle. 

As our sampling $\P_X$ becomes denser and denser on $X$ this lower bound will
approach the true directed Hausdorff distance. Unfortunately, an efficient
_upper bound_ is significantly more difficult to design.

#### Hausdorff distance for alignment optimization

Even if it _were_ cheap to compute, Hausdorff distance is difficult to
_optimize_ when aligning two surfaces. If we treat the Hausdorff distance
between surfaces $X$ and $Y$ as an energy to be minimized, then only change to
the surfaces that will decrease the energy will be moving the (in general)
isolated point on $X$ and isolated point on $Y$ generating the maximum-minimum
distance. In effect, the rest of the surface does not even matter or effect the
Hausdorff distance. This, or any type of $L^∞$ norm, will be much more
difficult to optimize.

Hausdorff distance can serve as a validation measure, while we turn to $L²$
norms for optimization.

## Integrated closest-point distance

We would like a distance measure between two surfaces that---like Hausdorff
distance---does not require a shared parameterization. Unlike Hausdorff
distance, we would like this distance to _diffuse_ the measurement over the
entire surfaces rather than generate it from the sole _worst offender_. We can
accomplish this by replacing the _supremum_ in the Hausdorff distance ($L^∞$)
with a integral of squared distances ($L²$). Let us first define a directed
_closest-point distance_ from  a surface $X$ to another surface $Y$, as the
integral of the squared distance from every point $\x$ on $X$ to its
closest-point projection $P_Y(\x)$ on the surfaces $Y$:

\\[
D_{\overrightarrow{C}}(X,Y) = \sqrt{\ ∫\limits_{\x∈X} ‖\x - P_Y(\x) ‖² \;dA }.
\\]

This distance will only be zero if all points on $X$ also lie on $Y$, but when
it is non-zero it is summing/averaging/diffusing the distance measures of all
of the points.

This distance is suitable to define a matching energy, but is not necessarily
welcoming for optimization: the function inside the square is non-linear. Let's
dig into it a bit. We'll define a directed _matching energy_
$E_{\overrightarrow{C}}(Z,Y)$ from $Z$ to $Y$ to be the squared directed
closest point distance from $X$ to $Y$:

\\[
E_{\overrightarrow{C}}(Z,Y) = ∫\limits_{\z∈Z} ‖\z - P_Y(\z) ‖² \;dA =
∫\limits_{\z∈Z} ‖f_Y(\z) ‖² \;dA
\\]

where we introduce the proximity function $\f_Y:\R³→\R³$ defined simply as the
vector from a point $\z$ to its closest-point projection onto $Y$:

\\[
\f(\z) = \z - P_Y(\z).
\\]

Suppose $Y$ was not a surface, but just a single point $Y = \{\y\}$. In this
case, $\f(\z) = \z - \y$ is clearly linear in $\z$.

Similarly, suppose $Y$ was an [infinite
plane](https://en.wikipedia.org/wiki/Plane_(geometry)) $Y = \{\y | (\y-\p)⋅\n =
0\}$ defined by some point $\p$ on the plane and the plane's unit normal vector
$\n$. Then $\f(\z) = ((\z-\p)⋅\n)\n)$ is also linear in $\z$.

But in general, if $Y$ is an interesting surface $\f(\z)$ will be non-linear; it
might not even be a continuous function.

![](images/closest-point-discontinuous.png)

In optimization, a common successful strategy to minimize energies composed of
squaring a non-linear functions $\f$ is to
[linearize](https://en.wikipedia.org/wiki/Linearization) the function about a
current input value (i.e., a current guess $\z₀$), minimize the energy built
from this linearization, then re-linearize around that solution, and then
repeat. 

This is the core idea behind [gradient
descent](https://en.wikipedia.org/wiki/Gradient_descent) and the
[Gauss-Newton](https://en.wikipedia.org/wiki/Gauss–Newton_algorithm) methods:

```
minimize f(z)²
  z₀ ← initial guess
  repeat until convergence
    f₀ ← linearize f(z) around z₀
    z₀ ← minimize f₀(z)²
```

Since our $\f$ is a geometric function, we can derive its linearizations
_geometrically_.

### Constant function approximation

If we make the convenient---however unrealistic---assumption that in the
neighborhood of the closest-point projection $P_Y(\z₀)$ of the current guess
$\z₀$ the surface $Y$ is simply the point $P_Y(\z₀)$ (perhaps imagine that $Y$
is makes a sharp needle-like point at $P_Y(\z₀)$ or that $Y$ is very far away
from $\x$), then we can approximate $\f(\z)$ in the proximity of our current
guess $\z₀$ as the vector between the input point $\z$ and $P_Y(\z₀)$:

\\[
\f(\z) \approx \f_\text{point}(\z) = \z-P_Y(\z₀)
\\]

In effect, we are assuming that the surface $Y$ is _constant_ function of its
parameterization: $\y(u,v) = P_Y(\z₀)$.

Minimizing $E_{\overrightarrow{C}}$ iteratively using this linearization (or
rather _constantization_) of $\f$ is equivalent to the [gradient
descent](https://en.wikipedia.org/wiki/Gradient_descent). We have simply
derived our gradients geometrically.

### Linear function approximation

If we make make a slightly more appropriate assuming that in the neighborhood
of the  $P_Y(\z₀)$ the surface $Y$ is a plane, then we can improve this
approximation while keeping $\f$ linear in $\z$:

\\[
\f(\z) \approx \f_\text{plane}(\z) = ((\z-P_Y(\z₀))⋅\n) \n.
\\]

where the plane that _best_ approximates $Y$ locally near $P_Y(\z₀)$ is the
[tangent plane](https://en.wikipedia.org/wiki/Tangent_space) defined by the
[normal vector](https://en.wikipedia.org/wiki/Normal_(geometry)) $\n$ at
$P_Y(\z₀)$.


Minimizing $E_{\overrightarrow{C}}$ iteratively using this linearization of
$\f$ is equivalent to the
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
$T(\x) = \Rot \x + \t$ for some rotation matrix $\Rot ∈ SO(3) ⊂ \R^{3×3}$
(i.e., an [orthogonal matrix with determinant
1](https://en.wikipedia.org/wiki/Rotation_group_SO(3))) and translation vector
$\t∈\R³$.

Our matching problem can be written as an optimization problem to find the best
possible rotation $\Rot$ and translation $\t$ that match surface $X$ to surface
$Y$:

\\[
\mathop{\text{minimize}}_{\t∈\R³,\ \Rot ∈ SO(3)} 
  ∫\limits_{\x∈X} ‖\Rot \x + \t - P_Y(T(\x)) ‖² \;dA
\\]

Even if $X$ is a triangle mesh, it is difficult to _integrate_ over _all_
points on the surface of $X$. _At any point_, we can approximate this energy by
_summing_ over a point-sampling of $X$:

\\[
\mathop{\text{minimize}}_{\t∈\R³,\ \Rot ∈ SO(3)} 
  ∑_{i=1}^k ‖\Rot \x_i + \t - P_Y(T(\x_i)) ‖²,
\\]

where $\X ∈ \R^{k×3}$ is a set of $k$ points on $X$ so that each point $\x_i$
might lie at a vertex, along an edge, or inside a triangle. We defer discussion
of _how_ to sample a triangle mesh surface.

### Pseudocode

As the name implies, the method proceeds by iteratively finding the closest
point on $Y$ to the current rigid transformation $\Rot \x + \t$ of each sample
point $\x$ in $\X$ and then minimizing the _linearized_ energy to update the
rotation $\Rot$ and translation $\t$. 

If $V_X$ and $F_X$ are the vertices and faces of a triangle mesh surface $X$
(and correspondingly for $Y$), then we can summarize a generic ICP algorithm in
pseudocode:

```
icp V_X, F_X, V_Y, F_Y
  R,t ← initialize (e.g., set to identity transformation)
  repeat until convergence
    X ← sample source mesh (V_X,F_X)
    P0 ← project all X onto target mesh (V_Y,F_Y)
    R,t ← update rigid transform to best match X and P0
    V_X ← rigidly transform original source mesh by R and t
```

### Updating the rigid transformation

We would like to find the rotation matrix $\Rot ∈ SO(3) ⊂ \R^{3×3}$ and
translation vector $\t∈\R³$ that _best_ aligns a given a set of points $\X ∈
\R^{k×3}$ on the source mesh and their current closest points $\P ∈ \P^{k×3}$
on the target mesh. We have two choices for _linearizing_ our matching energy:
point-to-point (gradient descent) and point-to-plane (Gauss-Newton).

![ICP using the point-to-point matching energy linearization is slow to
converge.](images/max-point-to-point.gif)

![ICP using the point-to-plane matching energy linearization is
faster.](images/max-point-to-plane.gif)

In either case, this is still a non-linear optimization problem. This time due
to the [constraints](https://en.wikipedia.org/wiki/Constrained_optimization)
rather than the energy term. 

We require that $\Rot$ is not just any 3×3 matrix, but a rotation matrix. We
can _linearize_ this constraint, by assuming that the rotation in $\Rot$ will
be very small and thus well approximated by the identity matrix $\I$ plus a
skew-symmetric matrix:

\\[
\Rot \approx \I + 
  \left(\begin{array}{ccc}
   0 & -γ &  β \\
   γ &  0 & -α \\
  -β &  α &  0 \\
  \end{array}\right)
\\]

where we can now work directly with the three scalar unknowns $α$, $β$ and $γ$.

### Approximate point-to-point minimizer

If we apply our linearization of $\Rot$ to the **point-to-point** distance
linearization of the matching energy, our minimization becomes:

\\[
\mathop{\text{minimize}}_{\t∈\R³, α, β, γ} 
  ∑_{i=1}^k \left\|
  \left(\begin{array}{ccc}
   0 & -γ &  β \\
   γ &  0 & -α \\
  -β &  α &  0 \\
  \end{array}\right)
  \x_i + \t - \p_i \right\|^2.
\\]

This energy is quadratic in the translation vector $\t$ and the linearized
rotation angles $α$, $β$ and $γ$. Let's gather these degrees of freedom into a
vector of unknowns: $\u = [α β γ \t^\transpose] ∈ \R⁶$. Then we can write our
problem in summation form as:

\\[
\mathop{\text{minimize}}_{\u∈\R⁶}
  ∑_{i=1}^k \left\| 
  \left(\begin{array}{cccccc}
         0 &  x_{i,3} & -x_{i,2} & 1 & 0 & 0 \\
  -x_{i,3} &        0 &  x_{i,1} & 0 & 1 & 0 \\
   x_{i,2} & -x_{i,1} &        0 & 0 & 0 & 1
  \end{array}\right) \u +
  \x_i - \p_i \right\|^2.
\\]

This can be written compactly in matrix form as:

\\[
\mathop{\text{minimize}}_{\u∈\R⁶}
  \left\|
  \underbrace{
  \left(\begin{array}{cccccc}
      0 &  \X_3 & -\X_2 & \One & 0    & 0 \\
  -\X_3 &     0 &  \X_1 & 0    & \One & 0 \\
   \X_2 & -\X_1 &     0 & 0    & 0    & \One
  \end{array}\right)
  }_{\A}
  \u +
\left[\begin{array}{c}
  \X_1-\P_1 \\
  \X_2-\P_2 \\
  \X_3-\P_3
\end{array}\right]
  \right\|_F^2,
\\]
where we introduce the matrix $\A ∈ \R^{3k × 6}$ that gathers the columns
$\X_i$ of $\X$ and columns of ones $\One ∈ \R^k$.

This quadratic energy is minimized with its partial derivatives with respect to
entries in $\u$ are all zero:

\\[
\begin{align}
\A^\transpose \A \u & = -\A^\transpose 
\left[\begin{array}{c}
  \X_1-\P_1 \\
  \X_2-\P_2 \\
  \X_3-\P_3
\end{array}\right]
, \\
\u & = \left(\A^\transpose \A\right)^{-1} \left(-\A^\transpose
\left[\begin{array}{c}
  \X_1-\P_1 \\
  \X_2-\P_2 \\
  \X_3-\P_3
\end{array}\right]
\right),
\end{align}
\\]

Solving this small 6×6 system gives us our translation vector $\t$ and the
linearized rotation angles $α$, $β$ and $γ$. If we simply assign 

\\[
\Rot ←  \M := \I + 
  \left(\begin{array}{ccc}
   0 & -γ &  β \\
   γ &  0 & -α \\
  -β &  α &  0 \\
  \end{array}\right)
\\]

then our transformation will _not_ be rigid. Instead, we should project $\M$
onto the space of rotation matrices.

#### Recovering a pure rotation from its linearization

If $α$, $β$ and $γ$ are all small, then it may be safe to _interpret_ these
values as rotation angles about the $x$, $y$, and $z$ axes respectively.

In general, it is better to find the closest rotation matrix to $\M$. In other
words, we'd like to solve the small optimization problem:

\\[
\begin{align}
\Rot^* 
&= \argmin_{\Rot ∈ SO(3)} \left\| \Rot - \M \right\|_F^2 \\\\
&= \argmin_{\Rot ∈ SO(3)} \left\| \M \right\|_F^2 + \left\| \Rot \right\|_F^2 - 2 \left<\Rot^\transpose, \M \right>_F\\\\
&= \argmax_{\Rot ∈ SO(3)} \left<\Rot^\transpose, \M \right>_F
\end{align}
\\]

We now take advantage of the [singular value
decomposition](https://en.wikipedia.org/wiki/Singular_value_decomposition) of
$\M = \U Σ \V^\transpose$, where $\U, \V ∈
\R^{3×3}$ are orthonormal matrices and $Σ∈\R^{3×3}$ is a diagonal matrix:

\\[
\begin{align}
\Rot^*  
  &= \argmax_{\Rot ∈ SO(3)} \left<\Rot,\V Σ \U^\transpose \right>_F \\\\
  &= \argmax_{\Rot ∈ SO(3)} \left<\V^\transpose \Rot \U, Σ \right>_F \\\\
  &= \U \left( \argmax_{Ω ∈ O(3),\ \det{Ω} = \det{\U\V^\transpose}} \left<Ω, Σ \right>_F \right) \V^\transpose,\\\\
\end{align}
\\]

where the optimization argument $Ω ∈ O(3)$ is an orthogonal matrix, but may be
either a reflection ($\det{Ω} = -1$) or a rotation (($\det{Ω} = 1$) depending
on the SVD on $\M$. This ensures that as a result $\R^*$ will have determinant
1. The optimal choice of $Ω$ is to set all values to zero except on the
diagonal, where we place all 1s except the bottom right corner (corresponding
to the smallest singular value in $Σ$) which is set to $\det{\U\V^\transpose}$:

\\[
Ω_{ij} = \begin{cases}
1 & \text{ if $i=j\lt3$} \\\\
\det{\U\V^\transpose} & \text{ if $i=j=3$} \\\\
0 & \text{ otherwise.}
\end{cases}
\\]

Finally, we have a formula for our optimal rotation:

\\[
\Rot = \U Ω \V^\transpose.
\\]

> ### Closed-form point-to-point minimizer
>
> 
> _Interestingly_, despite the non-linear constraint on $\Rot$ there is actually
> a closed-form solution to the point-to-point matching problem:
> 
> \\[
> \mathop{\text{minimize}}_{\t∈\R³,\ \Rot ∈ SO(3)} ∑_{i=1}^k ‖\Rot \x_i + \t - \p_i‖²,
> \\]
> 
> This is a variant of what's known as a [Procrustes
> problem](https://en.wikipedia.org/wiki/Orthogonal_Procrustes_problem), named
> after a [mythical psychopath](https://en.wikipedia.org/wiki/Procrustes) who
> would kidnap people and force them to fit in his bed by stretching them or
> cutting off their legs. In our case, we are forcing $\Rot$ to be perfectly
> orthogonal (no "longer", no "shorter).
> 
> #### Substituting out the translation terms
> 
> This energy is _quadratic_ in $\t$ and there are no other constraints on
> $\t$. We can immediately solve for the optimal $\t^*$---leaving $\Rot$ as an unknown---by
> setting all derivatives with respect to unknowns in $\t$ to zero:
> 
> \\[
> \begin{align}
> \t^*
>   &= \argmin_{\t} ∑_{i=1}^k ‖\Rot \x_i + \t - \p_i‖²  \\\\
>   &= \argmin_\t \left\|\Rot \X^\transpose + \t \One^\transpose - \P^\transpose\right\|^2_F,
> \end{align}
> \\]
> where $\One ∈ \R^{k}$ is a vector ones. Setting the partial derivative with
> respect to $\t$ of this
> quadratic energy to zero finds the minimum:
> \\[
> \begin{align}
> 0 
>   &= \frac{∂}{∂\t} \left\|\Rot \X^\transpose + \t \One^\transpose - \P^\transpose\right\|^2_F \\\\
>   &= \One^\transpose \One \t + \Rot \X^\transpose \One - \P^\transpose \One,
> \end{align}
> \\]
> 
> Rearranging terms above reveals that the optimal $\t$ is the vector aligning
> the [centroids](https://en.wikipedia.org/wiki/Centroid) of the points in $\P$
> and the points in $\X$ rotated by the---yet-unknown---$\Rot$. Introducing
> variables for the respective centroids $\hat{\p} = \tfrac{1}{k} ∑_{i=1}^k
> \p_i$ and $\hat{\x} = \tfrac{1}{k} ∑_{i=1}^k \x_i$, we can write the
> formula for the optimal  $\t$:
> 
> \\[
> \begin{align}
> \t 
>   &= \frac{\P^\transpose \One - \Rot \X^\transpose \One}{ \One^\transpose \One} \\\\
>   &= \hat{\p} - \Rot \hat{\x}.
> \end{align}
> \\]
> 
> Now we have a formula for the optimal translation vector $\t$ in terms of the
> unknown rotation $\R$. Let us
> [substitute](https://en.wikipedia.org/wiki/Substitution_(algebra)) this formula
> for all occurrences of $\t$ in our energy written in its original summation
> form:
> 
> \\[
> \mathop{\text{minimize}}_{\Rot ∈ SO(3)}  ∑\limits_{i=1}^k \left\| \Rot \x_i + ( \hat{\p} - \Rot\hat{\x}) - \p_i \right\|^2 \\\
> \mathop{\text{minimize}}_{\Rot ∈ SO(3)}  ∑\limits_{i=1}^k \left\| \Rot (\x_i - \hat{\x}) - (\p_i - \hat{\p}) \right\|^2 \\\\
> \mathop{\text{minimize}}_{\Rot ∈ SO(3)}  ∑\limits_{i=1}^k \left\| \Rot \overline{\x}_i - \overline{\p}_i \right\|^2 \\\\
> \mathop{\text{minimize}}_{\Rot ∈ SO(3)}  \left\| \Rot \overline{\X}^\transpose - \overline{\P}^\transpose \right\|_F^2,
> \\]
> 
> where we introduce $\overline{\X} ∈ \R^{k × 3}$ where the ith row contains the
> _relative position_ of the ith point to the centroid $\hat{\x}$: i.e.,
> $\overline{\x}_i = (\x_i - \hat{\x})$ (and analagously for $\overline{\P}$).
> 
> Now we have the canonical form of the [orthogonal procrustes
> problem](https://en.wikipedia.org/wiki/Orthogonal_Procrustes_problem). To
> find the optimal rotation matrix $\Rot^*$ we will massage the terms in the
> _minimization_ until we have a _maximization_ problem involving the [Frobenius
> inner-product](https://en.wikipedia.org/wiki/Frobenius_inner_product) of the
> unknown rotation $\Rot$ and [covariance
> matrix](https://en.wikipedia.org/wiki/Covariance_matrix) of $\X$ and $\P$:
> 
> \\[
> \begin{align}
> \Rot^* 
> &= \argmin_{\Rot ∈ SO(3)} \left\| \Rot \overline{\X}^\transpose - \overline{\P}^\transpose \right\|_F^2 \\\\
> &= \argmin_{\Rot ∈ SO(3)} \left<\Rot \overline{\X}^\transpose - \overline{\P}^\transpose , \Rot \overline{\X}^\transpose - \overline{\P}^\transpose \right>_F\\\\
> &= \argmin_{\Rot ∈ SO(3)} \left\| \overline{\X} \right\|_F^2 + \left\| \overline{\P} \right\|_F^2 - 2 \left<\Rot \overline{\X}^\transpose , \overline{\P}^\transpose \right>_F\\\\
> &= \argmax_{\Rot ∈ SO(3)} \left<\Rot,\overline{\P}\,\overline{\X}^\transpose\right>_F\\\\
> &= \argmax_{\Rot ∈ SO(3)} \left<\Rot,\M\right>_F\\\\
> \end{align}
> \\]
> 
> Letting $\M = \overline{\P}\,\overline{\X}^\transpose$ we can now follow the
> steps above using [singular value
> decomposition](https://en.wikipedia.org/wiki/Singular_value_decomposition) to
> find the optimal $\Rot$.

### Approximate point-to-plane minimizer

If we apply our linearization of $\Rot$ to the **point-to-plane** distance
linearization of the matching energy, our minimization is:

\\[
\mathop{\text{minimize}}_{\t∈\R³, α, β, γ} 
  ∑_{i=1}^k 
  \left( 
  \left(
  \left(\begin{array}{ccc}
   0 & -γ &  β \\
   γ &  0 & -α \\
  -β &  α &  0 \\
  \end{array}\right)\x_i +
  \x_i + \t - \p_i 
  \right)⋅\n_i
  \right)^2.
\\]

We can follow similar steps as above. Let's gather a vector of unknowns: $\u =
[α β γ \t^\transpose] ∈ \R⁶$. Then we can write our problem in summation form
as:

\\[
\mathop{\text{minimize}}_{\u∈\R⁶}
  ∑_{i=1}^k \left(\n_i^\transpose 
  \left(\begin{array}{cccccc}
         0 &  x_{i,3} & -x_{i,2} & 1 & 0 & 0 \\
  -x_{i,3} &        0 &  x_{i,1} & 0 & 1 & 0 \\
   x_{i,2} & -x_{i,1} &        0 & 0 & 0 & 1
  \end{array}\right) \u +
  \n_i^\transpose(\x_i - \p_i) \right)^2.
\\]

This can be written compactly in matrix form as:

\\[
\mathop{\text{minimize}}_{\u∈\R⁶}
  \left(
  \left[\begin{array}{ccc} \text{diag}(\N_1) & \text{diag}(\N_2) & \text{diag}(\N_2)\end{array}\right]
  \left( 
  \A
  \u +
\left[\begin{array}{c}
  \X_1-\P_1 \\
  \X_2-\P_2 \\
  \X_3-\P_3
\end{array}\right]\right)
  \right)^2,
\\]

where $\N_i$ is the ith column from the matrix of normals $\N ∈ \R^{k × 3}$,
$\text{diag}(\v)$ [creates a diagonal
matrix](https://en.wikipedia.org/wiki/Diagonal_matrix#Matrix_operations) from a
vector, and $\A ∈ \R^{3k × 6}$ is the same as above.

This energy is quadratic in $\u$ and can be solve by setting all partial
derivatives with respect to $\u$ to zero.

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
variable](https://en.wikipedia.org/wiki/Random_variable) $\x ∈ X$ to have a
uniform [probability density
function](https://en.wikipedia.org/wiki/Probability_density_function) $f(\x) =
1/A_X$, where $A_X$ is the [surface
area](https://en.wikipedia.org/wiki/Surface_area) of the triangle mesh $X$. We
can achieve this by breaking the problem into two steps: uniformly sampling in
a single triangle and sampling triangles non-uniformly according to their
area.

Suppose we have a way to evaluate a continuous random point $\x$ in a triangle
$T$ with uniform probability density function $g_T(\x) = 1/A_T$ _and_ we have a
away to evaluate a discrete random triangle index $T ∈ \{1,2,‥,m\}$ with [discrete
probability
distribution](https://en.wikipedia.org/wiki/Probability_distribution#Discrete_probability_distribution)
$h(T) = A_T/A_X$, then the joint probability of evaluating a certain triangle
index $T$ and then uniformly random point in that triangle $\x$ is indeed
uniform over the surface:

\\[
h(T) g_T(\x) = \frac{A_T}{A_X} \frac{1}{A_T} = \frac{1}{A_T} = f(\x).
\\]

### Uniform random sampling of a single triangle

In order to pick a point uniformly randomly in a triangle with corners $\v_1,
\v_2, \v_3 ∈ \R^3$ we will _first_ pick a point uniformly randomly in the
[parallelogram](https://en.wikipedia.org/wiki/Parallelogram) formed by
reflecting $\v_1$ across the line $\overline{\v_2\v_3}$:

\\[
\x = \v_1 + α (\v_2-\v_1) + β (\v_3 - \v_1)
\\]

where $α,β$ are uniformly sampled from the unit interval $[0,1]$. If $α+β > 1$
then the point $\x$ above will lie in the reflected triangle rather than the
original one. In this case, preprocess $α$ and $β$ by setting $α←1-α$ and
$β←1-β$ to reflect the point $\x$ back into the original triangle.

### Area-weighted random sampling of triangles

Assuming we know how to draw a _continuous_ uniform random variable $γ$ from
the unit interval $[0,1]$, we would now like to draw a _discrete_ random
triangle index $T$ from the sequence ${1,‥,m}$ with likelihood proportional to
the relative area of each triangle in the mesh.

We can achieve this by first computing the [cumulative
sum](https://en.wikipedia.org/wiki/Running_total) $\C ∈ \R^{m}$ of the relative
areas:

\\[
C_i = ∑_{j=1}^m \frac{A_j}{A_X},
\\]

Then our random index is found by identifying the first entry in $\C$ whose
value is greater than a uniform random variable $γ$. Since $\C$ is sorted,
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
Given a 3×3 matrix `M`, find the closest rotation matrix `R`.

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
