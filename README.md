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

    ./mesh-reconstruction [path to mesh1.obj] [path to mesh2.obj] ...

## Background

In this assignment, we will be implementing a version of the [iterative closest
point (ICP)](https://en.wikipedia.org/wiki/Iterative_closest_point), not to be
confused with [Insane Clown Posse](https://en.wikipedia.org/wiki/Insane_Clown_Posse).

This _algorithm_ and its many variants has been use for quite some time to
align discrete shapes. One of the first descriptions is given in "A Method for
Registration of 3-D Shapes" by Besl & McKay 1992. However, the award-winning
PhD thesis of Sofien Bouaziz ("Realtime Face Tracking and Animation" 2015,
section 3.2-3.3) contains a more modern view that unifies many of the variants
with respect to how they impact the same core optimization problem. 

Our goal is to _align_ shape $Z$ to shape $Y$. For now, let's assume that $Z$
and $Y$ are smooth surfaces. We will revisit this assumption later, and
investigate how things change if $Z$ and $Y$ are actually point clouds or
triangle meshes.

There are various ways to measure how well aligned two surfaces are. 

### Matching surfaces that share a parameterization

For example, consider if a surface $X$ is given with a parameterization so that
each point on $X$ can be written as a function of parameters $u$ and $v$,
$\x(u,v) ∈ X$. If $Y$ is another surface produced by the same parameterization
($\y(u,v) ∈ Y$), then we can think of $Y$ as a _deformation_ of $X$ (or
vice-versa). Each point $\y(u,v)$ on the surface $Y$ has a natural
corresponding point $\x(u,v)$ on $X$ via the parameters $u$ and $v$.

![The surface of a Beetle is _deformed_ into a new surface. The
parameterization of the original surface allows us to identify the
corresponding points on the deformed surface.
[image source](http://www.cs.cmu.edu/~kmcrane/)
](images/beetle-deformation.png)

A very natural way to _measure_ the difference between these two surfaces
_aggregate_ the distance between each pair of corresponding points. For
example, we could [integrate](https://en.wikipedia.org/wiki/Integral) this
distance over the parametric domain
([w.l.o.g.](https://en.wikipedia.org/wiki/Without_loss_of_generality) let's say
valid values are $u,v ∈ (0,1)$):

\\[
D_2(X,Y) = \sqrt{ ∫_0^1∫_0^1 ‖\x(u,v) - \y(u,v)‖² \;du\;dv }
\\]

This measure will be zero if the surfaces are the same for any choice of
parameters $u$ and $v$. The measure $D_2(X,Y)$ could large if every point on
$Y$ is _slightly_ deformed or if a few bad points are deformed a lot. This
distance is the [L²
norm](https://en.wikipedia.org/wiki/Norm_(mathematics)#Infinite-dimensional_case)
of the magnitude of the _displacement_ from $X$ to $Y$ (or vice-versa):

\\[
D_2(X,Y) = \sqrt{ ∫_0^1∫_0^1 d(u,v)² \;du\;dv }, \quad \text{ and } \quad d(u,v) = ‖\x(u,v) -
\y(u,v)‖
\\]

We can directly measure the _maximum_ distance between corresponding
points, the $L^∞$ norm:

\\[
D_∞(X,Y) = \lim_{p→∞} \sqrt[p]{∫_0^1∫_0^1 d(u,v)^p \;du\;dv } = \sup\limits_{u,v ∈
(0,1)} ‖\x(u,v) - \y(u,v)‖,
\\]

where $\sup$ takes the
[supremum](https://en.wikipedia.org/wiki/Infimum_and_supremum) (roughly the continuous math
analog of the [maximum](https://en.wikipedia.org/wiki/Maxima_and_minima)).

The measure $D_∞$ will also be exactly zero if the surfaces are the same.

#### Triangle meshes

On the computer, we can store an explicit surface as a triangle mesh. Triangle
meshes have an _implicit_ parameterization: each triangle can be trivially
and independently mapped to the unit triangle, via its [barycentric
coordinates](https://en.wikipedia.org/wiki/Barycentric_coordinate_system#Barycentric_coordinates_on_triangles).

Triangle meshes also afford an immediate analog of deformation: moving each
vertex of the mesh (without change the mesh combinatorics/topology).

So if $\V_X ∈ \R^{n × 3}$ represents the vertices of our surface $X$ with a set
$F$ of $m$
triangular faces and  $\V_Y ∈ \R^{n × 3}$ the vertices of deformed surface $Y$,
then we can rewrite our measure $D_2$ above as a sum of integrals over each
triangle:

\\[
∑\limits_{\{i,j,k\} ∈ T}  \sqrt{ ∫_0^{1-u} ∫_0^1 
\left\|
\underbrace{\left(u \v_x^i + v \v_x^j + (1-u-v) \v_x^k\right)}_\x
-
\underbrace{\left(u \v_y^i + v \v_y^j + (1-u-v) \v_y^k\right)}_\y
\right\|^2 \;du\;dv }
\\]

> **Note:** The _areas_ of the triangles in our mesh may be different. So this
> measure may be thrown off by a very large triangle with a small difference
> and may fail to measure a very small triangle with a large difference. We'll
> learn how to account for this, later.

Unfortunately, in many scenarios we do not have two co-parameterized surface or
a simple per-vertex mesh deformation. Instead, we may have two arbitrary
surfaces discretized with different meshes of different
topologies. We will need a measure of difference or distance between two
surfaces that does not assume a shared parameterization.

## Hausdorff distance

We can build a distance measure between two surfaces out of two ingredients:

 1. the distance between a single point to a surface, and 
 2. the supremum we used in $D_∞$ above.

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

It is easy to verify that $D_{\overrightarrow{H}}$ will only equal zero if all
points on $X$ also lie exactly on $Y$. 

The converse is not true: if $D_{\overrightarrow{H}}=0$ there may still be
points on $Y$ that do not lie on $X$. In other words, _in general_ the directed
Hausdorff distance from surface $X$ to surface $Y$ will not equal the Hausdorff
distance from surface $Y$ to surface $X$:

\\[
D_{\overrightarrow{H}}(X,Y) ≠ D_{\overrightarrow{H}}(Y,X).
\\]

### (undirected) Hausdorff distance

To form a [metric](https://en.wikipedia.org/wiki/Metric_(mathematics)) in the
mathematical sense, a distance measure should be symmetric: distance from $X$
to $Y$ equals distance from $Y$ to $X$. Hausdorff distance is defined as the
maximum of directed Haurdorff distances:

\\[
D_{H}(X,Y) = \max \left[ D_{\overrightarrow{H}}(X,Y)\ , \ D_{\overrightarrow{H}}(Y,X) \right].
\\]

Unlike each individual directed distance, the (undirected) Hausdorff distance
will measure zero [if and only
if](https://en.wikipedia.org/wiki/If_and_only_if) the [point
sets](https://en.wikipedia.org/wiki/Euclidean_space#Balls.2C_spheres.2C_and_hypersurfaces)
of the surface $X$ is identical to that of $Y$.

![](images/hausdorff-distance-2d.png)

#### Triangle meshes

On the computer, surfaces represented with triangle meshes (like any point set)
admit a well-defined Hausdorff distance between one-another. Unfortunately,
computing _exact_ Hausdorff distance between two triangle meshes remains a
difficult task: known exact algorithm are prohibitively inefficient.

We do not know [_a
priori_](https://en.wikipedia.org/wiki/A_priori_and_a_posteriori) which
point(s) of each triangle mesh will end up determining the maximum value. It is
tempting, optimistic, but ultimately incorrect to assume that the _generator_
points will be one of the vertices of the triangle mesh. Consider if a triangle
$t$ connected corners at $(1,1,0)$, $(1,0,1)$ and $(0,1,1)$ and $B$ was a mesh
with two triangles, the first connecting $(0,0,0)$, $(1,0,1)$ and $(1,1,0)$ and
the second connecting $(0,0,0)$, $(0,1,1)$ and $(1,1,0)$. The corners of $t$
also appear as vertices of $B$, so clearly their respective vertex-to-mesh
distances are zero, yet the maximum minimum distance from $t$ to $B$ is clearly
non-zero (it is $\sqrt{3}/3$).

![The directed Hausdorff distance from the orange triangle $A$ to
the blue, two-triangle mesh $B$ is non-zero (generated by red segment), but the
distance from each corner of $A$ to $B$ is
zero.](images/hausdorff-counterexample-3d.jpg)

One might also optimistically, but erroneously hope that by considering the
symmetric Hausdorff distance one of the _generator_ points must lie on a vertex
of $A$ or of $B$. Unfortunately, this only follows for convex shapes.

![The directed Hausdorff distance _to_ the blue "alligator" shape
in 2D _from_ its orange convex hull is generated by non-vertex points. Since
the symmetric Hausdorff distance is the maximum of this and the smaller
distance from the blue shape to the orange shape, it is also generated by these
non-vertex points.](images/hausdorff-non-convex-2d.svg)

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

This is similar to the $D_2$ measure above, but we are _not_ assuming that $X$
and $Y$ already have a matching parameterization. 

This distance will only be zero if all points on $X$ also lie on $Y$, but when
it is non-zero it is summing/averaging/diffusing the distance measures of all
of the points.

<!--
We can similarly define a symmetric distance by _summing_ the two directed
distances:

\\[
D_{C}(X,Y) = 
D_{\overrightarrow{C}}(X,Y) +
D_{\overrightarrow{C}}(Y,X) = 
\sqrt{\ ∫\limits_{\x∈X} ‖\x - P_Y(\x) ‖² \;dA }+
\sqrt{\ ∫\limits_{\y∈Y} ‖\y - P_X(\y) ‖² \;dA }.
\\]
-->

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
So far we have derived distances between a surface $X$ and another surface $Y$.
In the alignment and registration problem, we would like to
[transform](https://en.wikipedia.org/wiki/Transformation_(function)) one surface
$X$ into a new surface $T(X) = Z$ so that it best aligns with/matches the other
surface $Y$.

Our matching problem can be written as an optimization problem to find the best
possible transformation $T$ so that $T(X)$ best matches $Y$:

\\[
\mathop{\text{minimize}}_T ∫\limits_{\x∈X} ‖T(\x) - P_Y(T(\x)) ‖² \;dA
\\]

Even if $X$ is a triangle mesh, it is difficult to _integrate_ over _all_ points
on the surface of $X$. We can approximate this energy by _summing_ over a
dense point-sampling $X$:



\\[
\mathop{\text{minimize}}_T ∑_{i=1}^k ‖T(\x_i) - P_Y(T(\x_i)) ‖²,
\\]

where $\X ∈ ℝ^{k×3}$ is a set of $k$ points on $X$ so that each point $\x_i$ might lie at
a vertex, along an edge, or inside a triangle. We defer discussion of _how_ to
sample a triangle mesh surface.

As the name implies, the iterative closest point method will proceed by
iteratively finding the closest point on $Y$ to the current transformation
$T(\x)$ of each sample point $\x$ in $\X$ and then minimizing the linearized
energy to update $T$. So if $V_X$ and $F_X$ are the vertices and faces of a
triangle mesh surface $X$ (and correspondingly for $Y$), then we can summarize
a generic ICP algorithm in pseudocode:

```
icp V_X, F_X, V_Y, F_Y
  T ← initialize (e.g., set to identity transformation)
  X ← uniformly sample (V_X,F_X)
  repeat until convergence
    Z0 ← T(X)
    P0 ← project all transformed points Z0 onto (V_Y,F_Y)
    f₀ ← linearize f(z) = z - P_Y(z) around each z0
    T ← minimize ∑ f₀(T(x))²
```

where `linearize f(z) = z - P_Y(z)` could be accomplished by using the simple
point-to-point distance or by utilizing normal information via point-to-plane
distance. For now, let's a assume that we use the point-to-point distance. The
_inner_ minimization problem is then:

\\[
\mathop{\text{minimize}}_T ∑_{i=1}^k ‖T(\x_i) - \p_i ‖²,
\\]

where $\p_i$ is the closest-point projection of $\z_i = T(\x_i)$ using the
current guess of the transformation $T$.


If we place no restrictions or
[constraints](https://en.wikipedia.org/wiki/Constrained_optimization) on the
transformation $T: \R³ → \R³$, then there are many [trivial
solutions](https://en.wikipedia.org/wiki/Triviality_(mathematics)) to
minimizing $E_{\overrightarrow{C}}(T(X),Y)$. For example, we could simply
define $T$ so that every point $\x$ gets mapped to its closest point on $Y$:
$T(\x) := P_Y(\x)$. This would clearly induce $E_{\overrightarrow{C}}=0$.
Actually, we could get zero energy even if we define $T$ to map _all_ points on
$X$ to the same _arbitrary_ point on $Y$: $T(\x) := \y$.

<!--
> ### Symmetric energy for complete matches
> 
> If $X$ and $Y$ are _complete_ matches, then one way to remove these trivial
> solutions is to minimize the symmetric energy summing energies from $X$ to $Y$
> and from $Y$ to $X$:
> 
> \\[
> E_C(T(X),Y) = E_{\overrightarrow{C}}(T(X),Y) + E_{\overrightarrow{C}}(Y,T(X)).
> \\]
> 
> Appending the energy $E_{\overrightarrow{C}}(Y,T(X))$ measure the distance
> from all points on $Y$ to the transformed surface $T(X)$ ensures that we will
> not get zero energy if some part of $Y$ does not get matched to some part of
> $X$.
-->

### Rigid regularization for partial scans

<!--
In the context of scanning, we usually have surfaces that only partially match.
For example, suppose we have a complete surface $Y$ and we would like to align
a new scan of some smaller part of this surface $X$. We know that all points on
$X$ should have a corresponding match on $Y$, but not vice-versa. In this case,
only the directed energy $E_{\overrightarrow{C}}(T(X),Y)$ is appropriate. 

> We will assume (for now) that there are not also parts on $X$ without a match
> on $Y$.
-->

We can avoid the trivial solutions by constraining $T$ to be a specific type of
transformation. If $X$ and $Y$ are both from trustworthy scans of the same
[rigid object](https://en.wikipedia.org/wiki/Rigid_body), then the
transformation $T$ aligning $X$ to $Y$ should be a _rigid_ transformation:
$T(\x) = \Rot \x + \t$ where $\Rot ∈ SO(3) ⊂ \R^{3×3}$ is a rotation matrix
(i.e., an [orthogonal matrix with determinant
1](https://en.wikipedia.org/wiki/Rotation_group_SO(3))) and $\t ∈ \R^3$ is a
translation vector:


\\[
\mathop{\text{minimize}}_{\t∈\R³,\ \Rot ∈ SO(3)} ∑_{i=1}^k ‖\Rot \x_i + \t -
\p_i‖².
\\]

> It is worth pausing to notice that we have significantly reduced the problem
> to finding a single translation $\t$ and a single rotation matrix $\R$ to
> align all $k$ sample points on $X$ to $Y$.

This is a variant of what's known as a [Procrustes
problem](https://en.wikipedia.org/wiki/Orthogonal_Procrustes_problem), named
after a [mythical psychopath](https://en.wikipedia.org/wiki/Procrustes) who
would kidnap people and force them to fit in his bed by stretching them or
cutting off their legs. In our case, we are forcing $\Rot$ to be perfectly
orthogonal (no "longer", no "shorter).

Solving this kind of problem relies on some linear algebra kung fu, but the
solution will be beautifully simple.

This energy is _quadratic_ in $\t$ and there are no other constraints on
$\t$. We can immediately solve for the optimal $\t^*$---leaving $\Rot$ as an unknown---by
setting all derivatives with respect to unknowns in $\t$ to zero:

\\[
\begin{align}
\t^*
  &= \argmin_{\t} ∑_{i=1}^k ‖\Rot \x_i + \t - \p_i‖²  \\\\
  &= \argmin_\t \left\|\Rot \X^\transpose + \t \One^\transpose - \P^\transpose\right\|^2_F,
\end{align}
\\]
where $\One ∈ \R^{k}$ is a vector ones. Setting the partial derivative with
respect to $\t$ of this
quadratic energy to zero finds the minimum:
\\[
\begin{align}
0 
  &= \frac{∂}{∂\t} \left\|\Rot \X^\transpose + \t \One^\transpose - \P^\transpose\right\|^2_F \\\\
  &= \One^\transpose \One \t + \Rot \X^\transpose \One - \P^\transpose \One,
\end{align}
\\]

Rearranging terms above reveals that the optimal $\t$ is the vector aligning
the [centroids](https://en.wikipedia.org/wiki/Centroid) of the points in $\P$
and the points in $\X$ rotated by the---yet-unknown---$\Rot$. Introducing
variables for the respective centroids $\hat{\p} = \tfrac{1}{k} ∑_{i=1}^k
\p_i$ and $\hat{\x} = \tfrac{1}{k} ∑_{i=1}^k \x_i$, we can write the
formula for the optimal  $\t$:

\\[
\begin{align}
\t 
  &= \frac{\P^\transpose \One - \Rot \X^\transpose \One}{ \One^\transpose \One} \\\\
  &= \hat{\p} - \Rot \hat{\x}.
\end{align}
\\]

Now we have a formula for the optimal translation vector $\t$ in terms of the
unknown rotation $\R$. Let us
[substitute](https://en.wikipedia.org/wiki/Substitution_(algebra)) this formula
for all occurrences of $\t$ in our energy written in its original summation
form:

\\[
\mathop{\text{minimize}}_{\Rot ∈ SO(3)}  ∑\limits_{i=1}^k \left\| \Rot \x_i + ( \hat{\p} - \Rot\hat{\x}) - \p_i \right\|^2 \\\
\mathop{\text{minimize}}_{\Rot ∈ SO(3)}  ∑\limits_{i=1}^k \left\| \Rot (\x_i - \hat{\x}) - (\p_i - \hat{\p}) \right\|^2 \\\\
\mathop{\text{minimize}}_{\Rot ∈ SO(3)}  ∑\limits_{i=1}^k \left\| \Rot \overline{\x}_i - \overline{\p}_i \right\|^2 \\\\
\mathop{\text{minimize}}_{\Rot ∈ SO(3)}  \left\| \Rot \overline{\X}^\transpose - \overline{\P}^\transpose \right\|_F^2,
\\]

where we introduce $\overline{\X} ∈ \R^{k × 3}$ where the ith row contains the
_relative position_ of the ith point to the centroid $\hat{\x}$: i.e.,
$\overline{\x}_i = (\x_i - \hat{\x})$ (and analagously for $\overline{\P}$).

Now we have the canonical form of the [orthogonal procrustes
problem](https://en.wikipedia.org/wiki/Orthogonal_Procrustes_problem). To
find the optimal rotation matrix $\Rot^*$ we will massage the terms in the
_minimization_ until we have a _maximization_ problem involving the [Frobenius
inner-product](https://en.wikipedia.org/wiki/Frobenius_inner_product) of the
unknown rotation $\Rot$ and [covariance
matrix](https://en.wikipedia.org/wiki/Covariance_matrix) of $\X$ and $\P$:

\\[
\begin{align}
\Rot^* 
&= \argmin_{\Rot ∈ SO(3)} \left\| \Rot \overline{\X}^\transpose - \overline{\P}^\transpose \right\|_F^2 \\\\
&= \argmin_{\Rot ∈ SO(3)} \left<\Rot \overline{\X}^\transpose - \overline{\P}^\transpose , \Rot \overline{\X}^\transpose - \overline{\P}^\transpose \right>_F\\\\
&= \argmin_{\Rot ∈ SO(3)} \left\| \overline{\X} \right\|_F^2 + \left\| \overline{\P} \right\|_F^2 - 2 \left<\Rot \overline{\X}^\transpose , \overline{\P}^\transpose \right>_F\\\\
&= \argmax_{\Rot ∈ SO(3)} \left<\Rot,\overline{\P}\,\overline{\X}^\transpose\right>_F\\\\
\end{align}
\\]

We now take advantage of the [singular value
decomposition](https://en.wikipedia.org/wiki/Singular_value_decomposition) of
$\overline{\X}^\transpose \overline{P} = \U Σ \V^\transpose$, where $\U, \V ∈
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
on the SVD on $\overline{\X}^\transpose \overline{P}$. This ensures that as a
result $\R^*$ will have determinant 1. The optimal choice of $Ω$ is to set all
values to zero except on the diagonal, where we place all 1s except the bottom
right corner (corresponding to the smallest singular value in $Σ$) which is set
to $\det{\U\V^\transpose}$:

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

<!--
### Pseudocode of Rigid ICP algorithm

We are now prepared to describe an algorithm for finding the best rigid
transformation that matches a triangle mesh $X$ representing a partial scan to
a triangle mesh $Y$ representing a complete scan.


### point to plane

### point to plane

## Best fitting transformation

### Translation

### Affine

### Rigid
-->


## Sampling meshes

### `Uniform random sampling on a triangle mesh`
### `point to triangle distance`
### `point to mesh distance`
### `Lower bound on Hausdorff distance between two meshes`


Rather
than [registering multiple point
clouds](https://en.wikipedia.org/wiki/Point_set_registration), we will register
multiple triangle mesh surfaces. 


- how well does _this_ surface match _that_ surface?
  - for partially overlapping surfaces, use this measure as "score" for
    alignment/registration
- abstract distance
  - distance between points 
  - difference between "rest shape" S and "deformed shape" f(S)
    - l2 norm
    - l1 norm
  - implicitly assumes matching parameterization
    - can write as integral over same (u,v)
- Hausdorff distance
  - mathematically elegant measure but infeasible matching energy?
    - Difficult to fool: hard to get very small Hausdorff while being poorly
      aligned
  - perhaps too idealized
    - not robust to outliers/noise
    - hard to use for partial overlaps
- Integrated projection matching energy 
  - closest point projection onto triangle mesh
  - point to point error metric
  - point to plane error metric
- Rigid transformations: Procrustes
- uniform random sampling
- bounding volume hierarchies
  - give this for free?
- In practice life is not so rigid
  - allow non-rigid, need to regularize to tame subspace
