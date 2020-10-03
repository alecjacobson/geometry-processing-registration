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
complete scan of the surface <img src="./tex/91aac9730317276af725abd8cef04ca9.svg?invert_in_darkmode" align=middle width=13.19638649999999pt height=22.465723500000017pt/> of some [rigid
object](https://en.wikipedia.org/wiki/Rigid_body) and a new partial scan of
that surface <img src="./tex/cbfb1b2a33b28eab8a3e59464768e810.svg?invert_in_darkmode" align=middle width=14.908688849999992pt height=22.465723500000017pt/>.

![Example input: a partial scan mesh surface <img src="./tex/cbfb1b2a33b28eab8a3e59464768e810.svg?invert_in_darkmode" align=middle width=14.908688849999992pt height=22.465723500000017pt/> is misaligned with the 
mesh of the complete surface <img src="./tex/91aac9730317276af725abd8cef04ca9.svg?invert_in_darkmode" align=middle width=13.19638649999999pt height=22.465723500000017pt/>](images/max-inputs.jpg)

These meshes will not have the same number of vertices or the even the same
topology. We will first explore different ways to _measure_ how well aligned
two surfaces are and then how to optimize the _rigid_ alignment of the partial
surface <img src="./tex/cbfb1b2a33b28eab8a3e59464768e810.svg?invert_in_darkmode" align=middle width=14.908688849999992pt height=22.465723500000017pt/> to the complete surface <img src="./tex/91aac9730317276af725abd8cef04ca9.svg?invert_in_darkmode" align=middle width=13.19638649999999pt height=22.465723500000017pt/>.

## Hausdorff distance

We would like to compute a single scalar number that measures how poorly two
surfaces are matched. In other words, we would like to measure the _distance_
between two surfaces. Let's start by reviewing more familiar distances:

#### Point-to-point distance
The usually Euclidean distance between _two points_ <img src="./tex/b0ea07dc5c00127344a1cad40467b8de.svg?invert_in_darkmode" align=middle width=9.97711604999999pt height=14.611878600000017pt/> and <img src="./tex/1da18d2de6d16a18e780cd6c435a2936.svg?invert_in_darkmode" align=middle width=10.239687149999991pt height=14.611878600000017pt/> is the <img src="./tex/1ba11678e565f04e2212a9d8c00c72dc.svg?invert_in_darkmode" align=middle width=17.73978854999999pt height=26.76175259999998pt/>
norm of their difference :

<p align="center"><img src="./tex/b1ccd123952eb3057907ed4fe6d2f976.svg?invert_in_darkmode" align=middle width=132.09434535pt height=16.438356pt/></p>


#### Point-to-projection distance

When we consider the distance between a point <img src="./tex/b0ea07dc5c00127344a1cad40467b8de.svg?invert_in_darkmode" align=middle width=9.97711604999999pt height=14.611878600000017pt/> and some _larger_ object <img src="./tex/91aac9730317276af725abd8cef04ca9.svg?invert_in_darkmode" align=middle width=13.19638649999999pt height=22.465723500000017pt/> (a line,
a circle, a surface), the natural extension is to take the distance to the
closest point <img src="./tex/1da18d2de6d16a18e780cd6c435a2936.svg?invert_in_darkmode" align=middle width=10.239687149999991pt height=14.611878600000017pt/> on <img src="./tex/91aac9730317276af725abd8cef04ca9.svg?invert_in_darkmode" align=middle width=13.19638649999999pt height=22.465723500000017pt/>:

<p align="center"><img src="./tex/54367d492546be6175a0bcae2a535657.svg?invert_in_darkmode" align=middle width=157.38562785pt height=25.168946549999998pt/></p>


written in this way the
[infimum](https://en.wikipedia.org/wiki/Infimum_and_supremum) considers all
possible points <img src="./tex/1da18d2de6d16a18e780cd6c435a2936.svg?invert_in_darkmode" align=middle width=10.239687149999991pt height=14.611878600000017pt/> and keeps the minimum distance. We may equivalently write
this distance instead as simply the point-to-point distance between <img src="./tex/b0ea07dc5c00127344a1cad40467b8de.svg?invert_in_darkmode" align=middle width=9.97711604999999pt height=14.611878600000017pt/> and
the _closest-point projection_ <img src="./tex/43d1b46893b3e57ac2d78fc6241da8ef.svg?invert_in_darkmode" align=middle width=44.696402849999984pt height=24.65753399999998pt/>:

<p align="center"><img src="./tex/43bb920cf95f6cae8a04f17fe8dd7071.svg?invert_in_darkmode" align=middle width=281.13890475pt height=16.438356pt/></p>


If <img src="./tex/91aac9730317276af725abd8cef04ca9.svg?invert_in_darkmode" align=middle width=13.19638649999999pt height=22.465723500000017pt/> is a smooth surface, this projection will also be an [orthogonal
projection](https://en.wikipedia.org/wiki/Projection_(linear_algebra)#Orthogonal_projections).


![The distance between a surface <img src="./tex/91aac9730317276af725abd8cef04ca9.svg?invert_in_darkmode" align=middle width=13.19638649999999pt height=22.465723500000017pt/> (light blue) and a point <img src="./tex/b0ea07dc5c00127344a1cad40467b8de.svg?invert_in_darkmode" align=middle width=9.97711604999999pt height=14.611878600000017pt/> (orange) is
determined by the closest point <img src="./tex/43d1b46893b3e57ac2d78fc6241da8ef.svg?invert_in_darkmode" align=middle width=44.696402849999984pt height=24.65753399999998pt/> (blue)](images/max-point-mesh.gif)

### Directed Hausdorff distance

We might be tempted to define the distance from surface <img src="./tex/cbfb1b2a33b28eab8a3e59464768e810.svg?invert_in_darkmode" align=middle width=14.908688849999992pt height=22.465723500000017pt/> to <img src="./tex/91aac9730317276af725abd8cef04ca9.svg?invert_in_darkmode" align=middle width=13.19638649999999pt height=22.465723500000017pt/> as the
_infimum_ of _point-to-projection_ distances over all points <img src="./tex/b0ea07dc5c00127344a1cad40467b8de.svg?invert_in_darkmode" align=middle width=9.97711604999999pt height=14.611878600000017pt/> on <img src="./tex/cbfb1b2a33b28eab8a3e59464768e810.svg?invert_in_darkmode" align=middle width=14.908688849999992pt height=22.465723500000017pt/>:

<p align="center"><img src="./tex/91f6adf2b92c5fb76354ca7fcb38f619.svg?invert_in_darkmode" align=middle width=225.58776239999997pt height=23.802282899999998pt/></p>


but this will not be useful for registering two surfaces: it will measure zero
if even just a single point of <img src="./tex/b0ea07dc5c00127344a1cad40467b8de.svg?invert_in_darkmode" align=middle width=9.97711604999999pt height=14.611878600000017pt/> happens to lie on <img src="./tex/91aac9730317276af725abd8cef04ca9.svg?invert_in_darkmode" align=middle width=13.19638649999999pt height=22.465723500000017pt/>. Imagine the noses of
two faces touching at their tips.

Instead, we should take the _supremum_ of _point-to-projection_ distances over
all points <img src="./tex/b0ea07dc5c00127344a1cad40467b8de.svg?invert_in_darkmode" align=middle width=9.97711604999999pt height=14.611878600000017pt/> on <img src="./tex/cbfb1b2a33b28eab8a3e59464768e810.svg?invert_in_darkmode" align=middle width=14.908688849999992pt height=22.465723500000017pt/>:

<p align="center"><img src="./tex/fdb18156461a3ad105287867a0dbdfa7.svg?invert_in_darkmode" align=middle width=223.43276174999997pt height=26.998626599999998pt/></p>


This surface-to-surface distance measure is called the _directed_ [Hausdorff
distance](https://en.wikipedia.org/wiki/Hausdorff_distance). We may interpret
this as taking the worst of the best: we 
let each point <img src="./tex/b0ea07dc5c00127344a1cad40467b8de.svg?invert_in_darkmode" align=middle width=9.97711604999999pt height=14.611878600000017pt/> on <img src="./tex/cbfb1b2a33b28eab8a3e59464768e810.svg?invert_in_darkmode" align=middle width=14.908688849999992pt height=22.465723500000017pt/> declare its shortest distance to <img src="./tex/91aac9730317276af725abd8cef04ca9.svg?invert_in_darkmode" align=middle width=13.19638649999999pt height=22.465723500000017pt/> and then keep
the longest of those.

![The directed Hausdorff distance between from surface <img src="./tex/cbfb1b2a33b28eab8a3e59464768e810.svg?invert_in_darkmode" align=middle width=14.908688849999992pt height=22.465723500000017pt/> (light orange) to
another surface <img src="./tex/91aac9730317276af725abd8cef04ca9.svg?invert_in_darkmode" align=middle width=13.19638649999999pt height=22.465723500000017pt/> (light blue) is determined by the point on <img src="./tex/cbfb1b2a33b28eab8a3e59464768e810.svg?invert_in_darkmode" align=middle width=14.908688849999992pt height=22.465723500000017pt/> (orange)
whose closest point on <img src="./tex/91aac9730317276af725abd8cef04ca9.svg?invert_in_darkmode" align=middle width=13.19638649999999pt height=22.465723500000017pt/> (blue) is the farthest
away.](images/max-point-mesh-farthest.jpg)

It is easy to verify that <img src="./tex/cf08f61d7e71426b649e9e173e212be8.svg?invert_in_darkmode" align=middle width=26.51178254999999pt height=22.465723500000017pt/> will only equal zero if all
points on <img src="./tex/cbfb1b2a33b28eab8a3e59464768e810.svg?invert_in_darkmode" align=middle width=14.908688849999992pt height=22.465723500000017pt/> also lie exactly on <img src="./tex/91aac9730317276af725abd8cef04ca9.svg?invert_in_darkmode" align=middle width=13.19638649999999pt height=22.465723500000017pt/>. 

The converse is not true: if <img src="./tex/2f4e6856946394a0e03f4928feceaf72.svg?invert_in_darkmode" align=middle width=57.47051639999999pt height=22.465723500000017pt/> there may still be
points on <img src="./tex/91aac9730317276af725abd8cef04ca9.svg?invert_in_darkmode" align=middle width=13.19638649999999pt height=22.465723500000017pt/> that do not lie on <img src="./tex/cbfb1b2a33b28eab8a3e59464768e810.svg?invert_in_darkmode" align=middle width=14.908688849999992pt height=22.465723500000017pt/>. In other words, _in general_ the directed
Hausdorff distance from surface <img src="./tex/cbfb1b2a33b28eab8a3e59464768e810.svg?invert_in_darkmode" align=middle width=14.908688849999992pt height=22.465723500000017pt/> to surface <img src="./tex/91aac9730317276af725abd8cef04ca9.svg?invert_in_darkmode" align=middle width=13.19638649999999pt height=22.465723500000017pt/> will not equal the Hausdorff
distance from surface <img src="./tex/91aac9730317276af725abd8cef04ca9.svg?invert_in_darkmode" align=middle width=13.19638649999999pt height=22.465723500000017pt/> to surface <img src="./tex/cbfb1b2a33b28eab8a3e59464768e810.svg?invert_in_darkmode" align=middle width=14.908688849999992pt height=22.465723500000017pt/>:

<p align="center"><img src="./tex/0f8695adcd207a50844c3997aa6dc4cb.svg?invert_in_darkmode" align=middle width=173.89095569999998pt height=18.949258349999997pt/></p>


#### directed Hausdorff distance between triangle meshes

We can approximate a _lower bound_ on the Hausdorff distance between two meshes
by densely sampling surfaces <img src="./tex/cbfb1b2a33b28eab8a3e59464768e810.svg?invert_in_darkmode" align=middle width=14.908688849999992pt height=22.465723500000017pt/> and <img src="./tex/91aac9730317276af725abd8cef04ca9.svg?invert_in_darkmode" align=middle width=13.19638649999999pt height=22.465723500000017pt/>. We will discuss sampling methods,
later. For now consider that we have chosen a set <img src="./tex/cf9cc8738a6712e4c9ad70500cc2115a.svg?invert_in_darkmode" align=middle width=24.59703509999999pt height=22.55708729999998pt/> of <img src="./tex/63bb9849783d01d91403bc9a5fea12a2.svg?invert_in_darkmode" align=middle width=9.075367949999992pt height=22.831056599999986pt/> points on <img src="./tex/cbfb1b2a33b28eab8a3e59464768e810.svg?invert_in_darkmode" align=middle width=14.908688849999992pt height=22.465723500000017pt/>
(each point might lie at a vertex, along an edge, or inside a triangle). The
directed Hausdorff distance from <img src="./tex/cbfb1b2a33b28eab8a3e59464768e810.svg?invert_in_darkmode" align=middle width=14.908688849999992pt height=22.465723500000017pt/> to another triangle mesh <img src="./tex/91aac9730317276af725abd8cef04ca9.svg?invert_in_darkmode" align=middle width=13.19638649999999pt height=22.465723500000017pt/> must be
_greater_ than the directed Hausdorff distance from this [point
cloud](https://en.wikipedia.org/wiki/Point_cloud) <img src="./tex/cf9cc8738a6712e4c9ad70500cc2115a.svg?invert_in_darkmode" align=middle width=24.59703509999999pt height=22.55708729999998pt/> to <img src="./tex/91aac9730317276af725abd8cef04ca9.svg?invert_in_darkmode" align=middle width=13.19638649999999pt height=22.465723500000017pt/>:

<p align="center"><img src="./tex/9624b0223b4ac8537bc0acf869131f90.svg?invert_in_darkmode" align=middle width=345.59090115pt height=28.70995545pt/></p>


where we should be careful to ensure that the projection <img src="./tex/aad6ea016a5322e977ce3e691ab982d0.svg?invert_in_darkmode" align=middle width=50.69431124999999pt height=24.65753399999998pt/> of the
point <img src="./tex/f13e5bc0860402c82f869bcf883eb8b0.svg?invert_in_darkmode" align=middle width=15.15312644999999pt height=14.611878600000017pt/> onto the triangle mesh <img src="./tex/91aac9730317276af725abd8cef04ca9.svg?invert_in_darkmode" align=middle width=13.19638649999999pt height=22.465723500000017pt/> might lie at a vertex, along an edge or
inside a triangle. 

As our sampling <img src="./tex/cf9cc8738a6712e4c9ad70500cc2115a.svg?invert_in_darkmode" align=middle width=24.59703509999999pt height=22.55708729999998pt/> becomes denser and denser on <img src="./tex/cbfb1b2a33b28eab8a3e59464768e810.svg?invert_in_darkmode" align=middle width=14.908688849999992pt height=22.465723500000017pt/> this lower bound will
approach the true directed Hausdorff distance. Unfortunately, an efficient
_upper bound_ is significantly more difficult to design.

#### Hausdorff distance for alignment optimization

Even if it _were_ cheap to compute, Hausdorff distance is difficult to
_optimize_ when aligning two surfaces. If we treat the Hausdorff distance
between surfaces <img src="./tex/cbfb1b2a33b28eab8a3e59464768e810.svg?invert_in_darkmode" align=middle width=14.908688849999992pt height=22.465723500000017pt/> and <img src="./tex/91aac9730317276af725abd8cef04ca9.svg?invert_in_darkmode" align=middle width=13.19638649999999pt height=22.465723500000017pt/> as an energy to be minimized, then only change to
the surfaces that will decrease the energy will be moving the (in general)
isolated point on <img src="./tex/cbfb1b2a33b28eab8a3e59464768e810.svg?invert_in_darkmode" align=middle width=14.908688849999992pt height=22.465723500000017pt/> and isolated point on <img src="./tex/91aac9730317276af725abd8cef04ca9.svg?invert_in_darkmode" align=middle width=13.19638649999999pt height=22.465723500000017pt/> generating the maximum-minimum
distance. In effect, the rest of the surface does not even matter or effect the
Hausdorff distance. This, or any type of <img src="./tex/0bb003a2b7ef9dddb4804e47433f86e1.svg?invert_in_darkmode" align=middle width=24.292324649999987pt height=22.465723500000017pt/> norm, will be much more
difficult to optimize.

Hausdorff distance can serve as a validation measure, while we turn to <img src="./tex/1ba11678e565f04e2212a9d8c00c72dc.svg?invert_in_darkmode" align=middle width=17.73978854999999pt height=26.76175259999998pt/>
norms for optimization.

## Integrated closest-point distance

We would like a distance measure between two surfaces that---like Hausdorff
distance---does not require a shared parameterization. Unlike Hausdorff
distance, we would like this distance to _diffuse_ the measurement over the
entire surfaces rather than generate it from the sole _worst offender_. We can
accomplish this by replacing the _supremum_ in the Hausdorff distance (<img src="./tex/0bb003a2b7ef9dddb4804e47433f86e1.svg?invert_in_darkmode" align=middle width=24.292324649999987pt height=22.465723500000017pt/>)
with a integral of squared distances (<img src="./tex/1ba11678e565f04e2212a9d8c00c72dc.svg?invert_in_darkmode" align=middle width=17.73978854999999pt height=26.76175259999998pt/>). Let us first define a directed
_closest-point distance_ from  a surface <img src="./tex/cbfb1b2a33b28eab8a3e59464768e810.svg?invert_in_darkmode" align=middle width=14.908688849999992pt height=22.465723500000017pt/> to another surface <img src="./tex/91aac9730317276af725abd8cef04ca9.svg?invert_in_darkmode" align=middle width=13.19638649999999pt height=22.465723500000017pt/>, as the
integral of the squared distance from every point <img src="./tex/b0ea07dc5c00127344a1cad40467b8de.svg?invert_in_darkmode" align=middle width=9.97711604999999pt height=14.611878600000017pt/> on <img src="./tex/cbfb1b2a33b28eab8a3e59464768e810.svg?invert_in_darkmode" align=middle width=14.908688849999992pt height=22.465723500000017pt/> to its
closest-point projection <img src="./tex/43d1b46893b3e57ac2d78fc6241da8ef.svg?invert_in_darkmode" align=middle width=44.696402849999984pt height=24.65753399999998pt/> on the surfaces <img src="./tex/91aac9730317276af725abd8cef04ca9.svg?invert_in_darkmode" align=middle width=13.19638649999999pt height=22.465723500000017pt/>:

<p align="center"><img src="./tex/df955f1e03403cc6a3aa7800a7905b51.svg?invert_in_darkmode" align=middle width=279.08912295pt height=59.17867724999999pt/></p>


This distance will only be zero if all points on <img src="./tex/cbfb1b2a33b28eab8a3e59464768e810.svg?invert_in_darkmode" align=middle width=14.908688849999992pt height=22.465723500000017pt/> also lie on <img src="./tex/91aac9730317276af725abd8cef04ca9.svg?invert_in_darkmode" align=middle width=13.19638649999999pt height=22.465723500000017pt/>, but when
it is non-zero it is summing/averaging/diffusing the distance measures of all
of the points.

This distance is suitable to define a matching energy, but is not necessarily
welcoming for optimization: the function inside the square is non-linear. Let's
dig into it a bit. We'll define a directed _matching energy_
<img src="./tex/bb8adc1062948e6e5f2a70f3b24baf46.svg?invert_in_darkmode" align=middle width=70.63052039999998pt height=24.65753399999998pt/> from <img src="./tex/5b51bd2e6f329245d425b8002d7cf942.svg?invert_in_darkmode" align=middle width=12.397274999999992pt height=22.465723500000017pt/> to <img src="./tex/91aac9730317276af725abd8cef04ca9.svg?invert_in_darkmode" align=middle width=13.19638649999999pt height=22.465723500000017pt/> to be the squared directed
closest point distance from <img src="./tex/cbfb1b2a33b28eab8a3e59464768e810.svg?invert_in_darkmode" align=middle width=14.908688849999992pt height=22.465723500000017pt/> to <img src="./tex/91aac9730317276af725abd8cef04ca9.svg?invert_in_darkmode" align=middle width=13.19638649999999pt height=22.465723500000017pt/>:

<p align="center"><img src="./tex/617004ce27ea74300b489851cf3e63ed.svg?invert_in_darkmode" align=middle width=381.20427179999996pt height=48.00358365pt/></p>


where we introduce the proximity function <img src="./tex/f59d9f8399f137ba98915eb29b88acac.svg?invert_in_darkmode" align=middle width=94.09681709999998pt height=26.76175259999998pt/> defined simply as the
vector from a point <img src="./tex/da278ee0789447cfaae0380d4cda2fdb.svg?invert_in_darkmode" align=middle width=8.40178184999999pt height=14.611878600000017pt/> to its closest-point projection onto <img src="./tex/91aac9730317276af725abd8cef04ca9.svg?invert_in_darkmode" align=middle width=13.19638649999999pt height=22.465723500000017pt/>:

<p align="center"><img src="./tex/d1a6061378d42b58c37bbd663014a755.svg?invert_in_darkmode" align=middle width=126.8535378pt height=16.438356pt/></p>


Suppose <img src="./tex/91aac9730317276af725abd8cef04ca9.svg?invert_in_darkmode" align=middle width=13.19638649999999pt height=22.465723500000017pt/> was not a surface, but just a single point <img src="./tex/795b90e510e7828e3c6947201bc71afe.svg?invert_in_darkmode" align=middle width=61.79211389999999pt height=24.65753399999998pt/>. In this
case, <img src="./tex/feee31a378f76581ef1432d9329ee6d4.svg?invert_in_darkmode" align=middle width=89.40593309999998pt height=24.65753399999998pt/> is clearly linear in <img src="./tex/da278ee0789447cfaae0380d4cda2fdb.svg?invert_in_darkmode" align=middle width=8.40178184999999pt height=14.611878600000017pt/>.

Similarly, suppose <img src="./tex/91aac9730317276af725abd8cef04ca9.svg?invert_in_darkmode" align=middle width=13.19638649999999pt height=22.465723500000017pt/> was an [infinite
plane](https://en.wikipedia.org/wiki/Plane_(geometry)) <img src="./tex/6e2e89421be85b6f94a9d30106fbe831.svg?invert_in_darkmode" align=middle width=172.48792275pt height=24.65753399999998pt/> defined by some point <img src="./tex/980fcd4213d7b5d2ffcc82ec78c27ead.svg?invert_in_darkmode" align=middle width=10.502226899999991pt height=14.611878600000017pt/> on the plane and the plane's unit normal vector
<img src="./tex/b56595d2a30a0af329086562ca12d521.svg?invert_in_darkmode" align=middle width=10.502226899999991pt height=14.611878600000017pt/>. Then <img src="./tex/c08ae17a8da032e881964bc610bfcb58.svg?invert_in_darkmode" align=middle width=154.50849315pt height=24.65753399999998pt/> is also linear in <img src="./tex/da278ee0789447cfaae0380d4cda2fdb.svg?invert_in_darkmode" align=middle width=8.40178184999999pt height=14.611878600000017pt/>.

But in general, if <img src="./tex/91aac9730317276af725abd8cef04ca9.svg?invert_in_darkmode" align=middle width=13.19638649999999pt height=22.465723500000017pt/> is an interesting surface <img src="./tex/5c8431f6e02c7487c48b23b4d4d72b11.svg?invert_in_darkmode" align=middle width=28.75564229999999pt height=24.65753399999998pt/> will be non-linear; it
might not even be a continuous function.

![](images/closest-point-discontinuous.png)

In optimization, a common successful strategy to minimize energies composed of
squaring a non-linear functions <img src="./tex/47b0192f8f0819d64bce3612c46d15ea.svg?invert_in_darkmode" align=middle width=7.56844769999999pt height=22.831056599999986pt/> is to
[linearize](https://en.wikipedia.org/wiki/Linearization) the function about a
current input value (i.e., a current guess <img src="./tex/068e9f4c48765b7e6082c36543e13d10.svg?invert_in_darkmode" align=middle width=14.95432949999999pt height=14.611878600000017pt/>), minimize the energy built
from this linearization, then re-linearize around that solution, and then
repeat. 

This is the core idea behind [gradient
descent](https://en.wikipedia.org/wiki/Gradient_descent) and the
[Gauss-Newton](https://en.wikipedia.org/wiki/Gauss–Newton_algorithm) methods:

```
minimize f(z)^{2}
  z_{0} \Leftarrow  initial guess
  repeat until convergence
    f_{0} \Leftarrow  linearize f(z) around z_{0}
    z_{0} \Leftarrow  minimize f_{0}(z)^{2}
```

Since our <img src="./tex/47b0192f8f0819d64bce3612c46d15ea.svg?invert_in_darkmode" align=middle width=7.56844769999999pt height=22.831056599999986pt/> is a geometric function, we can derive its linearizations
_geometrically_.

### Constant function approximation

If we make the convenient---however unrealistic---assumption that in the
neighborhood of the closest-point projection <img src="./tex/5ea090d44e44319896420959310f0012.svg?invert_in_darkmode" align=middle width=50.49552749999999pt height=24.65753399999998pt/> of the current guess
<img src="./tex/068e9f4c48765b7e6082c36543e13d10.svg?invert_in_darkmode" align=middle width=14.95432949999999pt height=14.611878600000017pt/> the surface <img src="./tex/91aac9730317276af725abd8cef04ca9.svg?invert_in_darkmode" align=middle width=13.19638649999999pt height=22.465723500000017pt/> is simply the point <img src="./tex/5ea090d44e44319896420959310f0012.svg?invert_in_darkmode" align=middle width=50.49552749999999pt height=24.65753399999998pt/> (perhaps imagine that <img src="./tex/91aac9730317276af725abd8cef04ca9.svg?invert_in_darkmode" align=middle width=13.19638649999999pt height=22.465723500000017pt/>
is makes a sharp needle-like point at <img src="./tex/5ea090d44e44319896420959310f0012.svg?invert_in_darkmode" align=middle width=50.49552749999999pt height=24.65753399999998pt/> or that <img src="./tex/91aac9730317276af725abd8cef04ca9.svg?invert_in_darkmode" align=middle width=13.19638649999999pt height=22.465723500000017pt/> is very far away
from <img src="./tex/b0ea07dc5c00127344a1cad40467b8de.svg?invert_in_darkmode" align=middle width=9.97711604999999pt height=14.611878600000017pt/>), then we can approximate <img src="./tex/5c8431f6e02c7487c48b23b4d4d72b11.svg?invert_in_darkmode" align=middle width=28.75564229999999pt height=24.65753399999998pt/> in the proximity of our current
guess <img src="./tex/068e9f4c48765b7e6082c36543e13d10.svg?invert_in_darkmode" align=middle width=14.95432949999999pt height=14.611878600000017pt/> as the vector between the input point <img src="./tex/da278ee0789447cfaae0380d4cda2fdb.svg?invert_in_darkmode" align=middle width=8.40178184999999pt height=14.611878600000017pt/> and <img src="./tex/5ea090d44e44319896420959310f0012.svg?invert_in_darkmode" align=middle width=50.49552749999999pt height=24.65753399999998pt/>:

<p align="center"><img src="./tex/d300e867569228e54c52f050c2d3d802.svg?invert_in_darkmode" align=middle width=209.29638509999998pt height=17.031940199999998pt/></p>


In effect, we are assuming that the surface <img src="./tex/91aac9730317276af725abd8cef04ca9.svg?invert_in_darkmode" align=middle width=13.19638649999999pt height=22.465723500000017pt/> is _constant_ function of its
parameterization: <img src="./tex/4cb0776dc727de5904c46747e53cc2d8.svg?invert_in_darkmode" align=middle width=120.71227409999999pt height=24.65753399999998pt/>.

Minimizing <img src="./tex/ef03e3adb9f682700bd731e4c2ebeb55.svg?invert_in_darkmode" align=middle width=25.03690529999999pt height=22.465723500000017pt/> iteratively using this linearization (or
rather _constantization_) of <img src="./tex/47b0192f8f0819d64bce3612c46d15ea.svg?invert_in_darkmode" align=middle width=7.56844769999999pt height=22.831056599999986pt/> is equivalent to the [gradient
descent](https://en.wikipedia.org/wiki/Gradient_descent). We have simply
derived our gradients geometrically.

### Linear function approximation

If we make make a slightly more appropriate assumption that in the neighborhood
of the  <img src="./tex/5ea090d44e44319896420959310f0012.svg?invert_in_darkmode" align=middle width=50.49552749999999pt height=24.65753399999998pt/> the surface <img src="./tex/91aac9730317276af725abd8cef04ca9.svg?invert_in_darkmode" align=middle width=13.19638649999999pt height=22.465723500000017pt/> is a plane, then we can improve this
approximation while keeping <img src="./tex/47b0192f8f0819d64bce3612c46d15ea.svg?invert_in_darkmode" align=middle width=7.56844769999999pt height=22.831056599999986pt/> linear in <img src="./tex/da278ee0789447cfaae0380d4cda2fdb.svg?invert_in_darkmode" align=middle width=8.40178184999999pt height=14.611878600000017pt/>:

<p align="center"><img src="./tex/b27a9e33eecb71a0e19921f9e61a1151.svg?invert_in_darkmode" align=middle width=273.01770374999995pt height=17.031940199999998pt/></p>


where the plane that _best_ approximates <img src="./tex/91aac9730317276af725abd8cef04ca9.svg?invert_in_darkmode" align=middle width=13.19638649999999pt height=22.465723500000017pt/> locally near <img src="./tex/5ea090d44e44319896420959310f0012.svg?invert_in_darkmode" align=middle width=50.49552749999999pt height=24.65753399999998pt/> is the
[tangent plane](https://en.wikipedia.org/wiki/Tangent_space) defined by the
[normal vector](https://en.wikipedia.org/wiki/Normal_(geometry)) <img src="./tex/b56595d2a30a0af329086562ca12d521.svg?invert_in_darkmode" align=middle width=10.502226899999991pt height=14.611878600000017pt/> at
<img src="./tex/5ea090d44e44319896420959310f0012.svg?invert_in_darkmode" align=middle width=50.49552749999999pt height=24.65753399999998pt/>.


Minimizing <img src="./tex/ef03e3adb9f682700bd731e4c2ebeb55.svg?invert_in_darkmode" align=middle width=25.03690529999999pt height=22.465723500000017pt/> iteratively using this linearization of
<img src="./tex/47b0192f8f0819d64bce3612c46d15ea.svg?invert_in_darkmode" align=middle width=7.56844769999999pt height=22.831056599999986pt/> is equivalent to the
[Gauss-Newton](https://en.wikipedia.org/wiki/Gauss–Newton_algorithm) method. We
have simply derived our linear approximation geometrically.

Equipped with these linearizations, we may now describe an [optimization
algorithm](https://en.wikipedia.org/wiki/Mathematical_optimization#Optimization_algorithms)
for minimizing the matching energy between a surface <img src="./tex/5b51bd2e6f329245d425b8002d7cf942.svg?invert_in_darkmode" align=middle width=12.397274999999992pt height=22.465723500000017pt/> and another surface
<img src="./tex/91aac9730317276af725abd8cef04ca9.svg?invert_in_darkmode" align=middle width=13.19638649999999pt height=22.465723500000017pt/>.

## Iterative closest point algorithm

So far we have derived distances between a surface <img src="./tex/5b51bd2e6f329245d425b8002d7cf942.svg?invert_in_darkmode" align=middle width=12.397274999999992pt height=22.465723500000017pt/> and another surface <img src="./tex/91aac9730317276af725abd8cef04ca9.svg?invert_in_darkmode" align=middle width=13.19638649999999pt height=22.465723500000017pt/>.
In our _rigid_ alignment and registration problem, we would like to
[transform](https://en.wikipedia.org/wiki/Transformation_(function)) one
surface <img src="./tex/cbfb1b2a33b28eab8a3e59464768e810.svg?invert_in_darkmode" align=middle width=14.908688849999992pt height=22.465723500000017pt/> into a new surface <img src="./tex/ea84abc39a2543b8f4c51e99dbeb6e75.svg?invert_in_darkmode" align=middle width=73.89831734999999pt height=24.65753399999998pt/> so that it best aligns with/matches
the other surface <img src="./tex/91aac9730317276af725abd8cef04ca9.svg?invert_in_darkmode" align=middle width=13.19638649999999pt height=22.465723500000017pt/>. Further we require that <img src="./tex/2f118ee06d05f3c2d98361d9c30e38ce.svg?invert_in_darkmode" align=middle width=11.889314249999991pt height=22.465723500000017pt/> is a rigid transformation:
<img src="./tex/12b5e1c8ae1c2f7b00f2518449b288b5.svg?invert_in_darkmode" align=middle width=108.16736369999998pt height=24.65753399999998pt/> for some rotation matrix <img src="./tex/43b9b5d6dee02a08443aa04710f133ad.svg?invert_in_darkmode" align=middle width=136.46551874999997pt height=26.76175259999998pt/>
(i.e., an [orthogonal matrix with determinant
1](https://en.wikipedia.org/wiki/Rotation_group_SO(3))) and translation vector
<img src="./tex/4835152d95e6468a4f5aaf85cd0e3fed.svg?invert_in_darkmode" align=middle width=45.86742269999999pt height=26.76175259999998pt/>.

Our matching problem can be written as an optimization problem to find the best
possible rotation <img src="./tex/6423e0d54c2545769ad013e5f6a4cf94.svg?invert_in_darkmode" align=middle width=14.17800779999999pt height=22.55708729999998pt/> and translation <img src="./tex/f40598ec49a99f9a93c399f7dacc6d3e.svg?invert_in_darkmode" align=middle width=7.35155849999999pt height=20.87411699999998pt/> that match surface <img src="./tex/cbfb1b2a33b28eab8a3e59464768e810.svg?invert_in_darkmode" align=middle width=14.908688849999992pt height=22.465723500000017pt/> to surface
<img src="./tex/91aac9730317276af725abd8cef04ca9.svg?invert_in_darkmode" align=middle width=13.19638649999999pt height=22.465723500000017pt/>:

<p align="center"><img src="./tex/fe8a23212b6b42f08a7e371f1b119348.svg?invert_in_darkmode" align=middle width=318.75751214999997pt height=48.00358365pt/></p>


Even if <img src="./tex/cbfb1b2a33b28eab8a3e59464768e810.svg?invert_in_darkmode" align=middle width=14.908688849999992pt height=22.465723500000017pt/> is a triangle mesh, it is difficult to _integrate_ over _all_
points on the surface of <img src="./tex/cbfb1b2a33b28eab8a3e59464768e810.svg?invert_in_darkmode" align=middle width=14.908688849999992pt height=22.465723500000017pt/>. _At any point_, we can approximate this energy by
_summing_ over a point-sampling of <img src="./tex/cbfb1b2a33b28eab8a3e59464768e810.svg?invert_in_darkmode" align=middle width=14.908688849999992pt height=22.465723500000017pt/>:

<p align="center"><img src="./tex/5fbacc7d6decdf1dc93d52a18071ff97.svg?invert_in_darkmode" align=middle width=323.5505328pt height=35.433915pt/></p>


where <img src="./tex/838d3d65478c3e05d640b3b2a37b5104.svg?invert_in_darkmode" align=middle width=70.34808pt height=27.91243950000002pt/> is a set of <img src="./tex/63bb9849783d01d91403bc9a5fea12a2.svg?invert_in_darkmode" align=middle width=9.075367949999992pt height=22.831056599999986pt/> points on <img src="./tex/cbfb1b2a33b28eab8a3e59464768e810.svg?invert_in_darkmode" align=middle width=14.908688849999992pt height=22.465723500000017pt/> so that each point <img src="./tex/c416d0c6d8ab37f889334e2d1a9863c3.svg?invert_in_darkmode" align=middle width=14.628015599999989pt height=14.611878600000017pt/>
might lie at a vertex, along an edge, or inside a triangle. We defer discussion
of _how_ to sample a triangle mesh surface.

### Pseudocode

As the name implies, the method proceeds by iteratively finding the closest
point on <img src="./tex/91aac9730317276af725abd8cef04ca9.svg?invert_in_darkmode" align=middle width=13.19638649999999pt height=22.465723500000017pt/> to the current rigid transformation <img src="./tex/1f0619d1c005ea7ee747c5b3738a14ca.svg?invert_in_darkmode" align=middle width=51.59787269999999pt height=22.55708729999998pt/> of each sample
point <img src="./tex/b0ea07dc5c00127344a1cad40467b8de.svg?invert_in_darkmode" align=middle width=9.97711604999999pt height=14.611878600000017pt/> in <img src="./tex/d05b996d2c08252f77613c25205a0f04.svg?invert_in_darkmode" align=middle width=14.29216634999999pt height=22.55708729999998pt/> and then minimizing the _linearized_ energy to update the
rotation <img src="./tex/6423e0d54c2545769ad013e5f6a4cf94.svg?invert_in_darkmode" align=middle width=14.17800779999999pt height=22.55708729999998pt/> and translation <img src="./tex/f40598ec49a99f9a93c399f7dacc6d3e.svg?invert_in_darkmode" align=middle width=7.35155849999999pt height=20.87411699999998pt/>. 

If <img src="./tex/264301219ded214ecf2ec9d61203a03c.svg?invert_in_darkmode" align=middle width=21.263777699999988pt height=22.465723500000017pt/> and <img src="./tex/30e02450834ec46ab5f2f42b4262acc6.svg?invert_in_darkmode" align=middle width=22.245524399999987pt height=22.465723500000017pt/> are the vertices and faces of a triangle mesh surface <img src="./tex/cbfb1b2a33b28eab8a3e59464768e810.svg?invert_in_darkmode" align=middle width=14.908688849999992pt height=22.465723500000017pt/>
(and correspondingly for <img src="./tex/91aac9730317276af725abd8cef04ca9.svg?invert_in_darkmode" align=middle width=13.19638649999999pt height=22.465723500000017pt/>), then we can summarize a generic ICP algorithm in
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

We would like to find the rotation matrix <img src="./tex/43b9b5d6dee02a08443aa04710f133ad.svg?invert_in_darkmode" align=middle width=136.46551874999997pt height=26.76175259999998pt/> and
translation vector <img src="./tex/4835152d95e6468a4f5aaf85cd0e3fed.svg?invert_in_darkmode" align=middle width=45.86742269999999pt height=26.76175259999998pt/> that _best_ aligns a given a set of points <img src="./tex/c6fb0a239d93250461ae9e9483290505.svg?invert_in_darkmode" align=middle width=70.34808pt height=27.91243950000002pt/> on the source mesh and their current closest points <img src="./tex/3f351cddeb3ad7497953e6d06635431f.svg?invert_in_darkmode" align=middle width=70.02835124999999pt height=27.91243950000002pt/>
on the target mesh. We have two choices for _linearizing_ our matching energy:
point-to-point (gradient descent) and point-to-plane (Gauss-Newton).

![ICP using the point-to-point matching energy linearization is slow to
converge.](images/max-point-to-point.gif)

![ICP using the point-to-plane matching energy linearization is
faster.](images/max-point-to-plane.gif)

In either case, this is still a non-linear optimization problem. This time due
to the [constraints](https://en.wikipedia.org/wiki/Constrained_optimization)
rather than the energy term. 

We require that <img src="./tex/6423e0d54c2545769ad013e5f6a4cf94.svg?invert_in_darkmode" align=middle width=14.17800779999999pt height=22.55708729999998pt/> is not just any 3\times 3 matrix, but a rotation matrix. We
can _linearize_ this constraint, by assuming that the rotation in <img src="./tex/6423e0d54c2545769ad013e5f6a4cf94.svg?invert_in_darkmode" align=middle width=14.17800779999999pt height=22.55708729999998pt/> will
be very small and thus well approximated by the identity matrix <img src="./tex/d8471e559d932f20f66bec32f6002e08.svg?invert_in_darkmode" align=middle width=7.168923299999991pt height=22.55708729999998pt/> plus a
skew-symmetric matrix:

<p align="center"><img src="./tex/cbb823dc659c49f9d0d2b361d0cb77d1.svg?invert_in_darkmode" align=middle width=209.96017185pt height=59.1786591pt/></p>


where we can now work directly with the three scalar unknowns <img src="./tex/dbbd12c1d7f968c7fca71ae001318ee6.svg?invert_in_darkmode" align=middle width=10.57650494999999pt height=14.15524440000002pt/>, <img src="./tex/10bc4a49c81f5dd5800ca54295c8fc4e.svg?invert_in_darkmode" align=middle width=10.16555099999999pt height=22.831056599999986pt/> and <img src="./tex/193089f7a231633473714830d2edc62a.svg?invert_in_darkmode" align=middle width=9.423880949999988pt height=14.15524440000002pt/>.

### Approximate point-to-point minimizer

If we apply our linearization of <img src="./tex/6423e0d54c2545769ad013e5f6a4cf94.svg?invert_in_darkmode" align=middle width=14.17800779999999pt height=22.55708729999998pt/> to the **point-to-point** distance
linearization of the matching energy, our minimization becomes:

<p align="center"><img src="./tex/9541ffc65ff661a30e49ce2ecd7030a1.svg?invert_in_darkmode" align=middle width=377.9018958pt height=62.53032225pt/></p>


This energy is quadratic in the translation vector <img src="./tex/f40598ec49a99f9a93c399f7dacc6d3e.svg?invert_in_darkmode" align=middle width=7.35155849999999pt height=20.87411699999998pt/> and the linearized
rotation angles <img src="./tex/dbbd12c1d7f968c7fca71ae001318ee6.svg?invert_in_darkmode" align=middle width=10.57650494999999pt height=14.15524440000002pt/>, <img src="./tex/10bc4a49c81f5dd5800ca54295c8fc4e.svg?invert_in_darkmode" align=middle width=10.16555099999999pt height=22.831056599999986pt/> and <img src="./tex/193089f7a231633473714830d2edc62a.svg?invert_in_darkmode" align=middle width=9.423880949999988pt height=14.15524440000002pt/>. Let's gather these degrees of freedom into a
vector of unknowns: <img src="./tex/949c78fb9f2f5c932b0e333014efbd8d.svg?invert_in_darkmode" align=middle width=126.75812654999999pt height=27.91243950000002pt/>. Then we can write our
problem in summation form as:

<p align="center"><img src="./tex/eaccf6a2d1872cb0bdbbaa314c6df49a.svg?invert_in_darkmode" align=middle width=500.35957289999993pt height=62.53032225pt/></p>


This can be written compactly in matrix form as:

<p align="center"><img src="./tex/9d2cc4a31a2eec7830d2ee99138f8551.svg?invert_in_darkmode" align=middle width=496.9322391pt height=112.6677849pt/></p>

where we introduce the matrix <img src="./tex/7eab9509bd92f8ab739bdfe4383e7249.svg?invert_in_darkmode" align=middle width=76.90062764999999pt height=27.91243950000002pt/> that gathers the columns
<img src="./tex/9b01119ffd35fe6d8a8795a24fc11616.svg?invert_in_darkmode" align=middle width=18.943064249999992pt height=22.55708729999998pt/> of <img src="./tex/d05b996d2c08252f77613c25205a0f04.svg?invert_in_darkmode" align=middle width=14.29216634999999pt height=22.55708729999998pt/> and columns of ones <img src="./tex/0ab021958640a132ba4077ebcae7ec95.svg?invert_in_darkmode" align=middle width=48.68135084999999pt height=27.91243950000002pt/>.

This quadratic energy is minimized with its partial derivatives with respect to
entries in <img src="./tex/129c5b884ff47d80be4d6261a476e9f1.svg?invert_in_darkmode" align=middle width=10.502226899999991pt height=14.611878600000017pt/> are all zero:

<p align="center"><img src="./tex/9fea9b76fa7c8822aeb3e9021b84420b.svg?invert_in_darkmode" align=middle width=318.86380679999996pt height=124.93263584999998pt/></p>


Solving this small 6\times 6 system gives us our translation vector <img src="./tex/f40598ec49a99f9a93c399f7dacc6d3e.svg?invert_in_darkmode" align=middle width=7.35155849999999pt height=20.87411699999998pt/> and the
linearized rotation angles <img src="./tex/dbbd12c1d7f968c7fca71ae001318ee6.svg?invert_in_darkmode" align=middle width=10.57650494999999pt height=14.15524440000002pt/>, <img src="./tex/10bc4a49c81f5dd5800ca54295c8fc4e.svg?invert_in_darkmode" align=middle width=10.16555099999999pt height=22.831056599999986pt/> and <img src="./tex/193089f7a231633473714830d2edc62a.svg?invert_in_darkmode" align=middle width=9.423880949999988pt height=14.15524440000002pt/>. If we simply assign 

<p align="center"><img src="./tex/11f305c11d1f16925fe778eac8be2844.svg?invert_in_darkmode" align=middle width=258.04211564999997pt height=59.1786591pt/></p>


then our transformation will _not_ be rigid. Instead, we should project <img src="./tex/e6bb22a58889cb2e58f4fce2f3a80e02.svg?invert_in_darkmode" align=middle width=17.94511949999999pt height=22.55708729999998pt/>
onto the space of rotation matrices.

#### Recovering a pure rotation from its linearization

> In an effort to provide an alternative from "Least-Squares Rigid Motion Using
> SVD" [Sorkine 2009], this derivation purposefully _avoids_ the [trace
> operator](https://en.wikipedia.org/wiki/Trace_(linear_algebra)) and its
> various nice properties.

If <img src="./tex/dbbd12c1d7f968c7fca71ae001318ee6.svg?invert_in_darkmode" align=middle width=10.57650494999999pt height=14.15524440000002pt/>, <img src="./tex/10bc4a49c81f5dd5800ca54295c8fc4e.svg?invert_in_darkmode" align=middle width=10.16555099999999pt height=22.831056599999986pt/> and <img src="./tex/193089f7a231633473714830d2edc62a.svg?invert_in_darkmode" align=middle width=9.423880949999988pt height=14.15524440000002pt/> are all small, then it may be safe to _interpret_ these
values as rotation angles about the <img src="./tex/332cc365a4987aacce0ead01b8bdcc0b.svg?invert_in_darkmode" align=middle width=9.39498779999999pt height=14.15524440000002pt/>, <img src="./tex/deceeaf6940a8c7a5a02373728002b0f.svg?invert_in_darkmode" align=middle width=8.649225749999989pt height=14.15524440000002pt/>, and <img src="./tex/f93ce33e511096ed626b4719d50f17d2.svg?invert_in_darkmode" align=middle width=8.367621899999993pt height=14.15524440000002pt/> axes respectively.

In general, it is better to find the closest rotation matrix to <img src="./tex/e6bb22a58889cb2e58f4fce2f3a80e02.svg?invert_in_darkmode" align=middle width=17.94511949999999pt height=22.55708729999998pt/>. In other
words, we'd like to solve the small optimization problem:

<p align="center"><img src="./tex/842739ed55ed0979a3a9c4178899b2d7.svg?invert_in_darkmode" align=middle width=189.06322544999998pt height=33.1233078pt/></p>

where <img src="./tex/1f2a0ec2ecb40db723721c1512f99c66.svg?invert_in_darkmode" align=middle width=40.83682679999999pt height=26.76175259999998pt/> computes the squared [Frobenius
norm](https://en.wikipedia.org/wiki/Matrix_norm#Frobenius_norm) of the matrix
<img src="./tex/d05b996d2c08252f77613c25205a0f04.svg?invert_in_darkmode" align=middle width=14.29216634999999pt height=22.55708729999998pt/> (i.e., the sum of all squared element values. In MATLAB syntax:
`sum(sum(A.^2))`). We can expand the norm by taking advantage of the [associativity
property](https://en.wikipedia.org/wiki/Associative_property) of the Frobenius
norm:
<p align="center"><img src="./tex/5b23fe2de73cdb5d0b9da4b94718794d.svg?invert_in_darkmode" align=middle width=310.62239999999997pt height=33.1233078pt/></p>

where <img src="./tex/98a03379352cf0fa6a6609d8d3ac6c5c.svg?invert_in_darkmode" align=middle width=57.937128149999985pt height=24.65753399999998pt/> is the
[Frobenius inner
product](https://en.wikipedia.org/wiki/Frobenius_inner_product) of  <img src="./tex/96458543dc5abd380904d95cae6aa2bc.svg?invert_in_darkmode" align=middle width=14.29216634999999pt height=22.55708729999998pt/> and
<img src="./tex/ff44d867a998c08241beb49b30148782.svg?invert_in_darkmode" align=middle width=13.44741914999999pt height=22.55708729999998pt/> (i.e., the sum of all per-element products. In MATLAB syntax:
`sum(sum(A.*B))`). We can drop the Frobenius norm
of <img src="./tex/e6bb22a58889cb2e58f4fce2f3a80e02.svg?invert_in_darkmode" align=middle width=17.94511949999999pt height=22.55708729999998pt/> term (<img src="./tex/3ec5996c9e885d6d32796997cee23987.svg?invert_in_darkmode" align=middle width=44.48976344999999pt height=31.360807499999982pt/>) because it is constant with respect to the unknown rotation
matrix <img src="./tex/6423e0d54c2545769ad013e5f6a4cf94.svg?invert_in_darkmode" align=middle width=14.17800779999999pt height=22.55708729999998pt/>. We can also drop the Frobenius norm of <img src="./tex/6423e0d54c2545769ad013e5f6a4cf94.svg?invert_in_darkmode" align=middle width=14.17800779999999pt height=22.55708729999998pt/> term because it
must equal one (<img src="./tex/2fae87c07b255f956ea01283ff2c2109.svg?invert_in_darkmode" align=middle width=71.68138889999999pt height=31.360807499999982pt/>) since <img src="./tex/6423e0d54c2545769ad013e5f6a4cf94.svg?invert_in_darkmode" align=middle width=14.17800779999999pt height=22.55708729999998pt/> is required to be a orthonormal matrix
(<img src="./tex/9ef0dc2c2b529f2bad33e83fb198c711.svg?invert_in_darkmode" align=middle width=79.2965943pt height=24.65753399999998pt/>). We can drop the factor of <img src="./tex/76c5792347bb90ef71cfbace628572cf.svg?invert_in_darkmode" align=middle width=8.219209349999991pt height=21.18721440000001pt/> and flip the minus sign to
change our _minimization_ problem into a _maximization_ problem:
<p align="center"><img src="./tex/d04408a5bc2f9636cdaa41bbea578993.svg?invert_in_darkmode" align=middle width=164.4971691pt height=29.771669399999997pt/></p>


We now take advantage of the [singular value
decomposition](https://en.wikipedia.org/wiki/Singular_value_decomposition) of
<img src="./tex/ac02df3fc6b3f4c944ecfc0a1181ae51.svg?invert_in_darkmode" align=middle width=87.29426639999998pt height=27.91243950000002pt/>, where <img src="./tex/9b2a1625a0911294931a3f1c4aeea9ec.svg?invert_in_darkmode" align=middle width=91.74635744999999pt height=26.76175259999998pt/> are orthonormal matrices
and <img src="./tex/ad4f8ffa0b0d4f9b5cd0e611986d5870.svg?invert_in_darkmode" align=middle width=65.32531334999999pt height=26.76175259999998pt/> is a non-negative diagonal matrix:

<p align="center"><img src="./tex/62d93360ff63336cdc9bdde409d09426.svg?invert_in_darkmode" align=middle width=205.67292239999998pt height=32.2210416pt/></p>


The Frobenius inner product will let us move the products by <img src="./tex/26eb59da31fb48cb17abfe4c6dc80375.svg?invert_in_darkmode" align=middle width=14.554737449999989pt height=22.55708729999998pt/> and <img src="./tex/35531be55273dc37ee90083451d089ff.svg?invert_in_darkmode" align=middle width=14.54330789999999pt height=22.55708729999998pt/> from
the right argument to the left argument:

> Recall some linear algebra properties:
> 
>  1. Matrix multiplication (on the left) can be understood as _acting_ on each
>    column: <img src="./tex/9d3490fe55729135aecebd9021855136.svg?invert_in_darkmode" align=middle width=376.4558853pt height=24.65753399999998pt/>,
>  4. The [Kronecker product](https://en.wikipedia.org/wiki/Kronecker_product)
>    <img src="./tex/f751e30bb968304500a783f95a78637a.svg?invert_in_darkmode" align=middle width=41.55228164999998pt height=22.55708729999998pt/> of the identity matrix <img src="./tex/d8471e559d932f20f66bec32f6002e08.svg?invert_in_darkmode" align=middle width=7.168923299999991pt height=22.55708729999998pt/> of size <img src="./tex/63bb9849783d01d91403bc9a5fea12a2.svg?invert_in_darkmode" align=middle width=9.075367949999992pt height=22.831056599999986pt/> and a matrix <img src="./tex/96458543dc5abd380904d95cae6aa2bc.svg?invert_in_darkmode" align=middle width=14.29216634999999pt height=22.55708729999998pt/> simply
>    repeats <img src="./tex/96458543dc5abd380904d95cae6aa2bc.svg?invert_in_darkmode" align=middle width=14.29216634999999pt height=22.55708729999998pt/> along the diagonal k times. In MATLAB, `repdiag(A,k)`,
>  3. Properties 1. and 2. imply that the vectorization of a matrix product
>    <img src="./tex/9ac7623993ca6d8d5bc9b36cdde3c8ff.svg?invert_in_darkmode" align=middle width=27.10031444999999pt height=22.55708729999998pt/> can be written as the Kronecker product of the #-columns-in-<img src="./tex/12d3ebda1a212bd89197298f60cf3ce1.svg?invert_in_darkmode" align=middle width=13.652895299999988pt height=22.55708729999998pt/>
>    identity matrix and <img src="./tex/ff44d867a998c08241beb49b30148782.svg?invert_in_darkmode" align=middle width=13.44741914999999pt height=22.55708729999998pt/> times the vectorization of <img src="./tex/12d3ebda1a212bd89197298f60cf3ce1.svg?invert_in_darkmode" align=middle width=13.652895299999988pt height=22.55708729999998pt/>:
>    <img src="./tex/4f5a78bb9fa1f74a9ec8f953df314ce5.svg?invert_in_darkmode" align=middle width=187.39681455pt height=24.65753399999998pt/>,
>  4. The transpose of a Kronecker product is the Kronecker product of
>    transposes: <img src="./tex/b356aee8d966bb7a2f87b2f5a6c83c0e.svg?invert_in_darkmode" align=middle width=157.06021814999997pt height=27.91243950000002pt/>,
>  5. The Frobenius inner product can be written as a [dot
>    product](://en.wikipedia.org/wiki/Dot_product) of
>    [vectorized](https://en.wikipedia.org/wiki/Vectorization_(mathematics))
>    matrices: <img src="./tex/80da057916f1391ad58c67c29bb467e7.svg?invert_in_darkmode" align=middle width=351.72093329999996pt height=27.91243950000002pt/>,
>  6. Properties 3., 4., and 5. imply that Frobenius inner product of a matrix
>    <img src="./tex/96458543dc5abd380904d95cae6aa2bc.svg?invert_in_darkmode" align=middle width=14.29216634999999pt height=22.55708729999998pt/> and the matrix product of matrix <img src="./tex/ff44d867a998c08241beb49b30148782.svg?invert_in_darkmode" align=middle width=13.44741914999999pt height=22.55708729999998pt/> and <img src="./tex/12d3ebda1a212bd89197298f60cf3ce1.svg?invert_in_darkmode" align=middle width=13.652895299999988pt height=22.55708729999998pt/> is equal to the
>    Frobenius inner product of the matrix product of the transpose of <img src="./tex/ff44d867a998c08241beb49b30148782.svg?invert_in_darkmode" align=middle width=13.44741914999999pt height=22.55708729999998pt/> and
>    <img src="./tex/96458543dc5abd380904d95cae6aa2bc.svg?invert_in_darkmode" align=middle width=14.29216634999999pt height=22.55708729999998pt/>  and the matrix <img src="./tex/12d3ebda1a212bd89197298f60cf3ce1.svg?invert_in_darkmode" align=middle width=13.652895299999988pt height=22.55708729999998pt/>:
>    <img src="./tex/efeb486808dd99e587af1321ed719c5c.svg?invert_in_darkmode" align=middle width=700.2741735pt height=47.671235699999976pt/>.
>  

<p align="center"><img src="./tex/4021f2dd3092665d966f767d6d531816.svg?invert_in_darkmode" align=middle width=203.57247734999999pt height=32.2210416pt/></p>


Now, <img src="./tex/35531be55273dc37ee90083451d089ff.svg?invert_in_darkmode" align=middle width=14.54330789999999pt height=22.55708729999998pt/> and <img src="./tex/26eb59da31fb48cb17abfe4c6dc80375.svg?invert_in_darkmode" align=middle width=14.554737449999989pt height=22.55708729999998pt/> are both
[orthonormal](https://en.wikipedia.org/wiki/Orthogonal_matrix), so multiplying
them against a rotation matrix <img src="./tex/6423e0d54c2545769ad013e5f6a4cf94.svg?invert_in_darkmode" align=middle width=14.17800779999999pt height=22.55708729999998pt/> does not change its orthonormality. We can
pull them out of the maximization if we account for the reflection they _might_
incur: introduce <img src="./tex/84f085a8ee01510052b15faab48ecc9b.svg?invert_in_darkmode" align=middle width=139.41221085pt height=27.6567522pt/> with <img src="./tex/49fe195139d6ccf1d8e438cafa335ba9.svg?invert_in_darkmode" align=middle width=122.37995054999998pt height=27.91243950000002pt/>.
This implies that the optimal rotation for the original probklem is recovered
via <img src="./tex/e817d12342659e3fb1edb6e783d2554d.svg?invert_in_darkmode" align=middle width=100.53062744999998pt height=27.91243950000002pt/>.  When we move the <img src="./tex/a3e6efe097bf3b70a46b0e3e3611d182.svg?invert_in_darkmode" align=middle width=53.47052864999999pt height=14.15524440000002pt/> inside, we now
look for an orthonormal matrix <img src="./tex/bae5a06f813149fce8f3c8a39745d442.svg?invert_in_darkmode" align=middle width=65.96338484999998pt height=24.65753399999998pt/> that is a reflection (if
<img src="./tex/c3aaecd8804db3e504a466c05f005c59.svg?invert_in_darkmode" align=middle width=106.76355029999998pt height=27.91243950000002pt/>) or a rotation (if <img src="./tex/c4553215b14c8475895e752276942b6f.svg?invert_in_darkmode" align=middle width=93.97811609999998pt height=27.91243950000002pt/>):

<p align="center"><img src="./tex/d5a80c823b751b92e935f098eaec7c01.svg?invert_in_darkmode" align=middle width=318.32557844999997pt height=49.315569599999996pt/></p>


This ensures that as a result <img src="./tex/7418159b714ed42bd664b73099a6311f.svg?invert_in_darkmode" align=middle width=20.913202199999986pt height=22.63846199999998pt/> will be a rotation: <img src="./tex/e1aa55f8d78af82fc6c2c53b6de68935.svg?invert_in_darkmode" align=middle width=77.44268894999999pt height=22.831056599999986pt/>.

> Recall that <img src="./tex/ad4f8ffa0b0d4f9b5cd0e611986d5870.svg?invert_in_darkmode" align=middle width=65.32531334999999pt height=26.76175259999998pt/> is a non-negative diagonal matrix of singular values
> sorted so that the smallest value is in the bottom right corner.

Because <img src="./tex/9f531c9f3f1ebeef802ced46eabb0336.svg?invert_in_darkmode" align=middle width=11.87217899999999pt height=22.465723500000017pt/> is orthonormal, each column (or row) of <img src="./tex/9f531c9f3f1ebeef802ced46eabb0336.svg?invert_in_darkmode" align=middle width=11.87217899999999pt height=22.465723500000017pt/> must have unit norm.
Placing a non-zero on the off-diagonal will get "killed" when multiplied by the
corresponding zero in <img src="./tex/7aed918aa12a276a602e30e90b0b109d.svg?invert_in_darkmode" align=middle width=9.98290094999999pt height=14.15524440000002pt/>. So the optimal choice of <img src="./tex/9f531c9f3f1ebeef802ced46eabb0336.svg?invert_in_darkmode" align=middle width=11.87217899999999pt height=22.465723500000017pt/> is to set all values to
zero except on the diagonal. If <img src="./tex/c3aaecd8804db3e504a466c05f005c59.svg?invert_in_darkmode" align=middle width=106.76355029999998pt height=27.91243950000002pt/>, then we should set
one (and only one) of these values to <img src="./tex/e11a8cfcf953c683196d7a48677b2277.svg?invert_in_darkmode" align=middle width=21.00464354999999pt height=21.18721440000001pt/>. The best choice is the bottom right
corner since that will multiply against the smallest singular value in <img src="./tex/9f695bec305c31490f90808856401395.svg?invert_in_darkmode" align=middle width=17.35165739999999pt height=24.657735299999988pt/> (add
negatively affect the maximization the least):

<p align="center"><img src="./tex/15e07f4327285382d80cd51bfe1ea044.svg?invert_in_darkmode" align=middle width=226.23472739999997pt height=118.35736770000001pt/></p>


Finally, we have a formula for our optimal rotation:

<p align="center"><img src="./tex/651299b379f707cb354e1bb225872659.svg?invert_in_darkmode" align=middle width=98.36167769999999pt height=14.77813755pt/></p>


> ### Closed-form point-to-point minimizer
>
> 
> _Interestingly_, despite the non-linear constraint on <img src="./tex/6423e0d54c2545769ad013e5f6a4cf94.svg?invert_in_darkmode" align=middle width=14.17800779999999pt height=22.55708729999998pt/> there is actually
> a closed-form solution to the point-to-point matching problem:
> 
> <p align="center"><img src="./tex/a6a0de2a4895ba102a5ffc9db3f1eeeb.svg?invert_in_darkmode" align=middle width=297.5582379pt height=35.433915pt/></p>

> 
> This is a variant of what's known as a [Procrustes
> problem](https://en.wikipedia.org/wiki/Orthogonal_Procrustes_problem), named
> after a [mythical psychopath](https://en.wikipedia.org/wiki/Procrustes) who
> would kidnap people and force them to fit in his bed by stretching them or
> cutting off their legs. In our case, we are forcing <img src="./tex/6423e0d54c2545769ad013e5f6a4cf94.svg?invert_in_darkmode" align=middle width=14.17800779999999pt height=22.55708729999998pt/> to be perfectly
> orthogonal (no "longer", no "shorter).
> 
> #### Substituting out the translation terms
> 
> This energy is _quadratic_ in <img src="./tex/f40598ec49a99f9a93c399f7dacc6d3e.svg?invert_in_darkmode" align=middle width=7.35155849999999pt height=20.87411699999998pt/> and there are no other constraints on
> <img src="./tex/f40598ec49a99f9a93c399f7dacc6d3e.svg?invert_in_darkmode" align=middle width=7.35155849999999pt height=20.87411699999998pt/>. We can immediately solve for the optimal <img src="./tex/2dbfbbc26f524676be39b3f3df0ad0bc.svg?invert_in_darkmode" align=middle width=14.08675289999999pt height=22.63846199999998pt/>---leaving <img src="./tex/6423e0d54c2545769ad013e5f6a4cf94.svg?invert_in_darkmode" align=middle width=14.17800779999999pt height=22.55708729999998pt/> as an unknown---by
> setting all derivatives with respect to unknowns in <img src="./tex/f40598ec49a99f9a93c399f7dacc6d3e.svg?invert_in_darkmode" align=middle width=7.35155849999999pt height=20.87411699999998pt/> to zero:
> 
> <p align="center"><img src="./tex/411962325b614ad89018cbe9de7a4436.svg?invert_in_darkmode" align=middle width=337.16608365pt height=100.32167639999999pt/></p>

> where <img src="./tex/2ac7c6ff1056fe3737fa4395e2efb6c5.svg?invert_in_darkmode" align=middle width=48.68135084999999pt height=27.91243950000002pt/> is a vector ones. Setting the partial derivative with
> respect to <img src="./tex/f40598ec49a99f9a93c399f7dacc6d3e.svg?invert_in_darkmode" align=middle width=7.35155849999999pt height=20.87411699999998pt/> of this
> quadratic energy to zero finds the minimum:
> <p align="center"><img src="./tex/558c4e7b6ff601eec20574a000a5f46d.svg?invert_in_darkmode" align=middle width=278.3087109pt height=84.66329024999999pt/></p>

> 
> Rearranging terms above reveals that the optimal <img src="./tex/f40598ec49a99f9a93c399f7dacc6d3e.svg?invert_in_darkmode" align=middle width=7.35155849999999pt height=20.87411699999998pt/> is the vector aligning
> the [centroids](https://en.wikipedia.org/wiki/Centroid) of the points in <img src="./tex/384591906555413c452c93e493b2d4ec.svg?invert_in_darkmode" align=middle width=12.92230829999999pt height=22.55708729999998pt/>
> and the points in <img src="./tex/d05b996d2c08252f77613c25205a0f04.svg?invert_in_darkmode" align=middle width=14.29216634999999pt height=22.55708729999998pt/> rotated by the---yet-unknown---<img src="./tex/6423e0d54c2545769ad013e5f6a4cf94.svg?invert_in_darkmode" align=middle width=14.17800779999999pt height=22.55708729999998pt/>. Introducing
> variables for the respective centroids <img src="./tex/64af2fa219a12c6bb69c25da58441a8f.svg?invert_in_darkmode" align=middle width=120.17019629999999pt height=32.51169900000002pt/> and <img src="./tex/00085c1c3a7306fc9ff9974f9fe861dc.svg?invert_in_darkmode" align=middle width=97.20234314999998pt height=32.51169900000002pt/>, we can write the
> formula for the optimal  <img src="./tex/f40598ec49a99f9a93c399f7dacc6d3e.svg?invert_in_darkmode" align=middle width=7.35155849999999pt height=20.87411699999998pt/>:
> 
> <p align="center"><img src="./tex/365bcb6e3360952b527036dbc1c97837.svg?invert_in_darkmode" align=middle width=201.3531696pt height=84.59019029999999pt/></p>

> 
> Now we have a formula for the optimal translation vector <img src="./tex/f40598ec49a99f9a93c399f7dacc6d3e.svg?invert_in_darkmode" align=middle width=7.35155849999999pt height=20.87411699999998pt/> in terms of the
> unknown rotation <img src="./tex/6423e0d54c2545769ad013e5f6a4cf94.svg?invert_in_darkmode" align=middle width=14.17800779999999pt height=22.55708729999998pt/>. Let us
> [substitute](https://en.wikipedia.org/wiki/Substitution_(algebra)) this formula
> for all occurrences of <img src="./tex/f40598ec49a99f9a93c399f7dacc6d3e.svg?invert_in_darkmode" align=middle width=7.35155849999999pt height=20.87411699999998pt/> in our energy written in its original summation
> form:
> 
> <p align="center"><img src="./tex/ef620aca9542ebf83ccc978dd8db9215.svg?invert_in_darkmode" align=middle width=1085.32980435pt height=36.50245665pt/></p>

> 
> where we introduce <img src="./tex/0666e59c3940e6bce1da734268343091.svg?invert_in_darkmode" align=middle width=70.34807339999999pt height=27.91243950000002pt/> where the ith row contains the
> _relative position_ of the ith point to the centroid <img src="./tex/9a741103081d04ca0a4a97a8e6c28ce6.svg?invert_in_darkmode" align=middle width=9.97711604999999pt height=24.200985600000003pt/>: i.e.,
> <img src="./tex/049a4d4763f53136c65dd204d7dca240.svg?invert_in_darkmode" align=middle width=95.67119594999998pt height=24.65753399999998pt/> (and analagously for <img src="./tex/12096d414736789db645c9547a5804cc.svg?invert_in_darkmode" align=middle width=12.92230829999999pt height=27.817082700000007pt/>).
> 
> Now we have the canonical form of the [orthogonal procrustes
> problem](https://en.wikipedia.org/wiki/Orthogonal_Procrustes_problem). To
> find the optimal rotation matrix <img src="./tex/7418159b714ed42bd664b73099a6311f.svg?invert_in_darkmode" align=middle width=20.913202199999986pt height=22.63846199999998pt/> we will massage the terms in the
> _minimization_ until we have a _maximization_ problem involving the [Frobenius
> inner-product](https://en.wikipedia.org/wiki/Frobenius_inner_product) of the
> unknown rotation <img src="./tex/6423e0d54c2545769ad013e5f6a4cf94.svg?invert_in_darkmode" align=middle width=14.17800779999999pt height=22.55708729999998pt/> and [covariance
> matrix](https://en.wikipedia.org/wiki/Covariance_matrix) of <img src="./tex/d05b996d2c08252f77613c25205a0f04.svg?invert_in_darkmode" align=middle width=14.29216634999999pt height=22.55708729999998pt/> and <img src="./tex/384591906555413c452c93e493b2d4ec.svg?invert_in_darkmode" align=middle width=12.92230829999999pt height=22.55708729999998pt/>:
> 
> <p align="center"><img src="./tex/cbb24b64ac63695a2ee4bc9c622b6015.svg?invert_in_darkmode" align=middle width=406.3871988pt height=339.09967244999996pt/></p>

> 
> Letting <img src="./tex/83d29abc70b2d18383927c9c5f984546.svg?invert_in_darkmode" align=middle width=78.98938574999998pt height=36.98604359999998pt/> we can now follow the
> steps above using [singular value
> decomposition](https://en.wikipedia.org/wiki/Singular_value_decomposition) to
> find the optimal <img src="./tex/6423e0d54c2545769ad013e5f6a4cf94.svg?invert_in_darkmode" align=middle width=14.17800779999999pt height=22.55708729999998pt/>.

### Approximate point-to-plane minimizer

If we apply our linearization of <img src="./tex/6423e0d54c2545769ad013e5f6a4cf94.svg?invert_in_darkmode" align=middle width=14.17800779999999pt height=22.55708729999998pt/> to the **point-to-plane** distance
linearization of the matching energy, our minimization is:

<p align="center"><img src="./tex/3dd5d746af4f71d8f4483efd64be5082.svg?invert_in_darkmode" align=middle width=480.55944915pt height=62.53027769999999pt/></p>


We can follow similar steps as above. Let's gather a vector of unknowns: <img src="./tex/6bcbadace59521e7565cab9d0648a961.svg?invert_in_darkmode" align=middle width=126.75812654999999pt height=27.91243950000002pt/>. Then we can write our problem in summation form
as:

<p align="center"><img src="./tex/35069a4108be3a4d3a47e4dd606c1aa0.svg?invert_in_darkmode" align=middle width=565.7364008999999pt height=62.53027769999999pt/></p>


This can be written compactly in matrix form as:

<p align="center"><img src="./tex/2717b067b56472023341743a847aa92d.svg?invert_in_darkmode" align=middle width=546.11870775pt height=62.53027769999999pt/></p>


where <img src="./tex/d215374485dd1f9e71eba3fa2f247afe.svg?invert_in_darkmode" align=middle width=19.44535064999999pt height=22.55708729999998pt/> is the ith column from the matrix of normals <img src="./tex/5e65e079e680711d68cb1f973f9b02d9.svg?invert_in_darkmode" align=middle width=70.85036639999998pt height=27.91243950000002pt/>,
<img src="./tex/13f548e5efa74d654479d62c25ff2fd8.svg?invert_in_darkmode" align=middle width=53.16219644999998pt height=24.65753399999998pt/> [creates a diagonal
matrix](https://en.wikipedia.org/wiki/Diagonal_matrix#Matrix_operations) from a
vector, and <img src="./tex/7eab9509bd92f8ab739bdfe4383e7249.svg?invert_in_darkmode" align=middle width=76.90062764999999pt height=27.91243950000002pt/> is the same as above.

This energy is quadratic in <img src="./tex/129c5b884ff47d80be4d6261a476e9f1.svg?invert_in_darkmode" align=middle width=10.502226899999991pt height=14.611878600000017pt/> and can be solve by setting all partial
derivatives with respect to <img src="./tex/129c5b884ff47d80be4d6261a476e9f1.svg?invert_in_darkmode" align=middle width=10.502226899999991pt height=14.611878600000017pt/> to zero.

> ### Closed-form point-to-point minimizer
>
> To the best of my knowledge, no known closed-form solution exists. I am not
> sure whether it **_can not_** exist or just whether no one has figured it out
> (or they did and I just do not know about it).

## Uniform random sampling of a triangle mesh

Our last missing piece is to sample the surface of a triangle mesh <img src="./tex/cbfb1b2a33b28eab8a3e59464768e810.svg?invert_in_darkmode" align=middle width=14.908688849999992pt height=22.465723500000017pt/> with <img src="./tex/0e51a2dede42189d77627c4d742822c3.svg?invert_in_darkmode" align=middle width=14.433101099999991pt height=14.15524440000002pt/>
faces uniformly randomly. This allows us to approximate _continuous_ integrals
over the surface <img src="./tex/cbfb1b2a33b28eab8a3e59464768e810.svg?invert_in_darkmode" align=middle width=14.908688849999992pt height=22.465723500000017pt/> with a summation of the integrand evaluated at a finite
number of randomly selected points. This type of [numerical
integration](https://en.wikipedia.org/wiki/Numerical_integration) is called the
[Monte Carlo method](https://en.wikipedia.org/wiki/Monte_Carlo_method).

We would like our [random
variable](https://en.wikipedia.org/wiki/Random_variable) <img src="./tex/4871a4dd376e74c437a6f3f4e6724dba.svg?invert_in_darkmode" align=middle width=44.97694244999998pt height=22.465723500000017pt/> to have a
uniform [probability density
function](https://en.wikipedia.org/wiki/Probability_density_function) <img src="./tex/14f294cdc16d8c7298021ab50dddf896.svg?invert_in_darkmode" align=middle width=94.02629114999999pt height=24.65753399999998pt/>, where <img src="./tex/1fc24c1076ff1d4160b0235804985e21.svg?invert_in_darkmode" align=middle width=24.003525149999987pt height=22.465723500000017pt/> is the [surface
area](https://en.wikipedia.org/wiki/Surface_area) of the triangle mesh <img src="./tex/cbfb1b2a33b28eab8a3e59464768e810.svg?invert_in_darkmode" align=middle width=14.908688849999992pt height=22.465723500000017pt/>. We
can achieve this by breaking the problem into two steps: uniformly sampling in
a single triangle and sampling triangles non-uniformly according to their
area.

Suppose we have a way to evaluate a continuous random point <img src="./tex/b0ea07dc5c00127344a1cad40467b8de.svg?invert_in_darkmode" align=middle width=9.97711604999999pt height=14.611878600000017pt/> in a triangle
<img src="./tex/2f118ee06d05f3c2d98361d9c30e38ce.svg?invert_in_darkmode" align=middle width=11.889314249999991pt height=22.465723500000017pt/> with uniform probability density function <img src="./tex/7c4848e8bdfb6b4a8ec9dcf6175ada00.svg?invert_in_darkmode" align=middle width=100.26404684999999pt height=24.65753399999998pt/> _and_ we have a
away to evaluate a discrete random triangle index <img src="./tex/5f32952725df3b99025ce88ca4641561.svg?invert_in_darkmode" align=middle width=123.12568619999999pt height=24.65753399999998pt/> with [discrete
probability
distribution](https://en.wikipedia.org/wiki/Probability_distribution#Discrete_probability_distribution)
<img src="./tex/9f535ff671119867bec84b932d265b2d.svg?invert_in_darkmode" align=middle width=110.0573694pt height=24.65753399999998pt/>, then the joint probability of evaluating a certain triangle
index <img src="./tex/2f118ee06d05f3c2d98361d9c30e38ce.svg?invert_in_darkmode" align=middle width=11.889314249999991pt height=22.465723500000017pt/> and then uniformly random point in that triangle <img src="./tex/b0ea07dc5c00127344a1cad40467b8de.svg?invert_in_darkmode" align=middle width=9.97711604999999pt height=14.611878600000017pt/> is indeed
uniform over the surface:

<p align="center"><img src="./tex/6138d44be15c524729f1a70cff608559.svg?invert_in_darkmode" align=middle width=262.17446144999997pt height=36.09514755pt/></p>


### Uniform random sampling of a single triangle

In order to pick a point uniformly randomly in a triangle with corners <img src="./tex/cde4b125428933dde9b918ac57353a40.svg?invert_in_darkmode" align=middle width=105.18235694999998pt height=26.76175259999998pt/> we will _first_ pick a point uniformly randomly in the
[parallelogram](https://en.wikipedia.org/wiki/Parallelogram) formed by
reflecting <img src="./tex/f92f752167f1cb7515d84d4657942eb8.svg?invert_in_darkmode" align=middle width=16.529662049999992pt height=14.611878600000017pt/> across the line <img src="./tex/e68a9725799ff7c75d7fa243b723272e.svg?invert_in_darkmode" align=middle width=34.70315144999999pt height=19.871860799999983pt/>:

<p align="center"><img src="./tex/9b504a2cfd935e029fbb58ed2729951a.svg?invert_in_darkmode" align=middle width=245.33028794999998pt height=16.438356pt/></p>


where <img src="./tex/e00280302736d283121cea8b5081a8a0.svg?invert_in_darkmode" align=middle width=28.047932549999988pt height=22.831056599999986pt/> are uniformly sampled from the unit interval <img src="./tex/acf5ce819219b95070be2dbeb8a671e9.svg?invert_in_darkmode" align=middle width=32.87674994999999pt height=24.65753399999998pt/>. If <img src="./tex/5b266dab098952f2923405c8b7c5f778.svg?invert_in_darkmode" align=middle width=70.97006234999999pt height=22.831056599999986pt/>
then the point <img src="./tex/b0ea07dc5c00127344a1cad40467b8de.svg?invert_in_darkmode" align=middle width=9.97711604999999pt height=14.611878600000017pt/> above will lie in the reflected triangle rather than the
original one. In this case, preprocess <img src="./tex/dbbd12c1d7f968c7fca71ae001318ee6.svg?invert_in_darkmode" align=middle width=10.57650494999999pt height=14.15524440000002pt/> and <img src="./tex/10bc4a49c81f5dd5800ca54295c8fc4e.svg?invert_in_darkmode" align=middle width=10.16555099999999pt height=22.831056599999986pt/> by setting <img src="./tex/f3a788e0a068ea2e77b7ea3b8b30e6a2.svg?invert_in_darkmode" align=middle width=75.03400244999999pt height=21.18721440000001pt/> and
<img src="./tex/9e3953de653706db2d76757d620e922e.svg?invert_in_darkmode" align=middle width=74.21208464999998pt height=22.831056599999986pt/> to reflect the point <img src="./tex/b0ea07dc5c00127344a1cad40467b8de.svg?invert_in_darkmode" align=middle width=9.97711604999999pt height=14.611878600000017pt/> back into the original triangle.

### Area-weighted random sampling of triangles

Assuming we know how to draw a _continuous_ uniform random variable <img src="./tex/193089f7a231633473714830d2edc62a.svg?invert_in_darkmode" align=middle width=9.423880949999988pt height=14.15524440000002pt/> from
the unit interval <img src="./tex/acf5ce819219b95070be2dbeb8a671e9.svg?invert_in_darkmode" align=middle width=32.87674994999999pt height=24.65753399999998pt/>, we would now like to draw a _discrete_ random
triangle index <img src="./tex/2f118ee06d05f3c2d98361d9c30e38ce.svg?invert_in_darkmode" align=middle width=11.889314249999991pt height=22.465723500000017pt/> from the sequence <img src="./tex/12ae120cf77557772d65127ef3360fb5.svg?invert_in_darkmode" align=middle width=37.26407684999999pt height=21.18721440000001pt/> with likelihood proportional to
the relative area of each triangle in the mesh.

We can achieve this by first computing the [cumulative
sum](https://en.wikipedia.org/wiki/Running_total) <img src="./tex/386003f198eaba07b624962ab7a894b0.svg?invert_in_darkmode" align=middle width=57.281063399999994pt height=22.648391699999998pt/> of the relative
areas:

<p align="center"><img src="./tex/2ff5513f2ad6ffbf75f8fa24d37017d0.svg?invert_in_darkmode" align=middle width=119.79076395pt height=36.09514755pt/></p>


Then our random index is found by identifying the first entry in <img src="./tex/12d3ebda1a212bd89197298f60cf3ce1.svg?invert_in_darkmode" align=middle width=13.652895299999988pt height=22.55708729999998pt/> whose
value is greater than a uniform random variable <img src="./tex/193089f7a231633473714830d2edc62a.svg?invert_in_darkmode" align=middle width=9.423880949999988pt height=14.15524440000002pt/>. Since <img src="./tex/12d3ebda1a212bd89197298f60cf3ce1.svg?invert_in_darkmode" align=middle width=13.652895299999988pt height=22.55708729999998pt/> is sorted,
locating this entry can be done in <img src="./tex/03fb15b258f821e5f4081e2e77f57a5c.svg?invert_in_darkmode" align=middle width=64.18652789999999pt height=24.65753399999998pt/>
[time](https://en.wikipedia.org/wiki/Big_O_notation).

### Why is my code so slow?

Try profiling your code. Where is most of the computation time spent?

If you have done things right, the majority of time is spent computing
point-to-mesh distances. For each query point, the [computational
complexity](https://en.wikipedia.org/wiki/Computational_complexity_theory) of
computing its distance to a mesh with <img src="./tex/0e51a2dede42189d77627c4d742822c3.svg?invert_in_darkmode" align=middle width=14.433101099999991pt height=14.15524440000002pt/> faces is <img src="./tex/5e12273846712182a9633dc4c62b94f7.svg?invert_in_darkmode" align=middle width=40.21395839999999pt height=24.65753399999998pt/>.

This can be _dramatically_ improved (e.g., to <img src="./tex/03fb15b258f821e5f4081e2e77f57a5c.svg?invert_in_darkmode" align=middle width=64.18652789999999pt height=24.65753399999998pt/> on average) using an
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
- `igl::iterative_closest_point`
- `igl::point_mesh_squared_distance`
- `igl::point_simplex_squared_distance`
- `igl::polar_dec`
- `igl::polar_svd3x3`
- `igl::polar_svd`
- `igl::random_points_on_mesh`
- `igl::rigid_alignment`

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
randomly sampling the <img src="./tex/cbfb1b2a33b28eab8a3e59464768e810.svg?invert_in_darkmode" align=middle width=14.908688849999992pt height=22.465723500000017pt/> mesh.

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
the integral over <img src="./tex/cbfb1b2a33b28eab8a3e59464768e810.svg?invert_in_darkmode" align=middle width=14.908688849999992pt height=22.465723500000017pt/> and specify the `method` (point-to-point or
point-to-plane).
