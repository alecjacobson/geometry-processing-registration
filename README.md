# Geometry Processing — Registration

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

We would like a distance measure between two surfaces that — like Hausdorff
distance — does not require a shared parameterization. Unlike Hausdorff
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
[Gauss-Newton](https://en.wikipedia.org/wiki/Gauss—Newton_algorithm) methods:

```
minimize f(z)^{2}
  z_{0} ← initial guess
  repeat until convergence
    f_{0} ← linearize f(z) around z_{0}
    z_{0} ← minimize f_{0}(z)^{2}
```

Since our <img src="./tex/47b0192f8f0819d64bce3612c46d15ea.svg?invert_in_darkmode" align=middle width=7.56844769999999pt height=22.831056599999986pt/> is a geometric function, we can derive its linearizations
_geometrically_.

### Constant function approximation

If we make the convenient—however unrealistic—assumption that in the
neighborhood of the closest-point projection <img src="./tex/5ea090d44e44319896420959310f0012.svg?invert_in_darkmode" align=middle width=50.49552749999999pt height=24.65753399999998pt/> of the current guess
<img src="./tex/068e9f4c48765b7e6082c36543e13d10.svg?invert_in_darkmode" align=middle width=14.95432949999999pt height=14.611878600000017pt/> the surface <img src="./tex/91aac9730317276af725abd8cef04ca9.svg?invert_in_darkmode" align=middle width=13.19638649999999pt height=22.465723500000017pt/> is simply the point <img src="./tex/5ea090d44e44319896420959310f0012.svg?invert_in_darkmode" align=middle width=50.49552749999999pt height=24.65753399999998pt/> (perhaps imagine that <img src="./tex/91aac9730317276af725abd8cef04ca9.svg?invert_in_darkmode" align=middle width=13.19638649999999pt height=22.465723500000017pt/>
is makes a sharp needle-like point at <img src="./tex/5ea090d44e44319896420959310f0012.svg?invert_in_darkmode" align=middle width=50.49552749999999pt height=24.65753399999998pt/> or that <img src="./tex/91aac9730317276af725abd8cef04ca9.svg?invert_in_darkmode" align=middle width=13.19638649999999pt height=22.465723500000017pt/> is very far away
from <img src="./tex/b0ea07dc5c00127344a1cad40467b8de.svg?invert_in_darkmode" align=middle width=9.97711604999999pt height=14.611878600000017pt/>), then we can approximate <img src="./tex/5c8431f6e02c7487c48b23b4d4d72b11.svg?invert_in_darkmode" align=middle width=28.75564229999999pt height=24.65753399999998pt/> in the proximity of our current
guess <img src="./tex/068e9f4c48765b7e6082c36543e13d10.svg?invert_in_darkmode" align=middle width=14.95432949999999pt height=14.611878600000017pt/> as the vector between the input point <img src="./tex/da278ee0789447cfaae0380d4cda2fdb.svg?invert_in_darkmode" align=middle width=8.40178184999999pt height=14.611878600000017pt/> and <img src="./tex/5ea090d44e44319896420959310f0012.svg?invert_in_darkmode" align=middle width=50.49552749999999pt height=24.65753399999998pt/>:

<p align="center"><img src="./tex/d300e867569228e54c52f050c2d3d802.svg?invert_in_darkmode" align=middle width=209.29638509999998pt height=17.031940199999998pt/></p>


In effect, we are assuming that the surface <img src="./tex/91aac9730317276af725abd8cef04ca9.svg?invert_in_darkmode" align=middle width=13.19638649999999pt height=22.465723500000017pt/> is _constant_ function of its
parameterization: <img src="./tex/4cb0776dc727de5904c46747e53cc2d8.svg?invert_in_darkmode" align=middle width=120.71227409999999pt height=24.65753399999998pt/>.

Minimizing <img src="./tex/ef03e3adb9f682700bd731e4c2ebeb55.svg?invert_in_darkmode" align=middle width=25.03690529999999pt height=22.465723500000017pt/> iteratively using this linearization of
<img src="./tex/47b0192f8f0819d64bce3612c46d15ea.svg?invert_in_darkmode" align=middle width=7.56844769999999pt height=22.831056599999986pt/> is equivalent to [gradient
descent](https://en.wikipedia.org/wiki/Gradient_descent). We have simply derived
our gradients geometrically.

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
[Gauss-Newton](https://en.wikipedia.org/wiki/Gauss—Newton_algorithm) method. We
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

If `V_X` and `F_X` are the vertices and faces of a triangle mesh surface <img src="./tex/cbfb1b2a33b28eab8a3e59464768e810.svg?invert_in_darkmode" align=middle width=14.908688849999992pt height=22.465723500000017pt/>
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
translation vector <img src="./tex/4835152d95e6468a4f5aaf85cd0e3fed.svg?invert_in_darkmode" align=middle width=45.86742269999999pt height=26.76175259999998pt/> that _best_ aligns a given a set of points <img src="./tex/c6fb0a239d93250461ae9e9483290505.svg?invert_in_darkmode" align=middle width=70.34808pt height=27.91243950000002pt/> on the source mesh and their current closest points <img src="./tex/ed46e92825f1e61b4b5e43667657ffb2.svg?invert_in_darkmode" align=middle width=68.97822194999999pt height=27.91243950000002pt/>
on the target mesh. We have two choices for _linearizing_ our matching energy:
point-to-point (gradient descent) and point-to-plane (Gauss-Newton).

_**ICP using the point-to-point matching energy linearization is slow to converge.**_
![](images/max-point-to-point.gif)

_**ICP using the point-to-plane matching energy linearization is faster.**_
![](images/max-point-to-plane.gif)

In either case, this is still a non-linear optimization problem. This time due
to the [constraints](https://en.wikipedia.org/wiki/Constrained_optimization)
rather than the energy term. 

### Closed-form solution for point-to-point rigid matching

The point-to-point (gradient descent) rigid matching problem solves:

<p align="center"><img src="./tex/c7b374c6d51338a4900c611c8c44f5f8.svg?invert_in_darkmode" align=middle width=234.43698465pt height=47.93392394999999pt/></p>

This is a variant of what's known as a [Procrustes problem](https://en.wikipedia.org/wiki/Orthogonal_Procrustes_problem), named after a [mythical psychopath](https://en.wikipedia.org/wiki/Procrustes) who would kidnap people and force them to fit in his bed by stretching them or cutting off their legs. In our case, we are forcing <img src="./tex/6423e0d54c2545769ad013e5f6a4cf94.svg?invert_in_darkmode" align=middle width=14.17800779999999pt height=22.55708729999998pt/> to be perfectly orthogonal (no "longer", no "shorter").

#### Substituting out the translation terms

This energy is _quadratic_ in <img src="./tex/f40598ec49a99f9a93c399f7dacc6d3e.svg?invert_in_darkmode" align=middle width=7.35155849999999pt height=20.87411699999998pt/> and there are no other constraints on
<img src="./tex/f40598ec49a99f9a93c399f7dacc6d3e.svg?invert_in_darkmode" align=middle width=7.35155849999999pt height=20.87411699999998pt/>. We can immediately solve for the optimal <img src="./tex/2dbfbbc26f524676be39b3f3df0ad0bc.svg?invert_in_darkmode" align=middle width=14.08675289999999pt height=22.63846199999998pt/> — leaving <img src="./tex/6423e0d54c2545769ad013e5f6a4cf94.svg?invert_in_darkmode" align=middle width=14.17800779999999pt height=22.55708729999998pt/> as an unknown — by
setting all derivatives with respect to unknowns in <img src="./tex/f40598ec49a99f9a93c399f7dacc6d3e.svg?invert_in_darkmode" align=middle width=7.35155849999999pt height=20.87411699999998pt/> to zero:

<p align="center"><img src="./tex/dff2d51dc0bc2208fabf381da1ee9054.svg?invert_in_darkmode" align=middle width=479.1395680499999pt height=87.52486215pt/></p>

where <img src="./tex/2ac7c6ff1056fe3737fa4395e2efb6c5.svg?invert_in_darkmode" align=middle width=48.68135084999999pt height=27.91243950000002pt/> is a vector ones. Setting the partial derivative with
respect to <img src="./tex/f40598ec49a99f9a93c399f7dacc6d3e.svg?invert_in_darkmode" align=middle width=7.35155849999999pt height=20.87411699999998pt/> of this
quadratic energy to zero finds the minimum:

<p align="center"><img src="./tex/9d06bb65b0ec4d34e2278e2593224b6a.svg?invert_in_darkmode" align=middle width=457.47342464999997pt height=59.27515935pt/></p>



Rearranging terms above reveals that the optimal <img src="./tex/f40598ec49a99f9a93c399f7dacc6d3e.svg?invert_in_darkmode" align=middle width=7.35155849999999pt height=20.87411699999998pt/> is the vector aligning
the [centroids](https://en.wikipedia.org/wiki/Centroid) of the points in <img src="./tex/384591906555413c452c93e493b2d4ec.svg?invert_in_darkmode" align=middle width=12.92230829999999pt height=22.55708729999998pt/>
and the points in <img src="./tex/d05b996d2c08252f77613c25205a0f04.svg?invert_in_darkmode" align=middle width=14.29216634999999pt height=22.55708729999998pt/> rotated by the — yet-unknown — <img src="./tex/6423e0d54c2545769ad013e5f6a4cf94.svg?invert_in_darkmode" align=middle width=14.17800779999999pt height=22.55708729999998pt/>. Introducing
variables for the respective centroids <img src="./tex/e28e01c4d55f4abbe81b2eda4a4d3811.svg?invert_in_darkmode" align=middle width=103.73188319999998pt height=32.51169900000002pt/> and <img src="./tex/1879b1cd74e6963ce884b7a7e3196b6a.svg?invert_in_darkmode" align=middle width=97.20234314999998pt height=32.51169900000002pt/>, we can write the
formula for the optimal  <img src="./tex/f40598ec49a99f9a93c399f7dacc6d3e.svg?invert_in_darkmode" align=middle width=7.35155849999999pt height=20.87411699999998pt/>:


<p align="center"><img src="./tex/a9e6db9f6a09f07f15d7506f4ec1f6e4.svg?invert_in_darkmode" align=middle width=418.03393665pt height=60.845893350000004pt/></p>



Now we have a formula for the optimal translation vector <img src="./tex/f40598ec49a99f9a93c399f7dacc6d3e.svg?invert_in_darkmode" align=middle width=7.35155849999999pt height=20.87411699999998pt/> in terms of the
unknown rotation <img src="./tex/6423e0d54c2545769ad013e5f6a4cf94.svg?invert_in_darkmode" align=middle width=14.17800779999999pt height=22.55708729999998pt/>. Let us
[substitute](https://en.wikipedia.org/wiki/Substitution_(algebra)) this formula
for all occurrences of <img src="./tex/f40598ec49a99f9a93c399f7dacc6d3e.svg?invert_in_darkmode" align=middle width=7.35155849999999pt height=20.87411699999998pt/> in our energy written in its original summation
form:

<p align="center"><img src="./tex/00e00e26b2f40f54d1d1806f9d448fc6.svg?invert_in_darkmode" align=middle width=496.79341800000003pt height=167.46168615pt/></p>


where we introduce <img src="./tex/0666e59c3940e6bce1da734268343091.svg?invert_in_darkmode" align=middle width=70.34807339999999pt height=27.91243950000002pt/> where the ith row contains the
_relative position_ of the ith point to the centroid <img src="./tex/c28a7e764fb3e1d562204f0d05b4ec04.svg?invert_in_darkmode" align=middle width=9.97711604999999pt height=19.871860799999983pt/>: i.e.,
<img src="./tex/69f39cdb5b0d045862c4b5e8172d7988.svg?invert_in_darkmode" align=middle width=95.67119594999998pt height=24.65753399999998pt/> (and analagously for <img src="./tex/12096d414736789db645c9547a5804cc.svg?invert_in_darkmode" align=middle width=12.92230829999999pt height=27.817082700000007pt/>).

Now we have the canonical form of the [orthogonal procrustes
problem](https://en.wikipedia.org/wiki/Orthogonal_Procrustes_problem). To
find the optimal rotation matrix <img src="./tex/7418159b714ed42bd664b73099a6311f.svg?invert_in_darkmode" align=middle width=20.913202199999986pt height=22.63846199999998pt/> we will massage the terms in the
_minimization_ until we have a _maximization_ problem involving the [Frobenius
inner-product](https://en.wikipedia.org/wiki/Frobenius_inner_product) of the
unknown rotation <img src="./tex/6423e0d54c2545769ad013e5f6a4cf94.svg?invert_in_darkmode" align=middle width=14.17800779999999pt height=22.55708729999998pt/> and [covariance
matrix](https://en.wikipedia.org/wiki/Covariance_matrix) of <img src="./tex/d05b996d2c08252f77613c25205a0f04.svg?invert_in_darkmode" align=middle width=14.29216634999999pt height=22.55708729999998pt/> and <img src="./tex/384591906555413c452c93e493b2d4ec.svg?invert_in_darkmode" align=middle width=12.92230829999999pt height=22.55708729999998pt/>:


<p align="center"><img src="./tex/06667758b5dd0cc55d54c099049e9e42.svg?invert_in_darkmode" align=middle width=700.2739854pt height=336.1574403pt/></p>


Letting <img src="./tex/818e261a8ac45a73aabd65ba9bba3a1b.svg?invert_in_darkmode" align=middle width=80.91279569999999pt height=36.98604359999998pt/>. We can understand this problem as _projecting_ the matrix <img src="./tex/e6bb22a58889cb2e58f4fce2f3a80e02.svg?invert_in_darkmode" align=middle width=17.94511949999999pt height=22.55708729999998pt/> to the nearest rotation matrix <img src="./tex/6423e0d54c2545769ad013e5f6a4cf94.svg?invert_in_darkmode" align=middle width=14.17800779999999pt height=22.55708729999998pt/>.

#### Closest Rotation Matrix

> In an effort to provide an alternative from "Least-Squares Rigid Motion Using
> SVD" [Sorkine 2009], this derivation purposefully _avoids_ the [trace
> operator](https://en.wikipedia.org/wiki/Trace_(linear_algebra)) and its
> various nice properties.

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
<img src="./tex/6b131ebac8b7c63ef31cb6925661ad5a.svg?invert_in_darkmode" align=middle width=89.21769614999998pt height=27.91243950000002pt/>, where <img src="./tex/9b2a1625a0911294931a3f1c4aeea9ec.svg?invert_in_darkmode" align=middle width=91.74635744999999pt height=26.76175259999998pt/> are orthonormal matrices
and <img src="./tex/ad4f8ffa0b0d4f9b5cd0e611986d5870.svg?invert_in_darkmode" align=middle width=65.32531334999999pt height=26.76175259999998pt/> is a non-negative diagonal matrix:

<p align="center"><img src="./tex/53cafbd2848f7f70d7b4c3f03d9f6c94.svg?invert_in_darkmode" align=middle width=207.59633235pt height=32.2210416pt/></p>


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
>    transposes: <img src="./tex/30732bc83cf909d6edd8de25f2645089.svg?invert_in_darkmode" align=middle width=162.83046779999998pt height=27.91243950000002pt/>,
>  5. The Frobenius inner product can be written as a [dot
>    product](://en.wikipedia.org/wiki/Dot_product) of
>    [vectorized](https://en.wikipedia.org/wiki/Vectorization_(mathematics))
>    matrices: <img src="./tex/018ac9f687643eb5a4d16c0a4ab4bc06.svg?invert_in_darkmode" align=middle width=353.64434324999996pt height=27.91243950000002pt/>,
>  6. Properties 3., 4., and 5. imply that Frobenius inner product of a matrix
>    <img src="./tex/96458543dc5abd380904d95cae6aa2bc.svg?invert_in_darkmode" align=middle width=14.29216634999999pt height=22.55708729999998pt/> and the matrix product of matrix <img src="./tex/ff44d867a998c08241beb49b30148782.svg?invert_in_darkmode" align=middle width=13.44741914999999pt height=22.55708729999998pt/> and <img src="./tex/12d3ebda1a212bd89197298f60cf3ce1.svg?invert_in_darkmode" align=middle width=13.652895299999988pt height=22.55708729999998pt/> is equal to the
>    Frobenius inner product of the matrix product of the transpose of <img src="./tex/ff44d867a998c08241beb49b30148782.svg?invert_in_darkmode" align=middle width=13.44741914999999pt height=22.55708729999998pt/> and
>    <img src="./tex/96458543dc5abd380904d95cae6aa2bc.svg?invert_in_darkmode" align=middle width=14.29216634999999pt height=22.55708729999998pt/>  and the matrix <img src="./tex/12d3ebda1a212bd89197298f60cf3ce1.svg?invert_in_darkmode" align=middle width=13.652895299999988pt height=22.55708729999998pt/>:
>    <img src="./tex/4d9c6a6f9c0e0098e929887dddf57802.svg?invert_in_darkmode" align=middle width=702.93655245pt height=47.671235699999976pt/>.
>  

<p align="center"><img src="./tex/ef54fe83b00bd0937ccf189fac5b7fa3.svg?invert_in_darkmode" align=middle width=205.4958873pt height=32.2210416pt/></p>


Now, <img src="./tex/35531be55273dc37ee90083451d089ff.svg?invert_in_darkmode" align=middle width=14.54330789999999pt height=22.55708729999998pt/> and <img src="./tex/26eb59da31fb48cb17abfe4c6dc80375.svg?invert_in_darkmode" align=middle width=14.554737449999989pt height=22.55708729999998pt/> are both
[orthonormal](https://en.wikipedia.org/wiki/Orthogonal_matrix), so multiplying
them against a rotation matrix <img src="./tex/6423e0d54c2545769ad013e5f6a4cf94.svg?invert_in_darkmode" align=middle width=14.17800779999999pt height=22.55708729999998pt/> does not change its orthonormality. We can
pull them out of the maximization if we account for the reflection they _might_
incur: introduce <img src="./tex/84f085a8ee01510052b15faab48ecc9b.svg?invert_in_darkmode" align=middle width=139.41221085pt height=27.6567522pt/> with <img src="./tex/bc882f306e6b364898c14e65fcffdc53.svg?invert_in_darkmode" align=middle width=124.30338194999999pt height=27.91243950000002pt/>.
This implies that the optimal rotation for the original probklem is recovered
via <img src="./tex/923dfd11bdb9a8940133a2de8cf58616.svg?invert_in_darkmode" align=middle width=102.45405884999998pt height=27.91243950000002pt/>.  When we move the <img src="./tex/a3e6efe097bf3b70a46b0e3e3611d182.svg?invert_in_darkmode" align=middle width=53.47052864999999pt height=14.15524440000002pt/> inside, we now
look for an orthonormal matrix <img src="./tex/bae5a06f813149fce8f3c8a39745d442.svg?invert_in_darkmode" align=middle width=65.96338484999998pt height=24.65753399999998pt/> that is a reflection (if
<img src="./tex/d7696a03702713fc787fb978fa395ce5.svg?invert_in_darkmode" align=middle width=108.68696025pt height=27.91243950000002pt/>) or a rotation (if <img src="./tex/e535fddca4193bc96820d56b65569e96.svg?invert_in_darkmode" align=middle width=95.90152769999999pt height=27.91243950000002pt/>):

<p align="center"><img src="./tex/077b1c3e74148e3a3e68860032b4b776.svg?invert_in_darkmode" align=middle width=323.18850795pt height=49.315569599999996pt/></p>


This ensures that as a result <img src="./tex/7418159b714ed42bd664b73099a6311f.svg?invert_in_darkmode" align=middle width=20.913202199999986pt height=22.63846199999998pt/> will be a rotation: <img src="./tex/e1aa55f8d78af82fc6c2c53b6de68935.svg?invert_in_darkmode" align=middle width=77.44268894999999pt height=22.831056599999986pt/>.

> Recall that <img src="./tex/ad4f8ffa0b0d4f9b5cd0e611986d5870.svg?invert_in_darkmode" align=middle width=65.32531334999999pt height=26.76175259999998pt/> is a non-negative diagonal matrix of singular values
> sorted so that the smallest value is in the bottom right corner.

Because <img src="./tex/9f531c9f3f1ebeef802ced46eabb0336.svg?invert_in_darkmode" align=middle width=11.87217899999999pt height=22.465723500000017pt/> is orthonormal, each column (or row) of <img src="./tex/9f531c9f3f1ebeef802ced46eabb0336.svg?invert_in_darkmode" align=middle width=11.87217899999999pt height=22.465723500000017pt/> must have unit norm.
Placing a non-zero on the off-diagonal will get "killed" when multiplied by the
corresponding zero in <img src="./tex/7aed918aa12a276a602e30e90b0b109d.svg?invert_in_darkmode" align=middle width=9.98290094999999pt height=14.15524440000002pt/>. So the optimal choice of <img src="./tex/9f531c9f3f1ebeef802ced46eabb0336.svg?invert_in_darkmode" align=middle width=11.87217899999999pt height=22.465723500000017pt/> is to set all values to
zero except on the diagonal. If <img src="./tex/d7696a03702713fc787fb978fa395ce5.svg?invert_in_darkmode" align=middle width=108.68696025pt height=27.91243950000002pt/>, then we should set
one (and only one) of these values to <img src="./tex/e11a8cfcf953c683196d7a48677b2277.svg?invert_in_darkmode" align=middle width=21.00464354999999pt height=21.18721440000001pt/>. The best choice is the bottom right
corner since that will multiply against the smallest singular value in <img src="./tex/9f695bec305c31490f90808856401395.svg?invert_in_darkmode" align=middle width=17.35165739999999pt height=24.657735299999988pt/> (add
negatively affect the maximization the least):

<p align="center"><img src="./tex/2f2ebb3bacd43c9e9e6345a28182b942.svg?invert_in_darkmode" align=middle width=228.15813734999998pt height=69.0417981pt/></p>


Finally, we have a formula for our optimal rotation:

<p align="center"><img src="./tex/4ee960ed89d4b6ef9aa933ba448fa2e5.svg?invert_in_darkmode" align=middle width=100.28508765pt height=14.77813755pt/></p>


### Iterative linearization for point-to-plane rigid matching

We require that <img src="./tex/6423e0d54c2545769ad013e5f6a4cf94.svg?invert_in_darkmode" align=middle width=14.17800779999999pt height=22.55708729999998pt/> is not just any <img src="./tex/46e42d6ebfb1f8b50fe3a47153d01cd2.svg?invert_in_darkmode" align=middle width=36.52961069999999pt height=21.18721440000001pt/> matrix, but a rotation matrix. If we simply optimize the 9 matrix entries of <img src="./tex/6423e0d54c2545769ad013e5f6a4cf94.svg?invert_in_darkmode" align=middle width=14.17800779999999pt height=22.55708729999998pt/> directly, the result will be far from a rotation matrix. Instead, we _linearize_ the constraint that <img src="./tex/6423e0d54c2545769ad013e5f6a4cf94.svg?invert_in_darkmode" align=middle width=14.17800779999999pt height=22.55708729999998pt/> stays a rotation matrix and work with a reduced set of variables.

Any rotation <img src="./tex/6423e0d54c2545769ad013e5f6a4cf94.svg?invert_in_darkmode" align=middle width=14.17800779999999pt height=22.55708729999998pt/> in 3D can be written as scalar rotation angle <img src="./tex/27e556cf3caa0673ac49a8f0de3c73ca.svg?invert_in_darkmode" align=middle width=8.17352744999999pt height=22.831056599999986pt/> around a rotation axis defined by a unit vector <img src="./tex/38b405fb29be61635751df6f2ad66799.svg?invert_in_darkmode" align=middle width=52.431327299999985pt height=26.76175259999998pt/>.

If <img src="./tex/96c3741f62813242aaf853b649cf3780.svg?invert_in_darkmode" align=middle width=114.55434869999998pt height=24.65753399999998pt/>, we know that a rotation by <img src="./tex/27e556cf3caa0673ac49a8f0de3c73ca.svg?invert_in_darkmode" align=middle width=8.17352744999999pt height=22.831056599999986pt/> can be written as:

<p align="center"><img src="./tex/db60da1aeacc11d5cb8c2d8ed2486a28.svg?invert_in_darkmode" align=middle width=214.36958565pt height=59.1786591pt/></p>

For a general, rotation axis <img src="./tex/522c69add21191fefe7da3f76ad92547.svg?invert_in_darkmode" align=middle width=13.91546639999999pt height=23.28771720000001pt/>, we can write a generalized <a id=aa>_**axis-angle to matrix formula**_</a>:
<p align="center"><img src="./tex/69219d3de030a57e12eb79320ade7d3e.svg?invert_in_darkmode" align=middle width=602.1715788pt height=85.00052385000001pt/></p>

where <img src="./tex/c09091921b6bc51fd934869690606bfe.svg?invert_in_darkmode" align=middle width=75.14827979999998pt height=26.76175259999998pt/> is the [skew-symmetric](https://en.wikipedia.org/wiki/Skew-symmetric_matrix) [cross product matrix](https://en.wikipedia.org/wiki/Cross_product#Conversion_to_matrix_multiplication) of <img src="./tex/522c69add21191fefe7da3f76ad92547.svg?invert_in_darkmode" align=middle width=13.91546639999999pt height=23.28771720000001pt/> so that <img src="./tex/dd8d347bc29ac2275a6105fbc8615132.svg?invert_in_darkmode" align=middle width=95.42179679999998pt height=23.28771720000001pt/>

In this form, we can linearize by considering a small change in <img src="./tex/27e556cf3caa0673ac49a8f0de3c73ca.svg?invert_in_darkmode" align=middle width=8.17352744999999pt height=22.831056599999986pt/> and <img src="./tex/522c69add21191fefe7da3f76ad92547.svg?invert_in_darkmode" align=middle width=13.91546639999999pt height=23.28771720000001pt/>:

<p align="center"><img src="./tex/7f7877cc3239356dcc4f92ef547f2159.svg?invert_in_darkmode" align=middle width=218.50181924999998pt height=59.1786591pt/></p>

By defining <img src="./tex/c1d4dd9f8ef43f6ae2f3d7c1d37ace0e.svg?invert_in_darkmode" align=middle width=53.19605774999998pt height=23.28771720000001pt/>, we can write this in terms of only three simple scalar variables:

<p align="center"><img src="./tex/872476e5e4054f2b8c9267a3427db08a.svg?invert_in_darkmode" align=middle width=184.74316574999997pt height=59.1786591pt/></p>


### Approximate point-to-point minimizer

If we apply our linearization of <img src="./tex/6423e0d54c2545769ad013e5f6a4cf94.svg?invert_in_darkmode" align=middle width=14.17800779999999pt height=22.55708729999998pt/> to the **point-to-point** distance
linearization of the matching energy, our minimization becomes:

<p align="center"><img src="./tex/fbd893d3494efa498ebd7d964a94a92d.svg?invert_in_darkmode" align=middle width=412.36099079999997pt height=62.53032225pt/></p>


This energy is quadratic in the translation vector <img src="./tex/f40598ec49a99f9a93c399f7dacc6d3e.svg?invert_in_darkmode" align=middle width=7.35155849999999pt height=20.87411699999998pt/> and the linearized
rotation angles <img src="./tex/eec04193aead51ded846dbae8c3b4953.svg?invert_in_darkmode" align=middle width=15.24170009999999pt height=14.15524440000002pt/>, <img src="./tex/86ef8ef82733de9ba07432e88dd9757e.svg?invert_in_darkmode" align=middle width=15.24170009999999pt height=14.15524440000002pt/> and <img src="./tex/86ef8ef82733de9ba07432e88dd9757e.svg?invert_in_darkmode" align=middle width=15.24170009999999pt height=14.15524440000002pt/>. Let's gather these degrees of freedom into a
vector of unknowns: <img src="./tex/1fdaedd826e241163120cab972018fe4.svg?invert_in_darkmode" align=middle width=118.80099329999999pt height=27.91243950000002pt/>. Then we can write our
problem in summation form as:

<p align="center"><img src="./tex/eaccf6a2d1872cb0bdbbaa314c6df49a.svg?invert_in_darkmode" align=middle width=500.35957289999993pt height=62.53032225pt/></p>


This can be written compactly in matrix form as:

<p align="center"><img src="./tex/9d2cc4a31a2eec7830d2ee99138f8551.svg?invert_in_darkmode" align=middle width=496.9322391pt height=112.6677849pt/></p>

where we introduce the matrix <img src="./tex/7eab9509bd92f8ab739bdfe4383e7249.svg?invert_in_darkmode" align=middle width=76.90062764999999pt height=27.91243950000002pt/> that gathers the columns
<img src="./tex/9b01119ffd35fe6d8a8795a24fc11616.svg?invert_in_darkmode" align=middle width=18.943064249999992pt height=22.55708729999998pt/> of <img src="./tex/d05b996d2c08252f77613c25205a0f04.svg?invert_in_darkmode" align=middle width=14.29216634999999pt height=22.55708729999998pt/> and columns of ones <img src="./tex/0ab021958640a132ba4077ebcae7ec95.svg?invert_in_darkmode" align=middle width=48.68135084999999pt height=27.91243950000002pt/>.

This quadratic energy is minimized with its partial derivatives with respect to
entries in <img src="./tex/129c5b884ff47d80be4d6261a476e9f1.svg?invert_in_darkmode" align=middle width=10.502226899999991pt height=14.611878600000017pt/> are all zero:


<p align="center"><img src="./tex/45519f9d7e16159f36db93e8bf18b945.svg?invert_in_darkmode" align=middle width=512.4540267pt height=124.93263584999998pt/></p>



Solving this small <img src="./tex/6d89de257f7655e1691ec48411787787.svg?invert_in_darkmode" align=middle width=36.52961069999999pt height=21.18721440000001pt/> system gives us our translation vector <img src="./tex/f40598ec49a99f9a93c399f7dacc6d3e.svg?invert_in_darkmode" align=middle width=7.35155849999999pt height=20.87411699999998pt/> and the
linearized rotation angles <img src="./tex/eec04193aead51ded846dbae8c3b4953.svg?invert_in_darkmode" align=middle width=15.24170009999999pt height=14.15524440000002pt/>, <img src="./tex/86ef8ef82733de9ba07432e88dd9757e.svg?invert_in_darkmode" align=middle width=15.24170009999999pt height=14.15524440000002pt/> and <img src="./tex/86ef8ef82733de9ba07432e88dd9757e.svg?invert_in_darkmode" align=middle width=15.24170009999999pt height=14.15524440000002pt/>. If we simply assign 

<p align="center"><img src="./tex/46d4381fe98b9af4dc1f886f7d4b2d6b.svg?invert_in_darkmode" align=middle width=231.63804345pt height=59.1786591pt/></p>


then our transformation will _not_ be rigid. Instead, we should _recover_ the axis and angle of rotation from <img src="./tex/41f28962986ecdd9c1dc2af8b83fef84.svg?invert_in_darkmode" align=middle width=9.18943409999999pt height=14.611878600000017pt/> via <img src="./tex/d0a7030829b80628644fa618ce84fc92.svg?invert_in_darkmode" align=middle width=55.71901004999999pt height=24.65753399999998pt/> and <img src="./tex/a6581421740d0b194dfd7729e9a4bcc5.svg?invert_in_darkmode" align=middle width=61.41526379999999pt height=24.65753399999998pt/> and then update our rotation via the <a href=#aa>_**axis-angle to matrix formula**_ above</a>. Because we used a linearization of the rotation constraint, we cannot assume that we have _successful_ found the best rigid transformation. To converge on an optimal value, must set <img src="./tex/89b906c53bdb5b977d1e873cf0be3f7a.svg?invert_in_darkmode" align=middle width=79.82653799999999pt height=22.55708729999998pt/> and repeat this process (usually 5 times or so is sufficient).

#### Recovering a pure rotation from its linearization

> In an effort to provide an alternative from "Least-Squares Rigid Motion Using
> SVD" [Sorkine 2009], this derivation purposefully _avoids_ the [trace
> operator](https://en.wikipedia.org/wiki/Trace_(linear_algebra)) and its
> various nice properties.

If <img src="./tex/eec04193aead51ded846dbae8c3b4953.svg?invert_in_darkmode" align=middle width=15.24170009999999pt height=14.15524440000002pt/>, <img src="./tex/86ef8ef82733de9ba07432e88dd9757e.svg?invert_in_darkmode" align=middle width=15.24170009999999pt height=14.15524440000002pt/> and <img src="./tex/86ef8ef82733de9ba07432e88dd9757e.svg?invert_in_darkmode" align=middle width=15.24170009999999pt height=14.15524440000002pt/> are all small, then it may be safe to _interpret_ these
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
<img src="./tex/6b131ebac8b7c63ef31cb6925661ad5a.svg?invert_in_darkmode" align=middle width=89.21769614999998pt height=27.91243950000002pt/>, where <img src="./tex/9b2a1625a0911294931a3f1c4aeea9ec.svg?invert_in_darkmode" align=middle width=91.74635744999999pt height=26.76175259999998pt/> are orthonormal matrices
and <img src="./tex/ad4f8ffa0b0d4f9b5cd0e611986d5870.svg?invert_in_darkmode" align=middle width=65.32531334999999pt height=26.76175259999998pt/> is a non-negative diagonal matrix:

<p align="center"><img src="./tex/53cafbd2848f7f70d7b4c3f03d9f6c94.svg?invert_in_darkmode" align=middle width=207.59633235pt height=32.2210416pt/></p>


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
>    transposes: <img src="./tex/30732bc83cf909d6edd8de25f2645089.svg?invert_in_darkmode" align=middle width=162.83046779999998pt height=27.91243950000002pt/>,
>  5. The Frobenius inner product can be written as a [dot
>    product](://en.wikipedia.org/wiki/Dot_product) of
>    [vectorized](https://en.wikipedia.org/wiki/Vectorization_(mathematics))
>    matrices: <img src="./tex/018ac9f687643eb5a4d16c0a4ab4bc06.svg?invert_in_darkmode" align=middle width=353.64434324999996pt height=27.91243950000002pt/>,
>  6. Properties 3., 4., and 5. imply that Frobenius inner product of a matrix
>    <img src="./tex/96458543dc5abd380904d95cae6aa2bc.svg?invert_in_darkmode" align=middle width=14.29216634999999pt height=22.55708729999998pt/> and the matrix product of matrix <img src="./tex/ff44d867a998c08241beb49b30148782.svg?invert_in_darkmode" align=middle width=13.44741914999999pt height=22.55708729999998pt/> and <img src="./tex/12d3ebda1a212bd89197298f60cf3ce1.svg?invert_in_darkmode" align=middle width=13.652895299999988pt height=22.55708729999998pt/> is equal to the
>    Frobenius inner product of the matrix product of the transpose of <img src="./tex/ff44d867a998c08241beb49b30148782.svg?invert_in_darkmode" align=middle width=13.44741914999999pt height=22.55708729999998pt/> and
>    <img src="./tex/96458543dc5abd380904d95cae6aa2bc.svg?invert_in_darkmode" align=middle width=14.29216634999999pt height=22.55708729999998pt/>  and the matrix <img src="./tex/12d3ebda1a212bd89197298f60cf3ce1.svg?invert_in_darkmode" align=middle width=13.652895299999988pt height=22.55708729999998pt/>:
>    <img src="./tex/4d9c6a6f9c0e0098e929887dddf57802.svg?invert_in_darkmode" align=middle width=702.93655245pt height=47.671235699999976pt/>.
>  

<p align="center"><img src="./tex/ef54fe83b00bd0937ccf189fac5b7fa3.svg?invert_in_darkmode" align=middle width=205.4958873pt height=32.2210416pt/></p>


Now, <img src="./tex/35531be55273dc37ee90083451d089ff.svg?invert_in_darkmode" align=middle width=14.54330789999999pt height=22.55708729999998pt/> and <img src="./tex/26eb59da31fb48cb17abfe4c6dc80375.svg?invert_in_darkmode" align=middle width=14.554737449999989pt height=22.55708729999998pt/> are both
[orthonormal](https://en.wikipedia.org/wiki/Orthogonal_matrix), so multiplying
them against a rotation matrix <img src="./tex/6423e0d54c2545769ad013e5f6a4cf94.svg?invert_in_darkmode" align=middle width=14.17800779999999pt height=22.55708729999998pt/> does not change its orthonormality. We can
pull them out of the maximization if we account for the reflection they _might_
incur: introduce <img src="./tex/84f085a8ee01510052b15faab48ecc9b.svg?invert_in_darkmode" align=middle width=139.41221085pt height=27.6567522pt/> with <img src="./tex/bc882f306e6b364898c14e65fcffdc53.svg?invert_in_darkmode" align=middle width=124.30338194999999pt height=27.91243950000002pt/>.
This implies that the optimal rotation for the original probklem is recovered
via <img src="./tex/923dfd11bdb9a8940133a2de8cf58616.svg?invert_in_darkmode" align=middle width=102.45405884999998pt height=27.91243950000002pt/>.  When we move the <img src="./tex/a3e6efe097bf3b70a46b0e3e3611d182.svg?invert_in_darkmode" align=middle width=53.47052864999999pt height=14.15524440000002pt/> inside, we now
look for an orthonormal matrix <img src="./tex/bae5a06f813149fce8f3c8a39745d442.svg?invert_in_darkmode" align=middle width=65.96338484999998pt height=24.65753399999998pt/> that is a reflection (if
<img src="./tex/d7696a03702713fc787fb978fa395ce5.svg?invert_in_darkmode" align=middle width=108.68696025pt height=27.91243950000002pt/>) or a rotation (if <img src="./tex/e535fddca4193bc96820d56b65569e96.svg?invert_in_darkmode" align=middle width=95.90152769999999pt height=27.91243950000002pt/>):

<p align="center"><img src="./tex/077b1c3e74148e3a3e68860032b4b776.svg?invert_in_darkmode" align=middle width=323.18850795pt height=49.315569599999996pt/></p>


This ensures that as a result <img src="./tex/7418159b714ed42bd664b73099a6311f.svg?invert_in_darkmode" align=middle width=20.913202199999986pt height=22.63846199999998pt/> will be a rotation: <img src="./tex/e1aa55f8d78af82fc6c2c53b6de68935.svg?invert_in_darkmode" align=middle width=77.44268894999999pt height=22.831056599999986pt/>.

> Recall that <img src="./tex/ad4f8ffa0b0d4f9b5cd0e611986d5870.svg?invert_in_darkmode" align=middle width=65.32531334999999pt height=26.76175259999998pt/> is a non-negative diagonal matrix of singular values
> sorted so that the smallest value is in the bottom right corner.

Because <img src="./tex/9f531c9f3f1ebeef802ced46eabb0336.svg?invert_in_darkmode" align=middle width=11.87217899999999pt height=22.465723500000017pt/> is orthonormal, each column (or row) of <img src="./tex/9f531c9f3f1ebeef802ced46eabb0336.svg?invert_in_darkmode" align=middle width=11.87217899999999pt height=22.465723500000017pt/> must have unit norm.
Placing a non-zero on the off-diagonal will get "killed" when multiplied by the
corresponding zero in <img src="./tex/7aed918aa12a276a602e30e90b0b109d.svg?invert_in_darkmode" align=middle width=9.98290094999999pt height=14.15524440000002pt/>. So the optimal choice of <img src="./tex/9f531c9f3f1ebeef802ced46eabb0336.svg?invert_in_darkmode" align=middle width=11.87217899999999pt height=22.465723500000017pt/> is to set all values to
zero except on the diagonal. If <img src="./tex/d7696a03702713fc787fb978fa395ce5.svg?invert_in_darkmode" align=middle width=108.68696025pt height=27.91243950000002pt/>, then we should set
one (and only one) of these values to <img src="./tex/e11a8cfcf953c683196d7a48677b2277.svg?invert_in_darkmode" align=middle width=21.00464354999999pt height=21.18721440000001pt/>. The best choice is the bottom right
corner since that will multiply against the smallest singular value in <img src="./tex/9f695bec305c31490f90808856401395.svg?invert_in_darkmode" align=middle width=17.35165739999999pt height=24.657735299999988pt/> (add
negatively affect the maximization the least):

<p align="center"><img src="./tex/2f2ebb3bacd43c9e9e6345a28182b942.svg?invert_in_darkmode" align=middle width=228.15813734999998pt height=69.0417981pt/></p>


Finally, we have a formula for our optimal rotation:

<p align="center"><img src="./tex/4ee960ed89d4b6ef9aa933ba448fa2e5.svg?invert_in_darkmode" align=middle width=100.28508765pt height=14.77813755pt/></p>


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
> <img src="./tex/f40598ec49a99f9a93c399f7dacc6d3e.svg?invert_in_darkmode" align=middle width=7.35155849999999pt height=20.87411699999998pt/>. We can immediately solve for the optimal <img src="./tex/2dbfbbc26f524676be39b3f3df0ad0bc.svg?invert_in_darkmode" align=middle width=14.08675289999999pt height=22.63846199999998pt/> — leaving <img src="./tex/6423e0d54c2545769ad013e5f6a4cf94.svg?invert_in_darkmode" align=middle width=14.17800779999999pt height=22.55708729999998pt/> as an unknown — by
> setting all derivatives with respect to unknowns in <img src="./tex/f40598ec49a99f9a93c399f7dacc6d3e.svg?invert_in_darkmode" align=middle width=7.35155849999999pt height=20.87411699999998pt/> to zero:
> 
> <p align="center"><img src="./tex/9e362387214ebe1af7e853291abc1f78.svg?invert_in_darkmode" align=middle width=342.9363135pt height=74.02030679999999pt/></p>

> where <img src="./tex/2ac7c6ff1056fe3737fa4395e2efb6c5.svg?invert_in_darkmode" align=middle width=48.68135084999999pt height=27.91243950000002pt/> is a vector ones. Setting the partial derivative with
> respect to <img src="./tex/f40598ec49a99f9a93c399f7dacc6d3e.svg?invert_in_darkmode" align=middle width=7.35155849999999pt height=20.87411699999998pt/> of this
> quadratic energy to zero finds the minimum:
> <p align="center"><img src="./tex/b3bb46beb0a710d869c642bc8c1439c2.svg?invert_in_darkmode" align=middle width=284.0789424pt height=58.361920649999995pt/></p>

> 
> Rearranging terms above reveals that the optimal <img src="./tex/f40598ec49a99f9a93c399f7dacc6d3e.svg?invert_in_darkmode" align=middle width=7.35155849999999pt height=20.87411699999998pt/> is the vector aligning
> the [centroids](https://en.wikipedia.org/wiki/Centroid) of the points in <img src="./tex/384591906555413c452c93e493b2d4ec.svg?invert_in_darkmode" align=middle width=12.92230829999999pt height=22.55708729999998pt/>
> and the points in <img src="./tex/d05b996d2c08252f77613c25205a0f04.svg?invert_in_darkmode" align=middle width=14.29216634999999pt height=22.55708729999998pt/> rotated by the — yet-unknown — <img src="./tex/6423e0d54c2545769ad013e5f6a4cf94.svg?invert_in_darkmode" align=middle width=14.17800779999999pt height=22.55708729999998pt/>. Introducing
> variables for the respective centroids <img src="./tex/64af2fa219a12c6bb69c25da58441a8f.svg?invert_in_darkmode" align=middle width=120.17019629999999pt height=32.51169900000002pt/> and <img src="./tex/00085c1c3a7306fc9ff9974f9fe861dc.svg?invert_in_darkmode" align=middle width=97.20234314999998pt height=32.51169900000002pt/>, we can write the
> formula for the optimal  <img src="./tex/f40598ec49a99f9a93c399f7dacc6d3e.svg?invert_in_darkmode" align=middle width=7.35155849999999pt height=20.87411699999998pt/>:
> 
> <p align="center"><img src="./tex/f96a3a519c3060b59063229376b69f86.svg?invert_in_darkmode" align=middle width=205.19999115pt height=59.93265464999999pt/></p>

> 
> Now we have a formula for the optimal translation vector <img src="./tex/f40598ec49a99f9a93c399f7dacc6d3e.svg?invert_in_darkmode" align=middle width=7.35155849999999pt height=20.87411699999998pt/> in terms of the
> unknown rotation <img src="./tex/6423e0d54c2545769ad013e5f6a4cf94.svg?invert_in_darkmode" align=middle width=14.17800779999999pt height=22.55708729999998pt/>. Let us
> [substitute](https://en.wikipedia.org/wiki/Substitution_(algebra)) this formula
> for all occurrences of <img src="./tex/f40598ec49a99f9a93c399f7dacc6d3e.svg?invert_in_darkmode" align=middle width=7.35155849999999pt height=20.87411699999998pt/> in our energy written in its original summation
> form:
> 
> <p align="center"><img src="./tex/f63f9caf84eb6506928a49a848fac060.svg?invert_in_darkmode" align=middle width=1085.5236711pt height=36.50245665pt/></p>

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
> <p align="center"><img src="./tex/78e20bbb1884f37158772ccc9d08d39e.svg?invert_in_darkmode" align=middle width=410.23402035pt height=235.53802799999997pt/></p>

> 
> Letting <img src="./tex/818e261a8ac45a73aabd65ba9bba3a1b.svg?invert_in_darkmode" align=middle width=80.91279569999999pt height=36.98604359999998pt/> we can now follow the
> steps above using [singular value
> decomposition](https://en.wikipedia.org/wiki/Singular_value_decomposition) to
> find the optimal <img src="./tex/6423e0d54c2545769ad013e5f6a4cf94.svg?invert_in_darkmode" align=middle width=14.17800779999999pt height=22.55708729999998pt/>.

### Approximate point-to-plane minimizer

If we apply our linearization of <img src="./tex/6423e0d54c2545769ad013e5f6a4cf94.svg?invert_in_darkmode" align=middle width=14.17800779999999pt height=22.55708729999998pt/> to the **point-to-plane** distance
linearization of the matching energy, our minimization is:

<p align="center"><img src="./tex/a210a372dddce4251d005af80bfe1a79.svg?invert_in_darkmode" align=middle width=515.01854415pt height=62.53027769999999pt/></p>


We can follow similar steps as above. Let's gather a vector of unknowns: <img src="./tex/62521b121b6bcb4b180427b4424827f3.svg?invert_in_darkmode" align=middle width=146.7064203pt height=27.91243950000002pt/>. Then we can write our problem in summation form
as:

<p align="center"><img src="./tex/a5c0b6da39a1b42df1dfcca983aece61.svg?invert_in_darkmode" align=middle width=569.58322245pt height=62.53027769999999pt/></p>


This can be written compactly in matrix form as:

<p align="center"><img src="./tex/3d04698e460f14851ead5bf5b511e4af.svg?invert_in_darkmode" align=middle width=529.68035175pt height=62.53027769999999pt/></p>


where <img src="./tex/d215374485dd1f9e71eba3fa2f247afe.svg?invert_in_darkmode" align=middle width=19.44535064999999pt height=22.55708729999998pt/> is the ith column from the matrix of normals <img src="./tex/5e65e079e680711d68cb1f973f9b02d9.svg?invert_in_darkmode" align=middle width=70.85036639999998pt height=27.91243950000002pt/>,
<img src="./tex/13f548e5efa74d654479d62c25ff2fd8.svg?invert_in_darkmode" align=middle width=53.16219644999998pt height=24.65753399999998pt/> [creates a diagonal
matrix](https://en.wikipedia.org/wiki/Diagonal_matrix#Matrix_operations) from a
vector, and <img src="./tex/7eab9509bd92f8ab739bdfe4383e7249.svg?invert_in_darkmode" align=middle width=76.90062764999999pt height=27.91243950000002pt/> is the same as above.

This energy is quadratic in <img src="./tex/129c5b884ff47d80be4d6261a476e9f1.svg?invert_in_darkmode" align=middle width=10.502226899999991pt height=14.611878600000017pt/> and can be solve by setting all partial
derivatives with respect to <img src="./tex/129c5b884ff47d80be4d6261a476e9f1.svg?invert_in_darkmode" align=middle width=10.502226899999991pt height=14.611878600000017pt/> to zero.

> ### Closed-form point-to-planer minimizer
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

<p align="center"><img src="./tex/09a1e9943500d3845803cab7d3c0a426.svg?invert_in_darkmode" align=middle width=256.7154414pt height=16.438356pt/></p>


where <img src="./tex/c5ca62d9d8a39f2e2f82d053f4ec2275.svg?invert_in_darkmode" align=middle width=38.611176449999995pt height=14.15524440000002pt/> are uniformly sampled from the unit interval <img src="./tex/acf5ce819219b95070be2dbeb8a671e9.svg?invert_in_darkmode" align=middle width=32.87674994999999pt height=24.65753399999998pt/>. If <img src="./tex/f48e9ddd1da29c1be9cbfc3f7ea192b9.svg?invert_in_darkmode" align=middle width=82.35521579999998pt height=21.18721440000001pt/>
then the point <img src="./tex/b0ea07dc5c00127344a1cad40467b8de.svg?invert_in_darkmode" align=middle width=9.97711604999999pt height=14.611878600000017pt/> above will lie in the reflected triangle rather than the
original one. In this case, preprocess <img src="./tex/eec04193aead51ded846dbae8c3b4953.svg?invert_in_darkmode" align=middle width=15.24170009999999pt height=14.15524440000002pt/> and <img src="./tex/86ef8ef82733de9ba07432e88dd9757e.svg?invert_in_darkmode" align=middle width=15.24170009999999pt height=14.15524440000002pt/> by setting <img src="./tex/9c00a8bc01f19fc667b68ba3bb18ee18.svg?invert_in_darkmode" align=middle width=85.18629404999999pt height=21.18721440000001pt/> and
<img src="./tex/627a30497b1285c8664e46597b798464.svg?invert_in_darkmode" align=middle width=85.18629404999999pt height=21.18721440000001pt/> to reflect the point <img src="./tex/b0ea07dc5c00127344a1cad40467b8de.svg?invert_in_darkmode" align=middle width=9.97711604999999pt height=14.611878600000017pt/> back into the original triangle.

### Area-weighted random sampling of triangles

Assuming we know how to draw a _continuous_ uniform random variable <img src="./tex/86ef8ef82733de9ba07432e88dd9757e.svg?invert_in_darkmode" align=middle width=15.24170009999999pt height=14.15524440000002pt/> from
the unit interval <img src="./tex/acf5ce819219b95070be2dbeb8a671e9.svg?invert_in_darkmode" align=middle width=32.87674994999999pt height=24.65753399999998pt/>, we would now like to draw a _discrete_ random
triangle index <img src="./tex/2f118ee06d05f3c2d98361d9c30e38ce.svg?invert_in_darkmode" align=middle width=11.889314249999991pt height=22.465723500000017pt/> from the sequence <img src="./tex/12ae120cf77557772d65127ef3360fb5.svg?invert_in_darkmode" align=middle width=37.26407684999999pt height=21.18721440000001pt/> with likelihood proportional to
the relative area of each triangle in the mesh.

We can achieve this by first computing the [cumulative
sum](https://en.wikipedia.org/wiki/Running_total) <img src="./tex/386003f198eaba07b624962ab7a894b0.svg?invert_in_darkmode" align=middle width=57.281063399999994pt height=22.648391699999998pt/> of the relative
areas:

<p align="center"><img src="./tex/2ff5513f2ad6ffbf75f8fa24d37017d0.svg?invert_in_darkmode" align=middle width=119.79076395pt height=36.09514755pt/></p>


Then our random index is found by identifying the first entry in <img src="./tex/12d3ebda1a212bd89197298f60cf3ce1.svg?invert_in_darkmode" align=middle width=13.652895299999988pt height=22.55708729999998pt/> whose
value is greater than a uniform random variable <img src="./tex/86ef8ef82733de9ba07432e88dd9757e.svg?invert_in_darkmode" align=middle width=15.24170009999999pt height=14.15524440000002pt/>. Since <img src="./tex/12d3ebda1a212bd89197298f60cf3ce1.svg?invert_in_darkmode" align=middle width=13.652895299999988pt height=22.55708729999998pt/> is sorted,
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
