# Notes on _Putting Rigid Bodies to Rest_

* **WARNING:** This is a continuously evolving document. You may encounter incorrect info...

## Table of Contents

* [Abstract](#abstract)
* [Intro](#intro)
* [Algorithm](#algorithm)
  * [Derivation](#derivation)
  * [Tracing Paths](#tracing-paths)
  * [Categorize Paths With Morse-Smale Complex](#categorize-paths-with-morse-smale-complex)
  * [Trace Separatrix on Gauss Map](#trace-separatrix-on-gauss-map)
* [Manipulating the Probabilities](#manipulating-the-probabilities)
* [Inverse Design](#inverse-design)
* [Appendix](#appendix)

## Abstract

* _What to solve?_
  * Find all resting configs of 3D rigid body
  * Method is differentiable i.e. two-way solve
  * Forward version of method finds rest configs
  * Inverse version designs shapes with desired rest configs

* _How to solve?_
  * Compute probabilities for all rest configs
  * Use geometry instead of slower physical sim
  * Specifically, minimize distance from center of mass to ground
  * Use Morse-Smale (MS) complex over Gauss map

## Intro

* Assume body is rigid
* Ignore dynamic effects (e.g. velocity)
* Works with no assumptions on initial geometry
  * e.g. no need to be manifold
* Simplest case:
  * To compute probs of stable face configs...
    * Use normalized solid angles of each face of convex hull (hint, hint...)
* Related to fabrication tasks, can produce dice

## Algorithm

* _Input_
  * $\Omega$ = 3D shape (as mesh, point cloud, etc)
    * Assume non-negative density (need not be homogeneous)
  * $c$ = Center of mass (3D vector)
    * Need not be mesh barycenter
    * Lies within convex hull $H$
      * **convex hull** = shape with no inward dips that encloses mesh
* _Output_
  * $p$ = Array of $n$ probabilities
    * $n$ = num of faces of convex hull $H$
    * $p_f$ = probability in $p$ array for face $f$ in convex hull $H$
      * Tells chances of resting on face $f$ from random initial orientation
      * Each $p_f$ ranges from 0 to 1, i.e. $p_f \in [0, 1]$
      * Sum over all probabilities is 1, i.e. $\sum_{f \in H}p_f = 1$

### Derivation

* Minimize $R$ and $t$ in $\int_\Omega \rho(Rx + t) \cdot \hat{z} dV$
  * subject to $x_z ≥ 0, \forall x \in \Omega$
    * $\Omega$ = 3D shape
    * $\rho$ = density of shape (≥ 0)
    * $R$ = rotation during freefall (as 3x3 matrix)
    * $x$ = vertex position on shape (as 3D vector)
    * $t$ = translation during freefall (as 3D vector)
    * $\hat{z}$ = opposite dir of gravity = (0, 0, 1)
      * _Gravity points down, $\hat{z}$ points up_
    * V = volume of shape
    * Constraint says all z components of vertices must lie above ground at $z = 0$
* Can simplify integral to $(Rc + t) \cdot \hat{z}$
  * $c$ = center of mass
  * What remains is "height" (along z-axis) of transformed center of mass
  * Ignore translation b\c shape falls directly to ground
* For any rotation R,
  * $\hat{n} = R^{-1}\hat{z}$
    * _i.e. transform $\hat{z}$ from world space to object space_
    * $\hat{n}$ = unit vector pointing opposite to gravity in object space
      * _i.e. $\hat{n}$ points up_
  * Map $\hat{n}$ to face/edge/vertex of convex hull $H$ based on Gauss map
  * [**Gauss map**](#explanation) = maps surface vertex => dir of normal
    * Gauss map is a _unit sphere_
    * Gauss map of $H$ is dual graph that maps
      * Faces => points
      * Edges => "great arcs" (edges b/n sphere patches)
      * Vertices => "spherical polygons" (patches on sphere)
* **Gauss image** = points || great arcs || spherical polygons
* If hull vertex $x_j$ is resting on ground,
  * Get upward dir $\hat{z}$ by rotating normal $\hat{n}$ by R
  * **Height of center of mass (wrt $x_j$)**
    * $U_j(\hat{n}) = (c - x_j) \cdot \hat{n}$
      * i.e. scalar projection (scaling factor) of vector from $x_j$ to $c$ along $\hat{n}$
* For all unique vertices, edges, faces of convex hull $H$ that it could land on
  * $U(\hat{n}) = \sum_j \delta_j(\hat{n}) U_j(\hat{n})$
  * $\delta_j(\hat{n})$
    * 1 if $\hat{n}$ is on j's patch on Gauss map
    * $1/2$ if $\hat{n}$ is on edge of j's patch on Gauss map
    * $1/3$ if $\hat{n}$ is on boundary vertex of j's patch on Gauss map
    * else, 0
* BASICALLY, minimize $\hat{n}$ in $U(\hat{n})$

### Tracing Paths

* Given initial upward orientation $\hat{n_0}$,
  * Rolling a shape = Rotate in direction that minimizes U until rest
* Gauss map relates surface vertices to normals
* Note that since normals are unit length, _Gauss map is unit sphere_
* **You can "trace" paths on the Gauss map to minimize derivatives**
* Edges b/n patches are discontinuities in vector field of U
* GOAL: Solve for smallest $\partial U/\partial\hat{n}$
* Use **Appendix A, algorithm 1** = to trace all paths

#### Vertex

1. Locate patch on Gauss map corresponding to vertex $i$
2. Start from $\hat{n}$ in patch of $i$
3. Go along direction of $-\partial U/\partial\hat{n}$
4. Continue until you intersect with edge of patch

#### Edge

* Take vertices $i$ and $j$ which are endpoints of edge $ij$
* Locate their patches on Gauss map
* Need to watch out for edges between patches
  * They are discontinuities with undefined gradients!
* But tracing _along_ edge between patches is safe
* $\hat{n}$ is on edge between patches = convex hull rests on edge
* If resting on hull edge $ij$, there are 2 cases:
    1. Wheel edge (E1)
       * Shape $\Omega$ will rotate about either vertex $i$ or vertex $j$
       * Depends on contributions of $\partial U_i/\partial\hat{n}$ and $\partial U_j/\partial\hat{n}$
    2. Hinge edge (E2)
       * Shape $\Omega$ with rotate about edge $ij$ as if edge was a hinge
       * This is tracing along edge between patches (which is safe)
* How to check if an edge is E1 or E2 (wheel or hinge)?
    1. Take plane L containing $x_i$ and $x_j$
       * Impl tip: build eqn of plane L using normal and either $x_i$ or $x_j$
    2. Split intoi three strips, one parallel line thru $x_i$, one parallel line thru $x_j$
       * Both lines must be _orthogonal_ to edge $ij$
    3. Project center of mass $c$ onto L to get $\bar{c}$
       * How to project a point to a plane?
           * Get vector from point on plane (either $x_i$ or $x_j$) to $c$
               * $d = c - x_i$
           * Extract normal component of difference vector $d$
               * $d_{\hat{n}} = (d \cdot \hat{n}) \hat{n}$
           * Get difference of $c$ and $d_{\hat{n}}$ to get $\bar{c}$
               * $\bar{c} = c - d_{\hat{n}}$
    4. If $\bar{c}$ is in section between parallel lines, _edge is wheel (E1)_
    5. Else, _edge is hinge (E2)_
* If edge is wheel (E1), trace within patch of identified vertex (either $i$ or $j$)
* If edge is hinge (E2),
  * roll to next incident face based on direction of $\partial U/\partial\hat{n}$ along edge between patches

#### Face

* If $\hat{n}$ = normal of face $f$, i.e. $\hat{n}$ is vertex of Gauss map, there are three cases:
    1. Face is stable (F0, rolling stops)
    2. Face rolls onto vertex (F1, like a cartwheel)
    3. Face rolls onto edge (F2, like a hinge)
* How to check a face is F0 or F1 or F2?
    1. Split plane L of this face into 7 parts
        * Build triangle from each edge endpoint
        * Build three infinite strips away from triangle from each endpoint with parallel lines
        * Leave three "cone" regions between strips
    2. Project center of mass $c$ onto L as $\bar{c}$ (refer to earlier steps)
    3. If $\bar{c}$ is on strip, F2 (roll on edge)
    4. If $\bar{c}$ is on cone, F1 (roll on vertex)
    5. Else, F0

### Categorize Paths with Morse-Smale Complex

* **Morse-Smale complex** = partitions 2D domain of func into cells bounded by separatrices
* **Separatrix** = curve that connects saddle points to local minima/maxima
* **Ascending manifold** = union of cells that surround local minimum
* Local minima
  * Occur at stable hull faces (F0)
  * normal $\hat{n}_i$ for every face $i$ classified as F0
* Local maxima
  * Occur at unstable hull vertices (F1 or F2)
  * Compute max height of center of mass $c$
    * $\hat{n}_j = \argmin_{\hat{n}}U_j(\hat{n})=(x_j-c)\cdot\hat{n}$
    * if and only if maximizing normal is in dual graph
* Saddle points
  * Only on E2 (hinge edges)
  * $\hat{n}_{ij}^{*}=c-c_{ij}$
  * $c_{ij}=x_i+\frac{(c-x_i)\cdot v_{ij}} {v_{ij}\cdot v_{ij}}\cdot v_{ij}$
    * $c_{ij}$ is projection of center of mass $c$ on edge $ij$
    * $\hat{n}_{ij}^{*}$ is saddle point iff it lies in E2 edge on Gauss map
    * $v_{ij}=x_j - x_i$ is the edge vector

### Trace Separatrix on Gauss Map

* Refer to **Appendix A, algorithms 2 and 3**
* Separatrix doubles as gradient flow line
* Steps:
  * Start at saddle point on edge $ij$
  * Follow steeper gradient (either $\partial U_i/\partial \hat{n}$ or $\partial U_j/\partial \hat{n}$)
  * Pass through intersections with great arcs (edges b/n patches) until reaching local min/max
* Requirements
  * Precompute all max vertices, saddle edges, min (stable) faces
    * Only needs single pass
  * Subroutine for intersections w/great arcs (edges b/n patches)
  * Finally compute area $A_f$ of ascending manifold
* FINALLY, $p_f = \frac{A_f}{4\pi}$
  * Alternately, compute $A_f$ as sum of signed spherical triangle areas
    * $A_f = \sum_i \Omega(\hat{n}, u_i, u_{i + 1})$
      * $\Omega(\hat{n}, u_i, u_{i + 1})$ = signed area of spherical triangle w/vertices ${\hat{n}, u_i, u_{i + 1}}$
      * $\hat{n}$ = any point on sphere (consider as origin), also is normal of face $f$
      * How to compute signed spherical triangle areas?
        * Convert vertices from Cartesian to spherical
          * Remember Gauss map is unit sphere, so $r = 1$!
        * Compute double integral to get surface area patch?
          * TODO… arc length formulae might help?

#### Special Cases

* All faces of $H$ are _stable_ = all vertices of $H$ are _unstable_
  * Thus, $p_f$ = normalized solid angle of face $f$ seen from center of mass $c$

## Manipulating the Probabilities

* TODO...

## Inverse Design

* TODO...

## Rigid Body Simulation Validation

* Use Bullet for fast output, RigidIPC for more accurate output
* TODO...

## Appendix

### Curvature

* _Why discuss curvature?_
  * It's a core part of the definition of a Gauss map
* Resources
  * Short [notes](http://mesh.brown.edu/dgp/pdfs/Garland-diffgeom.pdf) by Garland (ignore Frenet formulae)
  * Neat and short [explanation](https://www.youtube.com/watch?v=UYiAlYlSwBo) on YouTube
  * A [longer one](https://www.youtube.com/watch?v=NlU1m-OfumE) by Crane (or honestly just skim thru his lectures in the Discrete Differential Geometry playlist 😉)

#### Explanation

* Consider a unit normal vector $\hat{n}$ of a point on a surface
  * Gather all unique surface normals and have them all share the same origin
  * They will form a sphere called the **Gauss map**
* Use the **shape operator** to transform a surface normal to the Gauss map
  * i.e. convert a surface normal to a sphere normal!
* How to build the shape operator?
  * Recall that the vector orthogonal to a normal is the **tangent**
  * Build three axes (aka basis vectors) with them
    * The normal $\hat{n}$, tangent $\hat{t}$, and bi-tangent $\hat{b}$
  * Bundle the vectors as columns to build a transformation matrix $S$

$$
  S = \begin{bmatrix}
        \hat{n}_x & \hat{t}_x & \hat{b}_x\\
        \hat{n}_y & \hat{t}_y & \hat{b}_y\\
        \hat{n}_z & \hat{t}_z & \hat{b}_z\\
      \end{bmatrix}
$$

* _Those three axes ($\hat{n}, \hat{t}, \hat{b}$) form a plane L, which is found on both the surface AND Gauss map!_
* To apply the shape operator on any surface point $x_s$ to the Gauss map $x_g$:
  * Just do $x_g = S \cdot x_s$
* _Problem:_ The plane L varies depending on the surface normal used as the basis...
  * _Solution:_ Extract constant properties of the matrix $S$, aka _invariants_
* What are the parts of matrix $S$ that don't change regardless of its elements?
  * Determinant ($\det$) = [definition is a bit complicated](https://en.wikipedia.org/wiki/Determinant)
  * Trace ($tr$) = sum of squares of diagonal elements of matrix
* We can now define the two types of curvature!!
  * **Gaussian curvature** = $\det(S)$
  * **Mean curvature** = $\frac{1}{2}tr(S)$
