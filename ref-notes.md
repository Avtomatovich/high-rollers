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
  * How to compute initial upward orientation?
    * i.e. how can we find an initial normal?
    * _Answer_ = sample from the Gauss map! (which is a unit sphere)
    * From the _Global Illumination Compendium_,
      * Generate a random point on sphere $(c_x, c_y, c_z, R)$ with density $p(\Theta)=\frac{1}{4\pi R^2}$ and $r_1$, $r_2$ are random numbers sampled from $[0, 1]$
        * $x = c_x + 2R\cos(2\pi r_1)\sqrt{r_2(1-r_2)}$
        * $y = c_y + 2R\sin(2\pi r_1)\sqrt{r_2(1-r_2)}$
        * $z = c_z + R(1-2r_2)$
          * where $(c_x, c_y, c_z)$ are coordinates of the sphere origin
          * $R$ is sphere radius, which is 1
          * $p(\Theta)$ is probability density, which is reciprocal of sphere area to make it uniform
      * For a unit sphere at the origin, this simplifies to...
        * $x = 2\cos(2\pi r_1)\sqrt{r_2(1-r_2)}$
        * $y = 2\sin(2\pi r_1)\sqrt{r_2(1-r_2)}$
        * $z = 1-2r_2$
* Gauss map relates surface normals to points on a sphere
* Note that since normals are unit length, _Gauss map is unit sphere_
* **You can "trace" paths on the Gauss map to minimize derivatives**
* Edges b/n patches are discontinuities in vector field of U
* GOAL: Solve for smallest $\partial U/\partial\hat{n}$
* Use **Appendix A, algorithm 1** to trace all paths
* IMPORTANT: while categorizing, need to cache vertex/edge/face to roll towards

#### Vertex

1. Locate patch on Gauss map corresponding to vertex $i$
2. Start from $\hat{n}$ in patch of $i$
3. Go along direction of $-\partial U/\partial\hat{n}$
4. Continue until you intersect with edge of patch

* How to compute partial derivative of $U$ wrt to $\hat{n}$?
  * $-\partial U/\partial\hat{n} = -\frac{\partial}{\partial\hat{n}}(c - x_j)\cdot\hat{n}$
  * $-(c - x_j) = x_j - c$

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
       * Impl tip: build eqn of plane L using $x_i$ and $x_j$ to compute plane normal
       * Or, you could use `Eigen::Hyperplane` instead!!
    2. Split into three strips, one parallel line thru $x_i$, one parallel line thru $x_j$
       * Both lines must be _orthogonal_ to edge $ij$
       * Maybe use `Eigen::ParametrizedLine` here?
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
  * Cache this vertex!
* If edge is hinge (E2),
  * roll to next incident face based on direction of $\partial U/\partial\hat{n}$ along edge between patches
  * Cache this face!

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
* Cache that edge/vertex!

### Element Fetching Based on Normals

* Combination of checking if a normal is that of a face, edge, or vertex (in that order!)

#### How to check if it is a face normal?

* Face normal = vertex on Gauss map
* Compute all face normals and do a linear search
* Could do a mapping of normals to faces, but floating point arithmetic won't guarantee equality

#### How to check if it is an edge normal?

* Edges on Gauss maps connect face normals (which are Gauss vertices)
* To check if normal corresponds to edge
  * Compute dihedral angle $\phi$ (angle between face normals)
  * Is edge normal if angle $\theta$ with either face normal is less than dihedral angle $\phi$
    * $\theta < \phi$

#### How to check if it is a vertex normal?

* Vertices on Gauss maps correspond to patches
  * Patch vertices are face normals
  * Patch boundaries are Gauss edges that connect face normals (Gauss vertices)
* Many ways to do this, but simplest method so far uses great circle intersections
  * Create circular slices of Gauss map sphere using patch boundaries
    * Circular slice = **Great circle**
  * Great circles are contained in a plane
    * To determine plane normal, take two points on plane (patch boundary endpoints = face normals)
    * Compute cross product of endpoints
  * Compute dot product of normal to be tested and all great-circle (plane) normals
  * If all dot products have same sign (all negative/all positive)
    * Normal is in vertex patch on Gauss map
  * Else, not in vertex patch on Gauss map

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
    * if and only if $\hat{n}_j$ is in vertex patch of j on Gauss map
      * Just compute $x_j - c$ for all vertices and check if in vertex patch of j
      * Use great-circle intersection method
* Saddle points
  * Only on E2 (hinge edges)
  * $\hat{n}_{ij}^{*}=c-c_{ij}$
  * $c_{ij}=x_i+\frac{(c-x_i)\cdot v_{ij}} {v_{ij}\cdot v_{ij}}\cdot v_{ij}$
    * $c_{ij}$ is projection of center of mass $c$ on edge $ij$
    * $v_{ij}=x_j - x_i$ is the edge vector
    * $\hat{n}_{ij}^{*}$ is saddle point iff it lies on edge on Gauss map
      * Use dihedral angle check from element fetching to determine

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
    * Using `rayArcInt`
* FINALLY, $p_f = \frac{A_f}{4\pi}$
  * where $A_f$ is the area of ascending manifold
    * Remember, ascending manifold is spherical patch around minima bounded by separatrices
  * How to compute $A_f$?
    * If ascending manifold is defined by vertices $[\hat{u}_1, \hat{u}_2, ..., \hat{u}_k]$, split into sum of signed spherical triangle areas:
      * $A_f = \sum_i\Omega(\hat{n}, \hat{u}_i, \hat{u}_{i + 1})$
        * where $\hat{n}$ is any point on sphere that is treated as origin
          * IMPORTANT: for ascending manifold, **$\hat{n}$ is normal of stable face** (since face is local minima that manifold is centered at)
    * Standard formula for spherical triangle is from Girard's theorem:
      * $A = \alpha + \beta + \gamma - \pi$
        * where $\alpha, \beta, \gamma$ are interior angles of triangle
      * How to compute interior angles? Two ways:
        * _Dihedral angles_
          * Compute normals of planes that intersect at vertex
          * Compute angle between normals
          * Repeat for all vertices
        * _Tangent method_
          * Given vertices A, B, C, start from vertex, say, A
          * Compute edge vectors from A, which are AB = B - A, and AC = C - A
          * Extract tangential components from AB and AC, where A is normal
            * $AB_t = AB - (AB \cdot A) A$
            * $AC_t = AC - (AC \cdot A) A$
          * Compute angle between tangential components
            * $\theta_A = \arccos(\frac{AB_t \cdot AC_t}{||AB_t||\cdot||AC_t||})$
          * Repeat for all vertices
    * IMPROVEMENT: Recall the formula for solid angles: $\Omega = A / r^2$
      * where $\Omega$ is solid angle with origin at sphere origin
      * $A$ is surface area of spherical patch bounded by solid angle
      * $r$ is radius of sphere
    * For unit sphere, $\Omega = A$, so we can use solid angle formulae to compute spherical triangle area
      * One example is L'Huilier's theorem:
        * $4 \cdot \arctan\Bigg(\sqrt{\tan(\frac{\theta_S}{2}) \cdot tan(\frac{\theta_S - \theta_A}{2}) \cdot tan(\frac{\theta_S - \theta_B}{2}) \cdot tan(\frac{\theta_S - \theta_C}{2})}\Bigg)$
          * where $\theta_A, \theta_B, \theta_C$ are interior angles of triangle
          * $\theta_S = \frac{\theta_A + \theta_B + \theta_C}{2}$
      * Better one proposed by Van Oosterom and Strackee [1983]
        * $A = 2 \cdot \arctan\Bigg(\frac{\left|a \cdot (b \times c)\right|}{||a||\cdot||b||\cdot||c|| + (a \cdot b)\cdot||c|| + (a \cdot c)\cdot||b|| + (b \cdot c)\cdot||a||}\Bigg)$
          * where $a, b, c$ are 3D coordinates of vertices of spherical triangle
          * Note absolute value around numerator
      * Even better one proposed by Eriksson [1990]
        * Since Gauss map is unit sphere, $||a|| = ||b|| = ||c|| = 1$
          * Simplify to $A = 2 \cdot \arctan\Bigg(\frac{\left|a \cdot (b \times c)\right|}{1 + (a \cdot b) + (a \cdot c) + (b \cdot c)}\Bigg)$
        * In code,

        ```cpp
        double N = std::abs(a.dot(b.cross(c))); \\ numerator
        double D = 1.0 + a.dot(b) + a.dot(c) + b.dot(c); \\ denominator
        double area = 2.0 * atan2(N, D);
        ```

    * Basically, sweep around stable face normal at center of ascending manifold
      * Compute signed spherical triangle area centered at face normal for each polygon vertex pair
      * Sum them up, and _VOILA_, you have the area $A_f$ of a spherical polygon!
* Divide $A_f$ by area of sphere $4\pi$, and you have probability of landing on face $f$!

#### Special Cases

* All faces of $H$ are _stable_ = all vertices of $H$ are _unstable_
  * Thus, $p_f$ = normalized solid angle of face $f$ seen from center of mass $c$

## Manipulating the Probabilities

* TODO...

## Inverse Design

* TODO...

## Rigid Body Simulation Validation

* Use Bullet for fast output, RigidIPC for more accurate output
* Use Rodrigues' (axis-angle) formula (with $\hat{z} = (0, 0, 1)$ as the axis and $\hat{n}$ to get the angle) to compute rotation $R$

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
