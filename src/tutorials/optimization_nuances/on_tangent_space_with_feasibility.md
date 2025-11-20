Great — let’s clarify this very cleanly and geometrically.
This is a common point of confusion and the key is to separate **the constraint manifold** from **the feasible *side*** of that manifold.

---

# **1. ∇c(x) is perpendicular to the constraint manifold**

For an equality constraint:

[
c(x) = 0,
]

the feasible set is the **manifold**:

[
\mathcal{M} = { x \mid c(x)=0 }.
]

Yes:
[
\nabla c(x)
]
is **normal (perpendicular)** to this manifold.

Geometrically:

* Tangent directions satisfy:
  [
  \nabla c(x)^\top p = 0
  ]
  (sliding *along* the constraint surface)

* Directions that increase/decrease (c(x)) are:
  [
  \nabla c(x)^\top p > 0 \quad \text{(moves to one side)}
  ]
  [
  \nabla c(x)^\top p < 0 \quad \text{(moves to the other side)}
  ]

So the gradient points **normal to the surface**, but that normal vector always defines **two half-spaces**: a positive side and a negative side.

---

# **2. Why does ∇c point toward “increasing violation”?**

Take an inequality:

[
c(x) \le 0.
]

The feasible set is the **half-space**:

[
{ x \mid c(x) \le 0 }.
]

Now linearize:

[
c(x + \alpha p) \approx c(x) + \alpha \nabla c(x)^\top p.
]

If you are infeasible:

[
c(x) > 0,
]

you want to move in directions that **decrease** (c(x)):

[
\nabla c(x)^\top p < 0.
]

This means:
→ (p) points **toward the negative side** of the gradient.

Meanwhile:

* A direction (p) with
  [
  \nabla c(x)^\top p > 0
  ]
  moves deeper into infeasibility — it increases the constraint value.

This is why we say the gradient “points toward increasing violation”:

> **Because moving in the positive gradient direction increases the scalar constraint value (c(x)).**

Even though ∇c is only perpendicular to the surface, it still tells you which direction increases the scalar function.

---

# **3. Half-space intuition**

Yes, your intuition is correct:

[
\nabla c(x)^\top p > 0
\quad \Longleftrightarrow \quad
p \text{ points into the same half-space as } \nabla c(x).
]

Similarly:

[
\nabla c(x)^\top p < 0
\quad \Longleftrightarrow \quad
p \text{ points into the *opposite* half-space}.
]

For an inequality constraint (c(x)\le 0):

* **Violating region** = (c(x)>0) = the “positive” half-space
* **Feasible region** = the “negative” half-space

Thus:

* (p) with (\nabla c(x)^\top p > 0) → more infeasible
* (p) with (\nabla c(x)^\top p < 0) → toward feasibility

---

# **4. Why this matters for KKT methods**

When a constraint is violated:

* its dual λ becomes large,
* the term ( -\lambda \nabla c(x) ) appears in the search direction,
* which forces the direction (p) to satisfy:

[
\nabla c(x)^\top p < 0.
]

Meaning:

**KKT-based solvers push you into the feasible half-space at each step.**

CRISP does *not* do this.

---

# **5. Summary**

* ∇c(x) is perpendicular to the constraint manifold (c(x)=0).
* But it still defines which direction increases or decreases the constraint value.
* Increasing violation = moving in the **positive** ∇c direction.
* Decreasing violation = moving in the **negative** ∇c direction.
* (\nabla c(x)^\top p > 0) = move deeper into infeasibility.
* (\nabla c(x)^\top p < 0) = move toward feasibility.

---

If you want, I can draw this out as an ASCII or plotted diagram to make the geometry even clearer.
