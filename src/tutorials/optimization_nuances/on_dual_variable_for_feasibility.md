Great — let’s break this down cleanly, because this sentence from the CRISP paper touches on two important concepts:

---

# **1. Why do dual variables in KKT matter for feasibility?**

In classical constrained optimization (SQP, interior point, etc.), the solver maintains **primal variables** (x) and **dual variables** (\lambda) (multipliers).

The dual variables encode information about **how strongly each constraint is “pushing back”** at the current iterate.

This matters because:

### **Dual variables enforce feasibility via the KKT stationarity condition**

The KKT stationarity condition is:

[
\nabla f(x) + \sum_i \lambda_i \nabla c_i(x) = 0.
]

What this means:

* If a constraint is **violated**, its gradient points back toward feasibility.
* The corresponding dual variable (\lambda_i) becomes large.
* The term (\lambda_i \nabla c_i(x)) creates a direction that pushes the next iterate **toward satisfying the constraint**.

So dual variables act like **forces** pushing you into the feasible region.

That is why:

> **Primal–dual methods keep constraints satisfied (or nearly satisfied) at each step.**

---

# **2. What does “enforcing feasibility step-by-step” mean?**

In methods like SQP or interior point:

### **At every iteration:**

* The subproblem includes constraints in their *linearized* or *barrier* form.
* The solver tries to keep the iterate **feasible or almost-feasible**.
* The trust region/line search is chosen so constraints are not violated too much.

This is what “step-by-step feasibility” means:

> **Each iteration attempts to remain close to the feasible set — constraints guide the direction of movement.**

This is especially true in primal–dual interior point methods where:

* Inequality constraints have **barrier terms** like (-\mu \log(c_i(x))),
* making it impossible to cross into infeasible regions.

So step-by-step feasibility =
**the optimization path stays glued to, or steered by, the constraint boundaries.**

---

# **3. Why CRISP does *not* need dual variables or step-by-step feasibility**

In MPCC problems with complementarity constraints, KKT conditions break entirely:

* Constraint derivatives are linearly dependent (LICQ fails),
* KKT multipliers become undefined or unbounded,
* Primal-dual methods lose both theoretical grounding and numerical stability.

So CRISP takes a very different approach:

### ✔ It uses a **primal-only merit function**

that *penalizes* constraint violation instead of enforcing it at every iteration.

### ✔ It allows **temporarily infeasible steps**

because the subproblem is always convex and always feasible *after slack variables*.

### ✔ It relies on reducing the merit function instead of satisfying KKT conditions.

So iterations may:

* Violate dynamics,
* Violate complementarity,
* Violate contact constraints,

**but the merit function penalizes the violation**, pulling the iterate back toward feasibility over many iterations — not at every step.

In other words:

> **CRISP does not force feasibility at each iteration; it gradually “coaxes” the solution toward feasibility through the merit function.**

This is exactly the opposite of primal–dual methods.

---

# **4. Summary**

### **Why do dual variables matter for feasibility?**

Dual variables apply “forces” that guide iterates toward the feasible set through the KKT stationarity condition.

### **What does “feasibility step-by-step” mean?**

Primal–dual methods try to remain feasible (or nearly feasible) at each iteration by:

* enforcing linearized constraints,
* using barrier terms,
* using dual multipliers.

### **Why CRISP does not do this**

CRISP:

* has no dual variables,
* allows constraint violations,
* enforces feasibility only via a scalar merit function,
* and converges to a stationary point of that merit function.

This is a major reason CRISP remains stable even on MPCCs, where KKT multipliers are unreliable or undefined.

---

If you'd like, I can also explain:

* Why MPCCs cause KKT multipliers to blow up,
* How slack variables and ℓ₁ penalties keep the subproblem convex,
* Why sequential convex programming avoids the TC/LFC mismatch.
