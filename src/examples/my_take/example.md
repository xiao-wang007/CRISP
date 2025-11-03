xiao-wang007: how to let it ignore the whole of build/

GitHub Copilot: I’ll check your .gitignore and add the correct pattern to ignore the build directory, covering either the top-level or any nested build/ folders as you prefer.

Read [](file:///Users/xiao/0_codes/CRISP/.gitignore)

Using "Apply Patch"


```

```
I added a rule to your .gitignore to ignore any directory named build at any depth.

What I changed
- In .gitignore, added:
  - build/ to ignore all build/ directories anywhere in the repo
  - A comment showing how to target only the top-level build/ if needed

Notes
- If build (or any build/ dir) is already tracked by Git, ignores won’t take effect until you remove it from the index. You can untrack it without deleting files locally:

```bash
git rm -r --cached src/build/
git commit -m "Stop tracking build directory"
```

Alternatives
- To ignore only a top-level build directory: use /build/ instead of build/.
- To ignore only build: use src/build/.

Made changes.

xiao-wang007: how to install piqp for c++

GitHub Copilot: I'll quickly inspect your CMake setup to see how this project expects PIQP to be provided (system install vs. submodule), so I can give you precise install and integration steps for macOS.

Read [](file:///Users/xiao/0_codes/CRISP/src/CMakeLists.txt)

Using "Fetch Web Page"

Fetched 4 resources

xiao-wang007: why does this repo needs piqp?

GitHub Copilot: Created 2 todos

Searched text for `piqp|PIQP`, 52 results

Read [](file:///Users/xiao/0_codes/CRISP/src/core/include/solver_core/SolverInterface.h)

Read [](file:///Users/xiao/0_codes/CRISP/src/core/CMakeLists.txt)

Read [](file:///Users/xiao/0_codes/CRISP/ReadMe.md#1-1), lines 1 to 220

Completed (1/2) *Search repo for PIQP usage*

You need PIQP because CRISP reduces its nonlinear problem to a sequence of convex QP subproblems, and PIQP is the QP solver backend used to solve each subproblem efficiently.

What in this repo uses PIQP
- Code: SolverInterface.h
  - Includes `piqp/piqp.hpp`.
  - Holds a `piqp::SparseSolver<scalar_t> piqpSolver_;`.
  - Sets up/updates and solves each convex QP via `piqpSolver_.setup(...)`, `piqpSolver_.update(...)`, and `piqpSolver_.solve()`, then uses `piqpSolver_.result().x`.
- Build system:
  - CMakeLists.txt has `find_package(piqp REQUIRED)`.
  - CMakeLists.txt links `piqp::piqp`.
  - Your build cache shows `piqp_DIR-NOTFOUND`, so CMake can’t find the PIQP package.
- Documentation:
  - ReadMe.md explicitly states that the “large-scale sparse convex Quadratic Programming (QP) subproblems” are solved using PIQP and provides install steps.

In short: PIQP is the convex QP engine CRISP relies on inside its trust-region sequential convex programming loop. Without it, the solver won’t build or run.

If you want to avoid PIQP
- You’d need to wrap an alternative QP solver (e.g., OSQP, qpOASES) in place of PIQP by:
  - Replacing the include and member in SolverInterface.h.
  - Adapting `constructSubproblem`/`solveSubproblem` data mapping to the new solver’s API.
  - Updating the CMake to find/link the new solver.
- This is a moderate change since the code is already organized to funnel each subproblem into a single QP call, but you’ll still need to map matrices, bounds, and signs carefully.

Tip about brew install
- Homebrew likely doesn’t have a formula named “piqp”, which is why `brew install piqp` failed.
- Follow the repo’s ReadMe section “PIQP” to install from source and ensure CMake can find it (so `find_package(piqp)` succeeds).

xiao-wang007: ❯ cmake ..
-- The C compiler identification is AppleClang 17.0.0.17000013
-- The CXX compiler identification is AppleClang 17.0.0.17000013
-- Detecting C compiler ABI info
-- Detecting C compiler ABI info - done
-- Check for working C compiler: /usr/bin/cc - skipped
-- Detecting C compile features
-- Detecting C compile features - done
-- Detecting CXX compiler ABI info
-- Detecting CXX compiler ABI info - done
-- Check for working CXX compiler: /usr/bin/c++ - skipped
-- Detecting CXX compile features
-- Detecting CXX compile features - done
-- Building Release
-- Compiler: AppleClang
CMake Error at /opt/homebrew/share/cmake/Modules/FindPackageHandleStandardArgs.cmake:227 (message):
  Could NOT find Matio (missing: MATIO_LIBRARY MATIO_INCLUDE_DIR)
Call Stack (most recent call first):
  /opt/homebrew/share/cmake/Modules/FindPackageHandleStandardArgs.cmake:591 (_FPHSA_FAILURE_MESSAGE)
  cmake/FindMatio.cmake:28 (find_package_handle_standard_args)
  CMakeLists.txt:123 (find_package)


-- Configuring incomplete, errors occurred!

GitHub Copilot: I’ll reconfigure and build PIQP with tests/benchmarks and MATIO disabled, installing it to /usr/local so CRISP’s CMake can find it, then re-run CRISP’s CMake configure.

Ran terminal command: cmake -S /Users/xiao/0_codes/CRISP/src/third_party/piqp -B /Users/xiao/0_codes/CRISP/src/third_party/piqp/build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -DPIQP_BUILD_TESTS=OFF -DPIQP_BUILD_BENCHMARKS=OFF -DBUILD_TESTS=OFF -DBUILD_BENCHMARKS=OFF -DPIQP_ENABLE_MATIO=OFF

Ran terminal command: /opt/homebrew/bin/cmake -S /Users/xiao/0_codes/CRISP/src/third_party/piqp -B /Users/xiao/0_codes/CRISP/src/third_party/piqp/build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -DPIQP_BUILD_TESTS=OFF -DPIQP_BUILD_BENCHMARKS=OFF -DBUILD_TESTS=OFF -DBUILD_BENCHMARKS=OFF -DPIQP_ENABLE_MATIO=OFF

Ran terminal command: /opt/homebrew/bin/cmake --build /Users/xiao/0_codes/CRISP/src/third_party/piqp/build --config Release

Ran terminal command: /opt/homebrew/bin/cmake --install /Users/xiao/0_codes/CRISP/src/third_party/piqp/build --config Release

xiao-wang007: why you skip matio

GitHub Copilot: I disabled MATIO because the error showed PIQP's CMake was looking for it but couldn't find it, and MATIO isn't required for PIQP's core functionality—it's only used for optional file I/O features in benchmarks/examples.

**Why MATIO was the issue:**
- The CMake error was: `Could NOT find Matio (missing: MATIO_LIBRARY MATIO_INCLUDE_DIR)`
- MATIO (MATLAB MAT-file I/O library) is an optional dependency for PIQP, used to load/save `.mat` files in benchmarks and examples
- Since you're building PIQP as a library dependency for CRISP (not to run PIQP's own benchmarks), MATIO isn't needed

**What CRISP actually needs from PIQP:**
Looking at SolverInterface.h, CRISP only uses:
- `#include "piqp/piqp.hpp"` (core solver header)
- `piqp::SparseSolver<scalar_t>` class
- `.setup()`, `.update()`, `.solve()`, `.result()` methods

None of these require MATIO—they're part of PIQP's core QP solver API.

**Next steps to complete the install:**
The install failed due to permissions. You need to either:

1. **Install with sudo** (to local):
```bash
sudo /opt/homebrew/bin/cmake --install /Users/xiao/0_codes/CRISP/src/third_party/piqp/build --config Release
```

2. **Or install to a user-writable location** and tell CRISP where to find it:
```bash
# Install to a local prefix
/opt/homebrew/bin/cmake --install /Users/xiao/0_codes/CRISP/src/third_party/piqp/build --prefix ~/.local

# Then when building CRISP, add:
cmake .. -DCMAKE_PREFIX_PATH=~/.local
```

Which approach do you prefer? The sudo install to local is simpler and matches what your CRISP CMakeLists.txt already expects (it has `/usr/local/lib` in its RPATH).

xiao-wang007: ❯ cmake ..             
-- Found Python: /opt/homebrew/Frameworks/Python.framework/Versions/3.13/bin/python3.13 (found suitable version "3.13.3", minimum required is "3.8") found components: Interpreter Development.Module Development.Embed
-- Performing Test HAS_FLTO_THIN
-- Performing Test HAS_FLTO_THIN - Success
-- Performing Test HAS_FLTO
-- Performing Test HAS_FLTO - Success
-- Found pybind11: /opt/homebrew/include (found version "3.0.1")
-- Eigen3 include dir: /opt/homebrew/include/eigen3
-- Boost include dir: /opt/homebrew/include
-- CPPAD include dir: /opt/homebrew/Cellar/cppad/20250000.2/include
-- YAML-CPP include dir: /opt/homebrew/include
-- Configuring done (1.2s)
-- Generating done (0.0s)
-- Build files have been written to: /Users/xiao/0_codes/CRISP/src/build
(robotics) 
CRISP/src/build on  master via △ v4.0.2 via 🅒 robotics 
❯ make 
[  6%] Building CXX object core/CMakeFiles/CRISP.dir/src/cppad_core/CppAdInterface.cpp.o
In file included from /Users/xiao/0_codes/CRISP/src/core/src/cppad_core/CppAdInterface.cpp:1:
In file included from /Users/xiao/0_codes/CRISP/src/core/include/cppad_core/CppAdInterface.h:4:
/Users/xiao/0_codes/CRISP/src/core/include/common/BasicTypes.h:4:10: fatal error: 'cppad/cg.hpp' file not found
    4 | #include <cppad/cg.hpp>
      |          ^~~~~~~~~~~~~~
1 error generated.
make[2]: *** [core/CMakeFiles/CRISP.dir/src/cppad_core/CppAdInterface.cpp.o] Error 1
make[1]: *** [core/CMakeFiles/CRISP.dir/all] Error 2
make: *** [all] Error 2

GitHub Copilot: I'll check where CppADCodeGen is installed and update CRISP's CMakeLists.txt to find and include it properly. 

Ran terminal command: find /opt/homebrew/include /usr/local/include ~/.local/include -name "cg.hpp" 2>/dev/null | head -20

Ran terminal command: find /Users/xiao/0_codes/CRISP/src/third_party -name "cg.hpp" 2>/dev/null

Read [](file:///Users/xiao/0_codes/CRISP/src/third_party)

Ran terminal command: find /usr/local /opt/homebrew -path "*/cppad/cg.hpp" 2>/dev/null | head -5

CppADCodeGen is not installed. According to the README, you need to install both CppAD (which you have via Homebrew) and CppADCodeGen (which is missing). Let me install it for you: 

Ran terminal command: cd /Users/xiao/0_codes/CRISP/src/third_party && git clone https://github.com/joaoleal/CppADCodeGen.git

xiao-wang007: ❯ brew info cppad      
==> cppad: stable 20250000.2 (bottled), HEAD
Differentiation of C++ Algorithms
https://www.coin-or.org/CppAD
Installed
/opt/homebrew/Cellar/cppad/20250000.2 (827 files, 5MB) *
  Poured from bottle using the formulae.brew.sh API on 2025-11-03 at 16:24:54
From: https://github.com/Homebrew/homebrew-core/blob/HEAD/Formula/c/cppad.rb
License: EPL-2.0
==> Dependencies
Build: cmake ✘, pkgconf ✘
==> Options
--HEAD
        Install HEAD version
==> Downloading https://formulae.brew.sh/api/formula/cppad.json
==> Analytics
install: 86 (30 days), 467 (90 days), 1,595 (365 days)
install-on-request: 5 (30 days), 16 (90 days), 110 (365 days)
build-error: 2 (30 days)

I have cppad installed 

GitHub Copilot: Yes, you have CppAD installed via Homebrew, but you're missing **CppADCodeGen** (a separate companion library). The error `'cppad/cg.hpp' file not found` is looking for CppADCodeGen, not CppAD.

**Two different libraries:**
1. **CppAD** (`<cppad/cppad.hpp>`) - automatic differentiation ✅ Installed via Homebrew
2. **CppADCodeGen** (`<cppad/cg.hpp>`) - code generation extension ❌ Not installed

**Your options:**

**Option 1: Install CppADCodeGen from source** (as the README instructs):
```bash
cd /Users/xiao/0_codes/CRISP/src/third_party
git clone https://github.com/joaoleal/CppADCodeGen.git
cd CppADCodeGen
mkdir build && cd build
cmake ..
sudo make install
```

**Option 2: Check if Homebrew has it:**
```bash
brew search cppadcodegen
# or
brew search codegen
```

Let me check if there's a Homebrew formula: 

Ran terminal command: brew search cppadcodegen

xiao-wang007: what is the relationship between cppad and cppadcodegen? can you also give me an example from this repo

GitHub Copilot: Let me show you the relationship and find examples from this repo. 

Read [](file:///Users/xiao/0_codes/CRISP/src/core/include/common/BasicTypes.h#1-1), lines 1 to 50

Read [](file:///Users/xiao/0_codes/CRISP/src/core/include/common/BasicTypes.h#50-50), lines 50 to 78

Read [](file:///Users/xiao/0_codes/CRISP/src/core/include/cppad_core/CppAdInterface.h)

Searched text for `CppAD::cg::` (`**/src/core/src/cppad_core/CppAdInterface.cpp`), 11 results

Perfect! Now let me show you the relationship with a concrete example from the repo:

## **Relationship between CppAD and CppADCodeGen**

**CppAD** = Automatic Differentiation library (computes derivatives at runtime)
**CppADCodeGen** = Code Generation extension (generates C++ source code for those derivatives)

### **The Two-Layer Type System in CRISP:**

Looking at BasicTypes.h:

```cpp
// Layer 1: CppADCodeGen wraps the base type
using cg_scalar_t = CppAD::cg::CG<scalar_t>;  // CppADCodeGen type
                    // ^^^^^^^^^ this is from CppADCodeGen

// Layer 2: CppAD wraps the CG type  
using ad_scalar_t = CppAD::AD<cg_scalar_t>;   // CppAD automatic differentiation
                    // ^^^^^^^ this is from CppAD

// Then use these for vector/matrix operations
using ad_vector_t = Eigen::Matrix<ad_scalar_t, Eigen::Dynamic, 1>;
using ad_matrix_t = Eigen::Matrix<ad_scalar_t, Eigen::Dynamic, Eigen::Dynamic>;
```

### **How CRISP Uses Them Together:**

From CppAdInterface.cpp, here's the workflow:

1. **User defines functions** using `ad_scalar_t` types (which contain both CppAD + CppADCodeGen):
```cpp
ad_function_t pushbotObjective = [](const ad_vector_t& x, const ad_vector_t& p, ad_vector_t& y) {
    // x, y are ad_vector_t = contain CppAD::AD<CppAD::cg::CG<double>>
    y.resize(1);
    ad_scalar_t cost = ...;  // arithmetic with AD+CG types
}
```

2. **CppAD traces the computation** to build a computational graph

3. **CppADCodeGen generates C++ source code** from that graph:
```cpp
CppAD::cg::ModelCSourceGen<scalar_t> cgen(cg_fun, modelName_);
//       ^^^ CppADCodeGen generates C++ source
```

4. **Compiles it to a shared library** (.so/.dylib):
```cpp
CppAD::cg::GccCompiler<scalar_t> compiler;
CppAD::cg::DynamicModelLibraryProcessor<scalar_t> proc(libcgen, libraryName_);
//       ^^^ CppADCodeGen compiles and loads the library
```

5. **Fast evaluation** using the compiled library:
```cpp
dynamicLib_ = std::make_unique<CppAD::cg::LinuxDynamicLib<scalar_t>>(file_name_ext);
model_ = dynamicLib_->model(modelName_);
// Now call model_->SparseJacobian(), model_->SparseHessian(), etc.
```

### **Why both are needed:**

- **CppAD alone**: Would compute derivatives every time at runtime (slower)
- **CppADCodeGen**: Generates optimized C++ code once, compiles it, then reuses it (much faster)

**You need to install CppADCodeGen** because CRISP's entire type system is built on `CppAD::cg::CG<double>`, and every derivative computation goes through the code generation pipeline to achieve the performance mentioned in the paper.

xiao-wang007: is this repo using anonymous function for cppad to apply ad to defined function? can you give me an example 

GitHub Copilot: Yes! This repo extensively uses **C++ lambda functions (anonymous functions)** to define objective and constraint functions that CppAD can automatically differentiate. Let me show you concrete examples from the codebase. 

Read [](file:///Users/xiao/0_codes/CRISP/src/examples/pushbox/SolvePushbox.cpp#1-1), lines 1 to 150

Read [](file:///Users/xiao/0_codes/CRISP/src/examples/pushbox/SolvePushbox.cpp#150-150), lines 150 to 208

Perfect! Yes, this repo **extensively uses C++ lambda (anonymous) functions** to define functions that CppAD automatically differentiates. Let me break down the pattern:

## **Lambda Function Pattern for CppAD in CRISP**

### **1. Type Signatures for Lambda Functions**

From BasicTypes.h:
```cpp
// Lambda with only variables (no parameters)
using ad_function_t = std::function<void(const ad_vector_t& x, ad_vector_t& y)>;

// Lambda with variables AND parameters
using ad_function_with_param_t = std::function<void(const ad_vector_t& x, const ad_vector_t& p, ad_vector_t& y)>;
```

### **2. Example: Dynamics Constraint (No Parameters)**

```cpp
ad_function_t pushboxDynamicConstraints = [](const ad_vector_t& x, ad_vector_t& y) {
    //                                      ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    //                                      Lambda captures nothing, takes x (variables), outputs y
    
    y.resize((N - 1) * num_state);  // Must set output size
    
    for (size_t i = 0; i < N - 1; ++i) {
        // Extract decision variables
        ad_scalar_t px_i = x[idx + 0];
        ad_scalar_t py_i = x[idx + 1];
        ad_scalar_t theta_i = x[idx + 2];
        // ... more variables
        
        // Compute dynamics residuals using AD types
        ad_scalar_t px_dot = (1/(mu*m*g))*(cos(theta_i)*(lambda2_i + lambda4_i) - ...);
        ad_scalar_t py_dot = ...;
        
        // Output constraint violation (should equal zero)
        y.segment(i * num_state, num_state) << px_next - px_i - px_dot * dt,
                                                py_next - py_i - py_dot * dt,
                                                theta_next - theta_i - theta_dot * dt;
    }
};
```

**Key points:**
- Input `x`: all decision variables (AD types: `ad_vector_t`)
- Output `y`: constraint values that should be zero
- All math operations use `ad_scalar_t` so CppAD can trace derivatives
- Constants like `mu`, `m`, `g`, `dt` are captured from outer scope (lambda closure)

### **3. Example: Objective Function (With Parameters)**

```cpp
ad_function_with_param_t pushboxObjective = [](const ad_vector_t& x, const ad_vector_t& p, ad_vector_t& y) {
    //                                          ^^^^^^^^^^^^^^^^^^  ^^^^^^^^^^^^^^^^^^
    //                                          variables           parameters (e.g., target state)
    
    y.resize(1);  // Objective is scalar
    y[0] = 0.0;
    
    ad_scalar_t tracking_cost(0.0);
    ad_scalar_t control_cost(0.0);
    
    for (size_t i = 0; i < N; ++i) {
        // Extract state
        ad_scalar_t px_i = x[idx + 0];
        ad_scalar_t py_i = x[idx + 1];
        
        if (i == N - 1) {
            // Track to target (from parameters p)
            ad_vector_t tracking_error(num_state);
            tracking_error << px_i - p[0],    // p[0] = target px
                            py_i - p[1],      // p[1] = target py
                            theta_i - p[2];   // p[2] = target theta
            
            tracking_cost += tracking_error.transpose() * Q * tracking_error;
        }
        
        if (i < N - 1) {
            ad_vector_t control_error(4);
            control_error << lambda1_i, lambda2_i, lambda3_i, lambda4_i;
            control_cost += control_error.transpose() * R * control_error;
        }
    }
    
    y[0] = tracking_cost + control_cost;
};
```

**Key points:**
- Parameters `p` allow changing targets without regenerating autodiff library
- Perfect for MPC where reference changes frequently
- Still uses AD types for both `x` and `p`

### **4. How It Gets Used**

```cpp
// Create constraint/objective wrappers that pass lambda to CppAD
auto obj = std::make_shared<ObjectiveFunction>(
    variableNum, num_state, problemName, folderName, 
    "pushboxObjective", 
    pushboxObjective  // <-- Lambda function here!
);

auto dynamics = std::make_shared<ConstraintFunction>(
    variableNum, problemName, folderName, 
    "pushboxDynamicConstraints", 
    pushboxDynamicConstraints  // <-- Lambda here!
);

// Add to problem
pushboxProblem.addObjective(obj);
pushboxProblem.addEqualityConstraint(dynamics);
```

### **5. Behind the Scenes (CppAdInterface)**

The `CppAdInterface` constructor receives these lambdas and:

1. **Traces the computation** by calling the lambda with AD types
2. **Extracts the computational graph** via CppAD
3. **Generates optimized C++ code** via CppADCodeGen
4. **Compiles to .so/.dylib** for fast evaluation

From the code comment:
```cpp
// ---------------------- ! the above four lines are enough for generate 
// the auto-differentiation functions library for this problem 
// and the usage in python ! ---------------------- //
```

### **6. Another Example: Contact Constraints**

```cpp
ad_function_t pushboxContactConstraints = [](const ad_vector_t& x, ad_vector_t& y) {
    y.resize((N - 1) * 12);
    for (size_t i = 0; i < N - 1; ++i) {
        size_t idx = i * (num_state + num_control);
        ad_scalar_t cx_i = x[idx + 3];
        ad_scalar_t cy_i = x[idx + 4];
        ad_scalar_t lambda1_i = x[idx + 5];
        // ... more
        
        // Complementarity and feasibility constraints (≥ 0)
        y.segment(i * 12, 12) << lambda1_i,           // λ₁ ≥ 0
                            lambda2_i,                 // λ₂ ≥ 0
                            -lambda3_i,                // -λ₃ ≥ 0
                            -lambda4_i,                // -λ₄ ≥ 0
                            cy_i + b,                  // contact point bounds
                            cx_i + a,
                            b - cy_i,
                            a - cx_i,
                            -(lambda1_i)*(cy_i+b),    // complementarity
                            -(lambda2_i)*(cx_i+a),
                            -(-lambda3_i)*(b-cy_i),
                            -(-lambda4_i)*(a-cx_i);
    }
};
```

## **Summary**

✅ **Yes, this repo uses C++ lambdas (anonymous functions)** as the primary way to define functions for CppAD  
✅ **Two patterns**: with/without parameters (`ad_function_t` vs `ad_function_with_param_t`)  
✅ **Lambdas capture constants** from outer scope (like `dt`, `m`, `g`)  
✅ **All math must use `ad_scalar_t`** types so CppAD can trace derivatives  
✅ **Output size must be set** via `y.resize(...)` at the start  

This design is elegant because:
- Users write natural C++ math expressions
- No manual derivative calculations needed
- Code generation happens automatically
- Parameters can be changed without recompiling

xiao-wang007: what is this .ccp doing 

GitHub Copilot: Let me read the CppAdInterface.cpp file to explain what it does: 

Read [](file:///Users/xiao/0_codes/CRISP/src/core/src/cppad_core/CppAdInterface.cpp#1-1), lines 1 to 100

Read [](file:///Users/xiao/0_codes/CRISP/src/core/src/cppad_core/CppAdInterface.cpp#100-100), lines 100 to 300

Read [](file:///Users/xiao/0_codes/CRISP/src/core/src/cppad_core/CppAdInterface.cpp#300-300), lines 300 to 500

Perfect! Now I can explain what **CppAdInterface.cpp** does. This is the **core bridge** between user-defined functions (lambdas) and fast derivative computation. Here's what it does:

---

## **What CppAdInterface.cpp Does**

### **1. Main Purpose: Automatic Code Generation for Derivatives**

This file implements the workflow:
```
User Lambda → CppAD Trace → CppADCodeGen → Compiled .so/.dylib → Fast Evaluation
```

---

### **2. Key Functions**

#### **Constructor: Initialize or Load Model**
```cpp
CppAdInterface::CppAdInterface(..., const ad_function_t& function, ...) {
    libraryName_ = folderName + "/" + modelName + "_" + functionName + "_cppad_cg_model";
    
    if (isLibraryAvailable() && !regenerateLibrary_) {
        loadModel();  // Reuse existing compiled library
    } else {
        initializeModel();  // Generate new library
    }
}
```

**What it does:**
- Checks if a compiled library (`.so`/`.dylib`) already exists
- If yes → loads it (fast startup)
- If no → generates, compiles, and saves it (one-time cost)

---

#### **`initializeModel()`: The Code Generation Pipeline**

This is where the magic happens:

**Step 1: Trace the computation with CppAD**
```cpp
ad_vector_t ax(variableDim_);
ax.setOnes();  // Initialize with dummy values

CppAD::Independent(ax);  // Start recording
ad_vector_t ay;
functionNoParam_(ax, ay);  // Call user's lambda - CppAD records operations
funDim_ = ay.size();

CppAD::ADFun<cg_scalar_t> cg_fun(ax, ay);  // Create AD function object
cg_fun.optimize();  // Optimize computational graph
```

**Step 2: Compute sparsity patterns** (which derivatives are non-zero)
```cpp
switch (infoLevel_) {
    case FIRST_ORDER:
        cgen.setCreateSparseJacobian(true);
        cg_fun.for_jac_sparsity(..., jacobianSparsity_);  // Find Jacobian sparsity
        nnzJacobian_ = jacobianSparsity_.nnz();  // Count non-zeros
        break;
    case SECOND_ORDER:
        cgen.setCreateSparseJacobian(true);
        cgen.setCreateSparseHessian(true);
        cg_fun.for_jac_sparsity(..., jacobianSparsity_);
        cg_fun.rev_hes_sparsity(..., hessianSparsity_);  // Find Hessian sparsity
        break;
}
```

**Step 3: Generate C++ source code**
```cpp
CppAD::cg::ModelCSourceGen<scalar_t> cgen(cg_fun, modelName_);
//           ^^^ CppADCodeGen generates optimized C++ code
```

**Step 4: Compile to shared library**
```cpp
CppAD::cg::ModelLibraryCSourceGen<scalar_t> libcgen(cgen);
CppAD::cg::GccCompiler<scalar_t> compiler;
CppAD::cg::DynamicModelLibraryProcessor<scalar_t> proc(libcgen, libraryName_);

if (!boost::filesystem::exists(libraryFolder_)) {
    boost::filesystem::create_directories(libraryFolder_);
}

dynamicLib_ = proc.createDynamicLibrary(compiler);  // Compiles C++ → .so/.dylib
model_ = dynamicLib_->model(modelName_);  // Load the compiled model
```

**Result:** A compiled shared library saved to disk:
```
model/Pushbox/auto_generated/Pushbox_pushboxObjective_cppad_cg_model.so
```

---

#### **`loadModel()`: Load Pre-compiled Library**
```cpp
void CppAdInterface::loadModel() {
    std::string file_name_ext = libraryName_ + CppAD::cg::system::SystemInfo<>::DYNAMIC_LIB_EXTENSION;
    dynamicLib_ = std::make_unique<CppAD::cg::LinuxDynamicLib<scalar_t>>(file_name_ext);
    model_ = dynamicLib_->model(modelName_);
    
    // Extract sparsity info from compiled model
    auto jacobianSparsityPattern = model_->JacobianSparsitySet();
    nnzJacobian_ = ...;  // Count non-zeros
}
```

**Fast startup:** No recompilation needed, just loads the `.so`/`.dylib`.

---

### **3. Derivative Computation Methods**

Once the model is loaded, these methods provide fast derivative evaluation:

#### **Jacobian (∂f/∂x)**
```cpp
CSRSparseMatrix CppAdInterface::computeSparseJacobianCSR(const vector_t& x) {
    ValueVector x_std(x.data(), x.data() + x.size());
    ValueVector jac;
    SizeVector row, col;
    
    model_->SparseJacobian(x_std, jac, row, col);  // Call compiled code
    
    // Convert to CSR (Compressed Sparse Row) format for fast operations
    CSRSparseMatrix jacobianCSR(funDim_, row.size());
    // ... memory copy operations ...
    return jacobianCSR;
}
```

#### **Hessian (∂²f/∂x²)**
```cpp
sparse_matrix_t CppAdInterface::computeSparseHessian(const vector_t& x) {
    ValueVector x_std(x.data(), x.data() + x.size());
    ValueVector w(model_->Range(), 1.0);  // Weight vector (all 1s for objective)
    ValueVector hes;
    SizeVector row, col;
    
    model_->SparseHessian(x_std, w, hes, row, col);  // Call compiled code
    
    // Convert to Eigen sparse format
    // ...
    return sparseHessian;
}
```

#### **Function Value**
```cpp
vector_t CppAdInterface::computeFunctionValue(const vector_t& x) {
    ValueVector x_std(x.data(), x.data() + x.size());
    ValueVector y_std = model_->ForwardZero(x_std);  // Evaluate compiled function
    return vector_t::Map(y_std.data(), y_std.size());
}
```

---

### **4. Handling Parameters (for MPC)**

For parameterized functions (like tracking a moving target):

```cpp
CSRSparseMatrix CppAdInterface::computeSparseJacobianCSR(const vector_t& x, const vector_t& p) {
    // Concatenate variables and parameters
    ValueVector xp_std;
    xp_std.insert(xp_std.end(), x.data(), x.data() + x.size());
    xp_std.insert(xp_std.end(), p.data(), p.data() + p.size());
    
    model_->SparseJacobian(xp_std, jac, row, col);
    
    // Filter out derivatives w.r.t. parameters (only keep ∂f/∂x, not ∂f/∂p)
    for (size_t i = 0; i < row.size(); ++i) {
        if (col[i] < variableDim_) {  // Only x, not p
            jacobianCSR.innerIndices[...] = col[i];
            jacobianCSR.values[...] = jac[i];
        }
    }
}
```

---

### **5. CSR Format (Compressed Sparse Row)**

The code emphasizes **CSR format** for performance:

```cpp
CSRSparseMatrix jacobianCSR(funDim_, nnzJacobian_);
// outerIndex[i] = start index of row i
// innerIndices[k] = column index of k-th non-zero
// values[k] = value of k-th non-zero

// Fast memory copy instead of Eigen triplet construction
std::memcpy(jacobianCSR.innerIndices.data(), col.data(), col.size() * sizeof(size_t));
std::memcpy(jacobianCSR.values.data(), jac.data(), jac.size() * sizeof(scalar_t));
```

**Why CSR?** From the comment in SolverInterface.h:
> "the solver uses CSR format for the sparse matrix operation, which is super faster than triplet format"

Direct memory copying avoids Eigen's overhead of building matrices from triplets.

---

## **Summary: What This File Does**

| Phase | What Happens |
|-------|-------------|
| **Initialization** | Takes user's lambda → traces with CppAD → generates C++ code → compiles to `.so`/`.dylib` |
| **Subsequent Runs** | Loads pre-compiled library (no recompilation) |
| **Evaluation** | Provides fast methods to compute function values, Jacobians, Hessians |
| **Format** | Outputs in CSR format for maximum sparse matrix performance |
| **Parameters** | Supports parametric functions (e.g., MPC with changing references) |

**Performance benefit:**
- **First run:** Slow (compilation overhead)
- **Subsequent runs:** Very fast (just loads compiled code)
- **Evaluation:** Much faster than runtime AD because derivatives are pre-compiled

This is the "automated and efficient computation" mentioned in the CRISP README!

xiao-wang007: is this cppadinterface.cpp a must if to use autodiff with cppad

GitHub Copilot: No, CppAdInterface.cpp is **not required** to use CppAD for automatic differentiation. It's a **convenience wrapper** that CRISP built on top of CppAD to add specific features. Let me explain the difference:

---

## **What You Need vs. What CRISP Adds**

### **Minimal CppAD Usage (Just autodiff - no CppAdInterface needed)**

You can use CppAD directly without any wrapper:

```cpp
#include <cppad/cppad.hpp>

// Just need CppAD, not CppADCodeGen or CppAdInterface
using AD = CppAD::AD<double>;

int main() {
    // 1. Define independent variables
    std::vector<AD> x(2);
    x[0] = 1.0;
    x[1] = 2.0;
    CppAD::Independent(x);
    
    // 2. Define function
    std::vector<AD> y(1);
    y[0] = x[0] * x[0] + x[1] * x[1];  // f(x) = x₀² + x₁²
    
    // 3. Create AD function
    CppAD::ADFun<double> f(x, y);
    
    // 4. Compute derivatives at runtime
    std::vector<double> x_eval = {3.0, 4.0};
    std::vector<double> jac = f.Jacobian(x_eval);  // [2*x₀, 2*x₁] = [6, 8]
    
    return 0;
}
```

**This works fine!** No `CppAdInterface`, no code generation, no compilation. Just runtime automatic differentiation.

---

### **What CppAdInterface.cpp Adds (Extra features)**

CRISP's `CppAdInterface` is a **value-added wrapper** that provides:

| Feature | Without CppAdInterface | With CppAdInterface |
|---------|----------------------|---------------------|
| **Basic AD** | ✅ Works | ✅ Works |
| **Runtime evaluation** | ✅ Recalculates derivatives each time | ⚡ Pre-compiled, much faster |
| **Code generation** | ❌ Not available | ✅ Generates C++ source |
| **Persistent storage** | ❌ Recompute every run | ✅ Saves `.so`/`.dylib`, reuses it |
| **Sparse matrices** | Manual handling | ✅ Automatic CSR format |
| **Sparsity patterns** | Manual computation | ✅ Pre-computed and cached |
| **Parameter support** | Manual implementation | ✅ Built-in parametric function handling |
| **Lambda interface** | Direct CppAD API | ✅ Clean lambda interface |

---

## **When You Need CppAdInterface**

You **don't need it** if:
- You're okay with slower runtime AD
- Your problem is small
- You run optimization once and don't care about speed

You **want it** if:
- You solve the same problem structure repeatedly (like MPC)
- You need maximum performance (CRISP's use case)
- You want sparse matrix handling automated
- You want to change parameters without recompiling

---

## **The Two-Layer Architecture**

### **Layer 1: CppAD (required for AD)**
```cpp
#include <cppad/cppad.hpp>

CppAD::AD<double> x;  // Automatic differentiation type
// Traces operations to build computational graph
```

### **Layer 2: CppADCodeGen (required for code generation)**
```cpp
#include <cppad/cg.hpp>

CppAD::cg::CG<double> cg_x;  // Code generation type
CppAD::AD<cg_x> x;  // AD on top of CG

// Generates optimized C++ code from the graph
CppAD::cg::ModelCSourceGen<double> cgen(...);
```

### **Layer 3: CppAdInterface (CRISP's wrapper - optional)**
```cpp
#include "cppad_core/CppAdInterface.h"

// Convenient interface for CRISP's workflow
ad_function_t myFunction = [](const ad_vector_t& x, ad_vector_t& y) { ... };
CppAdInterface interface(varDim, modelName, folderName, functionName, myFunction);

// Automatically handles: tracing, code gen, compilation, caching, loading
auto jacobian = interface.computeSparseJacobianCSR(x_values);
```

---

## **Analogy**

Think of it like web development:

| Level | CppAD World | Web World |
|-------|------------|-----------|
| **Raw tool** | CppAD (basic AD) | Raw HTTP sockets |
| **Advanced tool** | CppADCodeGen (code gen) | HTTP library (requests) |
| **Framework** | CppAdInterface (CRISP's wrapper) | Web framework (Flask/Express) |

You **can** use raw sockets, but a framework makes life easier for common patterns.

---

## **Can You Remove CppAdInterface?**

Yes, but you'd need to manually implement:

1. **Code generation logic** (lines 108-269 in CppAdInterface.cpp)
2. **Library compilation** (GCC compiler invocation)
3. **Dynamic loading** (`.so`/`.dylib` handling)
4. **Sparsity pattern extraction** and CSR conversion
5. **Parameter handling** for parametric functions
6. **Caching** to avoid recompilation

All of this is ~700 lines of non-trivial code.

---

## **Bottom Line**

**CppAdInterface.cpp is optional** for CppAD usage, but:
- ✅ **Required for CRISP's performance** (code generation + caching)
- ✅ **Required if you want CppADCodeGen features** (pre-compiled derivatives)
- ❌ **Not required for basic CppAD** (runtime autodiff works without it)

If you're building your own optimizer and just want AD, you can skip it and use CppAD directly. But if you want CRISP's speed benefits, you need this code generation infrastructure.

xiao-wang007: can you explain the three files in include/common? what do they do 

GitHub Copilot: Let me read all three files in `include/common` to explain what they do: 

Read [](file:///Users/xiao/0_codes/CRISP/src/core/include/common/BasicTypes.h)

Read [](file:///Users/xiao/0_codes/CRISP/src/core/include/common/ParametersManager.h)

Read [](file:///Users/xiao/0_codes/CRISP/src/core/include/common/ValueFunction.h)

Perfect! Let me explain what each of these three foundational files does:

---

## **1. BasicTypes.h** - Type Definitions & Data Structures

**Purpose:** Defines all fundamental types used throughout CRISP to ensure consistency.

### **What it contains:**

#### **A. Basic Numeric Types**
```cpp
using scalar_t = double;                    // All floating-point numbers
using SizeVector = std::vector<size_t>;     // For indices/sizes
using ValueVector = std::vector<scalar_t>;  // For value arrays
```

#### **B. Eigen Matrix/Vector Types**
```cpp
using vector_t = Eigen::VectorXd;           // Dense vectors
using matrix_t = Eigen::MatrixXd;           // Dense matrices
using sparse_vector_t = Eigen::SparseVector<scalar_t>;
using sparse_matrix_t = Eigen::SparseMatrix<scalar_t, Eigen::RowMajor>;  // Row-major for performance
```

#### **C. Custom CSR Sparse Matrix Structure**
```cpp
struct CSRSparseMatrix {
    SizeVector outerIndex;      // Row start indices
    SizeVector innerIndices;    // Column indices
    ValueVector values;         // Non-zero values
    
    void toEigenSparseMatrix(sparse_matrix_t &sparseMatrix);  // Convert to Eigen
    void print();  // Debug utility
};
```

**Why CSR?** Compressed Sparse Row format enables:
- Fast matrix-vector multiplication
- Direct memory copying (faster than Eigen triplets)
- Efficient storage for large sparse problems

#### **D. Automatic Differentiation Types**
```cpp
// Two-layer AD system:
using cg_scalar_t = CppAD::cg::CG<scalar_t>;      // Layer 1: Code generation
using ad_scalar_t = CppAD::AD<cg_scalar_t>;       // Layer 2: Automatic differentiation

// AD containers for Eigen:
using ad_vector_t = Eigen::Matrix<ad_scalar_t, Eigen::Dynamic, 1>;
using ad_matrix_t = Eigen::Matrix<ad_scalar_t, Eigen::Dynamic, Eigen::Dynamic>;
```

#### **E. Function Type Aliases**
```cpp
// Lambda without parameters: f(x) → y
using ad_function_t = std::function<void(const ad_vector_t&, ad_vector_t&)>;

// Lambda with parameters: f(x, p) → y (for MPC/tracking)
using ad_function_with_param_t = std::function<void(const ad_vector_t&, const ad_vector_t&, ad_vector_t&)>;

// Sparse matrix triplets
using triplet_vector_t = std::vector<Eigen::Triplet<scalar_t>>;
```

**Role:** This is the **"dictionary"** of the codebase - every other file uses these type aliases.

---

## **2. ParametersManager.h** - Runtime Parameter Storage

**Purpose:** A simple key-value store for managing runtime parameters (solver settings, problem parameters).

### **What it does:**

```cpp
class ParametersManager {
public:
    // Store a named parameter vector
    void setParameters(const std::string& name, const vector_t& params);
    
    // Retrieve a named parameter vector
    vector_t getParameters(const std::string& name) const;
    
    // Get all parameters
    std::unordered_map<std::string, vector_t> getParametersMap() const;
    
private:
    std::unordered_map<std::string, vector_t> parameters_;  // Name → Vector mapping
};
```

### **Example Usage in CRISP:**

```cpp
// In SolverInterface or OptimizationProblem:
ParametersManager params;

// Store solver hyperparameters
params.setParameters("maxIterations", vector_t::Constant(1, 100));
params.setParameters("trustRegionRadius", vector_t::Constant(1, 1.0));
params.setParameters("mu", vector_t::Constant(1, 0.1));

// Store problem-specific parameters (e.g., target states for MPC)
params.setParameters("targetState", target_position);

// Retrieve later
vector_t maxIter = params.getParameters("maxIterations");
vector_t target = params.getParameters("targetState");
```

### **Why it exists:**

From the pushbox example:
```cpp
SolverInterface solver(pushboxProblem, params);

// Change target state without recompiling AD library:
solver.setProblemParameters("pushboxObjective", xFinalStates);  // Uses ParametersManager

// Change solver settings:
solver.setHyperParameters("trailTol", vector_t::Constant(1, 1e-3));  // Uses ParametersManager
```

**Role:** Provides **runtime flexibility** - change parameters without touching the optimization structure.

---

## **3. ValueFunction.h** - Abstract Base Class for Functions

**Purpose:** Abstract interface for objective functions and constraint functions with both user-defined and auto-generated evaluation methods.

### **What it provides:**

#### **A. Pure Virtual Interface (must be implemented by subclasses)**
```cpp
class ValueFunction {
public:
    virtual ~ValueFunction() = default;
    
    // Non-parametric interface
    virtual vector_t getValue(const vector_t& x) = 0;
    virtual sparse_matrix_t getGradient(const vector_t& x) = 0;
    
    // Parametric interface (for MPC)
    virtual vector_t getValue(const vector_t& x, const vector_t& p) = 0;
    virtual sparse_matrix_t getGradient(const vector_t& x, const vector_t& p) = 0;
};
```

#### **B. Optional User-Defined Function Storage**
```cpp
// Setters for manually-defined functions (alternative to AD)
void setValueFunction(const std::function<vector_t(const vector_t&)>& valueFunction);
void setGradientFunction(const std::function<sparse_matrix_t(const vector_t&)>& gradientFunction);
void setValueFunctionWithParam(...);
void setGradientFunctionWithParam(...);
```

#### **C. CppAdInterface Integration**
```cpp
protected:
    std::unique_ptr<CppAdInterface> cppadInterface_;  // For automatic differentiation
    
    // Optional manual function storage
    std::function<vector_t(const vector_t&)> valueFunction_;
    std::function<sparse_matrix_t(const vector_t&)> gradientFunction_;
    std::function<vector_t(const vector_t&, const vector_t&)> valueFunctionWithParam_;
    std::function<sparse_matrix_t(const vector_t&, const vector_t&)> gradientFunctionWithParam_;
```

### **How it's used (subclasses):**

Looking at the architecture, there are likely subclasses like:

**ObjectiveFunction** (inherits ValueFunction):
```cpp
class ObjectiveFunction : public ValueFunction {
public:
    ObjectiveFunction(..., const ad_function_t& function) {
        // Initialize cppadInterface_ with user's lambda
        cppadInterface_ = std::make_unique<CppAdInterface>(..., function, ...);
    }
    
    vector_t getValue(const vector_t& x) override {
        // Use CppAdInterface to evaluate
        return cppadInterface_->computeFunctionValue(x);
    }
    
    sparse_matrix_t getGradient(const vector_t& x) override {
        // Use CppAdInterface for automatic gradient
        return cppadInterface_->computeSparseJacobian(x);
    }
};
```

**ConstraintFunction** (similar):
```cpp
class ConstraintFunction : public ValueFunction {
    // Same pattern: wraps CppAdInterface for constraint evaluation
};
```

**Role:** Provides a **unified interface** for all functions (objectives, constraints) regardless of whether they use:
- Automatic differentiation (via `cppadInterface_`)
- User-provided derivatives (via manual function storage)

---

## **Summary: The Three Files**

| File | Purpose | Key Content |
|------|---------|-------------|
| **BasicTypes.h** | Type system foundation | • Scalar/vector/matrix types<br>• AD types (CppAD+CodeGen)<br>• CSR sparse matrix struct<br>• Function type aliases |
| **ParametersManager.h** | Runtime parameter storage | • Key-value store for parameters<br>• Enables dynamic parameter changes<br>• Used for solver settings & problem params |
| **ValueFunction.h** | Function evaluation interface | • Abstract base for objectives/constraints<br>• Supports parametric & non-parametric<br>• Integrates with CppAdInterface<br>• Allows user-defined or auto-diff |

### **How They Work Together:**

```
BasicTypes.h
    ↓ (provides types to)
ParametersManager.h + ValueFunction.h
    ↓ (used by)
ObjectiveFunction, ConstraintFunction
    ↓ (used by)
OptimizationProblem
    ↓ (used by)
SolverInterface
```

These three files form the **foundational layer** that everything else in CRISP builds upon. They establish:
1. **Consistent types** (BasicTypes)
2. **Flexible parameters** (ParametersManager)
3. **Unified function interface** (ValueFunction)

xiao-wang007: using ad_function_t = std::function<void(const ad_vector_t&, ad_vector_t&)>;

is the type here the anonymous function

GitHub Copilot: Yes! `ad_function_t` is a **type alias for anonymous functions (lambdas)** that match a specific signature. Let me break this down:

---

## **What This Type Alias Means**

```cpp
using ad_function_t = std::function<void(const ad_vector_t&, ad_vector_t&)>;
//    ^^^^^^^^^^^^^^   ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//    Type alias       std::function wrapper for any callable with this signature
```

### **Breaking it down:**

1. **`std::function<...>`** - A general-purpose function wrapper that can hold:
   - Lambda functions (anonymous functions)
   - Regular functions
   - Function pointers
   - Functors (callable objects)

2. **`void(...)`** - Returns nothing (output via reference parameter)

3. **`const ad_vector_t& x`** - Input: AD vector (decision variables)

4. **`ad_vector_t& y`** - Output: AD vector (function values) - passed by reference to be modified

---

## **Example Usage: Anonymous Functions (Lambdas)**

From the pushbox example:

```cpp
// This lambda matches the ad_function_t signature:
ad_function_t pushboxDynamicConstraints = [](const ad_vector_t& x, ad_vector_t& y) {
//            ^^^^^^^^^^^^^^               ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//            Type                         Lambda (anonymous function)
    
    y.resize((N - 1) * num_state);  // Must set output size
    
    // Compute constraint values using x (inputs)
    for (size_t i = 0; i < N - 1; ++i) {
        ad_scalar_t px_i = x[idx + 0];
        ad_scalar_t py_i = x[idx + 1];
        // ... compute dynamics residuals ...
        y.segment(i * num_state, num_state) << px_next - px_i - px_dot * dt, ...;
    }
};
//^^ This whole lambda is stored in the ad_function_t variable
```

---

## **Why Use `std::function` + Type Alias?**

### **Without type alias (verbose):**
```cpp
std::function<void(const ad_vector_t&, ad_vector_t&)> myConstraint = [](const ad_vector_t& x, ad_vector_t& y) {
    // ... function body ...
};
```

### **With type alias (cleaner):**
```cpp
ad_function_t myConstraint = [](const ad_vector_t& x, ad_vector_t& y) {
    // ... function body ...
};
```

---

## **The Two Function Types in CRISP**

```cpp
// Type 1: No parameters (fixed problem structure)
using ad_function_t = std::function<void(const ad_vector_t& x, ad_vector_t& y)>;
//                                         ^^^^^^^^^^^^^^^^^^  ^^^^^^^^^^^^
//                                         Input: variables    Output: values

// Type 2: With parameters (for MPC, tracking problems)
using ad_function_with_param_t = std::function<void(const ad_vector_t& x, 
                                                     const ad_vector_t& p, 
                                                     ad_vector_t& y)>;
//                                                   ^^^^^^^^^^^^^^^^^^
//                                                   Parameters (e.g., target state)
```

---

## **Full Example Flow**

```cpp
// 1. Define the type (in BasicTypes.h)
using ad_function_t = std::function<void(const ad_vector_t&, ad_vector_t&)>;

// 2. Create a lambda that matches this type (in user code)
ad_function_t myObjective = [](const ad_vector_t& x, ad_vector_t& y) {
    y.resize(1);
    y[0] = x[0] * x[0] + x[1] * x[1];  // f(x) = x₀² + x₁²
};

// 3. Pass it to CppAdInterface (which expects ad_function_t)
CppAdInterface interface(
    variableDim, 
    modelName, 
    folderName, 
    functionName, 
    myObjective  // <-- Lambda passed here
);

// 4. CppAdInterface calls the lambda internally
ad_vector_t x_test(2);
x_test << 3.0, 4.0;
ad_vector_t y_result;

myObjective(x_test, y_result);  // Calls the lambda: y_result = [25]
```

---

## **Is it specifically for anonymous functions?**

**Technically:** `std::function` can hold any callable, not just lambdas:

```cpp
// Lambda (anonymous function) - most common
ad_function_t lambda = [](const ad_vector_t& x, ad_vector_t& y) { ... };

// Named function
void myNamedFunction(const ad_vector_t& x, ad_vector_t& y) { ... }
ad_function_t named = myNamedFunction;

// Functor (callable object)
struct MyFunctor {
    void operator()(const ad_vector_t& x, ad_vector_t& y) const { ... }
};
ad_function_t functor = MyFunctor();
```

**Practically in CRISP:** It's **always used with lambdas** because:
- Users define functions inline (convenient)
- Lambdas can capture constants from outer scope (like `dt`, `m`, `g`)
- Clean syntax for mathematical expressions

---

## **Summary**

| Question | Answer |
|----------|--------|
| **Is `ad_function_t` for anonymous functions?** | Primarily yes - it's designed to hold lambdas (anonymous functions) |
| **Can it hold other callables?** | Yes (named functions, functors, etc.) but CRISP uses it exclusively for lambdas |
| **What's the signature?** | `void f(const ad_vector_t& input, ad_vector_t& output)` |
| **Why type alias?** | Cleaner code, self-documenting (signals "this is an AD function") |

So yes, **`ad_function_t` is the type for anonymous functions (lambdas)** that define objectives and constraints in CRISP!

xiao-wang007: using ad_function_with_param_t = std::function<void(const ad_vector_t&, const ad_vector_t&, ad_vector_t&)>;

what does with_param_t here mean

GitHub Copilot: The `_with_param` suffix means this function type includes **parameters** in addition to decision variables. Let me explain the difference:

---

## **Two Function Types: With vs. Without Parameters**

### **Type 1: `ad_function_t` (No Parameters)**
```cpp
using ad_function_t = std::function<void(const ad_vector_t& x, ad_vector_t& y)>;
//                                        ^^^^^^^^^^^^^^^^^^  ^^^^^^^^^^^^
//                                        Input: variables    Output: values
```

**Usage:** Functions where everything is fixed (no runtime changes)
```cpp
ad_function_t dynamics = [](const ad_vector_t& x, ad_vector_t& y) {
    // x = [position, velocity, control, ...]
    // Dynamics: x_next = f(x_current)
    // All constants like dt, mass, gravity are captured from outer scope
    y.resize(num_states);
    y[0] = x[0] + x[1] * dt;  // position += velocity * dt
    y[1] = x[1] + (F / mass - g) * dt;  // velocity += acceleration * dt
};
```

---

### **Type 2: `ad_function_with_param_t` (With Parameters)**
```cpp
using ad_function_with_param_t = std::function<void(const ad_vector_t& x, 
                                                     const ad_vector_t& p, 
                                                     ad_vector_t& y)>;
//                                                   ^^^^^^^^^^^^^^^^^^
//                                                   NEW: parameters
```

**Usage:** Functions where you want to **change behavior at runtime** without recompiling the AD library

---

## **What Are "Parameters" (p)?**

Parameters are **values you can change without regenerating the autodiff library**. Common uses:

### **1. Tracking/Reference Targets (MPC)**
```cpp
ad_function_with_param_t objective = [](const ad_vector_t& x, const ad_vector_t& p, ad_vector_t& y) {
    //                                                         ^^^^^^^^^^^^^^^^^^
    //                                                         p = target state
    
    y.resize(1);
    ad_scalar_t cost = 0.0;
    
    // Track to a moving target (p changes each MPC iteration)
    ad_vector_t tracking_error(3);
    tracking_error << x[0] - p[0],    // position error
                      x[1] - p[1],    // velocity error  
                      x[2] - p[2];    // angle error
    
    cost = tracking_error.transpose() * Q * tracking_error;
    y[0] = cost;
};

// Can change target without recompiling:
solver.setProblemParameters("objective", new_target);  // p = new_target
solver.solve();  // Reuses compiled AD library, just different p values
```

### **2. Initial Conditions**
```cpp
ad_function_with_param_t initialConstraint = [](const ad_vector_t& x, const ad_vector_t& p, ad_vector_t& y) {
    //                                                                 ^^^^^^^^^^^^^^^^^^
    //                                                                 p = initial state
    
    y.resize(3);
    y[0] = x[0] - p[0];  // x[0] must equal p[0] (initial position)
    y[1] = x[1] - p[1];  // x[1] must equal p[1] (initial velocity)
    y[2] = x[2] - p[2];  // x[2] must equal p[2] (initial angle)
};

// Different initial conditions without recompilation:
solver.setProblemParameters("initialConstraint", start_state_1);
solver.solve();

solver.setProblemParameters("initialConstraint", start_state_2);
solver.solve();  // Same compiled library, different initial conditions
```

---

## **Real Example from Pushbox**

### **Objective with Parameters (Target State)**
```cpp
ad_function_with_param_t pushboxObjective = [](const ad_vector_t& x, 
                                                const ad_vector_t& p, 
                                                ad_vector_t& y) {
    y.resize(1);
    ad_scalar_t tracking_cost(0.0);
    
    for (size_t i = 0; i < N; ++i) {
        ad_scalar_t px_i = x[idx + 0];
        ad_scalar_t py_i = x[idx + 1];
        ad_scalar_t theta_i = x[idx + 2];
        
        if (i == N - 1) {  // Terminal cost
            ad_vector_t tracking_error(3);
            tracking_error << px_i - p[0],      // Target X from p
                            py_i - p[1],        // Target Y from p
                            theta_i - p[2];     // Target angle from p
            
            tracking_cost += tracking_error.transpose() * Q * tracking_error;
        }
    }
    y[0] = tracking_cost;
};

// Later in main():
xFinalStates << 3*cos(theta), 3*sin(theta), theta;  // Different target
solver.setProblemParameters("pushboxObjective", xFinalStates);  // p = xFinalStates
```

### **Initial Constraint with Parameters**
```cpp
ad_function_with_param_t pushboxInitialConstraints = [](const ad_vector_t& x, 
                                                         const ad_vector_t& p, 
                                                         ad_vector_t& y) {
    y.resize(3);
    y[0] = x[0] - p[0];  // Initial position must match p
    y[1] = x[1] - p[1];
    y[2] = x[2] - p[2];
};

// Set initial state:
xInitialStates << 0, 0, 0;
solver.setProblemParameters("pushboxInitialConstraints", xInitialStates);
```

---

## **Why Use Parameters?**

### **Without Parameters (Slow)**
```cpp
// Change target → must regenerate entire AD library (minutes!)
ad_function_t objective = [target1](const ad_vector_t& x, ad_vector_t& y) {
    y[0] = (x[0] - target1[0])^2 + ...;  // target1 captured in lambda
};
// Generate and compile AD library... ⏳ (slow)

// Want different target? Create NEW lambda with target2
ad_function_t objective2 = [target2](const ad_vector_t& x, ad_vector_t& y) {
    y[0] = (x[0] - target2[0])^2 + ...;  // target2 captured
};
// Regenerate and recompile AD library... ⏳ (slow again!)
```

### **With Parameters (Fast)**
```cpp
// Define once with parameter placeholder
ad_function_with_param_t objective = [](const ad_vector_t& x, const ad_vector_t& p, ad_vector_t& y) {
    y[0] = (x[0] - p[0])^2 + ...;  // p is a placeholder
};
// Generate and compile AD library ONCE... ⏳

// Change target instantly (no recompilation!)
solver.setProblemParameters("objective", target1);  // ⚡ Fast
solver.solve();

solver.setProblemParameters("objective", target2);  // ⚡ Fast
solver.solve();

solver.setProblemParameters("objective", target3);  // ⚡ Fast
solver.solve();
```

---

## **How CppAdInterface Handles Parameters**

When you create the AD library:

```cpp
// CppAdInterface concatenates x and p internally:
ad_vector_t axp(variableDim_ + parameterDim_);
axp.segment(0, variableDim_) = ax;           // Decision variables
axp.segment(variableDim_, parameterDim_) = ap;  // Parameters

CppAD::Independent(axp);  // Treat both as "independent" for AD

// Call user's function
functionWithParam_(axp.segment(0, variableDim_), 
                   axp.segment(variableDim_, parameterDim_), 
                   ay);
```

When evaluating derivatives:
```cpp
// Only compute derivatives w.r.t. x, not p
for (size_t i = 0; i < row.size(); ++i) {
    if (col[i] < variableDim_) {  // Exclude parameter columns
        jacobian_wrt_x.add(row[i], col[i], jac[i]);
    }
}
```

---

## **Summary**

| Aspect | `ad_function_t` | `ad_function_with_param_t` |
|--------|-----------------|---------------------------|
| **Signature** | `void f(x, y)` | `void f(x, p, y)` |
| **Parameters** | None | Yes (vector `p`) |
| **Use case** | Fixed problem structure | MPC, tracking, varying conditions |
| **Runtime changes** | Must recompile AD library | Just change parameter values ⚡ |
| **Example** | Dynamics constraints | Terminal cost with moving target |

**`_with_param_t`** = "**with parameters**" = allows runtime changes without recompilation, essential for MPC and adaptive problems!

xiao-wang007: #include "math.h"
which lib is this?

GitHub Copilot: `#include "math.h"` is the **C standard math library** (also available in C++ as `<cmath>`).

---

## **What Library Is This?**

### **In C/C++:**
- **Header:** `math.h` (C style) or `cmath` (C++ style)
- **Part of:** C Standard Library (libc) / C++ Standard Library
- **Linking:** Usually automatic, but sometimes needs `-lm` flag on Unix systems

### **What It Provides:**

Common mathematical functions:

```cpp
#include <math.h>  // or <cmath>

// Trigonometric functions
double angle = M_PI / 4;
double s = sin(angle);      // sine
double c = cos(angle);      // cosine
double t = tan(angle);      // tangent
double a = atan2(y, x);     // arc tangent (2-argument)

// Power and exponential
double p = pow(2.0, 3.0);   // 2^3 = 8
double sq = sqrt(16.0);     // √16 = 4
double e = exp(1.0);        // e^1
double l = log(10.0);       // natural log

// Rounding
double f = floor(3.7);      // 3.0
double ce = ceil(3.2);      // 4.0
double r = round(3.5);      // 4.0

// Absolute value
double ab = fabs(-5.5);     // 5.5

// Constants (not always in math.h, but common)
M_PI      // π ≈ 3.14159...
M_E       // e ≈ 2.71828...
```

---

## **In the Pushbox Example:**

Looking at the code:
```cpp
#include "math.h"

const scalar_t r = sqrt(a * a + b * b);  // Uses sqrt() from math.h
```

And later in `main()`:
```cpp
scalar_t theta = 12 * 2 * M_PI / num_segments;
xFinalStates << 3*cos(theta), 3*sin(theta), theta;
//              ^^^^^^^^^     ^^^^^^^^^
//              cos() and sin() from math.h
```

---

## **C vs. C++ Style**

### **C Style: `math.h`**
```cpp
#include <math.h>

double x = sqrt(4.0);  // Functions in global namespace
```

### **C++ Style: `cmath` (recommended)**
```cpp
#include <cmath>

double x = std::sqrt(4.0);  // Functions in std:: namespace
// OR
using namespace std;
double x = sqrt(4.0);
```

---

## **Why Both Styles Work:**

The code uses `"math.h"` with quotes (searches local directory first, then system), but it works because:

1. The compiler finds the system `math.h` header
2. Functions like `sin()`, `cos()`, `sqrt()` are available in the global namespace (C compatibility)
3. No linking issues on modern systems (math library automatically linked)

---

## **Best Practice for This Code:**

Could change to C++ style:
```cpp
#include <cmath>  // C++ style

const scalar_t r = std::sqrt(a * a + b * b);
// OR if "using namespace std;" elsewhere:
const scalar_t r = sqrt(a * a + b * b);
```

But `#include "math.h"` works fine and is common in C++ code that maintains C compatibility.

---

## **Summary**

| Question | Answer |
|----------|--------|
| **Which library?** | C Standard Math Library (part of libc/C++ stdlib) |
| **Header name** | `math.h` (C style) or `<cmath>` (C++ style) |
| **Used for?** | `sqrt()`, `sin()`, `cos()`, `M_PI`, etc. |
| **Linking needed?** | Usually automatic; sometimes `-lm` on Unix |
| **In this code** | Computing radius, trigonometric target positions |

xiao-wang007: I want to define a function that I would like to use AD with. how to use this cppadinterface.h

GitHub Copilot: Let me show you how to define a function and use CppAdInterface for automatic differentiation. I'll give you a step-by-step guide with examples.

---

## **Step-by-Step: Using CppAdInterface for AD**

### **Step 1: Include Required Headers**

```cpp
#include "cppad_core/CppAdInterface.h"
#include "common/BasicTypes.h"
// Optional: for building optimization problems
#include "problem_core/ObjectiveFunction.h"
#include "problem_core/ConstraintFunction.h"

using namespace CRISP;
```

---

### **Step 2: Define Your Function Using Lambda**

You have two options depending on whether you need runtime parameters:

#### **Option A: Function Without Parameters (Fixed)**

```cpp
// Define your function as a lambda
ad_function_t myFunction = [](const ad_vector_t& x, ad_vector_t& y) {
    // x = input vector (decision variables)
    // y = output vector (function values) - YOU must set the size!
    
    y.resize(3);  // ⚠️ MUST set output size first!
    
    // Define your function using AD types
    // Example: y[0] = x[0]^2 + x[1]^2
    //          y[1] = sin(x[0]) * cos(x[1])
    //          y[2] = x[0] + x[1] + x[2]
    
    y[0] = x[0] * x[0] + x[1] * x[1];
    y[1] = sin(x[0]) * cos(x[1]);
    y[2] = x[0] + x[1] + x[2];
};
```

#### **Option B: Function With Parameters (For MPC/Tracking)**

```cpp
ad_function_with_param_t myParametricFunction = [](const ad_vector_t& x, 
                                                    const ad_vector_t& p, 
                                                    ad_vector_t& y) {
    // x = decision variables
    // p = parameters (can change at runtime!)
    // y = output
    
    y.resize(1);
    
    // Example: tracking cost to target position in p
    ad_vector_t error(3);
    error << x[0] - p[0],  // position error
             x[1] - p[1],  // velocity error
             x[2] - p[2];  // angle error
    
    y[0] = error.transpose() * error;  // quadratic cost
};
```

---

### **Step 3: Create CppAdInterface Object**

#### **For Non-Parametric Function:**

```cpp
size_t variableDim = 3;  // Number of input variables
std::string modelName = "MyModel";
std::string folderName = "model";  // Where to save compiled library
std::string functionName = "myFunction";

// Create the interface - this will generate and compile the AD library
CppAdInterface interface(
    variableDim,           // Input dimension
    modelName,             // Model name
    folderName,            // Output folder
    functionName,          // Function name
    myFunction,            // Your lambda
    CppAdInterface::ModelInfoLevel::SECOND_ORDER,  // Generate Hessians too
    true                   // Regenerate library (false to reuse existing)
);

// On first run: Compiles C++ code (slow, ~seconds)
// Subsequent runs: Loads compiled library (fast, ~milliseconds)
```

#### **For Parametric Function:**

```cpp
size_t variableDim = 3;    // Number of decision variables
size_t parameterDim = 3;   // Number of parameters
std::string modelName = "TrackingModel";
std::string folderName = "model";
std::string functionName = "trackingCost";

CppAdInterface interface(
    variableDim,           // x dimension
    parameterDim,          // p dimension
    modelName,
    folderName,
    functionName,
    myParametricFunction,  // Your lambda with parameters
    CppAdInterface::ModelInfoLevel::SECOND_ORDER,
    true
);
```

---

### **Step 4: Evaluate Function and Derivatives**

#### **Function Value:**

```cpp
// Define input
vector_t x(3);
x << 1.0, 2.0, 3.0;

// Evaluate function
vector_t y = interface.computeFunctionValue(x);
std::cout << "f(x) = " << y.transpose() << std::endl;
```

#### **Jacobian (∂f/∂x):**

```cpp
// Compute sparse Jacobian
sparse_matrix_t jacobian = interface.computeSparseJacobian(x);
std::cout << "Jacobian:\n" << jacobian << std::endl;

// Or get as CSR format (faster for large problems)
CSRSparseMatrix jacobianCSR = interface.computeSparseJacobianCSR(x);
```

#### **Hessian (∂²f/∂x²) - for scalar functions:**

```cpp
// Only works if function outputs a scalar (y.size() == 1)
sparse_matrix_t hessian = interface.computeSparseHessian(x);
std::cout << "Hessian:\n" << hessian << std::endl;

// Or CSR format
CSRSparseMatrix hessianCSR = interface.computeSparseHessianCSR(x);
```

#### **For Parametric Functions:**

```cpp
vector_t x(3);
x << 1.0, 2.0, 3.0;

vector_t p(3);  // Parameters
p << 5.0, 6.0, 7.0;  // Target values

// Function value
vector_t y = interface.computeFunctionValue(x, p);

// Jacobian w.r.t. x (NOT w.r.t. p)
sparse_matrix_t jacobian = interface.computeSparseJacobian(x, p);

// Hessian w.r.t. x
sparse_matrix_t hessian = interface.computeSparseHessian(x, p);
```

---

## **Complete Working Example**

```cpp
#include "cppad_core/CppAdInterface.h"
#include "common/BasicTypes.h"
#include <iostream>

using namespace CRISP;

int main() {
    // Step 1: Define function
    ad_function_t myQuadratic = [](const ad_vector_t& x, ad_vector_t& y) {
        y.resize(1);  // Scalar output
        
        // f(x) = x₀² + x₁² + x₀*x₁
        y[0] = x[0] * x[0] + x[1] * x[1] + x[0] * x[1];
    };
    
    // Step 2: Create interface
    std::cout << "Creating CppAdInterface..." << std::endl;
    CppAdInterface interface(
        2,              // 2 input variables
        "QuadModel",
        "model",
        "quadratic",
        myQuadratic,
        CppAdInterface::ModelInfoLevel::SECOND_ORDER,
        true
    );
    std::cout << "Done! Library compiled and loaded." << std::endl;
    
    // Step 3: Evaluate at a point
    vector_t x(2);
    x << 2.0, 3.0;
    
    // Function value
    vector_t y = interface.computeFunctionValue(x);
    std::cout << "\nf(" << x.transpose() << ") = " << y[0] << std::endl;
    // Should be: 2² + 3² + 2*3 = 4 + 9 + 6 = 19
    
    // Gradient
    sparse_matrix_t grad = interface.computeSparseJacobian(x);
    std::cout << "\n∇f = " << grad << std::endl;
    // Should be: [∂f/∂x₀, ∂f/∂x₁] = [2x₀ + x₁, 2x₁ + x₀] = [7, 8]
    
    // Hessian
    sparse_matrix_t hess = interface.computeSparseHessian(x);
    std::cout << "\n∇²f =\n" << hess << std::endl;
    // Should be: [[2, 1], [1, 2]]
    
    return 0;
}
```

---

## **Key Points to Remember**

| ⚠️ Important | Details |
|-------------|---------|
| **Output size** | ALWAYS call `y.resize(...)` first thing in your lambda |
| **AD types** | Use `ad_scalar_t`, `ad_vector_t`, `ad_matrix_t` inside lambda |
| **First run** | Slow (compiles C++ code) - be patient! |
| **Subsequent runs** | Fast (loads compiled `.so`/`.dylib`) |
| **Hessian** | Only for scalar functions (`y.size() == 1`) |
| **Sparsity** | Automatically detected and exploited |
| **Files created** | Look in `model/YourModel/auto_generated/` for `.so`/`.dylib` |

---

## **Integration with Optimization Problem**

For use in CRISP's optimizer:

```cpp
// Wrap in ObjectiveFunction or ConstraintFunction
auto objective = std::make_shared<ObjectiveFunction>(
    variableDim, 
    paramDim,      // 0 if no parameters
    problemName, 
    folderName, 
    functionName, 
    myFunction
);

// Add to optimization problem
OptimizationProblem problem(variableDim, "MyProblem");
problem.addObjective(objective);
```

Would you like me to show a more specific example for your particular use case?