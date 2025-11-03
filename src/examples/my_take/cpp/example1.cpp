#include "cppad_core/CppAdInterface.h"
#include "common/BasicTypes.h"

using namespace CRISP;

/* define function as using Lambda */
// option 1: function without parameters passing in
ad_function_t example1_objective = [](const ad_vector_t& x, ad_vector_t& y) {
    y.resize(1);
    y[0] = x[0] * x[0] + x[1] * x[1];
};

// option 2: function with parameters passing in
ad_function_with_param_t example1_error = [](const ad_vector_t& x, const ad_vector_t& p, ad_vector_t& y) {
    y.resize(1);
    y[0] = (x[0] - p[0]) * (x[0] - p[0]) + (x[1] - p[1]) * (x[1] - p[1]);
};

// create CppAdInterface object for option 1
size_t variableDim = 2; // number of input variables 
std::string modelName = "my_first_take";
std::string folderName = "precompiled_grads";
std::string functionName = "example1_objective";

CppAdInterface interface(
    variableDim,
    modelName,
    folderName,
    functionName,
    example1_objective,
    CppAdInterface::ModelInfoLevel::SECOND_ORDER, // Generate Hessians too
    true // Regenerate the library
);

// create CppAdInterface object for option 2
size_t variableDim = 2;    // Number of decision variables
size_t parameterDim = 2;   // Number of parameters
std::string modelName = "my_first_take_error";
std::string folderName = "model";
std::string functionName = "example1_error";

CppAdInterface interface(
    variableDim,           // x dimension
    parameterDim,          // p dimension
    modelName,
    folderName,
    functionName,
    example1_error,  // Your lambda with parameters
    CppAdInterface::ModelInfoLevel::SECOND_ORDER,
    true
);