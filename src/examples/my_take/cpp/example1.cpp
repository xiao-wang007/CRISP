#include "cppad_core/CppAdInterface.h"
#include "common/BasicTypes.h"
#include "problem_core/ObjectiveFunction.h"
#include <iostream>
#include <boost/type_index.hpp>

using namespace CRISP;

/* Define functions using Lambda */
// Option 1: function without parameters passing in
ad_function_t example1_objective = [](const ad_vector_t& x, ad_vector_t& y) {
    y.resize(1);
    y[0] = x[0] * x[0] + x[1] * x[1];
};

int main() {

    /* 1st tutorial: the very basic usage */
    std::cout << "Creating CppAdInterface for simple objective function..." << std::endl;
    
    // Create CppAdInterface object inside main
    size_t variableDim = 2;
    std::string modelName = "my_first_take";
    std::string folderName = "model";
    std::string functionName = "example1_objective";

    CppAdInterface interface(
        variableDim,
        modelName,
        folderName,
        functionName,
        example1_objective,
        CppAdInterface::ModelInfoLevel::SECOND_ORDER,
        false // Regenerate the library (false to use existing)
    );

    std::cout << "CppAdInterface created successfully!" << std::endl;
    
    // Test evaluation
    vector_t x(2);
    x << 2.0, 3.0;
    
    // evaluate the function
    std::cout << "Evaluating f(x) where x = [" << x.transpose() << "]" << std::endl;
    vector_t y = interface.computeFunctionValue(x);
    std::cout << "Objective function value: f(x) = x[0]^2 + x[1]^2 = " << y[0] << std::endl;

    // compute derivatives
    sparse_matrix_t jac = interface.computeSparseJacobian(x);
    std::cout << "example1 objective function Jacobian: \n" << jac << std::endl;
    // in the form of compressed sparse row (CSR) format
    CSRSparseMatrix jac_csr = interface.computeSparseJacobianCSR(x);

    // compute Hessian (only works if y.size() == 1)
    sparse_matrix_t hess = interface.computeSparseHessian(x);
    std::cout << "Hessian:\n" << hess << std::endl;
    std::cout << "################################################################ \n"
              << std::endl;

    /* 2nd tutorial: using ObjectiveFunction.h */
    std::cout << "Creating ObjectiveFunction for simple objective function... \n" 
              << std::endl;
    // turning the function into an ObjectiveFunction object
    // auto obj = std::make_shared<ObjectiveFunction>(variableNum, num_state, problemName, folderName, "pushbotObjective", pushbotObjective);
    std::shared_ptr<ObjectiveFunction> obj1 = std::make_shared<ObjectiveFunction>(
        variableDim,
        modelName,
        folderName,
        functionName,
        example1_objective);

    std::cout << "ObjectiveFunction type of obj1: " 
              << boost::typeindex::type_id_with_cvr<decltype(obj1)>().pretty_name() 
              << std::endl;



    return 0;
}
