#include "RoboptimSparseSampleItem.h"
#include <roboptim/core.hh>

using namespace std::placeholders;
using namespace cnoid;
using namespace roboptim;

namespace cnoid {

  void RoboptimSparseSampleItem::initializeClass(ExtensionManager* ext)
  {
    ext->itemManager()
      .registerClass<RoboptimSparseSampleItem>("RoboptimSparseSampleItem");
  }


  RoboptimSparseSampleItem::RoboptimSparseSampleItem()
    : PlannerBaseItem()
  {
  }


  RoboptimSparseSampleItem::RoboptimSparseSampleItem(const RoboptimSparseSampleItem& org)
    : PlannerBaseItem(org)
  {
  }



  RoboptimSparseSampleItem::~RoboptimSparseSampleItem()
  {
  }


  namespace RoboptimSparseSample {

    struct F : public TwiceDifferentiableSparseFunction
    {
      F () : TwiceDifferentiableSparseFunction (4, 1, "x₀ * x₃ * (x₀ + x₁ + x₂) + x₂")
      {
      }

      void
      impl_compute (result_ref result, const_argument_ref x) const
      {
        result[0] = x[0] * x[3] * (x[0] + x[1] + x[2]) + x[2];
      }

      void
      impl_gradient (gradient_ref grad, const_argument_ref x, size_type) const
      {
        grad.coeffRef(0,0) = x[0] * x[3] + x[3] * (x[0] + x[1] + x[2]);
        grad.coeffRef(0,1) = x[0] * x[3];
        grad.coeffRef(0,2) = x[0] * x[3] + 1;
        grad.coeffRef(0,3) = x[0] * (x[0] + x[1] + x[2]);
      }

      void
      impl_hessian (hessian_ref h, const_argument_ref x, size_type) const
      {
        h.coeffRef(0,0)=2 * x[3]; h.coeffRef(0,1)=x[3]; h.coeffRef(0,2)=x[3]; h.coeffRef(0,3)=2 * x[0] + x[1] + x[2];
        h.coeffRef(1,0)=x[3]; h.coeffRef(1,3)=x[0];
        h.coeffRef(2,0)=x[3]; h.coeffRef(2,3)=x[1];//bug?
        h.coeffRef(3,0)=2 * x[0] + x[1] + x[2]; h.coeffRef(3,1)=x[0]; h.coeffRef(3,2)=x[0];
      }
    };


    struct G0 : public TwiceDifferentiableSparseFunction
    {
      G0 () : TwiceDifferentiableSparseFunction (4, 1, "x₀ * x₁ * x₂ * x₃")
      {
      }

      void
      impl_compute (result_ref result, const_argument_ref x) const
      {
        result[0] = x[0] * x[1] * x[2] * x[3];
      }

      void
      impl_gradient (gradient_ref grad, const_argument_ref x, size_type) const
      {
        grad.coeffRef(0,0) = x[1] * x[2] * x[3];
        grad.coeffRef(0,1) = x[0] * x[2] * x[3];
        grad.coeffRef(0,2) = x[0] * x[1] * x[3];
        grad.coeffRef(0,3) = x[0] * x[1] * x[2];
      }

      void
      impl_hessian (hessian_ref h, const_argument_ref x, size_type) const
      {
        h.coeffRef(0,1)=x[2] * x[3]; h.coeffRef(0,2)=x[1] * x[3]; h.coeffRef(0,3)=x[1] * x[2];
        h.coeffRef(1,0)=x[2] * x[3]; h.coeffRef(1,2)=x[0] * x[3]; h.coeffRef(1,3)=x[0] * x[2];
        h.coeffRef(2,0)=x[1] * x[3]; h.coeffRef(2,1)=x[0] * x[3]; h.coeffRef(2,3)=x[0] * x[1];
        h.coeffRef(3,0)=x[1] * x[2]; h.coeffRef(3,1)=x[0] * x[2]; h.coeffRef(3,2)=x[0] * x[1];
      }
    };

    struct G1 : public TwiceDifferentiableSparseFunction
    {
      G1 () : TwiceDifferentiableSparseFunction (4, 1, "x₀² + x₁² + x₂² + x₃²")
      {
      }

      void
      impl_compute (result_ref result, const_argument_ref x) const
      {
        result[0] = x[0] * x[0] + x[1] * x[1] + x[2] * x[2] + x[3] * x[3];
      }

      void
      impl_gradient (gradient_ref grad, const_argument_ref x, size_type) const
      {
        grad.coeffRef(0,0) = 2 * x[0];
        grad.coeffRef(0,1) = 2 * x[1];
        grad.coeffRef(0,2) = 2 * x[2];
        grad.coeffRef(0,3) = 2 * x[3];
      }

      void
      impl_hessian (hessian_ref h, const_argument_ref x, size_type) const
      {
        h.coeffRef(0,0)=2;
        h.coeffRef(1,1)=2;
        h.coeffRef(2,2)=2;
        h.coeffRef(3,3)=2;
      }
    };
  }

  void RoboptimSparseSampleItem::main()
  {
    using namespace RoboptimSparseSample;

    typedef Solver<EigenMatrixSparse> solver_t;

    // Create cost function.
    boost::shared_ptr<F> f (new F ());

    // Create problem.
    solver_t::problem_t pb (f);

    // Set bounds for all optimization parameters.
    // 1. < x_i < 5. (x_i in [1.;5.])
    for (SparseFunction::size_type i = 0; i < pb.function ().inputSize (); ++i)
      pb.argumentBounds ()[i] = SparseFunction::makeInterval (1., 5.);

    // Set the starting point.
    SparseFunction::vector_t start (pb.function ().inputSize ());
    start[0] = 1., start[1] = 5., start[2] = 5., start[3] = 1.;

    // Create constraints.
    boost::shared_ptr<G0> g0 (new G0 ());
    boost::shared_ptr<G1> g1 (new G1 ());

    F::intervals_t bounds;
    solver_t::problem_t::scaling_t scaling;

    // Add constraints
    bounds.push_back(SparseFunction::makeLowerInterval (25.));
    scaling.push_back (1.);
      pb.addConstraint
        (boost::static_pointer_cast<TwiceDifferentiableSparseFunction> (g0),
         bounds, scaling);

      bounds.clear ();
      scaling.clear ();

      bounds.push_back(SparseFunction::makeInterval (40., 40.));
      scaling.push_back (1.);
        pb.addConstraint
          (boost::static_pointer_cast<TwiceDifferentiableSparseFunction> (g1),
           bounds, scaling);

        // Initialize solver.

        // Here we are relying on a dummy solver.
        // You may change this string to load the solver you wish to use:
        //  - Ipopt: "ipopt", "ipopt-sparse", "ipopt-td"
        //  - Eigen: "eigen-levenberg-marquardt"
        //  etc.
        // The plugin is built for a given solver type, so choose it adequately.
        SolverFactory<solver_t> factory ("ipopt-sparse", pb);
        solver_t& solver = factory ();

        // Compute the minimum and retrieve the result.
        solver_t::result_t res = solver.minimum ();

        // Display solver information.
        this->mv->cout() << solver << std::endl;

        // Check if the minimization has succeeded.

        // Process the result
        switch (res.which ())
          {
          case solver_t::SOLVER_VALUE:
            {
              // Get the result.
              Result& result = boost::get<Result> (res);

              // Display the result.
              this->mv->cout() << "A solution has been found: " << std::endl
                        << result << std::endl;

              return;
            }

          case solver_t::SOLVER_ERROR:
            {
              this->mv->cout() << "A solution should have been found. Failing..."
                        << std::endl
                        << boost::get<SolverError> (res).what ()
                        << std::endl;

              return;
            }

          case solver_t::SOLVER_NO_SOLUTION:
            {
              this->mv->cout() << "The problem has not been solved yet."
                        << std::endl;

              return;
            }
          }

        // Should never happen.
        assert (0);
        return;
  }
}
