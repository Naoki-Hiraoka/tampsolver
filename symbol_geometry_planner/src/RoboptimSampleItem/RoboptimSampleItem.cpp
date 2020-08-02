#include "RoboptimSampleItem.h"
#include <roboptim/core.hh>

using namespace std::placeholders;
using namespace cnoid;
using namespace roboptim;

namespace cnoid {

  void RoboptimSampleItem::initializeClass(ExtensionManager* ext)
  {
    ext->itemManager()
      .registerClass<RoboptimSampleItem>("RoboptimSampleItem");
  }


  RoboptimSampleItem::RoboptimSampleItem()
    : PlannerBaseItem()
  {
  }


  RoboptimSampleItem::RoboptimSampleItem(const RoboptimSampleItem& org)
    : PlannerBaseItem(org)
  {
  }



  RoboptimSampleItem::~RoboptimSampleItem()
  {
  }



  struct F : public TwiceDifferentiableFunction
  {
    F () : TwiceDifferentiableFunction (4, 1, "x₀ * x₃ * (x₀ + x₁ + x₂) + x₂")
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
      grad << x[0] * x[3] + x[3] * (x[0] + x[1] + x[2]),
        x[0] * x[3],
        x[0] * x[3] + 1,
        x[0] * (x[0] + x[1] + x[2]);
    }

    void
    impl_hessian (hessian_ref h, const_argument_ref x, size_type) const
    {
      h << 2 * x[3],               x[3], x[3], 2 * x[0] + x[1] + x[2],
        x[3],                   0.,   0.,   x[0],
        x[3],                   0.,   0.,   x[1],
        2 * x[0] + x[1] + x[2], x[0], x[0], 0.;
    }
  };


  struct G0 : public TwiceDifferentiableFunction
  {
    G0 () : TwiceDifferentiableFunction (4, 1, "x₀ * x₁ * x₂ * x₃")
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
      grad << x[1] * x[2] * x[3],
        x[0] * x[2] * x[3],
        x[0] * x[1] * x[3],
        x[0] * x[1] * x[2];
    }

    void
    impl_hessian (hessian_ref h, const_argument_ref x, size_type) const
    {
      h << 0.,          x[2] * x[3], x[1] * x[3], x[1] * x[2],
        x[2] * x[3], 0.,          x[0] * x[3], x[0] * x[2],
        x[1] * x[3], x[0] * x[3], 0.,          x[0] * x[1],
        x[1] * x[2], x[0] * x[2], x[0] * x[1], 0.;
    }
  };

  struct G1 : public TwiceDifferentiableFunction
  {
    G1 () : TwiceDifferentiableFunction (4, 1, "x₀² + x₁² + x₂² + x₃²")
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
      grad = 2 * x;
    }

    void
    impl_hessian (hessian_ref h, const_argument_ref x, size_type) const
    {
      h << 2., 0., 0., 0.,
        0., 2., 0., 0.,
        0., 0., 2., 0.,
        0., 0., 0., 2.;
    }
  };


  void RoboptimSampleItem::main()
  {
    typedef Solver<EigenMatrixDense> solver_t;

    // Create cost function.
    boost::shared_ptr<F> f (new F ());

    // Create problem.
    solver_t::problem_t pb (f);

    // Set bounds for all optimization parameters.
    // 1. < x_i < 5. (x_i in [1.;5.])
    for (Function::size_type i = 0; i < pb.function ().inputSize (); ++i)
      pb.argumentBounds ()[i] = Function::makeInterval (1., 5.);

    // Set the starting point.
    Function::vector_t start (pb.function ().inputSize ());
    start[0] = 1., start[1] = 5., start[2] = 5., start[3] = 1.;

    // Create constraints.
    boost::shared_ptr<G0> g0 (new G0 ());
    boost::shared_ptr<G1> g1 (new G1 ());

    F::intervals_t bounds;
    solver_t::problem_t::scaling_t scaling;

    // Add constraints
    bounds.push_back(Function::makeLowerInterval (25.));
    scaling.push_back (1.);
      pb.addConstraint
        (boost::static_pointer_cast<TwiceDifferentiableFunction> (g0),
         bounds, scaling);

      bounds.clear ();
      scaling.clear ();

      bounds.push_back(Function::makeInterval (40., 40.));
      scaling.push_back (1.);
        pb.addConstraint
          (boost::static_pointer_cast<TwiceDifferentiableFunction> (g1),
           bounds, scaling);

        // Initialize solver.

        // Here we are relying on a dummy solver.
        // You may change this string to load the solver you wish to use:
        //  - Ipopt: "ipopt", "ipopt-sparse", "ipopt-td"
        //  - Eigen: "eigen-levenberg-marquardt"
        //  etc.
        // The plugin is built for a given solver type, so choose it adequately.
        SolverFactory<solver_t> factory ("ipopt", pb);
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
