#include <gtsam/nonlinear/DoglegOptimizerImpl.h>

namespace gtsam {

namespace DoglegOptimizerImplCustom {

  /* ************************************************************************* */
template<class M, class F, class VALUES>
typename DoglegOptimizerImpl::IterationResult Iterate(
    double delta, DoglegOptimizerImpl::TrustRegionAdaptationMode mode, const VectorValues& dx_u, const VectorValues& dx_n,
    const M& Rd, const F& f, const VALUES& x0, const double f_error, const bool verbose)
{
  gttic(M_error);
  const double M_error = Rd.error(VectorValues::Zero(dx_u));
  gttoc(M_error);

  // Result to return
  DoglegOptimizerImpl::IterationResult result;

  bool stay = true;
  enum { NONE, INCREASED_DELTA, DECREASED_DELTA } lastAction = NONE; // Used to prevent alternating between increasing and decreasing in one iteration
  while(stay) {
    gttic(Dog_leg_point);
    // Compute dog leg point
    result.dx_d = DoglegOptimizerImpl::ComputeDoglegPoint(delta, dx_u, dx_n, verbose);
    gttoc(Dog_leg_point);

    if(verbose) std::cout << "delta = " << delta << ", dx_d_norm = " << result.dx_d.norm() << std::endl;

    gttic(retract);
    // Compute expmapped solution
    const VALUES x_d(x0.retract(result.dx_d));
    gttoc(retract);

    gttic(decrease_in_f);
    // Compute decrease in f
    result.f_error = f.error(x_d);
    gttoc(decrease_in_f);

    gttic(new_M_error);
    // Compute decrease in M
    const double new_M_error = Rd.error(result.dx_d);
    gttoc(new_M_error);

    if(verbose) std::cout << std::setprecision(15) << "f error: " << f_error << " -> " << result.f_error << std::endl;
    if(verbose) std::cout << std::setprecision(15) << "M error: " << M_error << " -> " << new_M_error << std::endl;

    gttic(adjust_delta);
    // Compute gain ratio.  Here we take advantage of the invariant that the
    // Bayes' net error at zero is equal to the nonlinear error
    const double rho = std::abs(f_error - result.f_error) < 1e-15 || std::abs(M_error - new_M_error) < 1e-15 ?
        0.0 :
        (f_error - result.f_error) / (M_error - new_M_error);

    if(verbose) std::cout << std::setprecision(15) << "rho = " << rho << std::endl;

    // NEW CODE HERE
 
    if (rho == 0.0) {
      stay = false;
    }
    else if (rho > 0) {
      const double dx_d_norm = result.dx_d.norm();
      const double newDelta = std::min(2.0*delta, 1e5);
      if(mode == DoglegOptimizerImpl::ONE_STEP_PER_ITERATION || mode == DoglegOptimizerImpl::SEARCH_REDUCE_ONLY)
        stay = false;   // If not searching, just return with the new delta
      else if(mode == DoglegOptimizerImpl::SEARCH_EACH_ITERATION) {
        if( (dx_d_norm < delta) || lastAction == DECREASED_DELTA)
          stay = false; // Searching, but Newton's solution is within trust region so keep the same trust region
        else {
          stay = true;  // Searching and increased delta, so try again to increase delta
          lastAction = INCREASED_DELTA;
        }
      } else {
        assert(false); }

      delta = newDelta; // Update delta from new delta
    }
    else {
      assert(0.0 > rho);
      if(delta > 1e-7) {
        delta *= 0.5;
        stay = true;
        lastAction = DECREASED_DELTA;
      } else {
        if(verbose) std::cout << "Warning:  Dog leg stopping because cannot decrease error with minimum delta" << std::endl;
        result.dx_d.setZero(); // Set delta to zero - don't allow error to increase
        result.f_error = f_error;
        stay = false;
      }
    }

    // ORIGINAL CODE BELOW

  //   if(rho >= 0.4) {
  //     // M agrees very well with f, so try to increase lambda
  //     const double dx_d_norm = result.dx_d.norm();
  //     const double newDelta = std::max(delta, 3.0 * dx_d_norm); // Compute new delta

  //     if(mode == DoglegOptimizerImpl::ONE_STEP_PER_ITERATION || mode == DoglegOptimizerImpl::SEARCH_REDUCE_ONLY)
  //       stay = false;   // If not searching, just return with the new delta
  //     else if(mode == DoglegOptimizerImpl::SEARCH_EACH_ITERATION) {
  //       if(std::abs(newDelta - delta) < 1e-15 || lastAction == DECREASED_DELTA)
  //         stay = false; // Searching, but Newton's solution is within trust region so keep the same trust region
  //       else {
  //         stay = true;  // Searching and increased delta, so try again to increase delta
  //         lastAction = INCREASED_DELTA;
  //       }
  //     } else {
  //       assert(false); }

  //     delta = newDelta; // Update delta from new delta

  //   } else if(0.4 > rho && rho >= 0.15) {
  //     // M agrees so-so with f, keep the same delta
  //     stay = false;

  //   } else if(0.15 > rho && rho >= 0.0) {
  //     // M does not agree well with f, decrease delta until it does
  //     double newDelta;
  //     bool hitMinimumDelta;
  //     if(delta > 1e-5) {
  //       newDelta = 0.5 * delta;
  //       hitMinimumDelta = false;
  //     } else {
  //       newDelta = delta;
  //       hitMinimumDelta = true;
  //     }
  //     if(mode == DoglegOptimizerImpl::ONE_STEP_PER_ITERATION || /* mode == SEARCH_EACH_ITERATION && */ lastAction == INCREASED_DELTA || hitMinimumDelta)
  //       stay = false;   // If not searching, just return with the new smaller delta
  //     else if(mode == DoglegOptimizerImpl::SEARCH_EACH_ITERATION || mode == DoglegOptimizerImpl::SEARCH_REDUCE_ONLY) {
  //       stay = true;
  //       lastAction = DECREASED_DELTA;
  //     } else {
  //       assert(false); }

  //     delta = newDelta; // Update delta from new delta

  //   } else {
  //     // f actually increased, so keep decreasing delta until f does not decrease.
  //     // NOTE:  NaN and Inf solutions also will fall into this case, so that we
  //     // decrease delta if the solution becomes undetermined.
  //     assert(0.0 > rho);
  //     if(delta > 1e-5) {
  //       delta *= 0.5;
  //       stay = true;
  //       lastAction = DECREASED_DELTA;
  //     } else {
  //       if(verbose) std::cout << "Warning:  Dog leg stopping because cannot decrease error with minimum delta" << std::endl;
  //       result.dx_d.setZero(); // Set delta to zero - don't allow error to increase
  //       result.f_error = f_error;
  //       stay = false;
  //     }
  //   }
  //   gttoc(adjust_delta);
  }

  // dx_d and f_error have already been filled in during the loop
  result.delta = delta;
  return result;
}

} // end namespace DoglegOptimizerImplCustom

} // end namespace gtsam