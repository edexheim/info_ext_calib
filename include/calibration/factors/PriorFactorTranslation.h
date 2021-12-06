#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>

namespace gtsam {

  class PriorFactorTranslation : public NoiseModelFactor1<Pose3> {

  private:

    typedef NoiseModelFactor1<Pose3> Base;

    Point3 prior_; /** The measurement */

  public:

    /// shorthand for a smart pointer to a factor
    typedef typename boost::shared_ptr<PriorFactorTranslation> shared_ptr;

    /// Typedef to this class
    typedef PriorFactorTranslation This;

    /** default constructor - only use for serialization */
    PriorFactorTranslation() {}

    virtual ~PriorFactorTranslation() {}

    /** Constructor */
    PriorFactorTranslation(Key key, const Point3& prior, const SharedNoiseModel& model = nullptr) :
      Base(model, key), prior_(prior) {
    }

    /** Convenience constructor that takes a full covariance argument */
    PriorFactorTranslation(Key key, const Point3& prior, const Matrix& covariance) :
      Base(noiseModel::Gaussian::Covariance(covariance), key), prior_(prior) {
    }

    /// @return a deep copy of this factor
    gtsam::NonlinearFactor::shared_ptr clone() const override {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
          gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

    /** implement functions needed for Testable */

    /** print */
    void print(const std::string& s,
       const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
      std::cout << s << "PriorFactorExtrinsics on " << keyFormatter(this->key()) << "\n";
      traits<Point3>::Print(prior_, "  prior mean: ");
      if (this->noiseModel_)
        this->noiseModel_->print("  noise model: ");
      else
        std::cout << "no noise model" << std::endl;
    }

    /** equals */
    bool equals(const NonlinearFactor& expected, double tol=1e-9) const override {
      const This* e = dynamic_cast<const This*> (&expected);
      return e != nullptr && Base::equals(*e, tol) && traits<Point3>::Equals(prior_, e->prior_, tol);
    }

    /** implement functions needed to derive from Factor */

    /** vector of errors */
    Vector evaluateError(const Pose3& x,
       boost::optional<Matrix&> H = boost::none) const override {
      // if (H) (*H) = Matrix::Identity(traits<T>::GetDimension(x),traits<T>::GetDimension(x));
      // manifold equivalent of z-x -> Local(x,z)
      // TODO(ASL) Add Jacobians.
      // return -traits<T>::Local(x, prior_, H);

      // x^{-1} * prior_
      Point3 t = x.translation(H);
      Matrix3 H_tmp;
      Vector v = traits<Point3>::Local(t, prior_, H ? &H_tmp : 0);
      if (H) *H = H_tmp * (*H);
      return v;
    }

    const Point3& prior() const { return prior_; }

  private:

    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
      ar & boost::serialization::make_nvp("NoiseModelFactor1",
          boost::serialization::base_object<Base>(*this));
      ar & BOOST_SERIALIZATION_NVP(prior_);
    }

	// Alignment, see https://eigen.tuxfamily.org/dox/group__TopicStructHavingEigenMembers.html
	enum { NeedsToAlign = (sizeof(Point3) % 16) == 0 };
  public:
	GTSAM_MAKE_ALIGNED_OPERATOR_NEW_IF(NeedsToAlign)
  };

  /// traits
  // struct traits<PriorFactorTranslation> : public Testable<PriorFactorTranslation> {};


} /// namespace gtsam