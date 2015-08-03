#ifndef LEPP2_MODELS_OBJECT_MODEL_H__
#define LEPP2_MODELS_OBJECT_MODEL_H__

#include "lepp2/models/Coordinate.h"

namespace lepp {

// Forward declarations.
class ModelVisitor;
class SphereModel;
class CapsuleModel;

/**
 * The base class for all geometrical models that can be used to represent
 * objects.
 */
class ObjectModel {
public:
  ObjectModel() : id_(0) {}

  virtual void accept(ModelVisitor& visitor) = 0;
  virtual Coordinate center_point() const = 0;

  virtual ~ObjectModel() {}
  /**
   * Returns the ID associated with the object. If the ID is 0, it means that
   * no meaningful ID was associated.
   *
   * Therefore, 0 is the default value returned, unless `set_id` is called.
   */
  int id() const { return id_; }
  /**
   * Sets the object's ID.
   */
  void set_id(int id) { id_ = id; }

  friend std::ostream& operator<<(std::ostream& out, ObjectModel const& model);
private:
  int id_;
};

typedef boost::shared_ptr<ObjectModel> ObjectModelPtr;

class ModelVisitor {
public:
  virtual void visitSphere(SphereModel& sphere) = 0;
  virtual void visitCapsule(CapsuleModel& capsule) = 0;
  virtual ~ModelVisitor() {}
};

/**
 * Model class representing a sphere.
 */
class SphereModel : public ObjectModel {
public:
  SphereModel(double radius, Coordinate const& center);
  /**
   * Returns a model-specific representation of its coefficients packed into
   * an std::vector.
   */
  void accept(ModelVisitor& visitor) { visitor.visitSphere(*this); }
  Coordinate center_point() const { return center_; }

  double radius() const { return radius_; }
  Coordinate const& center() const { return center_; }
  void set_radius(double radius) { radius_ = radius; }
  void set_center(Coordinate const& center) { center_ = center; }

  friend std::ostream& operator<<(std::ostream& out, SphereModel const& sphere);
private:
  double radius_;
  Coordinate center_;
};

inline SphereModel::SphereModel(double radius, Coordinate const& center)
    : radius_(radius), center_(center) {}

inline std::ostream& operator<<(std::ostream& out, SphereModel const& sphere) {
  out << "[sphere; "
      << "radius = " << sphere.radius_ << "; "
      << "center = " << sphere.center_ << "]";

  return out;
}

/**
 * Model class that represents a capsule: a cylinder with two spheres at either
 * end of it.
 */
class CapsuleModel : public ObjectModel {
public:
  /**
   * Creates a new capsule with the given radius and two points representing the
   * centers of the two spheres at the ends of the capsule.
   */
  CapsuleModel(double radius, Coordinate const& first, Coordinate const& second)
      : radius_(radius), first_(first), second_(second) {}

  void accept(ModelVisitor& visitor) { visitor.visitCapsule(*this); }
  Coordinate center_point() const { return (second_ + first_) / 2; }

  double radius() const { return radius_; }
  Coordinate const& first() const { return first_; }
  Coordinate const& second() const { return second_; }

  void set_radius(double radius) { radius_ = radius; }
  void set_first(Coordinate const& first) { first_ = first; }
  void set_second(Coordinate const& second) { second_ = second; }

  friend std::ostream& operator<<(std::ostream& out, CapsuleModel const& caps);
private:
  double radius_;
  Coordinate first_;
  Coordinate second_;
};

inline std::ostream& operator<<(std::ostream& out, CapsuleModel const& capsule) {
  out << "[capsule; "
      << "radius = " << capsule.radius_ << "; "
      << "first = " << capsule.first_ << "; "
      << "second = " << capsule.second_ << "]";

  return out;
}

/**
 * A model implementation that can be composed of multiple basic models.
 */
class CompositeModel : public ObjectModel {
public:
  CompositeModel() {}
  explicit CompositeModel(std::vector<boost::shared_ptr<ObjectModel> > const& models)
      : models_(models) {}

  /**
   * Adds a new model to the composition. The model instance, which the given
   * pointer references, is not copied, but the composite will also contain a
   * pointer to the given element.
   */
  void addModel(boost::shared_ptr<ObjectModel> model) {
    models_.push_back(model);
  }

  /**
   * Add any model instance to the composite.
   * The given instance is copied (by invoking the copy constructor) before
   * storing in the composite.
   */
  template<class M>
  void addModel(M const& model) {
    models_.push_back(boost::shared_ptr<M>(new M(model)));
  }

  void set_models(std::vector<boost::shared_ptr<ObjectModel> > const& models) { models_ = models; }
  std::vector<boost::shared_ptr<ObjectModel> > const& models() { return models_; }

  void accept(ModelVisitor& visitor) {
    // We visit each individual part of the composite.
    size_t const sz = models_.size();
    for (size_t i = 0; i < sz; ++i) {
      models_[i]->accept(visitor);
    }
  }

  Coordinate center_point() const {
    // For now, no complex calculation to find the center of the entire
    // composition is made; rather, the center point of the first model is
    // returned.
    Coordinate center(0, 0, 0);
    if (models_.size() == 0) return center;

    for (size_t i = 0; i < models_.size(); ++i) {
      center = center + models_[i]->center_point();
    }
    center = center / models_.size();

    return center;
  }

private:
  /**
   * A list of all models that the composite is composed of.
   */
  std::vector<boost::shared_ptr<ObjectModel> > models_;
};

/**
 * A visitor implementation that outputs a textual representation of each model
 * instance to a given `std::ostream`.
 *
 * It requires that each `ObjectModel` implementation implements the
 * `operator<<` for an `std::ostream`.
 */
class PrintVisitor : public ModelVisitor {
public:
  /**
   * Create a new `PrintVisitor` that will output in the given `std::ostream`.
   */
  PrintVisitor(std::ostream& out) : out_(out) {}

  void visitSphere(SphereModel& sphere) { out_ << sphere; }
  void visitCapsule(CapsuleModel& capsule) { out_ << capsule; }
private:
  std::ostream& out_;
};

inline std::ostream& operator<<(std::ostream& out, ObjectModel const& model) {
  PrintVisitor printer(out);
  const_cast<ObjectModel&>(model).accept(printer);

  return out;
}

/**
 * A `ModelVisitor` implementation that can flatten out a given model into its
 * most primitive parts. Regardless how many composite model "levels" there are,
 * in the end it will yield only the primitive ones.
 *
 * After a model that should be flattened out has accepted an instance of this
 * visitor, the list of primitives can be obtained by using the `objs` accessor.
 *
 * Note: When using this visitor, extreme care must be taken not to let the list
 *       of primitives (or any of the pointers contained therein) outlive the
 *       actual model instance that it was obtained from, since then the
 *       pointers would be left dangling. In other words, the visitor does not
 *       in any way facilitate prolonging the lifetime of the primitive parts
 *       found in the model that it visits.
 */
class FlattenVisitor : public ModelVisitor {
public:
  void visitSphere(SphereModel& sphere) { objs_.push_back(&sphere); }
  void visitCapsule(CapsuleModel& capsule) { objs_.push_back(&capsule); }
  std::vector<ObjectModel*> const& objs() const { return objs_; }
private:
  std::vector<ObjectModel*> objs_;
};

}  // namespace lepp
#endif
