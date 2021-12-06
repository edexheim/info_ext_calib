#ifndef GTSAM_UTILS_H
#define GTSAM_UTILS_H

#include <gtsam/inference/LabeledSymbol.h>

// Symbol for working with poses in SE(3)
inline gtsam::Symbol TwvKey(uint64_t t) {
  return gtsam::Symbol('x', t); 
} 

inline gtsam::LabeledSymbol TwvKey(uint64_t t, unsigned char label) {
  return gtsam::LabeledSymbol('x', label, t);
}

// Symbol for working with extrinsic calibrations in SE(3)
inline gtsam::Symbol TvcKey(uint64_t t) {
  return gtsam::Symbol('c', t); 
}

// Symbol for intrinsic calibration
// inline gtsam::Symbol K(size_t t) {
//   return gtsam::Symbol('k', t); 
// }

// Symbol for landmarks
inline gtsam::Symbol lKey(uint64_t id) {
  return gtsam::Symbol('l', id);
}

// inline gtsam::LabeledSymbol lKey(uint64_t id, unsigned char label) {
//   return gtsam::LabeledSymbol('l', label, id);
// }

// Symbol for stereo landmarks
inline gtsam::Symbol lStereoKey(uint64_t id) {
  return gtsam::Symbol('s', id);
}

// inline gtsam::LabeledSymbol lStereoKey(uint64_t id, unsigned char label) {
//   return gtsam::LabeledSymbol('s', label, id);
// }


// Symbols for working with poses in SO(3) x R^3
inline gtsam::Symbol RwvKey(uint64_t t) {
  return gtsam::Symbol('a', t); 
}

inline gtsam::Symbol twvKey(uint64_t t) {
  return gtsam::Symbol('b', t); 
}

// Symbols for working with extrinsic calibrations in SO(3) x R^3
inline gtsam::Symbol RvcKey(uint64_t t) {
  return gtsam::Symbol('r', t); 
}

inline gtsam::Symbol tvcKey(uint64_t t) {
  return gtsam::Symbol('t', t); 
}


#endif