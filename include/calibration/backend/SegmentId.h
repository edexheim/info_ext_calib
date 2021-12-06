#ifndef SEGMENT_ID_H
#define SEGMENT_ID_H

#include <cstddef>
#include <functional>
#include <iostream>
#include <boost/serialization/nvp.hpp>


struct SegmentId {

  SegmentId()
    : sess_id_(0), seq_id_(0) {}

  SegmentId(size_t sess_id, size_t seq_id)
    : sess_id_(sess_id), seq_id_(seq_id) {}

  bool operator==(const SegmentId& other) const;

  bool operator!=(const SegmentId& other) const;
  
  bool operator<(const SegmentId& other) const;

  bool operator>(const SegmentId& other) const;

  bool operator<=(const SegmentId& other) const;

  bool operator>=(const SegmentId& other) const;

  bool isSequential(const SegmentId& seg_id) const;

  friend std::ostream& operator<<(std::ostream& os, const SegmentId& id);
  
  uint8_t sess_id_;
  size_t seq_id_;

  private:
    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
      ar & BOOST_SERIALIZATION_NVP(sess_id_);
      ar & BOOST_SERIALIZATION_NVP(seq_id_);
  }
};

#endif