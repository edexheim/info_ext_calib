#include "calibration/backend/SegmentId.h"

bool SegmentId::operator==(const SegmentId& other) const {
  return ( (sess_id_ == other.sess_id_)
      &&  (seq_id_  == other.seq_id_) );
}

bool SegmentId::operator!=(const SegmentId& other) const {
  return ( (sess_id_ != other.sess_id_)
      ||  (seq_id_  != other.seq_id_) );
}

bool SegmentId::operator<(const SegmentId& other) const {
  return ( (sess_id_ < other.sess_id_)
      ||  (sess_id_ == other.sess_id_ && seq_id_ < other.seq_id_) );
}

bool SegmentId::operator>(const SegmentId& other) const {
  return ( (sess_id_ > other.sess_id_)
      ||  (sess_id_ == other.sess_id_ && seq_id_ > other.seq_id_) );
}

bool SegmentId::operator<=(const SegmentId& other) const {
  return ( (*this) < other) || ( (*this) == other);
}

bool SegmentId::operator>=(const SegmentId& other) const {
  return ( (*this) > other) || ( (*this) == other);
}

bool SegmentId::isSequential(const SegmentId& other) const {
  return (sess_id_ == other.sess_id_ && seq_id_ == other.seq_id_+1)
      || (sess_id_ == other.sess_id_ && seq_id_+1 == other.seq_id_);
}

std::ostream& operator<<(std::ostream& os, const SegmentId& id) {
  os << "Session: " << static_cast<int>(id.sess_id_) << " Sequence: " << id.seq_id_;
  return os;
}