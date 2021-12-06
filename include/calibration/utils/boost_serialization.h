#ifndef BOOST_SERIALIZATION_H
#define BOOST_SERIALIZATION_H

#include <boost/serialization/binary_object.hpp>

namespace boost {
namespace serialization {

template<class Archive, class F, class S>
inline void serialize(
    Archive & ar,
    boost::tuple<F, S>& t,
    const unsigned int /* file_version */) {
  ar & boost::serialization::make_nvp("first" , t.template get<0>() );
  ar & boost::serialization::make_nvp("second", t.template get<1>() );
}

template<class Archive, class F, class S, class T>
inline void serialize(
    Archive & ar,
    boost::tuple<F, S, T>& t,
    const unsigned int /* file_version */) {
  ar & boost::serialization::make_nvp("first" , t.template get<0>() );
  ar & boost::serialization::make_nvp("second", t.template get<1>() );
  ar & boost::serialization::make_nvp("third" , t.template get<2>() );
}

} // serialization
} // namespace boost 

#endif