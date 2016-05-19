/*
 * MapDataMissingException.h
 *
 *  Created on: May 19, 2016
 *      Author: andreas
 */

#ifndef SRC_MAPDATAMISSINGEXCEPTION_H_
#define SRC_MAPDATAMISSINGEXCEPTION_H_

#include <ros/exception.h>

class MapDataMissingException: public ros::Exception {
public:
	MapDataMissingException(const std::string& what)
  : ros::Exception(what)
  {}
};

#endif /* SRC_MAPDATAMISSINGEXCEPTION_H_ */
