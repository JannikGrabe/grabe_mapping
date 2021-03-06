/**
 * @file scan_io_pts.h
 * @brief IO of a 3D scan in pts file format (right-handed coordinate system,
 * with x from left to right, y from bottom to up, z from front to back)
 * @author Hamidreza Houshiar. DenkmalDaten Winkler KG. Co. Muenster, Germany
 */

#ifndef __SCAN_IO_PTS_RRGB_H__
#define __SCAN_IO_PTS_RRGB_H__

#include "scan_io.h"

/**
 * @brief IO of a 3D scan in pts file format
 *
 * The compiled class is available as shared object file
 */
class ScanIO_pts_rrgb : public ScanIO {
protected:
  static const char* data_suffix;
  static IODataType spec[];
  static ScanDataTransform& transform2uos;

  virtual const char* dataSuffix() { return data_suffix; }
  virtual IODataType* getSpec() { return spec; }
  virtual ScanDataTransform& getTransform() { return transform2uos; }
};

#endif
