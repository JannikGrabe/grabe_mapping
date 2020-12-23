/**
 * @file scan_io_pcd.h
 * @brief IO of a 3D scan in pcd file format
 * @author Hamidreza Houshiar. DenkmalDaten Winkler KG. Co. Muenster, Germany
 */

#ifndef __SCAN_IO_PCD_H__
#define __SCAN_IO_PCD_H__

#include "scan_io.h"

/**
 * @brief IO of a 3D scan in pcd file format
 *
 * The compiled class is available as shared object file
 */
class ScanIO_pcd : public ScanIO {
protected:
  static const char* data_suffix;
  static ScanDataTransform& transform2uos;

  virtual const char* dataSuffix() { return data_suffix; }
  virtual ScanDataTransform& getTransform() { return transform2uos; }
};

#endif