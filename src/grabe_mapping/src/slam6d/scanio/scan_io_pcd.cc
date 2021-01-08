/*
 * scan_io_pcd implementation
 *
 * Copyright (C) Andreas Nuechter, Dorit Borrmann
 *
 * Released under the GPL version 3.
 *
 */


/**
 * @file scan_io_pcd.cc
 * @brief IO of a 3D scan in pcd file format (right-handed coordinate system,
 * with x from left to right, y from bottom to up, z from front to back)
 * @author Hamidreza Houshiar. DenkmalDaten Winkler KG. Co. Muenster, Germany
 */

#include "scanio/scan_io_pcd.h"

const char* ScanIO_pcd::data_suffix) = ".pcd";
ScanDataTransform_pcd scanio_pcd_tf;
ScanDataTransform& ScanIO_pcd::transform2uos = scanio_pcd_tf;


/**
 * class factory for object construction
 *
 * @return Pointer to new object
 */
#ifdef _MSC_VER
extern "C" __declspec(dllexport) ScanIO* create()
#else
extern "C" ScanIO* create()
#endif
{
  return new ScanIO_pcd;
}


/**
 * class factory for object construction
 *
 * @return Pointer to new object
 */
#ifdef _MSC_VER
extern "C" __declspec(dllexport) void destroy(ScanIO *sio)
#else
extern "C" void destroy(ScanIO *sio)
#endif
{
  delete sio;
}

#ifdef _MSC_VER
BOOL APIENTRY DllMain(HANDLE hModule, DWORD dwReason, LPVOID lpReserved)
{
    return TRUE;
}
#endif

/* vim: set ts=4 sw=4 et: */