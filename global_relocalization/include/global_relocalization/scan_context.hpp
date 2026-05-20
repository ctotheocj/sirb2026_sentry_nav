// Copyright 2026 Lihan Chen
//
// Licensed under the Apache License, Version 2.0

#ifndef GLOBAL_RELOCALIZATION__SCAN_CONTEXT_HPP_
#define GLOBAL_RELOCALIZATION__SCAN_CONTEXT_HPP_

#include <vector>

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

namespace global_relocalization
{

class ScanContextDescriptor
{
public:
  ScanContextDescriptor() = default;
  ScanContextDescriptor(int rings, int sectors, double max_range);

  void reset(int rings, int sectors, double max_range);
  bool build(const pcl::PointCloud<pcl::PointXYZ> & cloud);
  double similarityTo(const ScanContextDescriptor & other, int * best_shift = nullptr) const;
  bool empty() const {return data_.empty();}

  int rings() const {return rings_;}
  int sectors() const {return sectors_;}

private:
  double cosineForShift(const ScanContextDescriptor & other, int shift) const;

  int rings_{0};
  int sectors_{0};
  double max_range_{0.0};
  std::vector<float> data_;
};

}  // namespace global_relocalization

#endif  // GLOBAL_RELOCALIZATION__SCAN_CONTEXT_HPP_
