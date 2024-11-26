#ifndef LINE_EXTRACTION_H
#define LINE_EXTRACTION_H

#include <cmath>
#include <vector>
#include <boost/array.hpp>
#include <Eigen/Dense>
#include "laser_line_extraction/utilities.h"
#include "laser_line_extraction/line.h"
#include <dynamic_reconfigure/server.h>
#include <laser_line_extraction/ParameterConfig.h>

namespace line_extraction
{

class LineExtraction
{

typedef laser_line_extraction::ParameterConfig Config;
typedef dynamic_reconfigure::Server<Config> ParamterConfigServer;
typedef dynamic_reconfigure::Server<Config>::CallbackType CallbackType;

public:
  // Constructor / destructor
  LineExtraction();
  ~LineExtraction();
  // Run
  void extractLines(std::vector<Line>&);
  // Data setting
  void setCachedData(const std::vector<double>&, const std::vector<double>&,
                     const std::vector<double>&, const std::vector<unsigned int>&);
  void setRangeData(const std::vector<double>&);
  // Parameter setting
  void setBearingVariance(double);
  void setRangeVariance(double);
  void setLeastSqAngleThresh(double);
  void setLeastSqRadiusThresh(double);
  void setMaxLineGap(double);
  void setMinLineLength(double);
  void setMinLinePoints(unsigned int);
  void setMinRange(double);
  void setMaxRange(double);
  void setMinSplitDist(double);
  void setOutlierDist(double);

private:
  // Data structures
  CachedData c_data_;
  RangeData r_data_;
  Params params_;
  // Indices after filtering
  std::vector<unsigned int> filtered_indices_;
  // Line data
  std::vector<Line> lines_;
  // Methods
  double chiSquared(const Eigen::Vector2d&, const Eigen::Matrix2d&,
                    const Eigen::Matrix2d&);
  double distBetweenPoints(unsigned int index_1, unsigned int index_2);
  void   filterCloseAndFarPoints();
  void   filterOutlierPoints();
  void   filterLines();
  void   mergeLines();
  void   split(const std::vector<unsigned int>&);
  void reconfigureCB(Config& config, uint32_t level);

private:
  double max_line_gap_, min_line_length_, min_range_, max_range_, min_split_dist_, outlier_dist_;
  unsigned int min_line_points;
  ParamterConfigServer* dynamic_srv_;
};

} // namespace line_extraction

#endif
