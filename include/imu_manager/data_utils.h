#ifndef _IMU_MANAGER_DATA_UTILS_
#define _IMU_MANAGER_DATA_UTILS_

template <class T>
double calculateMean(T data_buffer)
{
  double sum = std::accumulate(data_buffer.begin(), data_buffer.end(), 0.0);
  double mean = sum / data_buffer.size();
  return mean;
}

template <class T>
double calculateStdDev(T data_buffer)
{
  double sum = std::accumulate(data_buffer.begin(), data_buffer.end(), 0.0);
  double mean = sum / data_buffer.size();

  std::vector<double> diff(data_buffer.size());
  std::transform(data_buffer.begin(), data_buffer.end(), diff.begin(), [mean](double x) { return x - mean; });

  double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
  double stddev = std::sqrt(sq_sum / data_buffer.size());
  return stddev;
}

#endif  // _IMU_MANAGER_DATA_UTILS_
