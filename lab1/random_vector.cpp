#include "random_vector.h"
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <numeric>
#include <random>
#include <cstdlib>
// TODO: add any include you might require

RandomVector::RandomVector(int size, double max_val) { 
  // TODO: Write your code here
  for(int i = 0; i < size; i++){
    double random_value = static_cast<double>(std::rand()) / RAND_MAX * max_val;
    values.push_back(random_value);
  }
}

void RandomVector::print(){
  // TODO: Write your code here
  for(const auto &value : values){
    std::cout << std::fixed << std::setprecision(6) << value << " ";
  }
  std::cout << std::endl;
}

double RandomVector::mean(){
  // TODO: Write your code here
  double sum = std::accumulate(values.begin(), values.end(), 0.0);
  return sum / values.size();
}

double RandomVector::max(){
  // TODO: Write your code here
  return *std::max_element(values.begin(), values.end());
}

double RandomVector::min(){
  //TODO:  Write your code here
  return *std::min_element(values.begin(), values.end());
}

void RandomVector::printHistogram(int bins){
  // TODO: Write your code here
  if(bins <= 0){
    std::cout << "Invalid number of bins" << std::endl;
    return;
  }
  double min_val = min();
  double max_val = max();
  double bin_width = (max_val - min_val) / bins;
  double range = max_val - min_val;
  std::vector<int> histogram(bins, 0);

  for(const auto &value : values){
    int bin_index = static_cast<int>((value - min_val) / range * bins);
    if(bin_index == bins){
      bin_index--;
    }
    histogram[bin_index]++;
  }

  for(int i = 0; i < bins; i++){
    double lower_bound = min_val + i * bin_width;
    double upper_bound = min_val + (i + 1) * bin_width;
    std::cout << std::fixed << std::setprecision(2) 
                  << "[" << lower_bound << " - " << upper_bound << "]: ";
        std::cout << std::string(histogram[i], '*') << std::endl;
  }
}
