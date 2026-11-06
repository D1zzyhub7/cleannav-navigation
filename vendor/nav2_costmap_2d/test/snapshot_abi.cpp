#include <iostream>
#include "nav2_costmap_2d/costmap_2d_publisher.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/costmap_filters/costmap_filter.hpp"
int main(){using namespace nav2_costmap_2d;std::cout << sizeof(Layer) << " " << sizeof(CostmapLayer) << " " << sizeof(CostmapFilter) << " " << sizeof(LayeredCostmap) << " " << sizeof(Costmap2DROS) << " " << sizeof(Costmap2DPublisher) << std::endl;}
