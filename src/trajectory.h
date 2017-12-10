#include <vector>
#include "vehicle.h"
#include "spline.h"

using namespace std;

class trajectory
{
public:
trajectory();

void get_trajectory(std::vector<double>& next_x_vals, std::vector<double>& next_y_vals, vehicle our_car, int prev_size, vector<double> previous_path_x, vector<double> previous_path_y, int lane, vector<double> map_waypoints_s, vector<double> map_waypoints_x, vector<double> map_waypoints_y, double ref_vel);

private:
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);
};
