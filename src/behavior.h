#include <vector>
#include "vehicle.h"
using namespace std;

class behavior
{

public:
behavior();

double lanecost(double vel_veh_ahead);
vector<double> getBehavior(int prev_size, vehicle our_car, vector<vehicle> other_cars, double ref_vel, int lane);
};
