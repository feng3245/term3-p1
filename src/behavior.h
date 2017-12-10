#include <vector>
#include "vehicle.h"
using namespace std;

class behavior
{
public:
behavior();

vector<double> getBehavior(int prev_size, vehicle our_car, vector<vehicle> other_cars, double ref_vel, int lane);
};
