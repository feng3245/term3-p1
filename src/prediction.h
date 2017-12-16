#include "classifier.h"

using namespace std;
class prediction
{
public:
prediction(classifier c);
//predict the lane of the vehicle
int predict(vehicle other_car, double previous_x, double previous_y);



};
