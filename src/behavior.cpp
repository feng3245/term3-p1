#include "behavior.h"

behavior::behavior() {}

vector<double> behavior::getBehavior(int prev_size, vehicle our_car, vector<vehicle> other_cars, double ref_vel, int lane)
{

bool too_close = false;

//find ref_v to use
for(int i = 0; i < other_cars.size(); i++)
{
        //car is in my lane
        float d = other_cars[i].D;
        if(d < (2+4*lane+2) && d>(2+4*lane-2) )
        {
        double check_speed = other_cars[i].Speed;
        double check_car_s = other_cars[i].S;
	auto other_car = other_cars[i];
       
        other_car.S+=((double)prev_size*.02*check_speed);
                if((other_car.S > our_car.S) && ((other_car.S - our_car.S) < 30))
                {
                        //ref_vel = check_speed*2.23694;
                        too_close = true;
                        if(lane>0)
                        {
                        lane = 0;
                        }
                }
        }
}
if(too_close)
{
ref_vel -= .224;
}
else if(ref_vel < 49.5)
{
ref_vel += .224;
}
return {ref_vel, (double)lane};
}
