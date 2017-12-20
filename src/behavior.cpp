#include "behavior.h"
#include <chrono>

using namespace std::chrono;

behavior::behavior() {}
//slowest vehcile ahead is what matters
double lastlaneswitch;
double timesincestart = 0;
double behavior::lanecost(double vel_veh_ahead)
{
//We assume lane cost is the speed going for the lane right now for simpllcity sake
if(vel_veh_ahead>=49.5/2.23694)
return 0;
return 0.8 *((49.5/2.23694-vel_veh_ahead)/(49.5/2.23694));
}
vector<double> behavior::getBehavior(int prev_size, vehicle our_car, vector<vehicle> other_cars, double ref_vel, int lane)
{

bool too_close = false;
double follow_speed = 49.5;
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
                if((other_car.S > our_car.S)&&(other_car.S - our_car.S)<45)
                {
			//Set it to the slowest car in the vacinity not just the first car within 45s
			if(check_speed*2.23694<follow_speed)
			{
                        	follow_speed = check_speed*2.23694;
			}
                        too_close = true;

                }
        }
}
double costoflane = 1;
int bestlane = 0;
for(int intendedlane = 0; intendedlane < 3; intendedlane++)
{
	
double slowestVehicle = 49.5/2.23694;
for(int i = 0; i < other_cars.size(); i++)
{
	float d = other_cars[i].D;		
	double check_speed = other_cars[i].Speed;
        double check_car_s = other_cars[i].S;
        auto other_car = other_cars[i];
	double othercars =  other_car.S+((double)prev_size*.02*check_speed);
	if(othercars > our_car.S)
	{
		if(d < (2 +4*intendedlane+ 2)&& d > (2 + 4*intendedlane-2))
		{
			if( check_speed<slowestVehicle)
			slowestVehicle = check_speed; 
		}
	}
}
double lanecosttemp = lanecost(slowestVehicle);
if(lanecosttemp < costoflane)
{
	costoflane = lanecosttemp;
	bestlane = intendedlane;
}
}

int intendedlane = lane;
//switch lane at most once every second
if(duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count()-lastlaneswitch>1000)
if(bestlane>intendedlane)
{
lastlaneswitch = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
intendedlane++;
}	
else if(bestlane<intendedlane)
{
lastlaneswitch = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
intendedlane--;
}		
//for(int intendedlane = 0; intendedlane < 2; intendedlane++)
//{

//if(intendedlane == lane )
//continue;

bool is_safe = true;
if(timesincestart == 0)
{
timesincestart = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();

}
//You need 15 seconds ramp up time
if(duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count()-timesincestart<15000)
is_safe = false;
else
for(int i = 0; i < other_cars.size(); i++)
{
	float d = other_cars[i].D;
	if(d < (2 +4*intendedlane+ 2)&& d > (2 + 4*intendedlane-2))
	{
		double check_speed = other_cars[i].Speed;
        	double check_car_s = other_cars[i].S;
        	auto other_car = other_cars[i];
		double othercars =  other_car.S+((double)prev_size*.02*check_speed);

		if( ((othercars <= our_car.S) && ((our_car.S - othercars) <
 20)&&(check_speed>our_car.Speed||(check_speed<our_car.Speed&&((our_car.S - othercars)<5))) ) || ((othercars >= our_car.S) && ((othercars - our_car.S) <
 20) && (check_speed<our_car.Speed||(check_speed>our_car.Speed&&((othercars-our_car.S)<5)))) )
		{
			is_safe = false;						
		}
	}
}
if(is_safe)
{
lane = intendedlane;
}

if(too_close&&ref_vel>follow_speed)
{
ref_vel -= .30;
}
else if(ref_vel < follow_speed)
{
ref_vel += .30;
}
return {ref_vel, (double)lane};
}
