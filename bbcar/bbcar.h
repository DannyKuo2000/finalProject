#ifndef BBCAR_H
#define BBCAR_H
#include "parallax_servo.h"
#include "parallax_ping.h"
#include "parallax_laserping.h"
#include "parallax_qti.h"

class BBCar{
	public:
        BBCar( PwmOut &pinc_servo0, PwmIn &pinf_servo0, PwmOut &pinc_servo1, 
                PwmIn &pinf_servo1, Ticker &servo_control_ticker, Ticker &servo_feedback_ticker );
		parallax_servo servo0;
		parallax_servo servo1;

		void controlWheel();
		void stop();
		void goStraight( double speed );
        void activeWeak( double speed); // edited

		// turn left/right with a factor of speed
		//void turn( double speed, double factor );
        void turn(double speed0, double speed1);
        void turnByStanding( double speed, double factor ); // edited

		// limit the value by max and min
		float clamp( float value, float max, float min );
		int turn2speed( float turn );
        
        //feedbackservo
        void feedbackWheel();
        void goCertainDistance(float distance);
		int checkDistance(int errorDistance_Range);
		
};

#endif
