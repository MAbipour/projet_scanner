#ifndef PI_REGULATOR_H
#define PI_REGULATOR_H


//start the PI regulator thread
void pi_regulator_start(void);
void activate_pi_regulator(void);
#define GOAL_DISTANCE 45	//distance en mm

#endif /* PI_REGULATOR_H */