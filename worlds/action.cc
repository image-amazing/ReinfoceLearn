#include "rl.h"

void action::initial(){
	action *act = new action[6];     
	act[0]->direction = "forward";
    act[1]->direction = "backward";
    act[2]->direction = "right";
    act[3]->direction = "half_right";
    act[4]->direction = "left";
    act[5]->direction = "half_left";
    
	for(int i = 0; i < 6; i++){
		act[i].speed = 1;
		act[i].turnrate = 0;
	}
}

void action::take_action(char *direction){
	

}
