/*
 * Zhang-Project3.cc
 * Reinforcement Learning for Robot Obstacle Avoidance and Wall Following
 * Chi Zhang, czhang24@utk.edu
 */

#include <libplayerc++/playerc++.h>
#include <iostream>
#include <fstream>
#include <time.h>
#include <math.h>
#include "args.h"

using namespace std;
using namespace PlayerCc;

#define move_num 8		
#define state_num 5 
#define Q_table_size 3125	// 5 states for 5 elements: 5^5=3125
#define PI 3.141592

double gama = 0.8;
double lamda = 0.9;
double alpha = 0.6;
double epsilon = 0.3;

double speed = 0.3;
double min_dist = 0.5;
double target_min_dist = 0.6;
double target_max_dist = 0.8; 
double max_dist = 1.2; 

struct move{
	double speed;
	double turnrate;
};
move *mov;
	
double reward(double *);
int greedy(double *, double**, int);
void read_state(double *, double **);
void print_state(double *);
int index(double *);
void convert(double *s);
void print_cfg();

int main(int argc, char **argv){
	parse_args(argc,argv);
	srand((unsigned)time(NULL));
	print_cfg();
  	// throw exceptions on creation if fail
	try{
		PlayerClient robot(gHostname, gPort);
		Position2dProxy pp(&robot, gIndex);
		LaserProxy lp(&robot, gIndex);
		SimulationProxy simproxy(&robot, 0);	

		std::cout << robot << std::endl;

		pp.SetMotorEnable (true);
		
		// Define move, speed & turnrate
		mov = new move[move_num];
		for(int i = 0; i < move_num; i++)
			mov[i].speed = speed;
		mov[0].turnrate = 0;	//forward	
		mov[1].turnrate = 45;	//week left
		mov[2].turnrate = 90;	//left
		mov[3].turnrate = 180;	//strong left
		mov[4].turnrate = -180;	//strong right
		mov[5].turnrate = -90;	//right
		mov[6].turnrate = -45;	//week right
		
		mov[7].turnrate = 0;	//backward
		mov[7].speed = -speed;

		// Initialize Q, e
		double **Q = new double*[Q_table_size];
		double **e = new double*[Q_table_size];
		for(int i = 0; i < Q_table_size; i++){
			Q[i] = new double[move_num];
			e[i] = new double[move_num];
			for(int j = 0; j < move_num; j++){
				Q[i][j] = (double)rand()/RAND_MAX;
				e[i][j] = 0;
			}
		}
		//define s, a	
		double *s = new double[state_num];
		double *s_next = new double[state_num];
		double **reading = new double*[361];
		for(int i = 0; i < 361; i++){
			reading[i] = new double[2];
			for(int j = 0; j< 2; j++)
				reading[i][j] = 0;
		}
		
		int total_step = 0;
		if(!strcmp(argv[1], "train")){
			cout << "Training ..." << endl;
			bool learn = false;
			double accum_reward[200] = {0};
			for(int episode = 0; episode < 2000; episode++){
				if(learn == true){
					cout << "A good policy has been learned. Drag the robot." << endl;
					break;
				}
				// Initialize s, e
				for(int i = 0; i< state_num; i++){	
					s[i] = 0.0;
					s_next[i] = 0.0;
				}
				int a = 0;	
				int a_next = 0;

				// Reset to a random position and read s
				simproxy.SetPose2d((char *)"robotProj3", 4, 4, dtor(250));
				
				robot.Read();		
				for(unsigned int i = 0; i < lp.GetCount(); i++){
					reading[i][0] = lp[i];
					reading[i][1] = lp.GetBearing(i)/PI*180;
				}
				read_state(s, reading);
				convert(s);	
				
				// Repeat for each step of episode
				int step = 0;
				bool follow = false;
				bool trap = false;
				int trap_counter = 0;
				int follow_counter = 0;
				for(;;){
					step++;
					total_step++;
					// current state, used to check trapping
					double curr_pos[3];
					curr_pos[0]= pp.GetXPos();
					curr_pos[1]= pp.GetYPos();
					curr_pos[2] = pp.GetYaw();			

					// Take action a 
					pp.SetSpeed(mov[a].speed, dtor(mov[a].turnrate));

					// Observe s'
					robot.Read();				
					for(unsigned int i = 0; i < lp.GetCount(); i++){
						reading[i][0] = lp[i];
						reading[i][1] = lp.GetBearing(i)/PI*180;
					}
					read_state(s_next, reading);
					convert(s_next);
					
					// Observe r
					double r = reward(s_next);	
					accum_reward[total_step%200] = r;
					if(r >= 0){						// follow counter
						follow = true;
						++follow_counter;
					}
					else{
						follow = false;
						follow_counter = 0;
					}
					
					// Choose a' from s' using epsilon-greedy
					a_next = greedy(s_next, Q, total_step);

					double delta = r + gama * Q[index(s_next)][a_next] - Q[index(s)][a];
					e[index(s)][a] = e[index(s)][a] + 1;
					
					// Update Q, e
					for(int i = 0; i < Q_table_size; i++){
						for(int j = 0; j < move_num; j++){
							Q[i][j] = Q[i][j] + alpha * delta * e[i][j];
							e[i][j] = gama * lamda * e[i][j];
						}
					}
					
					// Update s<-s', a<-a'
					for(int i = 0; i < state_num; i++)
						s[i] = s_next[i];
					a = a_next;
			
					// check termination
					if(follow_counter > 3000){
						cout << "Training Success!" << endl;
						learn = true;
						break;
					}

					if (curr_pos[0] == pp.GetXPos() && curr_pos[1] == pp.GetYPos() && curr_pos[2] == pp.GetYaw()){
						trap = true;
						++trap_counter;
					}
					else{
						trap = false;
						trap_counter = 0;
					}
					if(trap_counter > 1){
						trap = false;
						trap_counter = 0;
						break;
					}
					double sum_reward = 0;
					for(int i = 0; i< 200; i++)
						sum_reward += accum_reward[i];
					cout <<"step: " << total_step << "\tsum_reward: " << sum_reward << endl;
				} //within one training episode	
			} //end of one episode 
			
			//write the trained policy to file	
			ofstream output("output.txt");
			for(int i = 0; i < Q_table_size; i++){
				for(int j = 0; j < move_num; j++)
					output << Q[i][j] <<"\t";
				output << endl;
			}
				
			cout << "Policy saved to file." <<endl;
		} //end of training mode	
		else if(!strcmp(argv[1], "test")){			// testing mode
			cout << "Testing ..." << endl;
			ifstream myfile("best.txt");			// reading trained policy from file
			if (myfile.is_open()){
				std::istringstream istr;
				int i = 0;
				while(myfile.good()){
					char dontcare;
					string line;
					getline(myfile, line);
					istr.clear();
					istr.str(line);
					istr >> Q[i][0] >> dontcare >> Q[i][1] >> dontcare >> Q[i][2] >> dontcare >> Q[i][3] >> dontcare >> Q[i][4] >> dontcare >> Q[i][5] >> dontcare >> Q[i][6] >> dontcare >> Q[i][7];
					if(i == Q_table_size-1)
						break;
					++i;
				}
			}
			else cout << "Unable to open file.";
			myfile.close();
			
			pp.SetMotorEnable (true);			// drive according to Q table
			simproxy.SetPose2d((char *)"robotProj3", 4, 5, dtor(90));
			int a = -1;
			while(1){
				++total_step;
				robot.Read();
				for(unsigned int i = 0; i < lp.GetCount(); i++){
					reading[i][0] = lp[i];
					reading[i][1] = lp.GetBearing(i)/PI*180;
				}
				read_state(s, reading);
				convert(s);
				a = greedy(s, Q, total_step + 450);
				pp.SetSpeed(mov[a].speed, dtor(mov[a].turnrate));
			}
		}
		else{
			cout << "Error mode. type training or testing.";
		}
		pp.SetSpeed(0,0);
		delete s;
		delete s_next;
		delete[] reading;
		for(int i = 0; i < Q_table_size; i++){
			delete [] Q[i];
			delete [] e[i];
		}
		delete [] Q;
		delete [] e;
		delete [] mov;
		
		cout << "Exit normally" <<endl;
		return 0;
	}
	catch (PlayerCc::PlayerError & e){
		std::cerr << e << std::endl;
		return -1;
	}
}


void read_state(double *s, double **reading){
	double xx[5];
	for(int i = 0; i < 5; i++)
		xx[i] = 8.0;

	for(int i = 0; i < 361; i++){
		if(reading[i][1] >= 30 && reading[i][1] <= 90){			//left
			if (reading[i][0] < xx[0])
				xx[0] = reading[i][0];
			if (reading[i][1] < 60)								//left-front
				if(reading[i][0] < xx[1])
					xx[1] = reading[i][0];
		}
		else if(reading[i][1] < 30 && reading[i][1] >= -30){	//front
			if (reading[i][0] < xx[2])
				xx[2] = reading[i][0];
		}
		else if(reading[i][1] >= -90.01 && reading[i][1] < -30){//right
			if (reading[i][0] < xx[3])
				xx[3] = reading[i][0];
		}
		else{
			cout << "Discard laser reading." << endl;
		}
	}
	// s[4] orientation: 0-nothing, 1-approaching, 2-parallel, 3-moving away
	int mmin = -1;
	int mmax = -1;
	for(int i = 0; i < 361; i++){
		if(reading[i][1] <= 90 && reading[i][1] >= 60)	//most left
			if(reading[i][0] < max_dist && reading[i][0] > min_dist){
				mmin = i;
				break;
			}
	}
	for(int i = 360; i >= 0; i--){
		if(reading[i][1] <= 60 && reading[i][1] >= 30)	//left-front
			if(reading[i][0] < max_dist && reading[i][0] > min_dist){
				mmax = i;
				break;
			}
	}
	if((mmin != -1) && (mmax != -1)){
		double r2 = reading[mmax][0];
		double r1 = reading[mmin][0];
		double ang2 = reading[mmax][1] + 90;
		double ang1 = reading[mmin][1] + 90;
		if (r1*cos(ang1)-r2*cos(ang2) == 0){
			xx[4] = 2;
		}
		else{
			//orientation: 0-nothing, 1-approaching, 2-parallel, 3-moving away
			double slope = atan((r1*sin(ang1/180*PI) - r2*sin(ang2/180*PI))/(r1*cos(ang1/180*PI)-r2*cos(ang2/180*PI))) / PI *180;
			if((slope > 92 && slope <= 180) || (slope <= 0 && slope > -88))
				if(r1 < r2)
					xx[4] = 3;
				else
					xx[4] = 1;
			else if(abs(slope - 90) <= 2 || abs(slope+90) <= 2)
				xx[4] = 2;
			else
				xx[4] = 1;
			}
	}
	else{
		xx[4] = 0;
	}
	for(int i = 0; i < 5; i++){
		s[i] = xx[i];
	}
}

void print_state(double *x){
	for(int i = 0; i  < state_num; i++)
		cout << x[i];
}

void convert(double *s){
	// left, 5 pieces
	if (s[0] < min_dist)
		s[0] = 0;
	else if(s[0] >= min_dist && s[0] < target_min_dist)
		s[0] = 1;
	else if(s[0] >= target_min_dist && s[0] < target_max_dist)
		s[0] = 2;
	else if(s[0] >= target_max_dist && s[0] < max_dist)
		s[0] = 3;
	else
		s[0] = 4;
	
	// left-front, 2 pieces
	if(s[1] < max_dist)
		s[1] = 1;
	else
		s[1] = 2;

	// front, 4 pieces
	if(s[2] < min_dist)
		s[2] = 0;
	else if(s[2] >= min_dist && s[2] < target_min_dist)
		s[2] = 1;
	else if(s[2] >= target_min_dist && s[2] < max_dist)
		s[2] = 2;
	else
		s[2] = 3;

	// right, 2 pieces
	if(s[3] < min_dist)
		s[3] = 0;
	else
		s[3] = 1;
}

double reward(double *s){
	for(int i = 0;i < state_num-1; i++){
		if(s[i] == 0){
			return -1.0;
		}
	}

		if (s[0] == 4)
			return -1.0;
		else 
			return 0.0;
	/*	else if (s[0] == 2)	
			return 1.0;
		else if (s[0] == 1)
			return 0.5;
		else if(s[0] == 3)
			return 0.5;
		else
			return 0;
*/
}

int index(double *s){
	int x = 0;
	for(int i = state_num -1; i >= 0; i--){
		x += s[i] * pow(5, i);
	}
	return x;
}

int greedy(double *s, double **Q, int step){
	int best = rand() % move_num;
	int non_best = rand() % move_num;
	double max = Q[index(s)][best];

	for(int i = 0; i < move_num; i++){
		if(Q[index(s)][i] > max){
			max = Q[index(s)][i];
			best = i;
		}
	}

	double temperature = 1;
	if(step>100 && step < 502)
		temperature = 1/(step-99)*2;
	if(step >= 502)
		temperature = 1/(step - 500);
		
	if((double)rand()/RAND_MAX > epsilon * temperature){
		return best;
	}
	else{
		non_best = rand() % move_num;
		while (non_best == best)
			non_best = rand()%move_num;
		return non_best;
	}
}

void print_cfg(){
	cout<< "speed:\t" << speed
		<< "epsilon:\t" << epsilon
		<< "gama:\t"  << gama
		<< "lamda:\t" << lamda
		<< "alpha:\t" << alpha
		<< "move_num:\t" << move_num
		<< "state_num:\t" << state_num 
		<< "min:\t" << min_dist
		<< "target_min_dist:\t" << target_min_dist
		<< "target_max_dist:\t" << target_max_dist 
		<< "max:\t" << max_dist << endl; 
}
