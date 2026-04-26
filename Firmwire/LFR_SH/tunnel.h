#ifndef TUNNEL_H
#define TUNNEL_H




#define NUM_SONARS 3
extern const int sonarPins[NUM_SONARS];
extern volatile float distances[NUM_SONARS];
extern volatile bool readingReady;
extern volatile unsigned long echoStartTime;
extern int maxDistance;
extern int currentSensor;
extern unsigned long maxEchoDuration;



extern int wall_limit;
extern int obstacle_limit;

extern int turn90_delay;
// extern int errorWall;
// extern int lastErrorWall;
// extern bool forwardTurn;

void echoISR();
void updateSonarReading();
void sonarInit();
void wall_follow_1();


















#endif