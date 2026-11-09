/* Host adapter ONLY. Compile the unchanged v17 bridge.c against simulated motors.
 * It validates the actual firmware parser/qualification rules, not STM32 timing. */
#include <string.h>
#include "bridge.h"
static char output[4096];
static unsigned short positions[2][4] = {{512,512,512,512},{512,512,512,512}};
static unsigned short goals[2][4];
static int enabled[2];
static int read_failure;
void BridgeOutput(const char *s) { strncat(output, s, sizeof(output)-strlen(output)-1); }
void BridgeSetArmLed(int arm,int moving) { (void)arm; (void)moving; }
const char *take_output(void) { return output; }
void clear_output(void) { output[0]=0; }
void fail_read(int value) { read_failure=value; }
int Ax12WriteGoals(int arm,const unsigned short *p) {
    memcpy(goals[arm],p,sizeof(goals[arm]));
    if(enabled[arm]) memcpy(positions[arm],p,sizeof(positions[arm]));
    return 1;
}
int Ax12WriteTorque(int arm,int value) {
    enabled[arm]=value;
    if(value) memcpy(positions[arm],goals[arm],sizeof(positions[arm]));
    return 1;
}
int Ax12ReadPositions(int arm,unsigned short *p,Ax12ReadError *e) {
    if(read_failure) { e->code="DXL_TIMEOUT";e->id=arm?12:17;e->motorError=0;return 0; }
    memcpy(p,positions[arm],sizeof(positions[arm]));return 1;
}
