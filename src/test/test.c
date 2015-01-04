#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>

#define MRBFS_PATH  "/mrbfs/bus0/0xC0-mrb-iiab"
#define MRBFS_TXPKT "/mrbfs/bus0/txPacket"

#define RUN_TEST(fn)                                                                               \
	fn;                                                                                            
/*	if(fail)                                                                                       \
	{                                                                                              \
		if(system("cp /mrbfs/interfaces/ci2/pktLog pktlog.$(date +%Y%m%d-%H%M%S)") == -1)          \
			printf("system command failed (pktlog)\n");                                            \
		if(system("cp /home/data/design/mrbfs/mrbfs.log mrbfs.$(date +%Y%m%d-%H%M%S)") == -1)      \
			printf("system command failed (mrbfs.log)\n");                                         \
		exit(0);                                                                                   \
	}                                                                                              \
*/

#define START_TEST                                         \
	struct timeval tvBeginTest, tvEndTest, tvDiffTest;     \
	gettimeofday(&tvBeginTest, NULL);                      \

#define END_TEST                                                                            \
	gettimeofday(&tvEndTest, NULL);                                                         \
	printf("\nPass: %d\nFail: %d\n\n", pass, fail);                                         \
	timeval_subtract(&tvDiffTest, &tvEndTest, &tvBeginTest);                                \
	printf("Elapsed Time: %ld:%02ld\n\n", tvDiffTest.tv_sec / 60, tvDiffTest.tv_sec % 60);  \


#define SRC_ADDR 0xFD
#define DUT_ADDR 0xC0

#define RESET           0
#define BRIGHT          1
#define DIM             2
#define UNDERLINE       3
#define BLINK           4
#define REVERSE         7
#define HIDDEN          8

#define BLACK           0
#define RED             1
#define GREEN           2
#define YELLOW          3
#define BLUE            4
#define MAGENTA         5
#define CYAN            6
#define WHITE           7

int pass, fail;

#define MRBUS_EE_DEVICE_ADDR         0
#define MRBUS_EE_DEVICE_OPT_FLAGS    1
#define MRBUS_EE_DEVICE_UPDATE_H     2
#define MRBUS_EE_DEVICE_UPDATE_L     3

#define MRBUS_EE_DEVICE_UPDATE_H 2
#define MRBUS_EE_DEVICE_UPDATE_L 3

#define EE_TIMEOUT_SECONDS      0x10
#define EE_LOCKOUT_SECONDS      0x14
#define EE_TIMELOCK_SECONDS     0x15
#define EE_DEBOUNCE_SECONDS     0x16
#define EE_CLOCK_SOURCE_ADDRESS 0x18
#define EE_MAX_DEAD_RECKONING   0x19
#define EE_SIM_TRAIN_WINDOW     0x1A
#define EE_BLINKY_DECISECS      0x1F
#define EE_INPUT_POLARITY0      0x20
#define EE_INPUT_POLARITY1      0x21
#define EE_OUTPUT_POLARITY0     0x22
#define EE_OUTPUT_POLARITY1     0x23
#define EE_OUTPUT_POLARITY2     0x24
#define EE_OUTPUT_POLARITY3     0x25
#define EE_OUTPUT_POLARITY4     0x26
#define EE_MISC_CONFIG          0x30
#define EE_SIM_TRAINS           0x40

#define SIM_TRAIN_FLAGS_ENABLE          0x80
#define SIM_TRAIN_FLAGS_TRIGGER         0x40
#define SIM_TRAIN_FLAGS_INTERCHANGE     0x04
#define SIM_TRAIN_FLAGS_SOUND           0x03
#define SIM_TRAIN_DIRECTION_BITMASK     0x03


// Time variables
#define TIME_FLAGS_DISP_FAST       0x01
#define TIME_FLAGS_DISP_FAST_HOLD  0x02
#define TIME_FLAGS_DISP_REAL_AMPM  0x04
#define TIME_FLAGS_DISP_FAST_AMPM  0x08


typedef enum
{
	STATE_IDLE      = 0,
	STATE_CLEARANCE = 1,
	STATE_TIMEOUT   = 2,
	STATE_TIMER     = 3,
	STATE_OCCUPIED  = 4,
	STATE_LOCKOUT   = 5,
	STATE_CLEAR     = 6,
} InterlockState;


#define DIR_WEST  0
#define DIR_EAST  1
#define DIR_NORTH 2
#define DIR_SOUTH 3

// Aspect Definitions
#define ASPECT_LUNAR     0x07
#define ASPECT_FL_RED    0x06
#define ASPECT_FL_GREEN  0x05
#define ASPECT_RED       0x04
#define ASPECT_FL_YELLOW 0x03
#define ASPECT_YELLOW    0x02
#define ASPECT_GREEN     0x01
#define ASPECT_OFF       0x00

const char *aspectString[8];

#define OCC_WEST_MAIN    0x01
#define OCC_WEST_SIDING  0x02
#define OCC_EAST_MAIN    0x04
#define OCC_NORTH_MAIN   0x08
#define OCC_NORTH_SIDING 0x10
#define OCC_SOUTH_MAIN   0x20
#define OCC_INTERLOCKING 0x40

#define SIGNAL_EB_MAIN    0
#define SIGNAL_EB_SIDING  1
#define SIGNAL_WB_TOP     2
#define SIGNAL_WB_BOTTOM  3
#define SIGNAL_SB_MAIN    4
#define SIGNAL_SB_SIDING  5
#define SIGNAL_NB_TOP     6
#define SIGNAL_NB_BOTTOM  7

#define TURNOUT_WEST  0x01
#define TURNOUT_EAST  0x02
#define TURNOUT_NORTH 0x04
#define TURNOUT_SOUTH 0x08

#define   STATE_IDLE        0
#define   STATE_CLEARANCE   1
#define   STATE_TIMEOUT     2
#define   STATE_TIMER       3
#define   STATE_OCCUPIED    4
#define   STATE_LOCKOUT     5
#define   STATE_CLEAR       6

typedef struct
{
	uint8_t flags;         // See SIM_TRAIN_FLAGS defines
	uint8_t direction;
	uint16_t time;         // Minutes since midnight
	uint8_t totalTime;     // seconds
	uint8_t approachTime;  // seconds
} SimTrain;

#define NUM_SIM_TRAINS 32

typedef enum
{
	TRIGGER_DEAD_RECKONING,
	TRIGGER_PACKET,
	TRIGGER_WINDOW,
	TRIGGER_SKIP,
} TriggerType;

uint8_t occupancy;
uint8_t turnouts;

int timeoutTime, debounceTime, lockoutTime, timelockTime;

int timeval_subtract(struct timeval *result, struct timeval *t2, struct timeval *t1)
{
	long int diff = (t2->tv_usec + 1000000 * t2->tv_sec) - (t1->tv_usec + 1000000 * t1->tv_sec);
	result->tv_sec = diff / 1000000;
	result->tv_usec = diff % 1000000;

	return (diff<0);
}

void textcolor(int attr, int fg, int bg)
{
	printf("%c[%d;%d;%dm", 0x1B, attr, fg + 30, bg + 40);
}

void assertInt(char *msg, int actual, int expect)
{
	textcolor(BRIGHT, BLUE, BLACK);
	printf("%s: Actual=%d (0x%X), Expect=%d (0x%X) ", msg, actual, actual, expect, expect);

	if(actual == expect)
	{
		pass++;
	}
	else
	{
		textcolor(BRIGHT, RED, BLACK);
		printf("***Fail*** ");
		fail++;
	}
	
	textcolor(RESET, WHITE, BLACK);
	printf("\n");
}

void assertFloat(char *msg, float actual, float expect, float tolerance)
{
	textcolor(BRIGHT, BLUE, BLACK);
	printf("%s: Actual=%.3f, Expect=%.3f +/-%.3f ", msg, actual, expect, tolerance);

	if( (actual >= (expect-tolerance)) && (actual <= (expect+tolerance)) )
	{
		pass++;
	}
	else
	{
		textcolor(BRIGHT, RED, BLACK);
		printf("***Fail*** ");
		fail++;
	}
	
	textcolor(RESET, WHITE, BLACK);
	printf("\n");
}

void waitForNewPacket(void)
{
	char path[256];
	int startCount = 0, count = 0;
	FILE *fptr;

	// Wait for an updated status packet

	snprintf(path, sizeof(path), "%s/rxCounter", MRBFS_PATH);

	if( NULL != (fptr = fopen(path, "r")) )
	{
		if(!fscanf(fptr, "%d", &startCount))
		{
			textcolor(RESET, WHITE, RED);
			printf("Failed to read %s!\n", path);
			textcolor(RESET, WHITE, BLACK);
			fclose(fptr);
			return;
		}
	}
	else
	{
		textcolor(RESET, WHITE, RED);
		printf("Failed to open %s!\n", path);
		textcolor(RESET, WHITE, BLACK);
		return;
	}
	fclose(fptr);
	usleep(100000);

	do
	{
		if( NULL != (fptr = fopen(path, "r")) )
		{
			if(!fscanf(fptr, "%d", &count))
			{
				textcolor(RESET, WHITE, RED);
				printf("Failed to read %s!\n", path);
				textcolor(RESET, WHITE, BLACK);
				fclose(fptr);
				return;
			}
		}
		else
		{
			textcolor(RESET, WHITE, RED);
			printf("Failed to open %s!\n", path);
			textcolor(RESET, WHITE, BLACK);
			return;
		}
		fclose(fptr);
		usleep(100000);  // Delay after to give mrbfs time to process new information
	} while(count < (startCount+1));
}

int getSignal(int signal)
{
	char path[256], data[256];
	FILE *fptr;

	waitForNewPacket();
	
	snprintf(path, sizeof(path), "%s/signalhead_%d", MRBFS_PATH, signal);
	
	if( NULL != (fptr = fopen(path, "r")) )
	{
		if(!fscanf(fptr, "%s", data))
		{
			textcolor(RESET, WHITE, RED);
			printf("Failed to read %s!\n", path);
			textcolor(RESET, WHITE, BLACK);
			return -1;
		}
		fclose(fptr);
	}
	else
	{
		textcolor(RESET, WHITE, RED);
		printf("Failed to open %s!\n", path);
		textcolor(RESET, WHITE, BLACK);
		return -1;
	}
	
	if(!strcmp(data, "Red"))
		return ASPECT_RED;
	else if(!strcmp(data, "Yellow"))
		return ASPECT_YELLOW;
	else if(!strcmp(data, "Green"))
		return ASPECT_GREEN;
	
	return -1;
}


void assertAspect(char *msg, int signal, int expect)
{
	int passed = 0;
	int retries = 3;
	
	while((!passed) && retries)
	{
		textcolor(BRIGHT, BLUE, BLACK);
		int actual = getSignal(signal);
		printf("%s: Actual=%s, Expect=%s ", msg, aspectString[actual], aspectString[expect]);

		if(actual == expect)
			passed = 1;
		else if(--retries)
		{
			printf("***Retry***\n");
			waitForNewPacket();
		}
	}
	
	if(passed)
	{
		pass++;
	}
	else
	{
		textcolor(BRIGHT, RED, BLACK);
		printf("***Fail*** ");
		fail++;
	}

	textcolor(RESET, WHITE, BLACK);
	printf("\n");
}

void writeEeprom(int addr, int val)
{
	char pkt[32];
	FILE *fptr;
	
	snprintf(pkt, sizeof(pkt), "%02X->%02X %02X %02X %02X", SRC_ADDR, DUT_ADDR, 'W', addr, val);
	textcolor(RESET, YELLOW, BLACK);
	printf("Sending eeprom packet [%s]\n", pkt);
	textcolor(RESET, WHITE, BLACK);

	if( NULL != (fptr = fopen(MRBFS_TXPKT, "w")) )
	{
		fprintf(fptr, "%s\n", pkt);
		fclose(fptr);
	}
	else
	{
		textcolor(RESET, WHITE, RED);
		printf("Failed to open %s!\n", MRBFS_TXPKT);
		textcolor(RESET, WHITE, BLACK);
	}
	
	usleep(100000);
}

void writeEepromSim(int num, SimTrain sim)
{
	char pkt[256];
	FILE *fptr;
	
	snprintf(pkt, sizeof(pkt), "%02X->%02X %02X %02X %02X %02X %02X %02X %02X %02X", SRC_ADDR, DUT_ADDR, 'W', 0x40+(6*num), sim.flags, sim.direction, (sim.time >> 8)&0xFF, sim.time&0xFF, sim.totalTime, sim.approachTime);
	textcolor(RESET, YELLOW, BLACK);
	printf("Sending sim eeprom packet [%s]\n", pkt);
	textcolor(RESET, WHITE, BLACK);

	if( NULL != (fptr = fopen(MRBFS_TXPKT, "w")) )
	{
		fprintf(fptr, "%s\n", pkt);
		fclose(fptr);
	}
	else
	{
		textcolor(RESET, WHITE, RED);
		printf("Failed to open %s!\n", MRBFS_TXPKT);
		textcolor(RESET, WHITE, BLACK);
	}
	
	usleep(100000);
}



uint8_t maxDeadReckoning;

void setMaxDeadReckoning(uint8_t value)
{
	writeEeprom(EE_MAX_DEAD_RECKONING, value);
	maxDeadReckoning = value;
}

uint8_t getMaxDeadReckoning(void)
{
	return maxDeadReckoning;
}



uint8_t simTrainWindow;

void setSimTrainWindow(uint8_t value)
{
	writeEeprom(EE_SIM_TRAIN_WINDOW, value);
	simTrainWindow = value;
}

uint8_t getSimTrainWindow(void)
{
	return simTrainWindow;
}



InterlockState getState(int dir)
{
	char path[256];
	int data;
	FILE *fptr;
	
	snprintf(path, sizeof(path), "%s/stateMachine%d", MRBFS_PATH, dir);
	
	if( NULL != (fptr = fopen(path, "r")) )
	{
		if(!fscanf(fptr, "%d", &data))
		{
			textcolor(RESET, WHITE, RED);
			printf("Failed to read %s!\n", path);
			textcolor(RESET, WHITE, BLACK);
			return -1;
		}
		fclose(fptr);
	}
	else
	{
		textcolor(RESET, WHITE, RED);
		printf("Failed to open %s!\n", path);
		textcolor(RESET, WHITE, BLACK);
		return -1;
	}
	
	return data;
}

int getLockout(int dir)
{
	char path[256];
	int data;
	FILE *fptr;
	
	snprintf(path, sizeof(path), "%s/lockoutStatus%d", MRBFS_PATH, dir);
	
	if( NULL != (fptr = fopen(path, "r")) )
	{
		if(!fscanf(fptr, "%d", &data))
		{
			textcolor(RESET, WHITE, RED);
			printf("Failed to read %s!\n", path);
			textcolor(RESET, WHITE, BLACK);
			return -1;
		}
		fclose(fptr);
	}
	else
	{
		textcolor(RESET, WHITE, RED);
		printf("Failed to open %s!\n", path);
		textcolor(RESET, WHITE, BLACK);
		return -1;
	}
	
	return data;
}

int getTimeout(int dir)
{
	char path[256];
	int data;
	FILE *fptr;
	
	snprintf(path, sizeof(path), "%s/timeoutStatus%d", MRBFS_PATH, dir);
	
	if( NULL != (fptr = fopen(path, "r")) )
	{
		if(!fscanf(fptr, "%d", &data))
		{
			textcolor(RESET, WHITE, RED);
			printf("Failed to read %s!\n", path);
			textcolor(RESET, WHITE, BLACK);
			return -1;
		}
		fclose(fptr);
	}
	else
	{
		textcolor(RESET, WHITE, RED);
		printf("Failed to open %s!\n", path);
		textcolor(RESET, WHITE, BLACK);
		return -1;
	}
	
	return data;
}

int getDebounce(void)
{
	char path[256];
	int data;
	FILE *fptr;
	
	snprintf(path, sizeof(path), "%s/debounceTimer", MRBFS_PATH);
	
	if( NULL != (fptr = fopen(path, "r")) )
	{
		if(!fscanf(fptr, "%d", &data))
		{
			textcolor(RESET, WHITE, RED);
			printf("Failed to read %s!\n", path);
			textcolor(RESET, WHITE, BLACK);
			return -1;
		}
		fclose(fptr);
	}
	else
	{
		textcolor(RESET, WHITE, RED);
		printf("Failed to open %s!\n", path);
		textcolor(RESET, WHITE, BLACK);
		return -1;
	}
	
	return data;
}

int getTimelock(void)
{
	char path[256];
	int data;
	FILE *fptr;
	
	snprintf(path, sizeof(path), "%s/timelockTimer", MRBFS_PATH);
	
	if( NULL != (fptr = fopen(path, "r")) )
	{
		if(!fscanf(fptr, "%d", &data))
		{
			textcolor(RESET, WHITE, RED);
			printf("Failed to read %s!\n", path);
			textcolor(RESET, WHITE, BLACK);
			return -1;
		}
		fclose(fptr);
	}
	else
	{
		textcolor(RESET, WHITE, RED);
		printf("Failed to open %s!\n", path);
		textcolor(RESET, WHITE, BLACK);
		return -1;
	}
	
	return data;
}

int getOccupancy(int dir)
{
	char path[256];
	int data;
	FILE *fptr;
	
	switch (dir)
	{
		case OCC_WEST_MAIN:
			snprintf(path, sizeof(path), "%s/occ_dir0_main", MRBFS_PATH);
			break;
		case OCC_WEST_SIDING:
			snprintf(path, sizeof(path), "%s/occ_dir0_siding", MRBFS_PATH);
			break;
		case OCC_EAST_MAIN:
			snprintf(path, sizeof(path), "%s/occ_dir1_main", MRBFS_PATH);
			break;
		case OCC_NORTH_MAIN:
			snprintf(path, sizeof(path), "%s/occ_dir2_main", MRBFS_PATH);
			break;
		case OCC_NORTH_SIDING:
			snprintf(path, sizeof(path), "%s/occ_dir2_siding", MRBFS_PATH);
			break;
		case OCC_SOUTH_MAIN:
			snprintf(path, sizeof(path), "%s/occ_dir3_main", MRBFS_PATH);
			break;
		case OCC_INTERLOCKING:
			snprintf(path, sizeof(path), "%s/occ_interlocking_plant", MRBFS_PATH);
			break;
		default:
			snprintf(path, sizeof(path), "%s/unknown_occupancy_direction", MRBFS_PATH);
			break;
	}
	
	if( NULL != (fptr = fopen(path, "r")) )
	{
		if(!fscanf(fptr, "%d", &data))
		{
			textcolor(RESET, WHITE, RED);
			printf("Failed to read %s!\n", path);
			textcolor(RESET, WHITE, BLACK);
			return -1;
		}
		fclose(fptr);
	}
	else
	{
		textcolor(RESET, WHITE, RED);
		printf("Failed to open %s!\n", path);
		textcolor(RESET, WHITE, BLACK);
		return -1;
	}
	
	return data;
}

int getTurnout(int dir)
{
	char path[256];
	char position[16];
	int data;
	FILE *fptr;
	
	switch (dir)
	{
		case DIR_WEST:
			snprintf(path, sizeof(path), "%s/turnout_dir0", MRBFS_PATH);
			break;
		case DIR_EAST:
			snprintf(path, sizeof(path), "%s/turnout_dir1", MRBFS_PATH);
			break;
		case DIR_NORTH:
			snprintf(path, sizeof(path), "%s/turnout_dir2", MRBFS_PATH);
			break;
		case DIR_SOUTH:
			snprintf(path, sizeof(path), "%s/turnout_dir3", MRBFS_PATH);
			break;
		default:
			snprintf(path, sizeof(path), "%s/unknown_turnout", MRBFS_PATH);
			break;
	}
	
	if( NULL != (fptr = fopen(path, "r")) )
	{
		if(!fscanf(fptr, "%s", position))
		{
			textcolor(RESET, WHITE, RED);
			printf("Failed to read %s!\n", path);
			textcolor(RESET, WHITE, BLACK);
			return -1;
		}
		fclose(fptr);
	}
	else
	{
		textcolor(RESET, WHITE, RED);
		printf("Failed to open %s!\n", path);
		textcolor(RESET, WHITE, BLACK);
		return -1;
	}
	
	if(NULL != strstr(position, "normal"))
		data = 0;
	else
		data = 1;
	
	return data;
}

void sendCommand(void)
{
	char pkt[32];
	FILE *fptr;
	
	// FIXME: xor (invert) bits based on polarity configuration
	snprintf(pkt, sizeof(pkt), "%02X->%02X %02X %02X %02X %02X", SRC_ADDR, DUT_ADDR, 'C', 'T', occupancy, turnouts);

	int i=3;
	while(i)
	{
		// Try sending packet
		textcolor(RESET, YELLOW, BLACK);
		printf("Sending cmd packet [%s]\n", pkt);
		textcolor(RESET, WHITE, BLACK);

		if( NULL != (fptr = fopen(MRBFS_TXPKT, "w")) )
		{
			fprintf(fptr, "%s\n", pkt);
			fclose(fptr);
		}
		else
		{
			textcolor(RESET, WHITE, RED);
			printf("Failed to open %s!\n", MRBFS_TXPKT);
			textcolor(RESET, WHITE, BLACK);
		}
		
		// Wait for response
		usleep(100000);
		
		// Check state
		int foo = 0, j;
		for(j=0; j<7; j++)
		{
			if( getOccupancy(1 << j) != ((occupancy >> j)&0x01) )
				foo = 1;
		}
		if( getTurnout(0) != ((turnouts >> 0)&0x01) )
			foo = 1;
		if( getTurnout(2) != ((turnouts >> 2)&0x01) )
			foo = 1;
		
		if(!foo)
			break;
		i--;
	}
	
	if(!i)
	{
		textcolor(RESET, WHITE, RED);
		printf("Failed to confirm response to command!\n");
		textcolor(RESET, WHITE, BLACK);
	}

/*
	usleep(100000);

	// Send again 100ms later just to be safe
  	if( NULL != (fptr = fopen(MRBFS_TXPKT, "w")) )
	{
		fprintf(fptr, "%s\n", pkt);
		fclose(fptr);
	}
	else
	{
		textcolor(RESET, WHITE, RED);
		printf("Failed to open %s!\n", MRBFS_TXPKT);
		textcolor(RESET, WHITE, BLACK);
	}

	usleep(900000);  // Wait additional 900ms to make a complete 1 second delay (to give time for response packet)
*/
}

int getSound(int sound)
{
	char path[256];
	int data;
	FILE *fptr;
	
	snprintf(path, sizeof(path), "%s/soundOut%d", MRBFS_PATH, sound);
	
	if( NULL != (fptr = fopen(path, "r")) )
	{
		if(!fscanf(fptr, "%d", &data))
		{
			textcolor(RESET, WHITE, RED);
			printf("Failed to read %s!\n", path);
			textcolor(RESET, WHITE, BLACK);
			return -1;
		}
		fclose(fptr);
	}
	else
	{
		textcolor(RESET, WHITE, RED);
		printf("Failed to open %s!\n", path);
		textcolor(RESET, WHITE, BLACK);
		return -1;
	}
	
	return data;
}

void clearAll(void)
{
	printf("Clearing...\n");

	occupancy = 0;
	turnouts = 0;
	sendCommand();
	
	sleep(1);  // Wait for things to catch up
	
	int i=0;
	
	printf("Waiting for states and timers to clear.");
	while( 
		getState(DIR_WEST) ||
		getState(DIR_EAST) ||
		getState(DIR_NORTH) ||
		getState(DIR_SOUTH) ||
		getLockout(DIR_WEST) ||
		getLockout(DIR_EAST) ||
		getLockout(DIR_NORTH) ||
		getLockout(DIR_SOUTH) ||
		getTimelock()
		)
	{
		usleep(100000);
		if(++i > 10)
		{
			printf(".");
			fflush(stdout);
			i = 0;
		}
	}
	printf("\n");
}

void testEastboundSouthbound(int dir, int track, int leaveBeforeDebounce)
{
	START_TEST;
	
	int signal;
	
	printf("\n");
	printf("--------------------------------------------------------------------------------\n");
	printf("%s - %s %s\n", (DIR_EAST==dir)?"Eastbound":"Southbound", track?"Siding":"Main", leaveBeforeDebounce?"(Leave Before Debounce)":"");
	printf("--------------------------------------------------------------------------------\n");

	clearAll();

	if(DIR_EAST==dir)
	{
		occupancy |= track?OCC_WEST_SIDING:OCC_WEST_MAIN;
		turnouts |= track?TURNOUT_WEST:0;
		signal = 0 + track;
	}
	else
	{
		occupancy |= track?OCC_NORTH_SIDING:OCC_NORTH_MAIN;
		turnouts |= track?TURNOUT_NORTH:0;
		signal = 4 + track;
	}
	sendCommand();
	
	assertAspect("Train arrives, gets proceed indication", signal, track?ASPECT_YELLOW:ASPECT_GREEN);
	assertAspect("Red signal on other track", track?signal-1:signal+1, ASPECT_RED);

	occupancy |= OCC_INTERLOCKING;
	sendCommand();
	assertAspect("Enters interlocking, red signal", signal, ASPECT_RED);

	occupancy |= (DIR_EAST==dir)?OCC_EAST_MAIN:OCC_SOUTH_MAIN;
	sendCommand();
	assertAspect("Enters opposite approach, red signal", signal, ASPECT_RED);
	assertInt("No lockout", getLockout((DIR_EAST==dir)?DIR_EAST:DIR_SOUTH), 0);
	
	occupancy &= (DIR_EAST==dir) ? (track?~OCC_WEST_SIDING:~OCC_WEST_MAIN) : (track?~OCC_NORTH_SIDING:~OCC_NORTH_MAIN);
	sendCommand();
	assertAspect("Approach clear, red signal", signal, ASPECT_RED);
	assertInt("No lockout", getLockout((DIR_EAST==dir)?DIR_EAST:DIR_SOUTH), 0);

	occupancy &= ~OCC_INTERLOCKING;
	sendCommand();
	assertAspect("Interlocking cleared, red signal", signal, ASPECT_RED);
	waitForNewPacket();
	assertInt("DEBOUNCE timer running", 0!=getDebounce(), 1);
	
	if(leaveBeforeDebounce)
	{
		occupancy &= (DIR_EAST==dir)?~OCC_EAST_MAIN:~OCC_SOUTH_MAIN;
		sendCommand();
		assertAspect("Opposite approach cleared, red signal", signal, ASPECT_RED);
		assertInt("DEBOUNCE timer running", 0!=getDebounce(), 1);
	}
	
	while(getDebounce())
	{
		printf(".");
		fflush(stdout);
		sleep(1);
	}
	printf("\n");

	assertAspect("DEBOUNCE expired, red signal", signal, ASPECT_RED);
	waitForNewPacket();
	assertInt("LOCKOUT timer running", getLockout((DIR_EAST==dir)?DIR_EAST:DIR_SOUTH), 1);

	if(!leaveBeforeDebounce)
	{
		occupancy &= (DIR_EAST==dir)?~OCC_EAST_MAIN:~OCC_SOUTH_MAIN;
		sendCommand();
		assertAspect("Opposite approach cleared, red signal", signal, ASPECT_RED);
		assertInt("LOCKOUT timer running", getLockout((DIR_EAST==dir)?DIR_EAST:DIR_SOUTH), 1);
	}

	while(getLockout((DIR_EAST==dir)?DIR_EAST:DIR_SOUTH))
	{
		printf(".");
		fflush(stdout);
		sleep(1);
	}
	printf("\n");

	assertAspect("LOCKOUT complete, red signal", signal, ASPECT_RED);

	END_TEST;
}

void testWestboundNorthbound(int dir, int track, int leaveBeforeDebounce)
{
	START_TEST;

	int signal;
	
	printf("\n");
	printf("--------------------------------------------------------------------------------\n");
	printf("%s - %s %s\n", (DIR_WEST==dir)?"Westbuond":"Northbound", track?"Siding":"Main", leaveBeforeDebounce?"(Leave Before Debounce)":"");
	printf("--------------------------------------------------------------------------------\n");

	clearAll();

	if(DIR_WEST==dir)
	{
		occupancy |= OCC_EAST_MAIN;
		turnouts |= track?TURNOUT_WEST:0;
		signal = 2;
	}
	else
	{
		occupancy |= OCC_SOUTH_MAIN;
		turnouts |= track?TURNOUT_NORTH:0;
		signal = 6;
	}
	sendCommand();
	
	assertAspect("Train arrives, gets proceed indication (top)", signal, track?ASPECT_RED:ASPECT_GREEN);
	assertAspect("Train arrives, gets proceed indication (bottom)", signal+1, track?ASPECT_YELLOW:ASPECT_RED);

	occupancy |= OCC_INTERLOCKING;
	sendCommand();
	assertAspect("Enters interlocking, red signal (top)", signal, ASPECT_RED);
	assertAspect("Enters interlocking, red signal (bottom)", signal+1, ASPECT_RED);

	occupancy |= (DIR_WEST==dir)?(track?OCC_WEST_SIDING:OCC_WEST_MAIN):(track?OCC_NORTH_SIDING:OCC_NORTH_MAIN);
	sendCommand();
	assertAspect("Enters opposite approach, red signal (top)", signal, ASPECT_RED);
	assertAspect("Enters opposite approach, red signal (bottom)", signal+1, ASPECT_RED);
	assertInt("No lockout", getLockout((DIR_WEST==dir)?DIR_WEST:DIR_NORTH), 0);
	
	occupancy &= (DIR_WEST==dir) ? ~OCC_EAST_MAIN : ~OCC_SOUTH_MAIN;
	sendCommand();
	assertAspect("Approach clear, red signal (top)", signal, ASPECT_RED);
	assertAspect("Approach clear, red signal (bottom)", signal+1, ASPECT_RED);
	assertInt("No lockout", getLockout((DIR_WEST==dir)?DIR_WEST:DIR_NORTH), 0);

	occupancy &= ~OCC_INTERLOCKING;
	sendCommand();
	assertAspect("Interlocking cleared, red signal (top)", signal, ASPECT_RED);
	assertAspect("Interlocking cleared, red signal (bottom)", signal+1, ASPECT_RED);
	waitForNewPacket();
	assertInt("DEBOUNCE timer running", 0!=getDebounce(), 1);
	
	if(leaveBeforeDebounce)
	{
		occupancy &= (DIR_WEST==dir)?(track?~OCC_WEST_SIDING:~OCC_WEST_MAIN):(track?~OCC_NORTH_SIDING:~OCC_NORTH_MAIN);
		sendCommand();
		assertAspect("Opposite approach cleared, red signal (top)", signal, ASPECT_RED);
		assertAspect("Opposite approach cleared, red signal (bottom)", signal+1, ASPECT_RED);
		assertInt("DEBOUNCE timer still running", 0!=getDebounce(), 1);
	}
	
	while(getDebounce())
	{
		printf(".");
		fflush(stdout);
		sleep(1);
	}
	printf("\n");

	assertAspect("DEBOUNCE expired, red signal (top)", signal, ASPECT_RED);
	assertAspect("DEBOUNCE expired, red signal (bottom)", signal+1, ASPECT_RED);
	waitForNewPacket();
	assertInt("LOCKOUT timer running", getLockout((DIR_WEST==dir)?DIR_WEST:DIR_NORTH), 1);

	if(!leaveBeforeDebounce)
	{
		occupancy &= (DIR_WEST==dir)?(track?~OCC_WEST_SIDING:~OCC_WEST_MAIN):(track?~OCC_NORTH_SIDING:~OCC_NORTH_MAIN);
		sendCommand();
		assertAspect("Opposite approach cleared, red signal (top)", signal, ASPECT_RED);
		assertAspect("Opposite approach cleared, red signal (bottom)", signal+1, ASPECT_RED);
		assertInt("LOCKOUT timer running", getLockout((DIR_WEST==dir)?DIR_WEST:DIR_NORTH), 1);
	}

	while(getLockout((DIR_WEST==dir)?DIR_WEST:DIR_NORTH))
	{
		printf(".");
		fflush(stdout);
		sleep(1);
	}
	printf("\n");

	assertAspect("LOCKOUT complete, red signal (top)", signal, ASPECT_RED);
	assertAspect("LOCKOUT complete, red signal (bottom)", signal+1, ASPECT_RED);

	END_TEST;
}

void testMeetBeforeInterlocking(int firstOcc)
{
	int i;
	struct timeval tvDiff, tvTimerA, tvTimerB;
	int signal = 0, track = 0;
	int secondOcc = 0;
	
	START_TEST;

	printf("\n");
	printf("--------------------------------------------------------------------------------\n");
	printf("Meet Before Interlocking - ");
	switch(firstOcc)
	{
		case OCC_WEST_MAIN:
			printf("Arrive West Main");
			break;
		case OCC_WEST_SIDING:
			printf("Arrive West Siding");
			break;
		case OCC_NORTH_MAIN:
			printf("Arrive North Main");
			break;
		case OCC_NORTH_SIDING:
			printf("Arrive North Siding");
			break;
	}
	printf("\n");
	printf("--------------------------------------------------------------------------------\n");

	clearAll();

	occupancy |= firstOcc;
	switch(firstOcc)
	{
		case OCC_WEST_MAIN:
			signal = SIGNAL_EB_MAIN;
			secondOcc = OCC_EAST_MAIN;
			track = 0;
			break;
		case OCC_WEST_SIDING:
			turnouts |= TURNOUT_WEST;
			signal = SIGNAL_EB_SIDING;
			secondOcc = OCC_EAST_MAIN;
			track = 1;
			break;
		case OCC_NORTH_MAIN:
			signal = SIGNAL_SB_MAIN;
			secondOcc = OCC_SOUTH_MAIN;
			track = 0;
			break;
		case OCC_NORTH_SIDING:
			turnouts |= TURNOUT_NORTH;
			signal = SIGNAL_SB_SIDING;
			secondOcc = OCC_SOUTH_MAIN;
			track = 1;
			break;
	}
	sendCommand();
	
	assertAspect("First train arrives, gets proceed indication", signal, track?ASPECT_YELLOW:ASPECT_GREEN);
	
	occupancy |= secondOcc;
	sendCommand();

	assertAspect("Second train arrives, gets stop indication (top)", (OCC_EAST_MAIN==secondOcc)?SIGNAL_WB_TOP:SIGNAL_NB_TOP, ASPECT_RED);
	assertAspect("Second train arrives, gets stop indication (bottom)", (OCC_EAST_MAIN==secondOcc)?SIGNAL_WB_BOTTOM:SIGNAL_NB_BOTTOM, ASPECT_RED);
	
	printf("Change turnout to break the deadlock\n");
	switch(firstOcc)
	{
		case OCC_WEST_MAIN:
		case OCC_WEST_SIDING:
			turnouts ^= TURNOUT_WEST;
			break;
		case OCC_NORTH_MAIN:
		case OCC_NORTH_SIDING:
			turnouts ^= TURNOUT_NORTH;
			break;
	}
	sendCommand();

	gettimeofday(&tvTimerA, NULL);  // Timelock start

	assertAspect("First train's signal drops to stop", signal, ASPECT_RED);
	
	i = 0;
	printf("Wait for timelock to expire.");
	while(getTimelock())
	{
		usleep(100000);
		if(++i > 10)
		{
			printf(".");
			fflush(stdout);
			i = 0;
		}
	}
	printf("\n");
	gettimeofday(&tvTimerB, NULL);
	timeval_subtract(&tvDiff, &tvTimerB, &tvTimerA);

	assertFloat("TIMELOCK timer", (float)tvDiff.tv_sec + (float)tvDiff.tv_usec/1000000, timelockTime, 1);
	
	int aspect = ((OCC_WEST_MAIN==firstOcc)||(OCC_NORTH_MAIN==firstOcc))?ASPECT_RED:ASPECT_GREEN;
	assertAspect("Second train gets proceed indication (top)", (OCC_EAST_MAIN==secondOcc)?SIGNAL_WB_TOP:SIGNAL_NB_TOP, aspect);
	aspect = ((OCC_WEST_MAIN==firstOcc)||(OCC_NORTH_MAIN==firstOcc))?ASPECT_YELLOW:ASPECT_RED;
	assertAspect("Second train gets proceed indication (bottom)", (OCC_EAST_MAIN==secondOcc)?SIGNAL_WB_BOTTOM:SIGNAL_NB_BOTTOM, aspect);

	occupancy |= OCC_INTERLOCKING;
	sendCommand();

	occupancy &= ~secondOcc;
	sendCommand();
	
	switch(firstOcc)
	{
		case OCC_WEST_MAIN:
			occupancy |= OCC_WEST_SIDING;
			break;
		case OCC_WEST_SIDING:
			occupancy |= OCC_WEST_MAIN;
			break;
		case OCC_NORTH_MAIN:
			occupancy |= OCC_NORTH_SIDING;
			break;
		case OCC_NORTH_SIDING:
			occupancy |= OCC_NORTH_MAIN;
			break;
	}
	sendCommand();

	occupancy &= ~OCC_INTERLOCKING;
	sendCommand();

	waitForNewPacket();

	printf("Wait for debounce and timelock to expire.");
	while(getTimelock() || getDebounce())
	{
		usleep(100000);
		if(++i > 10)
		{
			printf(".");
			fflush(stdout);
			i = 0;
		}
	}
	printf("\n");

	waitForNewPacket();

	switch(firstOcc)
	{
		case OCC_WEST_MAIN:
		case OCC_WEST_SIDING:
			assertInt("West in LOCKOUT", getLockout(DIR_WEST), 1);
			break;
		case OCC_NORTH_MAIN:
		case OCC_NORTH_SIDING:
			assertInt("North in LOCKOUT", getLockout(DIR_NORTH), 1);
			break;
	}


	printf("Change turnout back\n");
	switch(firstOcc)
	{
		case OCC_WEST_MAIN:
		case OCC_WEST_SIDING:
			turnouts ^= TURNOUT_WEST;
			break;
		case OCC_NORTH_MAIN:
		case OCC_NORTH_SIDING:
			turnouts ^= TURNOUT_NORTH;
			break;
	}
	sendCommand();

	waitForNewPacket();

	assertAspect("First train gets proceed indication again", signal, track?ASPECT_YELLOW:ASPECT_GREEN);
	
	END_TEST;
}

void testArriveOpposingTurnout(int dir, int track)
{
	START_TEST;

	int signal;
	
	printf("\n");
	printf("--------------------------------------------------------------------------------\n");
	printf("Arrive %s to Opposing Turnout - %s\n", (DIR_EAST==dir)?"Eastbound":"Southbound", track?"Siding":"Main");
	printf("--------------------------------------------------------------------------------\n");

	clearAll();

	if(DIR_EAST==dir)
	{
		turnouts = track?0:TURNOUT_WEST;
	}
	else
	{
		turnouts = track?0:TURNOUT_NORTH;
	}
	sendCommand();
	
	if(DIR_EAST==dir)
	{
		occupancy |= track?OCC_WEST_SIDING:OCC_WEST_MAIN;
		signal = 0 + track;
	}
	else
	{
		occupancy |= track?OCC_NORTH_SIDING:OCC_NORTH_MAIN;
		signal = 4 + track;
	}
	sendCommand();
	
	assertAspect("Train arrives to opposing turnout, red signal", signal, ASPECT_RED);
	assertAspect("Red signal on other track", track?signal-1:signal+1, ASPECT_RED);

	if(DIR_EAST==dir)
	{
		turnouts = track?TURNOUT_WEST:0;
	}
	else
	{
		turnouts = track?TURNOUT_NORTH:0;
	}
	sendCommand();

	assertAspect("Turnout changed, proceed indication", signal, track?ASPECT_YELLOW:ASPECT_GREEN);
	assertAspect("Red signal on other track", track?signal-1:signal+1, ASPECT_RED);

	if(DIR_EAST==dir)
	{
		turnouts = track?0:TURNOUT_WEST;
	}
	else
	{
		turnouts = track?0:TURNOUT_NORTH;
	}
	sendCommand();

	assertAspect("Turnout changed back, red signal", signal, ASPECT_RED);
	assertAspect("Red signal on other track", track?signal-1:signal+1, ASPECT_RED);

	if(DIR_EAST==dir)
	{
		occupancy &= track?~OCC_WEST_SIDING:~OCC_WEST_MAIN;
	}
	else
	{
		occupancy &= track?~OCC_NORTH_SIDING:~OCC_NORTH_MAIN;
	}
	sendCommand();

	END_TEST;
}

void testTimelockMeet(int firstBlock, int secondBlock)
{
	int i;
	struct timeval tvDiff, tvTimerA, tvTimerB;

	START_TEST;
	
	printf("\n");
	printf("--------------------------------------------------------------------------------\n");
	printf("Timelock Timer with Meet\n");
	printf("--------------------------------------------------------------------------------\n");

	clearAll();
	
	turnouts = 0;

	if((firstBlock == OCC_WEST_SIDING) || (secondBlock == OCC_WEST_SIDING))
	{
		turnouts |= TURNOUT_WEST;
	}
	
	if((firstBlock == OCC_NORTH_SIDING) || (secondBlock == OCC_NORTH_SIDING))
	{
		turnouts |= TURNOUT_NORTH;
	}

	occupancy |= firstBlock;
	sendCommand();
	switch(firstBlock)
	{
		case OCC_WEST_MAIN:
			assertAspect("First train arrives, gets green", SIGNAL_EB_MAIN, ASPECT_GREEN);
			break;
		case OCC_WEST_SIDING:
			assertAspect("First train arrives, gets yellow", SIGNAL_EB_SIDING, ASPECT_YELLOW);
			break;
		case OCC_EAST_MAIN:
			assertAspect("First train arrives, gets green", SIGNAL_WB_TOP, ASPECT_GREEN);
			assertAspect("...over red", SIGNAL_WB_BOTTOM, ASPECT_RED);
			break;
		case OCC_NORTH_MAIN:
			assertAspect("First train arrives, gets green", SIGNAL_SB_MAIN, ASPECT_GREEN);
			break;
		case OCC_NORTH_SIDING:
			assertAspect("First train arrives, gets yellow", SIGNAL_SB_SIDING, ASPECT_YELLOW);
			break;
		case OCC_SOUTH_MAIN:
			assertAspect("First train arrives, gets green", SIGNAL_NB_TOP, ASPECT_GREEN);
			assertAspect("...over red", SIGNAL_NB_BOTTOM, ASPECT_RED);
			break;
	}

	occupancy |= secondBlock;
	sendCommand();
	switch(secondBlock)
	{
		case OCC_WEST_MAIN:
			assertAspect("Second train arrives to a red", SIGNAL_EB_MAIN, ASPECT_RED);
			break;
		case OCC_WEST_SIDING:
			assertAspect("Second train arrives to a red", SIGNAL_EB_SIDING, ASPECT_RED);
			break;
		case OCC_EAST_MAIN:
			assertAspect("Second train arrives to a red", SIGNAL_WB_TOP, ASPECT_RED);
			assertAspect("...over red", SIGNAL_WB_BOTTOM, ASPECT_RED);
			break;
		case OCC_NORTH_MAIN:
			assertAspect("Second train arrives to a red", SIGNAL_SB_MAIN, ASPECT_RED);
			break;
		case OCC_NORTH_SIDING:
			assertAspect("Second train arrives to a red", SIGNAL_SB_SIDING, ASPECT_RED);
			break;
		case OCC_SOUTH_MAIN:
			assertAspect("Second train arrives to a red", SIGNAL_NB_TOP, ASPECT_RED);
			assertAspect("...over red", SIGNAL_NB_BOTTOM, ASPECT_RED);
			break;
	}

	occupancy &= ~firstBlock;
	sendCommand();

	int timeoutDir = 0;
	switch(firstBlock)
	{
		case OCC_WEST_MAIN:
			assertAspect("First train backs up and leaves, signal stays green", SIGNAL_EB_MAIN, ASPECT_GREEN);
			timeoutDir = 0;
			break;
		case OCC_WEST_SIDING:
			assertAspect("First train backs up and leaves, signal stays yellow", SIGNAL_EB_SIDING, ASPECT_YELLOW);
			timeoutDir = 0;
			break;
		case OCC_EAST_MAIN:
			assertAspect("First train backs up and leaves, signal stays green", SIGNAL_WB_TOP, ASPECT_GREEN);
			assertAspect("...over red", SIGNAL_WB_BOTTOM, ASPECT_RED);
			timeoutDir = 1;
			break;
		case OCC_NORTH_MAIN:
			assertAspect("First train backs up and leaves, signal stays green", SIGNAL_SB_MAIN, ASPECT_GREEN);
			timeoutDir = 2;
			break;
		case OCC_NORTH_SIDING:
			assertAspect("First train backs up and leaves, signal stays yellow", SIGNAL_SB_SIDING, ASPECT_YELLOW);
			timeoutDir = 2;
			break;
		case OCC_SOUTH_MAIN:
			assertAspect("First train backs up and leaves, signal stays green", SIGNAL_NB_TOP, ASPECT_GREEN);
			assertAspect("...over red", SIGNAL_NB_BOTTOM, ASPECT_RED);
			timeoutDir = 3;
			break;
	}

	assertInt("TIMEOUT timer running", getTimeout(timeoutDir), 1);

	while(getTimeout(timeoutDir))
	{
		printf(".");
		fflush(stdout);
		sleep(1);
	}
	printf("\n");

	gettimeofday(&tvTimerA, NULL);  // Timelock start

	waitForNewPacket();

	i = 0;
	printf("Wait for timelock to expire.");
	while(getTimelock())
	{
		usleep(100000);
		if(++i > 10)
		{
			printf(".");
			fflush(stdout);
			i = 0;
		}
	}
	printf("\n");
	gettimeofday(&tvTimerB, NULL);
	timeval_subtract(&tvDiff, &tvTimerB, &tvTimerA);

	assertFloat("TIMELOCK timer", (float)tvDiff.tv_sec + (float)tvDiff.tv_usec/1000000, timelockTime, 1);

	switch(secondBlock)
	{
		case OCC_WEST_MAIN:
			assertAspect("Second train gets green", SIGNAL_EB_MAIN, ASPECT_GREEN);
			break;
		case OCC_WEST_SIDING:
			assertAspect("Second train gets green", SIGNAL_EB_SIDING, ASPECT_YELLOW);
			break;
		case OCC_NORTH_MAIN:
			assertAspect("Second train gets green", SIGNAL_SB_MAIN, ASPECT_GREEN);
			break;
		case OCC_NORTH_SIDING:
			assertAspect("Second train gets green", SIGNAL_SB_SIDING, ASPECT_YELLOW);
			break;
		case OCC_EAST_MAIN:
			assertAspect("Second train gets green", SIGNAL_WB_TOP, ASPECT_GREEN);
			assertAspect("...over red", SIGNAL_WB_BOTTOM, ASPECT_RED);
			break;
		case OCC_SOUTH_MAIN:
			assertAspect("Second train gets green", SIGNAL_NB_TOP, ASPECT_GREEN);
			assertAspect("...over red", SIGNAL_NB_BOTTOM, ASPECT_RED);
			break;
	}

	END_TEST;
}

void testTimelockTurnout(int firstBlock)
{
	int i;
	struct timeval tvDiff, tvTimerA, tvTimerB;

	START_TEST;
	
	printf("\n");
	printf("--------------------------------------------------------------------------------\n");
	printf("Timelock Timer with Turnout Change\n");
	printf("--------------------------------------------------------------------------------\n");

	clearAll();
	
	turnouts = 0;

	occupancy |= firstBlock;
	sendCommand();
	switch(firstBlock)
	{
		case OCC_WEST_MAIN:
			assertAspect("Train arrives, gets green", SIGNAL_EB_MAIN, ASPECT_GREEN);
			break;
		case OCC_WEST_SIDING:
			assertAspect("Train arrives, gets yellow", SIGNAL_EB_SIDING, ASPECT_YELLOW);
			break;
		case OCC_EAST_MAIN:
			assertAspect("Train arrives, gets green", SIGNAL_WB_TOP, ASPECT_GREEN);
			assertAspect("...over red", SIGNAL_WB_BOTTOM, ASPECT_RED);
			break;
		case OCC_NORTH_MAIN:
			assertAspect("Train arrives, gets green", SIGNAL_SB_MAIN, ASPECT_GREEN);
			break;
		case OCC_NORTH_SIDING:
			assertAspect("Train arrives, gets yellow", SIGNAL_SB_SIDING, ASPECT_YELLOW);
			break;
		case OCC_SOUTH_MAIN:
			assertAspect("Train arrives, gets green", SIGNAL_NB_TOP, ASPECT_GREEN);
			assertAspect("...over red", SIGNAL_NB_BOTTOM, ASPECT_RED);
			break;
	}

	if(firstBlock == OCC_EAST_MAIN)
	{
		turnouts |= TURNOUT_WEST;
	}
	
	if(firstBlock == OCC_SOUTH_MAIN)
	{
		turnouts |= TURNOUT_NORTH;
	}
	sendCommand();

	gettimeofday(&tvTimerA, NULL);  // Timelock start

	switch(firstBlock)
	{
		case OCC_EAST_MAIN:
			assertAspect("Turnout changes, signal goes to red", SIGNAL_WB_TOP, ASPECT_RED);
			assertAspect("...over red", SIGNAL_WB_BOTTOM, ASPECT_RED);
			break;
		case OCC_SOUTH_MAIN:
			assertAspect("Turnout changes, signal goes to red", SIGNAL_NB_TOP, ASPECT_RED);
			assertAspect("...over red", SIGNAL_NB_BOTTOM, ASPECT_RED);
			break;
	}

	waitForNewPacket();

	i = 0;
	printf("Wait for timelock to expire.");
	while(getTimelock())
	{
		usleep(100000);
		if(++i > 10)
		{
			printf(".");
			fflush(stdout);
			i = 0;
		}
	}
	printf("\n");
	gettimeofday(&tvTimerB, NULL);
	timeval_subtract(&tvDiff, &tvTimerB, &tvTimerA);

	assertFloat("TIMELOCK timer", (float)tvDiff.tv_sec + (float)tvDiff.tv_usec/1000000, timelockTime, 1);

	switch(firstBlock)
	{
		case OCC_EAST_MAIN:
			assertAspect("Signal goes to red", SIGNAL_WB_TOP, ASPECT_RED);
			assertAspect("...over yellow", SIGNAL_WB_BOTTOM, ASPECT_YELLOW);
			break;
		case OCC_SOUTH_MAIN:
			assertAspect("Turnout changes, signal goes to red", SIGNAL_NB_TOP, ASPECT_RED);
			assertAspect("...over yellow", SIGNAL_NB_BOTTOM, ASPECT_YELLOW);
			break;
	}

	turnouts = 0;
	sendCommand();

	gettimeofday(&tvTimerA, NULL);  // Timelock start

	switch(firstBlock)
	{
		case OCC_EAST_MAIN:
			assertAspect("Turnout changes, signal goes to red", SIGNAL_WB_TOP, ASPECT_RED);
			assertAspect("...over red", SIGNAL_WB_BOTTOM, ASPECT_RED);
			break;
		case OCC_SOUTH_MAIN:
			assertAspect("Turnout changes, signal goes to red", SIGNAL_NB_TOP, ASPECT_RED);
			assertAspect("...over red", SIGNAL_NB_BOTTOM, ASPECT_RED);
			break;
	}

	waitForNewPacket();

	i = 0;
	printf("Wait for timelock to expire.");
	while(getTimelock())
	{
		usleep(100000);
		if(++i > 10)
		{
			printf(".");
			fflush(stdout);
			i = 0;
		}
	}
	printf("\n");
	gettimeofday(&tvTimerB, NULL);
	timeval_subtract(&tvDiff, &tvTimerB, &tvTimerA);

	assertFloat("TIMELOCK timer", (float)tvDiff.tv_sec + (float)tvDiff.tv_usec/1000000, timelockTime, 1);

	switch(firstBlock)
	{
		case OCC_EAST_MAIN:
			assertAspect("Signal goes to green", SIGNAL_WB_TOP, ASPECT_GREEN);
			assertAspect("...over red", SIGNAL_WB_BOTTOM, ASPECT_RED);
			break;
		case OCC_SOUTH_MAIN:
			assertAspect("Turnout changes, signal goes to green", SIGNAL_NB_TOP, ASPECT_GREEN);
			assertAspect("...over red", SIGNAL_NB_BOTTOM, ASPECT_RED);
			break;
	}

	END_TEST;
}

void testLockout(int firstBlock, int secondBlock, int trainPresent)
{
	int i;
	struct timeval tvDiff, tvTimerA, tvTimerB;

	START_TEST;
	
	printf("\n");
	printf("--------------------------------------------------------------------------------\n");
	printf("Lockout Timer\n");
	printf("--------------------------------------------------------------------------------\n");

	clearAll();
	
	turnouts = 0;

	if((firstBlock == OCC_WEST_SIDING) || (secondBlock == OCC_WEST_SIDING))
	{
		turnouts |= TURNOUT_WEST;
	}
	
	if((firstBlock == OCC_NORTH_SIDING) || (secondBlock == OCC_NORTH_SIDING))
	{
		turnouts |= TURNOUT_NORTH;
	}

	occupancy |= firstBlock;
	sendCommand();
	switch(firstBlock)
	{
		case OCC_WEST_MAIN:
			assertAspect("Train arrives, gets green", SIGNAL_EB_MAIN, ASPECT_GREEN);
			break;
		case OCC_WEST_SIDING:
			assertAspect("Train arrives, gets yellow", SIGNAL_EB_SIDING, ASPECT_YELLOW);
			break;
		case OCC_EAST_MAIN:
			if(turnouts)
			{
				assertAspect("Train arrives, gets red", SIGNAL_WB_TOP, ASPECT_RED);
				assertAspect("...over yellow", SIGNAL_WB_BOTTOM, ASPECT_YELLOW);
			}
			else
			{
				assertAspect("Train arrives, gets green", SIGNAL_WB_TOP, ASPECT_GREEN);
				assertAspect("...over red", SIGNAL_WB_BOTTOM, ASPECT_RED);
			}
			break;
		case OCC_NORTH_MAIN:
			assertAspect("Train arrives, gets green", SIGNAL_SB_MAIN, ASPECT_GREEN);
			break;
		case OCC_NORTH_SIDING:
			assertAspect("Train arrives, gets yellow", SIGNAL_SB_SIDING, ASPECT_YELLOW);
			break;
		case OCC_SOUTH_MAIN:
			if(turnouts)
			{
				assertAspect("Train arrives, gets red", SIGNAL_NB_TOP, ASPECT_RED);
				assertAspect("...over yellow", SIGNAL_NB_BOTTOM, ASPECT_YELLOW);
			}
			else
			{
				assertAspect("Train arrives, gets green", SIGNAL_NB_TOP, ASPECT_GREEN);
				assertAspect("...over red", SIGNAL_NB_BOTTOM, ASPECT_RED);
			}
			break;
	}

	occupancy |= OCC_INTERLOCKING;
	sendCommand();

	occupancy = OCC_INTERLOCKING;
	sendCommand();

	occupancy |= secondBlock;
	sendCommand();

	occupancy = secondBlock;
	sendCommand();
	
	if(!trainPresent)
	{
		occupancy = 0;
		sendCommand();
	}

	switch(secondBlock)
	{
		case OCC_WEST_MAIN:
			assertAspect("Train proceeds, opposing signal is red", SIGNAL_EB_MAIN, ASPECT_RED);
			break;
		case OCC_WEST_SIDING:
			assertAspect("Train proceeds, opposing signal is red", SIGNAL_EB_SIDING, ASPECT_RED);
			break;
		case OCC_EAST_MAIN:
			assertAspect("Train proceeds, opposing signal is red", SIGNAL_WB_TOP, ASPECT_RED);
			assertAspect("...over red", SIGNAL_WB_BOTTOM, ASPECT_RED);
			break;
		case OCC_NORTH_MAIN:
			assertAspect("Train proceeds, opposing signal is red", SIGNAL_SB_MAIN, ASPECT_RED);
			break;
		case OCC_NORTH_SIDING:
			assertAspect("Train proceeds, opposing signal is red", SIGNAL_SB_SIDING, ASPECT_RED);
			break;
		case OCC_SOUTH_MAIN:
			assertAspect("Train proceeds, opposing signal is red", SIGNAL_NB_TOP, ASPECT_RED);
			assertAspect("...over red", SIGNAL_NB_BOTTOM, ASPECT_RED);
			break;
	}

	waitForNewPacket();

	printf("Wait for debounce and timelock to expire.");
	while(getTimelock() || getDebounce())
	{
		usleep(100000);
		if(++i > 10)
		{
			printf(".");
			fflush(stdout);
			i = 0;
		}
	}
	printf("\n");

	gettimeofday(&tvTimerA, NULL);  // Lockout start

	i = 0;
	printf("Wait for lockout to expire.");

	waitForNewPacket();

	int lockoutDir;
	if((secondBlock == OCC_NORTH_MAIN) || (secondBlock == OCC_NORTH_SIDING))
		lockoutDir = DIR_NORTH;
	if(secondBlock == OCC_SOUTH_MAIN)
		lockoutDir = DIR_SOUTH;
	if((secondBlock == OCC_WEST_MAIN) || (secondBlock == OCC_WEST_SIDING))
		lockoutDir = DIR_WEST;
	if(secondBlock == OCC_EAST_MAIN)
		lockoutDir = DIR_EAST;
	while(getLockout(lockoutDir))
	{
		usleep(100000);
		if(++i > 10)
		{
			printf(".");
			fflush(stdout);
			i = 0;
		}
	}
	printf("\n");
	gettimeofday(&tvTimerB, NULL);
	timeval_subtract(&tvDiff, &tvTimerB, &tvTimerA);

	assertFloat("LOCKOUT timer", (float)tvDiff.tv_sec + (float)tvDiff.tv_usec/1000000, lockoutTime, 1);

	if(trainPresent)
	{
		switch(secondBlock)
		{
			case OCC_WEST_MAIN:
				assertAspect("Signal goes green", SIGNAL_EB_MAIN, ASPECT_GREEN);
				break;
			case OCC_WEST_SIDING:
				assertAspect("Signal goes yellow", SIGNAL_EB_SIDING, ASPECT_YELLOW);
				break;
			case OCC_EAST_MAIN:
				if(firstBlock == OCC_WEST_MAIN)
				{
					assertAspect("Signal goes green", SIGNAL_WB_TOP, ASPECT_GREEN);
					assertAspect("...over red", SIGNAL_WB_BOTTOM, ASPECT_RED);
				}
				else
				{
					assertAspect("Signal goes red", SIGNAL_WB_TOP, ASPECT_RED);
					assertAspect("...over yellow", SIGNAL_WB_BOTTOM, ASPECT_YELLOW);
				}
				break;
			case OCC_NORTH_MAIN:
				assertAspect("Signal goes green", SIGNAL_SB_MAIN, ASPECT_GREEN);
				break;
			case OCC_NORTH_SIDING:
				assertAspect("Signal goes yellow", SIGNAL_SB_SIDING, ASPECT_YELLOW);
				break;
			case OCC_SOUTH_MAIN:
				if(firstBlock == OCC_NORTH_MAIN)
				{
					assertAspect("Signal goes green", SIGNAL_NB_TOP, ASPECT_GREEN);
					assertAspect("...over red", SIGNAL_NB_BOTTOM, ASPECT_RED);
				}
				else
				{
					assertAspect("Signal goes red", SIGNAL_NB_TOP, ASPECT_RED);
					assertAspect("...over yellow", SIGNAL_NB_BOTTOM, ASPECT_YELLOW);
				}
				break;
		}
	}
	else
	{
		switch(secondBlock)
		{
			case OCC_WEST_MAIN:
				assertAspect("Signal still red", SIGNAL_EB_MAIN, ASPECT_RED);
				break;
			case OCC_WEST_SIDING:
				assertAspect("Signal still red", SIGNAL_EB_SIDING, ASPECT_RED);
				break;
			case OCC_EAST_MAIN:
				assertAspect("Signal still red", SIGNAL_WB_TOP, ASPECT_RED);
				assertAspect("...over red", SIGNAL_WB_BOTTOM, ASPECT_RED);
				break;
			case OCC_NORTH_MAIN:
				assertAspect("Signal still red", SIGNAL_SB_MAIN, ASPECT_RED);
				break;
			case OCC_NORTH_SIDING:
				assertAspect("Signal still red", SIGNAL_SB_SIDING, ASPECT_RED);
				break;
			case OCC_SOUTH_MAIN:
				assertAspect("Signal still red", SIGNAL_NB_TOP, ASPECT_RED);
				assertAspect("...over red", SIGNAL_NB_BOTTOM, ASPECT_RED);
				break;
		}
	}

	if( trainPresent && ((DIR_WEST == lockoutDir) || (DIR_NORTH == lockoutDir)) )
	{
		if(turnouts)
		{
			turnouts = 0;
		}
		else if(secondBlock == OCC_WEST_MAIN)
		{
			turnouts |= TURNOUT_WEST;
		}
		else if(secondBlock == OCC_NORTH_MAIN)
		{
			turnouts |= TURNOUT_NORTH;
		}
		sendCommand();

		switch(secondBlock)
		{
			case OCC_WEST_MAIN:
				assertAspect("Change switch, signal is red", SIGNAL_EB_MAIN, ASPECT_RED);
				break;
			case OCC_WEST_SIDING:
				assertAspect("Change switch, signal is red", SIGNAL_EB_SIDING, ASPECT_RED);
				break;
			case OCC_NORTH_MAIN:
				assertAspect("Change switch, signal is red", SIGNAL_SB_MAIN, ASPECT_RED);
				break;
			case OCC_NORTH_SIDING:
				assertAspect("Change switch, signal is red", SIGNAL_SB_SIDING, ASPECT_RED);
				break;
		}
	}

	END_TEST;
}

void testTimeout(int firstBlock, int turnout)
{
	int i;
	struct timeval tvDiff, tvTimerA, tvTimerB;

	START_TEST;
	
	printf("\n");
	printf("--------------------------------------------------------------------------------\n");
	printf("Timeout Timer\n");
	printf("--------------------------------------------------------------------------------\n");

	clearAll();
	
	turnouts = turnout;

	occupancy |= firstBlock;
	sendCommand();
	switch(firstBlock)
	{
		case OCC_WEST_MAIN:
			assertAspect("Train arrives, gets green", SIGNAL_EB_MAIN, ASPECT_GREEN);
			break;
		case OCC_WEST_SIDING:
			assertAspect("Train arrives, gets yellow", SIGNAL_EB_SIDING, ASPECT_YELLOW);
			break;
		case OCC_EAST_MAIN:
			if(turnouts)
			{
				assertAspect("Train arrives, gets red", SIGNAL_WB_TOP, ASPECT_RED);
				assertAspect("...over yellow", SIGNAL_WB_BOTTOM, ASPECT_YELLOW);
			}
			else
			{
				assertAspect("Train arrives, gets green", SIGNAL_WB_TOP, ASPECT_GREEN);
				assertAspect("...over red", SIGNAL_WB_BOTTOM, ASPECT_RED);
			}
			break;
		case OCC_NORTH_MAIN:
			assertAspect("Train arrives, gets green", SIGNAL_SB_MAIN, ASPECT_GREEN);
			break;
		case OCC_NORTH_SIDING:
			assertAspect("Train arrives, gets yellow", SIGNAL_SB_SIDING, ASPECT_YELLOW);
			break;
		case OCC_SOUTH_MAIN:
			if(turnouts)
			{
				assertAspect("Train arrives, gets red", SIGNAL_NB_TOP, ASPECT_RED);
				assertAspect("...over yellow", SIGNAL_NB_BOTTOM, ASPECT_YELLOW);
			}
			else
			{
				assertAspect("Train arrives, gets green", SIGNAL_NB_TOP, ASPECT_GREEN);
				assertAspect("...over red", SIGNAL_NB_BOTTOM, ASPECT_RED);
			}
			break;
	}

	occupancy = 0;
	sendCommand();

	gettimeofday(&tvTimerA, NULL);  // Timeout start

	switch(firstBlock)
	{
		case OCC_WEST_MAIN:
			assertAspect("Train leaves, still green", SIGNAL_EB_MAIN, ASPECT_GREEN);
			break;
		case OCC_WEST_SIDING:
			assertAspect("Train leaves, still yellow", SIGNAL_EB_SIDING, ASPECT_YELLOW);
			break;
		case OCC_EAST_MAIN:
			if(turnouts)
			{
				assertAspect("Train leaves, still red", SIGNAL_WB_TOP, ASPECT_RED);
				assertAspect("...over yellow", SIGNAL_WB_BOTTOM, ASPECT_YELLOW);
			}
			else
			{
				assertAspect("Train leaves, still green", SIGNAL_WB_TOP, ASPECT_GREEN);
				assertAspect("...over red", SIGNAL_WB_BOTTOM, ASPECT_RED);
			}
			break;
		case OCC_NORTH_MAIN:
			assertAspect("Train leaves, still green", SIGNAL_SB_MAIN, ASPECT_GREEN);
			break;
		case OCC_NORTH_SIDING:
			assertAspect("Train leaves, still yellow", SIGNAL_SB_SIDING, ASPECT_YELLOW);
			break;
		case OCC_SOUTH_MAIN:
			if(turnouts)
			{
				assertAspect("Train leaves, still red", SIGNAL_NB_TOP, ASPECT_RED);
				assertAspect("...over yellow", SIGNAL_NB_BOTTOM, ASPECT_YELLOW);
			}
			else
			{
				assertAspect("Train leaves, still green", SIGNAL_NB_TOP, ASPECT_GREEN);
				assertAspect("...over red", SIGNAL_NB_BOTTOM, ASPECT_RED);
			}
			break;
	}

	i = 0;
	printf("Wait for timeout to expire.");

	waitForNewPacket();

	int timeoutDir;
	if((firstBlock == OCC_NORTH_MAIN) || (firstBlock == OCC_NORTH_SIDING))
		timeoutDir = DIR_NORTH;
	if(firstBlock == OCC_SOUTH_MAIN)
		timeoutDir = DIR_SOUTH;
	if((firstBlock == OCC_WEST_MAIN) || (firstBlock == OCC_WEST_SIDING))
		timeoutDir = DIR_WEST;
	if(firstBlock == OCC_EAST_MAIN)
		timeoutDir = DIR_EAST;
	while(getTimeout(timeoutDir))
	{
		usleep(100000);
		if(++i > 10)
		{
			printf(".");
			fflush(stdout);
			i = 0;
		}
	}
	printf("\n");
	gettimeofday(&tvTimerB, NULL);
	timeval_subtract(&tvDiff, &tvTimerB, &tvTimerA);

	assertFloat("TIMEOUT timer", (float)tvDiff.tv_sec + (float)tvDiff.tv_usec/1000000, timeoutTime, 1);

	switch(firstBlock)
	{
		case OCC_WEST_MAIN:
			assertAspect("Signal red", SIGNAL_EB_MAIN, ASPECT_RED);
			break;
		case OCC_WEST_SIDING:
			assertAspect("Signal red", SIGNAL_EB_SIDING, ASPECT_RED);
			break;
		case OCC_EAST_MAIN:
			assertAspect("Signal red", SIGNAL_WB_TOP, ASPECT_RED);
			assertAspect("...over red", SIGNAL_WB_BOTTOM, ASPECT_RED);
			break;
		case OCC_NORTH_MAIN:
			assertAspect("Signal red", SIGNAL_SB_MAIN, ASPECT_RED);
			break;
		case OCC_NORTH_SIDING:
			assertAspect("Signal red", SIGNAL_SB_SIDING, ASPECT_RED);
			break;
		case OCC_SOUTH_MAIN:
			assertAspect("Signal red", SIGNAL_NB_TOP, ASPECT_RED);
			assertAspect("...over red", SIGNAL_NB_BOTTOM, ASPECT_RED);
			break;
	}

	END_TEST;
}


void testDebounceTimer(void)
{
	int i;
	struct timeval tvDiff, tvTimerA, tvTimerB;

	START_TEST;
	
	printf("\n");
	printf("--------------------------------------------------------------------------------\n");
	printf("Debounce Timer\n");
	printf("--------------------------------------------------------------------------------\n");

	clearAll();

	occupancy |= OCC_WEST_MAIN;
	sendCommand();
	assertAspect("Eastbound train arrives, gets green", 0, ASPECT_GREEN);

	occupancy |= OCC_INTERLOCKING;
	sendCommand();
	gettimeofday(&tvTimerA, NULL);  // Timelock start
	assertAspect("Enters interlocking, red signal", 0, ASPECT_RED);

	occupancy &= ~OCC_WEST_MAIN;
	sendCommand();
	assertAspect("Occupies only interlocking, red signal", 0, ASPECT_RED);

	occupancy |= OCC_NORTH_MAIN;
	sendCommand();
	assertAspect("Southbound train arrives, gets red", 4, ASPECT_RED);

	occupancy |= OCC_EAST_MAIN;
	sendCommand();
	assertAspect("Eastbound continues, red signal", 0, ASPECT_RED);
	assertAspect("Southbound still red", 4, ASPECT_RED);

	waitForNewPacket();

	i = 0;
	printf("Wait for timelock to expire.");
	while(getTimelock())
	{
		usleep(100000);
		if(++i > 10)
		{
			printf(".");
			fflush(stdout);
			i = 0;
		}
	}
	printf("\n");
	gettimeofday(&tvTimerB, NULL);
	timeval_subtract(&tvDiff, &tvTimerB, &tvTimerA);

	assertFloat("TIMELOCK timer", (float)tvDiff.tv_sec + (float)tvDiff.tv_usec/1000000, timelockTime, 1);

	occupancy &= ~OCC_INTERLOCKING;
	sendCommand();
	gettimeofday(&tvTimerA, NULL);  // Debounce start
	assertAspect("Eastbound clears interlocking, red signal", 0, ASPECT_RED);
	assertAspect("Southbound still red", 4, ASPECT_RED);

	waitForNewPacket();

	i = 0;
	printf("Wait for debounce to expire.");
	while(getDebounce())
	{
		usleep(100000);
		if(++i > 10)
		{
			printf(".");
			fflush(stdout);
			i = 0;
		}
	}
	printf("\n");
	gettimeofday(&tvTimerB, NULL);
	timeval_subtract(&tvDiff, &tvTimerB, &tvTimerA);

	assertFloat("DEBOUNCE timer", (float)tvDiff.tv_sec + (float)tvDiff.tv_usec/1000000, debounceTime, 1);

	assertAspect("Southbound now green", 4, ASPECT_GREEN);

	END_TEST;
}



void testBogusInterlocking(void)
{
	int i;
	
	START_TEST;
	
	printf("\n");
	printf("--------------------------------------------------------------------------------\n");
	printf("Bogus Interlocking\n");
	printf("--------------------------------------------------------------------------------\n");

	clearAll();

	printf("Interlocking occupancy triggered.\n");
	occupancy |= OCC_INTERLOCKING;
	sendCommand();

	occupancy |= OCC_EAST_MAIN;
	sendCommand();
	assertAspect("Westbound arrives to red", SIGNAL_WB_TOP, ASPECT_RED);
	assertAspect("...over red", SIGNAL_WB_BOTTOM, ASPECT_RED);

	printf("Interlocking occupancy disappears.\n");
	occupancy &= ~OCC_INTERLOCKING;
	sendCommand();

	waitForNewPacket();

	printf("Wait for debounce to expire.");
	while(getDebounce())
	{
		usleep(100000);
		if(++i > 10)
		{
			printf(".");
			fflush(stdout);
			i = 0;
		}
	}
	printf("\n");

	assertAspect("Westbound gets green", SIGNAL_WB_TOP, ASPECT_GREEN);
	assertAspect("...over red", SIGNAL_WB_BOTTOM, ASPECT_RED);

	END_TEST;
}


void sendFastTimePacket(uint16_t time, uint16_t scale, uint8_t flags)
{
	char pkt[256];
	FILE *fptr;

	uint8_t hour = time / 60;
	uint8_t minute = time % 60;
	uint8_t second = 0;

	snprintf(pkt, sizeof(pkt), "%02X->%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X", SRC_ADDR, 0xFF, 'T', 0, 0, 0, flags, hour, minute, second, (scale >> 8)&0xFF, scale&0xFF);

	textcolor(RESET, YELLOW, BLACK);
	printf("Sending time packet [%s] >> %02d:%02d:%02d %2.1fx\n", pkt, hour, minute, second, scale/10.0);
	textcolor(RESET, WHITE, BLACK);

	if( NULL != (fptr = fopen(MRBFS_TXPKT, "w")) )
	{
		fprintf(fptr, "%s\n", pkt);
		fclose(fptr);
	}
	else
	{
		textcolor(RESET, WHITE, RED);
		printf("Failed to open %s!\n", MRBFS_TXPKT);
		textcolor(RESET, WHITE, BLACK);
	}
}


void testSimulatedTrain(int num, SimTrain sim, TriggerType trigger, int writeConfiguration)
{
	int i;
	SimTrain nullSim;
	uint16_t time;
	float elapsedTime;
	struct timeval tvDiff, tvTimerA, tvTimerB;
	
	START_TEST;
	
	// Not enabled, otherwise null
	nullSim.flags = 0;
	nullSim.direction = 0;
	nullSim.time = 0;
	nullSim.totalTime = 0;
	nullSim.approachTime = 0;
	
	printf("\n");
	printf("--------------------------------------------------------------------------------\n");
	printf("Simulated Train #%d\n", num);
	printf("--------------------------------------------------------------------------------\n");
	printf("Direction = %d\n", sim.direction);
	printf("Sound     = %d\n", sim.flags & SIM_TRAIN_FLAGS_SOUND);
	printf("Trigger   = ");
	switch(trigger)
	{
		case TRIGGER_DEAD_RECKONING:
			printf("Dead Reckoning\n");
			break;
		case TRIGGER_PACKET:
			printf("Packet\n");
			break;
		case TRIGGER_WINDOW:
			printf("Window\n");
			break;
		case TRIGGER_SKIP:
			printf("Skip\n");
			break;
	}
	printf("--------------------------------------------------------------------------------\n");

	if(writeConfiguration)
	{
		for(i=0; i<NUM_SIM_TRAINS; i++)
		{
			if(i == num)
			{
				writeEepromSim(i, sim);
			}
			else
			{
				writeEepromSim(i, nullSim);
			}
		}
	}

	int deadReckoningRatio = 10;

	// Do configuration before sending time packet since EEPROM write will reset device
	if(writeConfiguration)
	{
		switch(trigger)
		{
			case TRIGGER_DEAD_RECKONING:
				setMaxDeadReckoning(((60/deadReckoningRatio) + 1)*10);  // Set dead reckoning to 1 real second longer than delay (1 fast minute) we'll be using
				break;
			case TRIGGER_PACKET:
				break;
			case TRIGGER_WINDOW:
				break;
			case TRIGGER_SKIP:
				setMaxDeadReckoning(50);  // 5 real seconds
				setSimTrainWindow(5);  // 5 fast minutes
				break;
		}
	}
	
	// Pre-Trigger
	time = sim.time;
	if(time > (23*60 + 59))
		time = 23*60 + 59;  // Clamp to max inside the test
	
	if(time < 5)
	{
		time = time + (24*60) - 5;
	}
	else
	{
		time -= 5;
	}

	sendFastTimePacket(time, 10, TIME_FLAGS_DISP_FAST);  // time - 5 minutes, 1x, Fast enabled, not hold
	sleep(1);

	switch(trigger)
	{
		case TRIGGER_DEAD_RECKONING:
			time = sim.time;
			if(time > (23*60 + 59))
				time = 23*60 + 59;  // Clamp to max inside the test
			// Set time packet to 1 minute before scheduled time
			if(time < 1)
			{
				time = time + (24*60) - 1;
			}
			else
			{
				time -= 1;
			}
			sendFastTimePacket(time, 10*deadReckoningRatio, TIME_FLAGS_DISP_FAST);  // time - 1 minutes, 10x, Fast enabled, not hold
			gettimeofday(&tvTimerA, NULL);
			printf("Waiting for trigger time...\n");
			while(STATE_IDLE == getState(sim.direction)) {}
			gettimeofday(&tvTimerB, NULL);
			timeval_subtract(&tvDiff, &tvTimerB, &tvTimerA);
			assertFloat("Dead Reckoning Time", (float)tvDiff.tv_sec + (float)tvDiff.tv_usec/1000000, 60/deadReckoningRatio, 1);
			break;
		case TRIGGER_PACKET:
			time = sim.time;
			if(time > (23*60 + 59))
				time = 23*60 + 59;  // Clamp to max inside the test
			sendFastTimePacket(time, 10, TIME_FLAGS_DISP_FAST);
			break;
		case TRIGGER_WINDOW:
			setSimTrainWindow(5);  // Fast minutes
			time = sim.time;
			if(time > (23*60 + 59))
				time = 23*60 + 59;  // Clamp to max inside the test
			time += 2;
			if(time >= (24*60))
			{
				time -= (24*60);
			}
			sendFastTimePacket(time, 10, TIME_FLAGS_DISP_FAST);
			break;
		case TRIGGER_SKIP:
			time = sim.time;
			if(time > (23*60 + 59))
				time = 23*60 + 59;  // Clamp to max inside the test
			time += 6;  // Trigger 6 minutes after sim start time
			if(time >= (24*60))
			{
				time -= (24*60);
			}
			sendFastTimePacket(time, 10, TIME_FLAGS_DISP_FAST);
			break;
	}

	// Triggered Time
	gettimeofday(&tvTimerA, NULL);

	int track = (0 != turnouts);
	if(TRIGGER_SKIP != trigger)
	{
		assertAspect("Simulated train arrives, gets proceed indication (main/top)", sim.direction*2, track?ASPECT_RED:ASPECT_GREEN);
		assertAspect("Simulated train arrives, gets proceed indication (siding/bottom)", sim.direction*2+1, track?ASPECT_YELLOW:ASPECT_RED);

		assertInt("Sound0", getSound(0), (sim.flags&0x01)?1:0);
		assertInt("Sound1", getSound(1), (sim.flags&0x02)?1:0);

		printf("Waiting for approach time to expire...\n");
		while(STATE_OCCUPIED != getState(sim.direction)) {}

		gettimeofday(&tvTimerB, NULL);
		timeval_subtract(&tvDiff, &tvTimerB, &tvTimerA);
		if(sim.approachTime > sim.totalTime)
			assertFloat("Approach Time", (float)tvDiff.tv_sec + (float)tvDiff.tv_usec/1000000, sim.totalTime, 1);  // Clamp to totalTime
		else
			assertFloat("Approach Time", (float)tvDiff.tv_sec + (float)tvDiff.tv_usec/1000000, sim.approachTime, 1);

		assertAspect("Simulated train occupies interlocking (main/top)", sim.direction*2, ASPECT_RED);
		assertAspect("Simulated train occupies interlocking (siding/bottom)", sim.direction*2+1, ASPECT_RED);

		printf("Waiting for total time to expire...\n");
		while(STATE_IDLE != getState(sim.direction)) {}

		gettimeofday(&tvTimerB, NULL);
		timeval_subtract(&tvDiff, &tvTimerB, &tvTimerA);
		assertFloat("Total Time", (float)tvDiff.tv_sec + (float)tvDiff.tv_usec/1000000, sim.totalTime, (sim.approachTime>=sim.totalTime)?1.5:1.0);
		// Give a little extra time tolerance if approach == total

		assertInt("Sound0", getSound(0), 0);
		assertInt("Sound1", getSound(1), 0);
	}
	else
	{
		// Skip trigger
		do
		{
			gettimeofday(&tvTimerB, NULL);
			timeval_subtract(&tvDiff, &tvTimerB, &tvTimerA);
			elapsedTime = (float)tvDiff.tv_sec + (float)tvDiff.tv_usec/1000000;
		} while( (elapsedTime < (getMaxDeadReckoning() / 10.0)) && (STATE_IDLE == getState(sim.direction)) );

		assertFloat("Dead Reckoning Run Out", elapsedTime, getMaxDeadReckoning()/10, 1);
	}

	clearAll();

	END_TEST;
}

void runAllBasicSimulatedTrains(void)
{
	SimTrain simTrain;
	simTrain.flags = 0;
	simTrain.direction = 0;
	simTrain.time = 5*60 + 0;  // 5:00 AM
	simTrain.totalTime = 10;
	simTrain.approachTime = 5;

	printf("\n");
	printf("--------------------------------------------------------------------------------\n");
	printf("All Simulate Trains\n");
	printf("--------------------------------------------------------------------------------\n");

	for(int i=0; i<NUM_SIM_TRAINS; i++)
	{
		TriggerType t;
		switch(i%3)
		{
			case 0:
				t = TRIGGER_PACKET;
				break;
			case 1:
				t = TRIGGER_WINDOW;
				break;
			case 2:
				t = TRIGGER_DEAD_RECKONING;
				break;
		}
		
		simTrain.direction = i % 4;
		
		uint8_t sound;
		if( (i / 4) % 2 )
		{
			sound = ~(i % 4);
		}
		else
		{
			sound = i % 4;
		}
		simTrain.flags = SIM_TRAIN_FLAGS_ENABLE | (sound & SIM_TRAIN_FLAGS_SOUND);

		occupancy = 0;
		if( (i / 4) % 2 )
			turnouts = TURNOUT_WEST | TURNOUT_NORTH;
		else
			turnouts = 0;
		sendCommand();
		RUN_TEST(testSimulatedTrain(i, simTrain, t, 1));
	}		
}

void testSimulatedTrainSkipTime(void)
{
	SimTrain simTrain;
	simTrain.flags = SIM_TRAIN_FLAGS_ENABLE;
	simTrain.direction = 0;
	simTrain.time = 5*60 + 0;
	simTrain.totalTime = 10;
	simTrain.approachTime = 5;

	printf("\n");
	printf("--------------------------------------------------------------------------------\n");
	printf("Skip Simulated Train\n");
	printf("--------------------------------------------------------------------------------\n");

	testSimulatedTrain(0, simTrain, TRIGGER_SKIP, 1);
}

void testSimulatedTrainRetrigger(void)
{
	uint8_t hour = 16;
	uint8_t minute = 25;

	printf("\n");
	printf("--------------------------------------------------------------------------------\n");
	printf("Retrigger Train\n");
	printf("--------------------------------------------------------------------------------\n");

	SimTrain simTrain;
	simTrain.flags = SIM_TRAIN_FLAGS_ENABLE;
	simTrain.direction = 0;
	simTrain.time = hour*60 + minute;
	simTrain.totalTime = 10;
	simTrain.approachTime = 5;
	
	testSimulatedTrain(0, simTrain, TRIGGER_DEAD_RECKONING, 1);
	
	sendFastTimePacket((hour+1)*60 + minute, 10, TIME_FLAGS_DISP_FAST); // Advance past scheduled time
	
	sleep(1);

	sendFastTimePacket((hour-1)*60 + minute, 10, TIME_FLAGS_DISP_FAST); // Go 1 hour before scheduled time
	
	testSimulatedTrain(0, simTrain, TRIGGER_DEAD_RECKONING, 0);  // Don't rewrite EEPROM
}

void testSimulatedTrainInvalidTime(void)
{
	SimTrain simTrain;
	simTrain.flags = SIM_TRAIN_FLAGS_ENABLE;
	simTrain.direction = 0;
	simTrain.totalTime = 10;
	simTrain.approachTime = 5;

	printf("\n");
	printf("--------------------------------------------------------------------------------\n");
	printf("Invalid Start Time");
	printf("--------------------------------------------------------------------------------\n");

	simTrain.time = 24*60;  // Just Past 23:59
	testSimulatedTrain(0, simTrain, TRIGGER_DEAD_RECKONING, 1);
	simTrain.time = 65535;  // Way Past 23:59
	testSimulatedTrain(0, simTrain, TRIGGER_DEAD_RECKONING, 1);
}

void testSimulatedTrainEnable(void)
{
	// Note: All Enabled doesn't work since it is indeterminate which direction will get the interlocking first (depends where the state machine is at the time)
	int allEnabled = 0;

	int i;
	struct timeval tvDiff, tvTimerA, tvTimerB;

	SimTrain simTrain;
	simTrain.flags = 0;
	simTrain.direction = 0;
	simTrain.time = 13*60 + 0;
	simTrain.totalTime = 15;
	simTrain.approachTime = 10;

	SimTrain nullSim;
	nullSim.flags = 0;
	nullSim.direction = 0;
	nullSim.time = 0;
	nullSim.totalTime = 0;
	nullSim.approachTime = 0;
	
	int enabledDirection = 2;  // Only enable train #2 (north)
	
	START_TEST;
	
	printf("\n");
	printf("--------------------------------------------------------------------------------\n");
	printf("Simulated Train Enable (%s)\n", allEnabled?"All Enabled":"One Enabled");
	printf("--------------------------------------------------------------------------------\n");

	for(i=0; i<NUM_SIM_TRAINS; i++)
	{
		if(i < 4)
		{
			simTrain.direction = i;
			simTrain.flags = i;
			if( allEnabled || (enabledDirection == i) )
				simTrain.flags |= SIM_TRAIN_FLAGS_ENABLE;
			writeEepromSim(i, simTrain);
		}
		else
		{
			writeEepromSim(i, nullSim);
		}
	}

	// Pre-Trigger
	sendFastTimePacket(simTrain.time - 5, 10, TIME_FLAGS_DISP_FAST);  // time - 5 minutes, 1x, Fast enabled, not hold
	sleep(1);
	sendFastTimePacket(simTrain.time, 10, TIME_FLAGS_DISP_FAST);

	// Triggered Time
	gettimeofday(&tvTimerA, NULL);

	for(i=0; i<4; i++)
	{
		if( (allEnabled & (0 == i)) || (!allEnabled & (enabledDirection == i)) )
		{
			assertAspect("Simulated train arrives, gets proceed indication (main/top)", 2*i+0, ASPECT_GREEN);
			assertAspect("Simulated train arrives, gets proceed indication (siding/bottom)", 2*i+1, ASPECT_RED);
		}
		else
		{
			char buffer[256];
			snprintf(buffer, sizeof(buffer), "Direction %d not enabled, red (main/top)", i);
			assertAspect(buffer, 2*i+0, ASPECT_RED);
			snprintf(buffer, sizeof(buffer), "Direction %d not enabled, red (siding/bottom)", i);
			assertAspect(buffer, 2*i+1, ASPECT_RED);
		}
	}

	assertInt("Sound0", getSound(0), 0);
	assertInt("Sound1", getSound(1), 1);

	printf("Waiting for approach time to expire...\n");
	while(STATE_OCCUPIED != getState(enabledDirection)) {}

	gettimeofday(&tvTimerB, NULL);
	timeval_subtract(&tvDiff, &tvTimerB, &tvTimerA);
	assertFloat("Approach Time", (float)tvDiff.tv_sec + (float)tvDiff.tv_usec/1000000, simTrain.approachTime, 1);

	assertAspect("Simulated train occupies interlocking (main/top)", enabledDirection+0, ASPECT_RED);
	assertAspect("Simulated train occupies interlocking (siding/bottom)", enabledDirection+1, ASPECT_RED);

	printf("Waiting for total time to expire...\n");
	while(STATE_IDLE != getState(enabledDirection)) {}

	gettimeofday(&tvTimerB, NULL);
	timeval_subtract(&tvDiff, &tvTimerB, &tvTimerA);
	assertFloat("Total Time", (float)tvDiff.tv_sec + (float)tvDiff.tv_usec/1000000, simTrain.totalTime, 1);

	assertInt("Sound0", getSound(0), 0);
	assertInt("Sound1", getSound(1), 0);

	clearAll();

	END_TEST;
}

void testSimulatedTrainSequence(int stompOnFirst)
{
	int i;
	struct timeval tvDiff, tvTimerA, tvTimerB;

	SimTrain simTrain[2];
	simTrain[0].flags = SIM_TRAIN_FLAGS_ENABLE;
	simTrain[0].direction = 0;
	simTrain[0].time = 9*60 + 43;
	simTrain[0].totalTime = 10;
	simTrain[0].approachTime = 5;

	simTrain[1] = simTrain[0];
	simTrain[1].time = simTrain[0].time + 5;
	simTrain[1].totalTime = 9;
	simTrain[1].approachTime = 7;
	
	SimTrain nullSim;
	nullSim.flags = 0;
	nullSim.direction = 0;
	nullSim.time = 0;
	nullSim.totalTime = 0;
	nullSim.approachTime = 0;

	START_TEST;
	
	printf("\n");
	printf("--------------------------------------------------------------------------------\n");
	printf("Simulated Train Sequence %s\n", stompOnFirst?"(Stomp On First)":"");
	printf("--------------------------------------------------------------------------------\n");

	for(i=0; i<NUM_SIM_TRAINS; i++)
	{
		if(i < 2)
		{
			writeEepromSim(i, simTrain[i]);
		}
		else
		{
			writeEepromSim(i, nullSim);
		}
	}

	// Pre-Trigger
	printf("Pre-Trigger\n");
	sendFastTimePacket(simTrain[0].time - 5, 10, TIME_FLAGS_DISP_FAST);  // time - 5 minutes, 1x, Fast enabled, not hold
	sleep(1);
	printf("Trigger First Train\n");
	sendFastTimePacket(simTrain[0].time, 10, TIME_FLAGS_DISP_FAST);

	// Triggered Time
	gettimeofday(&tvTimerA, NULL);

	assertAspect("Simulated train arrives, gets proceed indication (main/top)", 0, ASPECT_GREEN);
	assertAspect("Simulated train arrives, gets proceed indication (siding/bottom)", 1, ASPECT_RED);

	if(stompOnFirst)
	{
		// Trigger second train
		printf("Trigger Second Train\n");
		sendFastTimePacket(simTrain[1].time, 10, TIME_FLAGS_DISP_FAST);
	}

	printf("Waiting for approach time to expire...\n");
	while(STATE_OCCUPIED != getState(0)) {}

	gettimeofday(&tvTimerB, NULL);
	timeval_subtract(&tvDiff, &tvTimerB, &tvTimerA);
	assertFloat("Approach Time", (float)tvDiff.tv_sec + (float)tvDiff.tv_usec/1000000, simTrain[0].approachTime, 1);

	assertAspect("Simulated train occupies interlocking (main/top)", 0, ASPECT_RED);
	assertAspect("Simulated train occupies interlocking (siding/bottom)", 1, ASPECT_RED);

	printf("Waiting for total time to expire...\n");
	while(STATE_IDLE != getState(0)) {}

	gettimeofday(&tvTimerB, NULL);
	timeval_subtract(&tvDiff, &tvTimerB, &tvTimerA);
	assertFloat("Total Time", (float)tvDiff.tv_sec + (float)tvDiff.tv_usec/1000000, simTrain[0].totalTime, 1);

	if(stompOnFirst)
	{
		sleep(2);
		assertAspect("2nd Triggered but Ignored (main/top)", 0, ASPECT_RED);
		assertAspect("2nd Triggered but Ignored (siding/bottom)", 1, ASPECT_RED);
	}

	if(!stompOnFirst)
	{
		// Trigger second train
		printf("Trigger Second Train\n");
		sendFastTimePacket(simTrain[1].time, 10, TIME_FLAGS_DISP_FAST);
		gettimeofday(&tvTimerA, NULL);

		assertAspect("Simulated train arrives, gets proceed indication (main/top)", 0, ASPECT_GREEN);
		assertAspect("Simulated train arrives, gets proceed indication (siding/bottom)", 1, ASPECT_RED);

		printf("Waiting for approach time to expire...\n");
		while(STATE_OCCUPIED != getState(0)) {}

		gettimeofday(&tvTimerB, NULL);
		timeval_subtract(&tvDiff, &tvTimerB, &tvTimerA);
		assertFloat("Approach Time", (float)tvDiff.tv_sec + (float)tvDiff.tv_usec/1000000, simTrain[1].approachTime, 1);

		assertAspect("Simulated train occupies interlocking (main/top)", 0, ASPECT_RED);
		assertAspect("Simulated train occupies interlocking (siding/bottom)", 1, ASPECT_RED);

		printf("Waiting for total time to expire...\n");
		while(STATE_IDLE != getState(0)) {}

		gettimeofday(&tvTimerB, NULL);
		timeval_subtract(&tvDiff, &tvTimerB, &tvTimerA);
		assertFloat("Total Time", (float)tvDiff.tv_sec + (float)tvDiff.tv_usec/1000000, simTrain[1].totalTime, 1);
	}

	clearAll();

	END_TEST;
}

void testSimulatedTrainApproachLarger(void)
{
	SimTrain simTrain;
	simTrain.flags = SIM_TRAIN_FLAGS_ENABLE;
	simTrain.direction = 0;
	simTrain.time = 5*60 + 0;
	simTrain.totalTime = 5;
	simTrain.approachTime = 10;  // Larger than totalTime

	printf("\n");
	printf("--------------------------------------------------------------------------------\n");
	printf("Approach Time Larger than Total Time\n");
	printf("--------------------------------------------------------------------------------\n");

	testSimulatedTrain(0, simTrain, TRIGGER_DEAD_RECKONING, 1);
}

void testSimulatedTrainMeet(int firstDirection, int secondDirection)
{
	int i;
	struct timeval tvDiff, tvTimerA, tvTimerB, tvTimerC;

	SimTrain simTrain[2];
	simTrain[0].flags = SIM_TRAIN_FLAGS_ENABLE;
	simTrain[0].direction = firstDirection;
	simTrain[0].time = 8*60 + 0;
	simTrain[0].totalTime = 10;
	simTrain[0].approachTime = 5;

	simTrain[1] = simTrain[0];
	simTrain[1].direction = secondDirection;
	simTrain[1].time = simTrain[0].time + 1;  // 1 fast minute later
	simTrain[1].totalTime = 9;
	simTrain[1].approachTime = 7;
	
	SimTrain nullSim;
	nullSim.flags = 0;
	nullSim.direction = 0;
	nullSim.time = 0;
	nullSim.totalTime = 0;
	nullSim.approachTime = 0;

	START_TEST;
	
	printf("\n");
	printf("--------------------------------------------------------------------------------\n");
	printf("Simulated Train Meet %d -> %d\n", firstDirection, secondDirection);
	printf("--------------------------------------------------------------------------------\n");

	timelockTime = simTrain[0].totalTime - simTrain[0].approachTime + 5;  // Set longer than totalTime-approachTime (first train)
	writeEeprom(EE_TIMELOCK_SECONDS, timelockTime);

	for(i=0; i<NUM_SIM_TRAINS; i++)
	{
		if(i < 2)
		{
			writeEepromSim(i, simTrain[i]);
		}
		else
		{
			writeEepromSim(i, nullSim);
		}
	}

	// Pre-Trigger
	printf("Pre-Trigger\n");
	sendFastTimePacket(simTrain[0].time - 5, 10, TIME_FLAGS_DISP_FAST);  // time - 5 minutes, 1x, Fast enabled, not hold
	sleep(1);
	printf("Trigger First Train\n");
	sendFastTimePacket(simTrain[0].time, 10, TIME_FLAGS_DISP_FAST);

	// Triggered Time
	gettimeofday(&tvTimerA, NULL);

	assertAspect("Simulated train arrives, gets proceed indication (main/top)", 2*firstDirection+0, ASPECT_GREEN);
	assertAspect("Simulated train arrives, gets proceed indication (siding/bottom)", 2*firstDirection+1, ASPECT_RED);

	// Trigger second train
	printf("Trigger Second Train\n");
	sendFastTimePacket(simTrain[1].time, 10, TIME_FLAGS_DISP_FAST);

	printf("Waiting for approach time to expire...\n");
	while(STATE_OCCUPIED != getState(firstDirection)) {}

	gettimeofday(&tvTimerB, NULL);
	gettimeofday(&tvTimerC, NULL);
	timeval_subtract(&tvDiff, &tvTimerB, &tvTimerA);
	assertFloat("Approach Time", (float)tvDiff.tv_sec + (float)tvDiff.tv_usec/1000000, simTrain[0].approachTime, 1);

	assertAspect("Simulated train occupies interlocking (main/top)", 0, ASPECT_RED);
	assertAspect("Simulated train occupies interlocking (siding/bottom)", 1, ASPECT_RED);

	printf("Waiting for total time to expire...\n");
	while(STATE_IDLE != getState(firstDirection)) {}

	gettimeofday(&tvTimerB, NULL);
	timeval_subtract(&tvDiff, &tvTimerB, &tvTimerA);
	assertFloat("Total Time", (float)tvDiff.tv_sec + (float)tvDiff.tv_usec/1000000, simTrain[0].totalTime, 1);

	i = 0;
	printf("Wait for timelock to expire.");
	while(getTimelock())
	{
		usleep(100000);
		if(++i > 10)
		{
			printf(".");
			fflush(stdout);
			i = 0;
		}
	}
	printf("\n");
	gettimeofday(&tvTimerB, NULL);
	timeval_subtract(&tvDiff, &tvTimerB, &tvTimerC);
	assertFloat("TIMELOCK timer", (float)tvDiff.tv_sec + (float)tvDiff.tv_usec/1000000, timelockTime, 1);

	gettimeofday(&tvTimerA, NULL);

	assertAspect("Simulated train arrives, gets proceed indication (main/top)", 2*secondDirection+0, ASPECT_GREEN);
	assertAspect("Simulated train arrives, gets proceed indication (siding/bottom)", 2*secondDirection+1, ASPECT_RED);

	printf("Waiting for approach time to expire...\n");
	while(STATE_OCCUPIED != getState(secondDirection)) {}

	gettimeofday(&tvTimerB, NULL);
	timeval_subtract(&tvDiff, &tvTimerB, &tvTimerA);
	assertFloat("Approach Time", (float)tvDiff.tv_sec + (float)tvDiff.tv_usec/1000000, simTrain[1].approachTime, 1);

	assertAspect("Simulated train occupies interlocking (main/top)", 0, ASPECT_RED);
	assertAspect("Simulated train occupies interlocking (siding/bottom)", 1, ASPECT_RED);

	printf("Waiting for total time to expire...\n");
	while(STATE_IDLE != getState(secondDirection)) {}

	gettimeofday(&tvTimerB, NULL);
	timeval_subtract(&tvDiff, &tvTimerB, &tvTimerA);
	assertFloat("Total Time", (float)tvDiff.tv_sec + (float)tvDiff.tv_usec/1000000, simTrain[1].totalTime, 1);

	clearAll();

	END_TEST;
}

void testRealMeetSimulated(int realDirection, int simulatedDirection, int realBeforeSim)
{
	int i;
	struct timeval tvDiff, tvTimerA, tvTimerB, tvTimerC;

	SimTrain simTrain;
	simTrain.flags = SIM_TRAIN_FLAGS_ENABLE;
	simTrain.direction = simulatedDirection;
	simTrain.time = 8*60 + 0;
	simTrain.totalTime = 10;
	simTrain.approachTime = 5;

	SimTrain nullSim;
	nullSim.flags = 0;
	nullSim.direction = 0;
	nullSim.time = 0;
	nullSim.totalTime = 0;
	nullSim.approachTime = 0;

	START_TEST;
	
	printf("\n");
	printf("--------------------------------------------------------------------------------\n");
	printf("Real Train Meet Simulated Train %d -> %d ", realDirection, simulatedDirection);
	if(realBeforeSim)
	{
		printf("(Real Before Sim)\n");
	}
	else
	{
		printf("(Sim Before Real)\n");
	}
	printf("--------------------------------------------------------------------------------\n");

	clearAll();

	debounceTime = 3;
	writeEeprom(EE_DEBOUNCE_SECONDS, debounceTime);
	timelockTime = simTrain.totalTime - simTrain.approachTime + 5;  // Set longer than totalTime-approachTime (first train)
	writeEeprom(EE_TIMELOCK_SECONDS, timelockTime);

	for(i=0; i<NUM_SIM_TRAINS; i++)
	{
		if(0 == i)
		{
			writeEepromSim(i, simTrain);
		}
		else
		{
			writeEepromSim(i, nullSim);
		}
	}

	// Pre-Trigger
	printf("Pre-Trigger\n");
	sendFastTimePacket(simTrain.time - 5, 10, TIME_FLAGS_DISP_FAST);  // time - 5 minutes, 1x, Fast enabled, not hold
	sleep(1);
	
	if(!realBeforeSim)
	{
		// Sim before real
		printf("Trigger Simulated Train\n");
		sendFastTimePacket(simTrain.time, 10, TIME_FLAGS_DISP_FAST);
		// Triggered Time
		gettimeofday(&tvTimerA, NULL);
		
		printf("Real Train Arrives\n");
		turnouts = 0;
		switch(realDirection)
		{
			case DIR_WEST:
				occupancy = OCC_WEST_MAIN;
				break;
			case DIR_EAST:
				occupancy = OCC_EAST_MAIN;
				break;
			case DIR_NORTH:
				occupancy = OCC_NORTH_MAIN;
				break;
			case DIR_SOUTH:
				occupancy = OCC_SOUTH_MAIN;
				break;
		}
		sendCommand();

		assertAspect("Simulated train arrives, gets proceed indication (main/top)", 2*simulatedDirection+0, ASPECT_GREEN);
		assertAspect("Simulated train arrives, gets proceed indication (siding/bottom)", 2*simulatedDirection+1, ASPECT_RED);
		printf("Waiting for approach time to expire...\n");
		while(STATE_OCCUPIED != getState(simulatedDirection)) {}

		gettimeofday(&tvTimerB, NULL);
		gettimeofday(&tvTimerC, NULL);
		timeval_subtract(&tvDiff, &tvTimerB, &tvTimerA);
		assertFloat("Approach Time", (float)tvDiff.tv_sec + (float)tvDiff.tv_usec/1000000, simTrain.approachTime, 1);

		assertAspect("Simulated train occupies interlocking (main/top)", 0, ASPECT_RED);
		assertAspect("Simulated train occupies interlocking (siding/bottom)", 1, ASPECT_RED);

		printf("Waiting for total time to expire...\n");
		while(STATE_IDLE != getState(simulatedDirection)) {}

		gettimeofday(&tvTimerB, NULL);
		timeval_subtract(&tvDiff, &tvTimerB, &tvTimerA);
		assertFloat("Total Time", (float)tvDiff.tv_sec + (float)tvDiff.tv_usec/1000000, simTrain.totalTime, 1);

		i = 0;
		printf("Wait for timelock to expire.");
		while(getTimelock())
		{
			usleep(100000);
			if(++i > 10)
			{
				printf(".");
				fflush(stdout);
				i = 0;
			}
		}
		printf("\n");
		gettimeofday(&tvTimerB, NULL);
		timeval_subtract(&tvDiff, &tvTimerB, &tvTimerC);
		assertFloat("TIMELOCK timer", (float)tvDiff.tv_sec + (float)tvDiff.tv_usec/1000000, timelockTime, 1);

		assertAspect("Real train gets interlocking, gets proceed indication (main/top)", 2*realDirection+0, ASPECT_GREEN);
		assertAspect("Real train gets interlocking, gets proceed indication (siding/bottom)", 2*realDirection+1, ASPECT_RED);
	}
	else
	{
		// Real before sim
		printf("Real Train Arrives\n");
		turnouts = 0;
		switch(realDirection)
		{
			case DIR_WEST:
				occupancy = OCC_WEST_MAIN;
				break;
			case DIR_EAST:
				occupancy = OCC_EAST_MAIN;
				break;
			case DIR_NORTH:
				occupancy = OCC_NORTH_MAIN;
				break;
			case DIR_SOUTH:
				occupancy = OCC_SOUTH_MAIN;
				break;
		}
		sendCommand();

		printf("Trigger Simulated Train\n");
		sendFastTimePacket(simTrain.time, 10, TIME_FLAGS_DISP_FAST);

		assertAspect("Real train gets interlocking, gets proceed indication (main/top)", 2*realDirection+0, ASPECT_GREEN);
		assertAspect("Real train gets interlocking, gets proceed indication (siding/bottom)", 2*realDirection+1, ASPECT_RED);

		sleep(1);

		occupancy = OCC_INTERLOCKING;
		sendCommand();
		gettimeofday(&tvTimerC, NULL);

		assertAspect("Real train takes interlocking (main/top)", 2*realDirection+0, ASPECT_RED);
		assertAspect("Real train takes interlocking (siding/bottom)", 2*realDirection+1, ASPECT_RED);

		sleep(1);

		occupancy = 0;
		sendCommand();

		i = 0;
		printf("Wait for timelock to expire.");
		while(getTimelock())
		{
			usleep(100000);
			if(++i > 10)
			{
				printf(".");
				fflush(stdout);
				i = 0;
			}
		}
		printf("\n");
		gettimeofday(&tvTimerB, NULL);
		timeval_subtract(&tvDiff, &tvTimerB, &tvTimerC);
		assertFloat("TIMELOCK timer", (float)tvDiff.tv_sec + (float)tvDiff.tv_usec/1000000, timelockTime, 1);

		assertAspect("Simulated train gets interlocking, gets proceed indication (main/top)", 2*simulatedDirection+0, ASPECT_GREEN);
		assertAspect("Simulated train gets interlocking, gets proceed indication (siding/bottom)", 2*simulatedDirection+1, ASPECT_RED);
	}

	clearAll();

	END_TEST;
}



int main(void)
{
	struct timeval tvBegin, tvEnd, tvDiff;
	
	pass = 0;
	fail = 0;

	timeoutTime = 5;
	debounceTime = 5;
	lockoutTime = 5;
	timelockTime = 5;
	
	printf("Configuring EEPROM...\n");
	writeEeprom(MRBUS_EE_DEVICE_UPDATE_H, 0);
	writeEeprom(MRBUS_EE_DEVICE_UPDATE_L, 5);
	writeEeprom(EE_TIMEOUT_SECONDS+0, timeoutTime);
	writeEeprom(EE_TIMEOUT_SECONDS+1, timeoutTime);
	writeEeprom(EE_TIMEOUT_SECONDS+2, timeoutTime);
	writeEeprom(EE_TIMEOUT_SECONDS+3, timeoutTime);
	writeEeprom(EE_DEBOUNCE_SECONDS, debounceTime);
	writeEeprom(EE_LOCKOUT_SECONDS, lockoutTime);
	writeEeprom(EE_TIMELOCK_SECONDS, timelockTime);

	setSimTrainWindow(3);     // 3 fast minutes
	setMaxDeadReckoning(50);  // 5 real seconds
	
	// 0.5sec update rate
	writeEeprom(MRBUS_EE_DEVICE_UPDATE_H, 0);
	writeEeprom(MRBUS_EE_DEVICE_UPDATE_L, 5);
	
	aspectString[7] = "LUNAR";
	aspectString[6] = "FL RED";
	aspectString[5] = "FL GREEN";
	aspectString[4] = "RED";
	aspectString[3] = "FL YELLOW";
	aspectString[2] = "YELLOW";
	aspectString[1] = "GREEN";
	aspectString[0] = "OFF";
	
	gettimeofday(&tvBegin, NULL);
	
	while(1)
	{
/*		// eastbound_main, eastbound_siding*/
/*		RUN_TEST(testEastboundSouthbound(DIR_EAST, 0, 0));*/
/*		RUN_TEST(testEastboundSouthbound(DIR_EAST, 0, 1));*/
/*		RUN_TEST(testEastboundSouthbound(DIR_EAST, 1, 0));*/
/*		RUN_TEST(testEastboundSouthbound(DIR_EAST, 1, 1));*/
/*		RUN_TEST(testEastboundSouthbound(DIR_SOUTH, 0, 0));*/
/*		RUN_TEST(testEastboundSouthbound(DIR_SOUTH, 0, 1));*/
/*		RUN_TEST(testEastboundSouthbound(DIR_SOUTH, 1, 0));*/
/*		RUN_TEST(testEastboundSouthbound(DIR_SOUTH, 1, 1));*/
/*	*/
/*		// eastbound_turnout*/
/*		RUN_TEST(testArriveOpposingTurnout(DIR_EAST, 0));*/
/*		RUN_TEST(testArriveOpposingTurnout(DIR_EAST, 1));*/
/*		RUN_TEST(testArriveOpposingTurnout(DIR_SOUTH, 0));*/
/*		RUN_TEST(testArriveOpposingTurnout(DIR_SOUTH, 1));*/

/*		// westbound_main, westbound_siding*/
/*		RUN_TEST(testWestboundNorthbound(DIR_WEST, 0, 0));*/
/*		RUN_TEST(testWestboundNorthbound(DIR_WEST, 0, 1));*/
/*		RUN_TEST(testWestboundNorthbound(DIR_WEST, 1, 0));*/
/*		RUN_TEST(testWestboundNorthbound(DIR_WEST, 1, 1));*/
/*		RUN_TEST(testWestboundNorthbound(DIR_NORTH, 0, 0));*/
/*		RUN_TEST(testWestboundNorthbound(DIR_NORTH, 0, 1));*/
/*		RUN_TEST(testWestboundNorthbound(DIR_NORTH, 1, 0));*/
/*		RUN_TEST(testWestboundNorthbound(DIR_NORTH, 1, 1));*/
/*	*/
/*		// meet_before_interlocking*/
/*		RUN_TEST(testMeetBeforeInterlocking(OCC_WEST_MAIN));*/
/*		RUN_TEST(testMeetBeforeInterlocking(OCC_WEST_SIDING));*/
/*		RUN_TEST(testMeetBeforeInterlocking(OCC_NORTH_MAIN));*/
/*		RUN_TEST(testMeetBeforeInterlocking(OCC_NORTH_SIDING));*/
/*		*/
/*		// meet_after_interlocking*/
/*		*/
/*		// timelock_meet*/
/*		timelockTime = 15;*/
/*		writeEeprom(EE_TIMELOCK_SECONDS, timelockTime);*/
/*		RUN_TEST(testTimelockMeet(OCC_EAST_MAIN,OCC_NORTH_MAIN));*/
/*		timelockTime = 5;*/
/*		writeEeprom(EE_TIMELOCK_SECONDS, timelockTime);*/
/*		RUN_TEST(testTimelockMeet(OCC_EAST_MAIN,OCC_NORTH_MAIN));*/
/*		RUN_TEST(testTimelockMeet(OCC_EAST_MAIN,OCC_NORTH_SIDING));*/
/*		RUN_TEST(testTimelockMeet(OCC_EAST_MAIN,OCC_SOUTH_MAIN));*/
/*		RUN_TEST(testTimelockMeet(OCC_WEST_MAIN,OCC_NORTH_MAIN));*/
/*		RUN_TEST(testTimelockMeet(OCC_WEST_MAIN,OCC_NORTH_SIDING));*/
/*		RUN_TEST(testTimelockMeet(OCC_WEST_MAIN,OCC_SOUTH_MAIN));*/
/*		RUN_TEST(testTimelockMeet(OCC_WEST_SIDING,OCC_NORTH_MAIN));*/
/*		RUN_TEST(testTimelockMeet(OCC_WEST_SIDING,OCC_NORTH_SIDING));*/
/*		RUN_TEST(testTimelockMeet(OCC_WEST_SIDING,OCC_SOUTH_MAIN));*/

/*		RUN_TEST(testTimelockMeet(OCC_SOUTH_MAIN,OCC_WEST_MAIN));*/
/*		RUN_TEST(testTimelockMeet(OCC_SOUTH_MAIN,OCC_WEST_SIDING));*/
/*		RUN_TEST(testTimelockMeet(OCC_SOUTH_MAIN,OCC_EAST_MAIN));*/
/*		RUN_TEST(testTimelockMeet(OCC_NORTH_MAIN,OCC_WEST_MAIN));*/
/*		RUN_TEST(testTimelockMeet(OCC_NORTH_MAIN,OCC_WEST_SIDING));*/
/*		RUN_TEST(testTimelockMeet(OCC_NORTH_MAIN,OCC_EAST_MAIN));*/
/*		RUN_TEST(testTimelockMeet(OCC_NORTH_SIDING,OCC_WEST_MAIN));*/
/*		RUN_TEST(testTimelockMeet(OCC_NORTH_SIDING,OCC_WEST_SIDING));*/
/*		RUN_TEST(testTimelockMeet(OCC_NORTH_SIDING,OCC_EAST_MAIN));*/

/*		// timelock*/
/*		RUN_TEST(testTimelockTurnout(OCC_EAST_MAIN));*/
/*		RUN_TEST(testTimelockTurnout(OCC_SOUTH_MAIN));*/

/*		// lockout*/
/*		lockoutTime = 15;*/
/*		writeEeprom(EE_LOCKOUT_SECONDS, lockoutTime);*/
/*		RUN_TEST(testLockout(OCC_EAST_MAIN,OCC_WEST_MAIN,0));*/
/*		lockoutTime = 5;*/
/*		writeEeprom(EE_LOCKOUT_SECONDS, lockoutTime);*/
/*		*/
/*		// lockout (no train present)*/
/*		RUN_TEST(testLockout(OCC_EAST_MAIN,OCC_WEST_MAIN,0));*/
/*		RUN_TEST(testLockout(OCC_EAST_MAIN,OCC_WEST_SIDING,0));*/
/*		RUN_TEST(testLockout(OCC_SOUTH_MAIN,OCC_NORTH_MAIN,0));*/
/*		RUN_TEST(testLockout(OCC_SOUTH_MAIN,OCC_NORTH_SIDING,0));*/
/*		RUN_TEST(testLockout(OCC_WEST_MAIN,OCC_EAST_MAIN,0));*/
/*		RUN_TEST(testLockout(OCC_WEST_SIDING,OCC_EAST_MAIN,0));*/
/*		RUN_TEST(testLockout(OCC_NORTH_MAIN,OCC_SOUTH_MAIN,0));*/
/*		RUN_TEST(testLockout(OCC_NORTH_SIDING,OCC_SOUTH_MAIN,0));*/

/*		// lockout_expire_with_train_present*/
/*		RUN_TEST(testLockout(OCC_EAST_MAIN,OCC_WEST_MAIN,1));*/
/*		RUN_TEST(testLockout(OCC_EAST_MAIN,OCC_WEST_SIDING,1));*/
/*		RUN_TEST(testLockout(OCC_SOUTH_MAIN,OCC_NORTH_MAIN,1));*/
/*		RUN_TEST(testLockout(OCC_SOUTH_MAIN,OCC_NORTH_SIDING,1));*/
/*		RUN_TEST(testLockout(OCC_WEST_MAIN,OCC_EAST_MAIN,1));*/
/*		RUN_TEST(testLockout(OCC_WEST_SIDING,OCC_EAST_MAIN,1));*/
/*		RUN_TEST(testLockout(OCC_NORTH_MAIN,OCC_SOUTH_MAIN,1));*/
/*		RUN_TEST(testLockout(OCC_NORTH_SIDING,OCC_SOUTH_MAIN,1));*/

/*		// timeout_expire (covered by below?)*/
/*		// timeout_expire_momentary_train*/
/*		timeoutTime = 15;*/
/*		writeEeprom(EE_TIMEOUT_SECONDS+0, timeoutTime);*/
/*		writeEeprom(EE_TIMEOUT_SECONDS+1, 4*timeoutTime);*/
/*		writeEeprom(EE_TIMEOUT_SECONDS+2, 4*timeoutTime);*/
/*		writeEeprom(EE_TIMEOUT_SECONDS+3, 4*timeoutTime);*/
/*		RUN_TEST(testTimeout(OCC_WEST_MAIN, 0));*/
/*		RUN_TEST(testTimeout(OCC_WEST_SIDING, TURNOUT_WEST));*/
/*		writeEeprom(EE_TIMEOUT_SECONDS+0, 4*timeoutTime);*/
/*		writeEeprom(EE_TIMEOUT_SECONDS+1, timeoutTime);*/
/*		writeEeprom(EE_TIMEOUT_SECONDS+2, 4*timeoutTime);*/
/*		writeEeprom(EE_TIMEOUT_SECONDS+3, 4*timeoutTime);*/
/*		RUN_TEST(testTimeout(OCC_EAST_MAIN, 0));*/
/*		RUN_TEST(testTimeout(OCC_EAST_MAIN, TURNOUT_WEST));*/
/*		writeEeprom(EE_TIMEOUT_SECONDS+0, 4*timeoutTime);*/
/*		writeEeprom(EE_TIMEOUT_SECONDS+1, 4*timeoutTime);*/
/*		writeEeprom(EE_TIMEOUT_SECONDS+2, timeoutTime);*/
/*		writeEeprom(EE_TIMEOUT_SECONDS+3, 4*timeoutTime);*/
/*		RUN_TEST(testTimeout(OCC_NORTH_MAIN, 0));*/
/*		RUN_TEST(testTimeout(OCC_NORTH_SIDING, TURNOUT_NORTH));*/
/*		writeEeprom(EE_TIMEOUT_SECONDS+0, 4*timeoutTime);*/
/*		writeEeprom(EE_TIMEOUT_SECONDS+1, 4*timeoutTime);*/
/*		writeEeprom(EE_TIMEOUT_SECONDS+2, 4*timeoutTime);*/
/*		writeEeprom(EE_TIMEOUT_SECONDS+3, timeoutTime);*/
/*		RUN_TEST(testTimeout(OCC_SOUTH_MAIN, 0));*/
/*		RUN_TEST(testTimeout(OCC_SOUTH_MAIN, TURNOUT_NORTH));*/

/*		timeoutTime = 5;*/
/*		writeEeprom(EE_TIMEOUT_SECONDS+0, timeoutTime);*/
/*		writeEeprom(EE_TIMEOUT_SECONDS+1, 4*timeoutTime);*/
/*		writeEeprom(EE_TIMEOUT_SECONDS+2, 4*timeoutTime);*/
/*		writeEeprom(EE_TIMEOUT_SECONDS+3, 4*timeoutTime);*/
/*		RUN_TEST(testTimeout(OCC_WEST_MAIN, 0));*/
/*		RUN_TEST(testTimeout(OCC_WEST_SIDING, TURNOUT_WEST));*/
/*		writeEeprom(EE_TIMEOUT_SECONDS+0, 4*timeoutTime);*/
/*		writeEeprom(EE_TIMEOUT_SECONDS+1, timeoutTime);*/
/*		writeEeprom(EE_TIMEOUT_SECONDS+2, 4*timeoutTime);*/
/*		writeEeprom(EE_TIMEOUT_SECONDS+3, 4*timeoutTime);*/
/*		RUN_TEST(testTimeout(OCC_EAST_MAIN, 0));*/
/*		RUN_TEST(testTimeout(OCC_EAST_MAIN, TURNOUT_WEST));*/
/*		writeEeprom(EE_TIMEOUT_SECONDS+0, 4*timeoutTime);*/
/*		writeEeprom(EE_TIMEOUT_SECONDS+1, 4*timeoutTime);*/
/*		writeEeprom(EE_TIMEOUT_SECONDS+2, timeoutTime);*/
/*		writeEeprom(EE_TIMEOUT_SECONDS+3, 4*timeoutTime);*/
/*		RUN_TEST(testTimeout(OCC_NORTH_MAIN, 0));*/
/*		RUN_TEST(testTimeout(OCC_NORTH_SIDING, TURNOUT_NORTH));*/
/*		writeEeprom(EE_TIMEOUT_SECONDS+0, 4*timeoutTime);*/
/*		writeEeprom(EE_TIMEOUT_SECONDS+1, 4*timeoutTime);*/
/*		writeEeprom(EE_TIMEOUT_SECONDS+2, 4*timeoutTime);*/
/*		writeEeprom(EE_TIMEOUT_SECONDS+3, timeoutTime);*/
/*		RUN_TEST(testTimeout(OCC_SOUTH_MAIN, 0));*/
/*		RUN_TEST(testTimeout(OCC_SOUTH_MAIN, TURNOUT_NORTH));*/

/*		// debounce_timer*/
/*		debounceTime = 15;*/
/*		writeEeprom(EE_DEBOUNCE_SECONDS, debounceTime);*/
/*		RUN_TEST(testDebounceTimer());*/

/*		debounceTime = 5;*/
/*		writeEeprom(EE_DEBOUNCE_SECONDS, debounceTime);*/
/*		RUN_TEST(testDebounceTimer());*/

/*		// What happens if interlocking block gets occupancy out of the blue?  Then something real shows up?*/
/*		RUN_TEST(testBogusInterlocking());*/


/*		runAllBasicSimulatedTrains();*/
/*		RUN_TEST(testSimulatedTrainSkipTime());*/
/*		RUN_TEST(testSimulatedTrainRetrigger());*/
/*		RUN_TEST(testSimulatedTrainInvalidTime());*/
/*		RUN_TEST(testSimulatedTrainEnable());*/
/*		RUN_TEST(testSimulatedTrainSequence(0));*/
/*		RUN_TEST(testSimulatedTrainSequence(1));*/
/*		RUN_TEST(testSimulatedTrainApproachLarger());*/
/*		RUN_TEST(testSimulatedTrainMeet(0, 2));*/
/*		RUN_TEST(testSimulatedTrainMeet(2, 1));*/
/*		RUN_TEST(testSimulatedTrainMeet(1, 3));*/
/*		RUN_TEST(testSimulatedTrainMeet(3, 0));*/

		RUN_TEST(testRealMeetSimulated(0, 2, 0));
		RUN_TEST(testRealMeetSimulated(2, 1, 0));
		RUN_TEST(testRealMeetSimulated(1, 3, 0));
		RUN_TEST(testRealMeetSimulated(3, 0, 0));
		RUN_TEST(testRealMeetSimulated(0, 2, 1));
		RUN_TEST(testRealMeetSimulated(2, 1, 1));
		RUN_TEST(testRealMeetSimulated(1, 3, 1));
		RUN_TEST(testRealMeetSimulated(3, 0, 1));

// Real train meet simulated train (before and after)
// Auto Interchange
// Train scheduled on same track as real
// Train scheduled on opposite track as real

		
		break;
	}
	
	clearAll();
	
	gettimeofday(&tvEnd, NULL);

	printf("--------------------------------------------------------------------------------\n");

	printf("\nPass: %d\nFail: %d\n\n", pass, fail);
	
	timeval_subtract(&tvDiff, &tvEnd, &tvBegin);
	printf("Elapsed Time: %ld:%02ld\n\n", tvDiff.tv_sec / 60, tvDiff.tv_sec % 60);
	
	return 0;
}


