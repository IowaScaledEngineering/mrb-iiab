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

#define EE_TIMEOUT_SECONDS_0  0x10
#define EE_TIMEOUT_SECONDS_1  0x11
#define EE_TIMEOUT_SECONDS_2  0x12
#define EE_TIMEOUT_SECONDS_3  0x13
#define EE_LOCKOUT_SECONDS    0x14
#define EE_TIMELOCK_SECONDS   0x15
#define EE_DEBOUNCE_SECONDS   0x16

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
	printf("Sending packet [%s]\n", pkt);
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
	
	sleep(1);	
}

int getState(int dir)
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
		printf("Sending packet [%s]\n", pkt);
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
	writeEeprom(EE_TIMEOUT_SECONDS_0, timeoutTime);
	writeEeprom(EE_TIMEOUT_SECONDS_1, timeoutTime);
	writeEeprom(EE_TIMEOUT_SECONDS_2, timeoutTime);
	writeEeprom(EE_TIMEOUT_SECONDS_3, timeoutTime);
	writeEeprom(EE_DEBOUNCE_SECONDS, debounceTime);
	writeEeprom(EE_LOCKOUT_SECONDS, lockoutTime);
	writeEeprom(EE_TIMELOCK_SECONDS, timelockTime);

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
		// eastbound_main, eastbound_siding
		RUN_TEST(testEastboundSouthbound(DIR_EAST, 0, 0));
		RUN_TEST(testEastboundSouthbound(DIR_EAST, 0, 1));
		RUN_TEST(testEastboundSouthbound(DIR_EAST, 1, 0));
		RUN_TEST(testEastboundSouthbound(DIR_EAST, 1, 1));
		RUN_TEST(testEastboundSouthbound(DIR_SOUTH, 0, 0));
		RUN_TEST(testEastboundSouthbound(DIR_SOUTH, 0, 1));
		RUN_TEST(testEastboundSouthbound(DIR_SOUTH, 1, 0));
		RUN_TEST(testEastboundSouthbound(DIR_SOUTH, 1, 1));
	
		// eastbound_turnout
		RUN_TEST(testArriveOpposingTurnout(DIR_EAST, 0));
		RUN_TEST(testArriveOpposingTurnout(DIR_EAST, 1));
		RUN_TEST(testArriveOpposingTurnout(DIR_SOUTH, 0));
		RUN_TEST(testArriveOpposingTurnout(DIR_SOUTH, 1));

		// westbound_main, westbound_siding
		RUN_TEST(testWestboundNorthbound(DIR_WEST, 0, 0));
		RUN_TEST(testWestboundNorthbound(DIR_WEST, 0, 1));
		RUN_TEST(testWestboundNorthbound(DIR_WEST, 1, 0));
		RUN_TEST(testWestboundNorthbound(DIR_WEST, 1, 1));
		RUN_TEST(testWestboundNorthbound(DIR_NORTH, 0, 0));
		RUN_TEST(testWestboundNorthbound(DIR_NORTH, 0, 1));
		RUN_TEST(testWestboundNorthbound(DIR_NORTH, 1, 0));
		RUN_TEST(testWestboundNorthbound(DIR_NORTH, 1, 1));
	
		// meet_before_interlocking
		RUN_TEST(testMeetBeforeInterlocking(OCC_WEST_MAIN));
		RUN_TEST(testMeetBeforeInterlocking(OCC_WEST_SIDING));
		RUN_TEST(testMeetBeforeInterlocking(OCC_NORTH_MAIN));
		RUN_TEST(testMeetBeforeInterlocking(OCC_NORTH_SIDING));
		
		// meet_after_interlocking
		
		// real_before_phantom
	
		// phantom_before_real
	
		// timelock_meet
		timelockTime = 15;
		writeEeprom(EE_TIMELOCK_SECONDS, timelockTime);
		RUN_TEST(testTimelockMeet(OCC_EAST_MAIN,OCC_NORTH_MAIN));
		timelockTime = 5;
		writeEeprom(EE_TIMELOCK_SECONDS, timelockTime);
		RUN_TEST(testTimelockMeet(OCC_EAST_MAIN,OCC_NORTH_MAIN));
		RUN_TEST(testTimelockMeet(OCC_EAST_MAIN,OCC_NORTH_SIDING));
		RUN_TEST(testTimelockMeet(OCC_EAST_MAIN,OCC_SOUTH_MAIN));
		RUN_TEST(testTimelockMeet(OCC_WEST_MAIN,OCC_NORTH_MAIN));
		RUN_TEST(testTimelockMeet(OCC_WEST_MAIN,OCC_NORTH_SIDING));
		RUN_TEST(testTimelockMeet(OCC_WEST_MAIN,OCC_SOUTH_MAIN));
		RUN_TEST(testTimelockMeet(OCC_WEST_SIDING,OCC_NORTH_MAIN));
		RUN_TEST(testTimelockMeet(OCC_WEST_SIDING,OCC_NORTH_SIDING));
		RUN_TEST(testTimelockMeet(OCC_WEST_SIDING,OCC_SOUTH_MAIN));

		RUN_TEST(testTimelockMeet(OCC_SOUTH_MAIN,OCC_WEST_MAIN));
		RUN_TEST(testTimelockMeet(OCC_SOUTH_MAIN,OCC_WEST_SIDING));
		RUN_TEST(testTimelockMeet(OCC_SOUTH_MAIN,OCC_EAST_MAIN));
		RUN_TEST(testTimelockMeet(OCC_NORTH_MAIN,OCC_WEST_MAIN));
		RUN_TEST(testTimelockMeet(OCC_NORTH_MAIN,OCC_WEST_SIDING));
		RUN_TEST(testTimelockMeet(OCC_NORTH_MAIN,OCC_EAST_MAIN));
		RUN_TEST(testTimelockMeet(OCC_NORTH_SIDING,OCC_WEST_MAIN));
		RUN_TEST(testTimelockMeet(OCC_NORTH_SIDING,OCC_WEST_SIDING));
		RUN_TEST(testTimelockMeet(OCC_NORTH_SIDING,OCC_EAST_MAIN));

		// timelock
		RUN_TEST(testTimelockTurnout(OCC_EAST_MAIN));
		RUN_TEST(testTimelockTurnout(OCC_SOUTH_MAIN));

		// lockout
		lockoutTime = 15;
		writeEeprom(EE_LOCKOUT_SECONDS, lockoutTime);
		RUN_TEST(testLockout(OCC_EAST_MAIN,OCC_WEST_MAIN,0));
		lockoutTime = 5;
		writeEeprom(EE_LOCKOUT_SECONDS, lockoutTime);
		
		// lockout (no train present)
		RUN_TEST(testLockout(OCC_EAST_MAIN,OCC_WEST_MAIN,0));
		RUN_TEST(testLockout(OCC_EAST_MAIN,OCC_WEST_SIDING,0));
		RUN_TEST(testLockout(OCC_SOUTH_MAIN,OCC_NORTH_MAIN,0));
		RUN_TEST(testLockout(OCC_SOUTH_MAIN,OCC_NORTH_SIDING,0));
		RUN_TEST(testLockout(OCC_WEST_MAIN,OCC_EAST_MAIN,0));
		RUN_TEST(testLockout(OCC_WEST_SIDING,OCC_EAST_MAIN,0));
		RUN_TEST(testLockout(OCC_NORTH_MAIN,OCC_SOUTH_MAIN,0));
		RUN_TEST(testLockout(OCC_NORTH_SIDING,OCC_SOUTH_MAIN,0));

		// lockout_expire_with_train_present
		RUN_TEST(testLockout(OCC_EAST_MAIN,OCC_WEST_MAIN,1));
		RUN_TEST(testLockout(OCC_EAST_MAIN,OCC_WEST_SIDING,1));
		RUN_TEST(testLockout(OCC_SOUTH_MAIN,OCC_NORTH_MAIN,1));
		RUN_TEST(testLockout(OCC_SOUTH_MAIN,OCC_NORTH_SIDING,1));
		RUN_TEST(testLockout(OCC_WEST_MAIN,OCC_EAST_MAIN,1));
		RUN_TEST(testLockout(OCC_WEST_SIDING,OCC_EAST_MAIN,1));
		RUN_TEST(testLockout(OCC_NORTH_MAIN,OCC_SOUTH_MAIN,1));
		RUN_TEST(testLockout(OCC_NORTH_SIDING,OCC_SOUTH_MAIN,1));

		// timeout_expire (covered by below?)
		// timeout_expire_momentary_train
		timeoutTime = 15;
		writeEeprom(EE_TIMEOUT_SECONDS_0, timeoutTime);
		writeEeprom(EE_TIMEOUT_SECONDS_1, 4*timeoutTime);
		writeEeprom(EE_TIMEOUT_SECONDS_2, 4*timeoutTime);
		writeEeprom(EE_TIMEOUT_SECONDS_3, 4*timeoutTime);
		RUN_TEST(testTimeout(OCC_WEST_MAIN, 0));
		RUN_TEST(testTimeout(OCC_WEST_SIDING, TURNOUT_WEST));
		writeEeprom(EE_TIMEOUT_SECONDS_0, 4*timeoutTime);
		writeEeprom(EE_TIMEOUT_SECONDS_1, timeoutTime);
		writeEeprom(EE_TIMEOUT_SECONDS_2, 4*timeoutTime);
		writeEeprom(EE_TIMEOUT_SECONDS_3, 4*timeoutTime);
		RUN_TEST(testTimeout(OCC_EAST_MAIN, 0));
		RUN_TEST(testTimeout(OCC_EAST_MAIN, TURNOUT_WEST));
		writeEeprom(EE_TIMEOUT_SECONDS_0, 4*timeoutTime);
		writeEeprom(EE_TIMEOUT_SECONDS_1, 4*timeoutTime);
		writeEeprom(EE_TIMEOUT_SECONDS_2, timeoutTime);
		writeEeprom(EE_TIMEOUT_SECONDS_3, 4*timeoutTime);
		RUN_TEST(testTimeout(OCC_NORTH_MAIN, 0));
		RUN_TEST(testTimeout(OCC_NORTH_SIDING, TURNOUT_NORTH));
		writeEeprom(EE_TIMEOUT_SECONDS_0, 4*timeoutTime);
		writeEeprom(EE_TIMEOUT_SECONDS_1, 4*timeoutTime);
		writeEeprom(EE_TIMEOUT_SECONDS_2, 4*timeoutTime);
		writeEeprom(EE_TIMEOUT_SECONDS_3, timeoutTime);
		RUN_TEST(testTimeout(OCC_SOUTH_MAIN, 0));
		RUN_TEST(testTimeout(OCC_SOUTH_MAIN, TURNOUT_NORTH));

		timeoutTime = 5;
		writeEeprom(EE_TIMEOUT_SECONDS_0, timeoutTime);
		writeEeprom(EE_TIMEOUT_SECONDS_1, 4*timeoutTime);
		writeEeprom(EE_TIMEOUT_SECONDS_2, 4*timeoutTime);
		writeEeprom(EE_TIMEOUT_SECONDS_3, 4*timeoutTime);
		RUN_TEST(testTimeout(OCC_WEST_MAIN, 0));
		RUN_TEST(testTimeout(OCC_WEST_SIDING, TURNOUT_WEST));
		writeEeprom(EE_TIMEOUT_SECONDS_0, 4*timeoutTime);
		writeEeprom(EE_TIMEOUT_SECONDS_1, timeoutTime);
		writeEeprom(EE_TIMEOUT_SECONDS_2, 4*timeoutTime);
		writeEeprom(EE_TIMEOUT_SECONDS_3, 4*timeoutTime);
		RUN_TEST(testTimeout(OCC_EAST_MAIN, 0));
		RUN_TEST(testTimeout(OCC_EAST_MAIN, TURNOUT_WEST));
		writeEeprom(EE_TIMEOUT_SECONDS_0, 4*timeoutTime);
		writeEeprom(EE_TIMEOUT_SECONDS_1, 4*timeoutTime);
		writeEeprom(EE_TIMEOUT_SECONDS_2, timeoutTime);
		writeEeprom(EE_TIMEOUT_SECONDS_3, 4*timeoutTime);
		RUN_TEST(testTimeout(OCC_NORTH_MAIN, 0));
		RUN_TEST(testTimeout(OCC_NORTH_SIDING, TURNOUT_NORTH));
		writeEeprom(EE_TIMEOUT_SECONDS_0, 4*timeoutTime);
		writeEeprom(EE_TIMEOUT_SECONDS_1, 4*timeoutTime);
		writeEeprom(EE_TIMEOUT_SECONDS_2, 4*timeoutTime);
		writeEeprom(EE_TIMEOUT_SECONDS_3, timeoutTime);
		RUN_TEST(testTimeout(OCC_SOUTH_MAIN, 0));
		RUN_TEST(testTimeout(OCC_SOUTH_MAIN, TURNOUT_NORTH));

		// debounce_timer
		debounceTime = 15;
		writeEeprom(EE_DEBOUNCE_SECONDS, debounceTime);
		RUN_TEST(testDebounceTimer());

		debounceTime = 5;
		writeEeprom(EE_DEBOUNCE_SECONDS, debounceTime);
		RUN_TEST(testDebounceTimer());

		// What happens if interlocking block gets occupancy out of the blue?  Then something real shows up?
		RUN_TEST(testBogusInterlocking());
		
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


