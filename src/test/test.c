#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>

#define MRBFS_PATH  "/mrbfs/bus0/0x03-mrb-iiab"
#define MRBFS_TXPKT "/mrbfs/bus0/txPacket"

#define SRC_ADDR 0xFD
#define DUT_ADDR 0x03

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

#define EE_TIMEOUT_SECONDS  0x10
#define EE_LOCKOUT_SECONDS  0x11
#define EE_TIMELOCK_SECONDS 0x12
#define EE_DEBOUNCE_SECONDS 0x13

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
#define OCC_EAST_SIDING  0x00
#define OCC_NORTH_MAIN   0x08
#define OCC_NORTH_SIDING 0x10
#define OCC_SOUTH_MAIN   0x20
#define OCC_SOUTH_SIDING 0x00
#define OCC_INTERLOCKING 0x40

#define TURNOUT_WEST  0x01
#define TURNOUT_EAST  0x02
#define TURNOUT_NORTH 0x04
#define TURNOUT_SOUTH 0x08

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

void assertAspect(char *msg, int actual, int expect)
{
	textcolor(BRIGHT, BLUE, BLACK);
	printf("%s: Actual=%s, Expect=%s ", msg, aspectString[actual], aspectString[expect]);

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

void sendCommand(void)
{
	char pkt[32];
	FILE *fptr;
	
	// FIXME: xor (invert) bits based on polarity configuration
	snprintf(pkt, sizeof(pkt), "%02X->%02X %02X %02X %02X %02X", SRC_ADDR, DUT_ADDR, 'C', 'T', occupancy, turnouts);
	textcolor(RESET, YELLOW, BLACK);
	printf("Sending packet [%s]\n", pkt);
	textcolor(RESET, WHITE, BLACK);

	// FIXME: Send, wait 1 sec, check that turnout and occupancy are correct.  If not, resend.

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

int getSignal(int signal)
{
	char path[256], data[256];
	FILE *fptr;
	
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

void clearAll(void)
{
	printf("Clearing...\n");

	occupancy = 0;
	turnouts = 0;
	sendCommand();
	
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
		);
}

void testEastboundSouthbound(int dir, int track, int leaveBeforeDebounce)
{
	struct timeval tvBegin, tvEnd, tvDiff;
	gettimeofday(&tvBegin, NULL);
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
	
	assertAspect("Train arrives, gets proceed indication", getSignal(signal), track?ASPECT_YELLOW:ASPECT_GREEN);
	assertAspect("Red signal on other track", getSignal(track?signal-1:signal+1), ASPECT_RED);

	occupancy |= OCC_INTERLOCKING;
	sendCommand();
	assertAspect("Enters interlocking, red signal", getSignal(signal), ASPECT_RED);

	occupancy |= (DIR_EAST==dir)?OCC_EAST_MAIN:OCC_SOUTH_MAIN;
	sendCommand();
	assertAspect("Enters opposite approach, red signal", getSignal(signal), ASPECT_RED);
	assertInt("No lockout", getLockout((DIR_EAST==dir)?DIR_EAST:DIR_SOUTH), 0);
	
	occupancy &= (DIR_EAST==dir) ? (track?~OCC_WEST_SIDING:~OCC_WEST_MAIN) : (track?~OCC_NORTH_SIDING:~OCC_NORTH_MAIN);
	sendCommand();
	assertAspect("Approach clear, red signal", getSignal(signal), ASPECT_RED);
	assertInt("No lockout", getLockout((DIR_EAST==dir)?DIR_EAST:DIR_SOUTH), 0);

	occupancy &= ~OCC_INTERLOCKING;
	sendCommand();
	assertAspect("Interlocking cleared, red signal", getSignal(signal), ASPECT_RED);
	sleep(1);
	assertFloat("DEBOUNCE timer running", getDebounce(), debounceTime-1, 1);
	
	if(leaveBeforeDebounce)
	{
		occupancy &= (DIR_EAST==dir)?~OCC_EAST_MAIN:~OCC_SOUTH_MAIN;
		sendCommand();
		assertAspect("Opposite approach cleared, red signal", getSignal(signal), ASPECT_RED);
		assertFloat("DEBOUNCE timer still running", getDebounce(), debounceTime-2, 1);
	}
	
	while(getDebounce())
	{
		printf(".");
		fflush(stdout);
		sleep(1);
	}
	printf("\n");

	assertAspect("DEBOUNCE expired, red signal", getSignal(signal), ASPECT_RED);
	sleep(1);
	assertInt("LOCKOUT timer running", getLockout((DIR_EAST==dir)?DIR_EAST:DIR_SOUTH), 1);

	if(!leaveBeforeDebounce)
	{
		occupancy &= (DIR_EAST==dir)?~OCC_EAST_MAIN:~OCC_SOUTH_MAIN;
		sendCommand();
		assertAspect("Opposite approach cleared, red signal", getSignal(signal), ASPECT_RED);
		assertInt("LOCKOUT timer running", getLockout((DIR_EAST==dir)?DIR_EAST:DIR_SOUTH), 1);
	}

	while(getLockout((DIR_EAST==dir)?DIR_EAST:DIR_SOUTH))
	{
		printf(".");
		fflush(stdout);
		sleep(1);
	}
	printf("\n");

	assertAspect("LOCKOUT complete, red signal", getSignal(signal), ASPECT_RED);

	gettimeofday(&tvEnd, NULL);
	printf("\nPass: %d\nFail: %d\n\n", pass, fail);
	timeval_subtract(&tvDiff, &tvEnd, &tvBegin);
	printf("Elapsed Time: %ld:%02ld\n\n", tvDiff.tv_sec / 60, tvDiff.tv_sec % 60);
}

void testWestboundNorthbound(int dir, int track, int leaveBeforeDebounce)
{
	struct timeval tvBegin, tvEnd, tvDiff;
	gettimeofday(&tvBegin, NULL);
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
	
	assertAspect("Train arrives, gets proceed indication (top)", getSignal(signal), track?ASPECT_RED:ASPECT_GREEN);
	assertAspect("Train arrives, gets proceed indication (bottom)", getSignal(signal+1), track?ASPECT_YELLOW:ASPECT_RED);

	occupancy |= OCC_INTERLOCKING;
	sendCommand();
	assertAspect("Enters interlocking, red signal (top)", getSignal(signal), ASPECT_RED);
	assertAspect("Enters interlocking, red signal (bottom)", getSignal(signal+1), ASPECT_RED);

	occupancy |= (DIR_WEST==dir)?(track?OCC_WEST_SIDING:OCC_WEST_MAIN):(track?OCC_NORTH_SIDING:OCC_NORTH_MAIN);
	sendCommand();
	assertAspect("Enters opposite approach, red signal (top)", getSignal(signal), ASPECT_RED);
	assertAspect("Enters opposite approach, red signal (bottom)", getSignal(signal+1), ASPECT_RED);
	assertInt("No lockout", getLockout((DIR_WEST==dir)?DIR_WEST:DIR_NORTH), 0);
	
	occupancy &= (DIR_WEST==dir) ? ~OCC_EAST_MAIN : ~OCC_SOUTH_MAIN;
	sendCommand();
	assertAspect("Approach clear, red signal (top)", getSignal(signal), ASPECT_RED);
	assertAspect("Approach clear, red signal (bottom)", getSignal(signal+1), ASPECT_RED);
	assertInt("No lockout", getLockout((DIR_WEST==dir)?DIR_WEST:DIR_NORTH), 0);

	occupancy &= ~OCC_INTERLOCKING;
	sendCommand();
	assertAspect("Interlocking cleared, red signal (top)", getSignal(signal), ASPECT_RED);
	assertAspect("Interlocking cleared, red signal (bottom)", getSignal(signal+1), ASPECT_RED);
	sleep(1);
	assertFloat("DEBOUNCE timer running", getDebounce(), debounceTime-1, 1);
	
	if(leaveBeforeDebounce)
	{
		occupancy &= (DIR_WEST==dir)?(track?~OCC_WEST_SIDING:~OCC_WEST_MAIN):(track?~OCC_NORTH_SIDING:~OCC_NORTH_MAIN);
		sendCommand();
		assertAspect("Opposite approach cleared, red signal (top)", getSignal(signal), ASPECT_RED);
		assertAspect("Opposite approach cleared, red signal (bottom)", getSignal(signal+1), ASPECT_RED);
		assertFloat("DEBOUNCE timer still running", getDebounce(), debounceTime-2, 1);
	}
	
	while(getDebounce())
	{
		printf(".");
		fflush(stdout);
		sleep(1);
	}
	printf("\n");

	assertAspect("DEBOUNCE expired, red signal (top)", getSignal(signal), ASPECT_RED);
	assertAspect("DEBOUNCE expired, red signal (bottom)", getSignal(signal+1), ASPECT_RED);
	sleep(1);
	assertInt("LOCKOUT timer running", getLockout((DIR_WEST==dir)?DIR_WEST:DIR_NORTH), 1);

	if(!leaveBeforeDebounce)
	{
		occupancy &= (DIR_WEST==dir)?(track?~OCC_WEST_SIDING:~OCC_WEST_MAIN):(track?~OCC_NORTH_SIDING:~OCC_NORTH_MAIN);
		sendCommand();
		assertAspect("Opposite approach cleared, red signal (top)", getSignal(signal), ASPECT_RED);
		assertAspect("Opposite approach cleared, red signal (bottom)", getSignal(signal+1), ASPECT_RED);
		assertInt("LOCKOUT timer running", getLockout((DIR_WEST==dir)?DIR_WEST:DIR_NORTH), 1);
	}

	while(getLockout((DIR_WEST==dir)?DIR_WEST:DIR_NORTH))
	{
		printf(".");
		fflush(stdout);
		sleep(1);
	}
	printf("\n");

	assertAspect("LOCKOUT complete, red signal (top)", getSignal(signal), ASPECT_RED);
	assertAspect("LOCKOUT complete, red signal (bottom)", getSignal(signal+1), ASPECT_RED);

	gettimeofday(&tvEnd, NULL);
	printf("\nPass: %d\nFail: %d\n\n", pass, fail);
	timeval_subtract(&tvDiff, &tvEnd, &tvBegin);
	printf("Elapsed Time: %ld:%02ld\n\n", tvDiff.tv_sec / 60, tvDiff.tv_sec % 60);
}

void testArriveOpposingTurnout(int dir, int track)
{
	struct timeval tvBegin, tvEnd, tvDiff;
	gettimeofday(&tvBegin, NULL);
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
	
	assertAspect("Train arrives to opposing turnout, red signal", getSignal(signal), ASPECT_RED);
	assertAspect("Red signal on other track", getSignal(track?signal-1:signal+1), ASPECT_RED);

	if(DIR_EAST==dir)
	{
		turnouts = track?TURNOUT_WEST:0;
	}
	else
	{
		turnouts = track?TURNOUT_NORTH:0;
	}
	sendCommand();

	assertAspect("Turnout changed, proceed indication", getSignal(signal), track?ASPECT_YELLOW:ASPECT_GREEN);
	assertAspect("Red signal on other track", getSignal(track?signal-1:signal+1), ASPECT_RED);

	if(DIR_EAST==dir)
	{
		turnouts = track?0:TURNOUT_WEST;
	}
	else
	{
		turnouts = track?0:TURNOUT_NORTH;
	}
	sendCommand();

	assertAspect("Turnout changed back, red signal", getSignal(signal), ASPECT_RED);
	assertAspect("Red signal on other track", getSignal(track?signal-1:signal+1), ASPECT_RED);

	if(DIR_EAST==dir)
	{
		occupancy &= track?~OCC_WEST_SIDING:~OCC_WEST_MAIN;
	}
	else
	{
		occupancy &= track?~OCC_NORTH_SIDING:~OCC_NORTH_MAIN;
	}
	sendCommand();

	gettimeofday(&tvEnd, NULL);
	printf("\nPass: %d\nFail: %d\n\n", pass, fail);
	timeval_subtract(&tvDiff, &tvEnd, &tvBegin);
	printf("Elapsed Time: %ld:%02ld\n\n", tvDiff.tv_sec / 60, tvDiff.tv_sec % 60);
}

void testDebounceTimer(void)
{
	int i;
	struct timeval tvBegin, tvEnd, tvDiff;
	gettimeofday(&tvBegin, NULL);
	
	printf("\n");
	printf("--------------------------------------------------------------------------------\n");
	printf("Debounce Timer\n");
	printf("--------------------------------------------------------------------------------\n");

	clearAll();

	occupancy |= OCC_WEST_MAIN;
	sendCommand();
	assertAspect("Eastbound train arrives, gets green", getSignal(0), ASPECT_GREEN);

	occupancy |= OCC_INTERLOCKING;
	sendCommand();
	assertAspect("Enters interlocking, red signal", getSignal(0), ASPECT_RED);

	occupancy &= ~OCC_WEST_MAIN;
	sendCommand();
	assertAspect("Occupies only interlocking, red signal", getSignal(0), ASPECT_RED);

	occupancy |= OCC_NORTH_MAIN;
	sendCommand();
	assertAspect("Southbound train arrives, gets red", getSignal(4), ASPECT_RED);

	occupancy |= OCC_EAST_MAIN;
	sendCommand();
	assertAspect("Eastbound continues, red signal", getSignal(0), ASPECT_RED);
	assertAspect("Southbound still red", getSignal(4), ASPECT_RED);

	printf("Wait for timelock to expire");
	while(getTimelock())
	{
		printf(".");
		fflush(stdout);
		sleep(1);
	}
	printf("\n");

	occupancy &= ~OCC_INTERLOCKING;
	sendCommand();
	assertAspect("Eastbound clears interlocking, red signal", getSignal(0), ASPECT_RED);
	assertAspect("Southbound still red", getSignal(4), ASPECT_RED);

	assertFloat("DEBOUNCE timer running", getDebounce(), debounceTime-1, 1);
	
	printf("Waiting for DEBOUNCE to expire");
	for(i=0;i<debounceTime-1;i++)
	{
		printf(".");
		fflush(stdout);
		sleep(1);
	}
	printf("\n");

	assertInt("DEBOUNCE expired", getDebounce(), 0);
	sleep(1);
	assertAspect("Southbound now green", getSignal(4), ASPECT_GREEN);

	sleep(1);

	gettimeofday(&tvEnd, NULL);
	printf("\nPass: %d\nFail: %d\n\n", pass, fail);
	timeval_subtract(&tvDiff, &tvEnd, &tvBegin);
	printf("Elapsed Time: %ld:%02ld\n\n", tvDiff.tv_sec / 60, tvDiff.tv_sec % 60);
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
	writeEeprom(EE_TIMEOUT_SECONDS, timeoutTime);
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
    
	// eastbound_main, eastbound_siding
/*	testEastboundSouthbound(DIR_EAST, 0, 0);*/
/*	testEastboundSouthbound(DIR_EAST, 0, 1);*/
/*	testEastboundSouthbound(DIR_EAST, 1, 0);*/
/*	testEastboundSouthbound(DIR_EAST, 1, 1);*/
/*	testEastboundSouthbound(DIR_SOUTH, 0, 0);*/
/*	testEastboundSouthbound(DIR_SOUTH, 0, 1);*/
/*	testEastboundSouthbound(DIR_SOUTH, 1, 0);*/
/*	testEastboundSouthbound(DIR_SOUTH, 1, 1);*/
	
	// eastbound_turnout
/*	testArriveOpposingTurnout(DIR_EAST, 0);*/
/*	testArriveOpposingTurnout(DIR_EAST, 1);*/
/*	testArriveOpposingTurnout(DIR_SOUTH, 0);*/
/*	testArriveOpposingTurnout(DIR_SOUTH, 1);*/

	// westbound_main, westbound_siding
	testWestboundNorthbound(DIR_WEST, 0, 0);
/*	testWestboundNorthbound(DIR_WEST, 0, 1);*/
/*	testWestboundNorthbound(DIR_WEST, 1, 0);*/
/*	testWestboundNorthbound(DIR_WEST, 1, 1);*/
/*	testWestboundNorthbound(DIR_NORTH, 0, 0);*/
/*	testWestboundNorthbound(DIR_NORTH, 0, 1);*/
	testWestboundNorthbound(DIR_NORTH, 1, 0);
/*	testWestboundNorthbound(DIR_NORTH, 1, 1);*/
	
	// meet_before_interlocking
	
	// meet_after_interlocking
	
	// real_before_phantom
	
	// phantom_before_real
	
	// timelock_meet
	
	// timelock
	
	// lockout_expire_with_train_present
	
	// timeout_expire
	
	// timeout_expire_momentary_train
	
	// debounce_timer
/*	testDebounceTimer();*/
	
/*
	while(1)
	{
		testDebounceTimer();
		if(fail)
		{
			system("cp /mrbfs/interfaces/ci2/pktLog pktlog.$(date +%Y%m%d-%H%M%S)");
			break;
		}
	}
*/

	clearAll();
	
	gettimeofday(&tvEnd, NULL);

	printf("--------------------------------------------------------------------------------\n");

	printf("\nPass: %d\nFail: %d\n\n", pass, fail);
	
	timeval_subtract(&tvDiff, &tvEnd, &tvBegin);
	printf("Elapsed Time: %ld:%02ld\n\n", tvDiff.tv_sec / 60, tvDiff.tv_sec % 60);
	
	return 0;
}


