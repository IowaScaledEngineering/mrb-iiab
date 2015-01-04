#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define MRBFS_PATH "/mrbfs/bus0/0xC0-mrb-iiab"

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

void textcolor(int attr, int fg, int bg)
{
	printf("%c[%d;%d;%dm", 0x1B, attr, fg + 30, bg + 40);
}

void clearscreen()
{
	printf("%c[%dJ", 0x1B, 2);
}

void clearline()
{
	printf("%c[K", 0x1B);
}

void hidecursor(void)
{
	printf("%c[?%dl", 0x1B, 25);
}

void movecursor(int x, int y)
{
	printf("%c[%d;%dH", 0x1B, x, y);
}

void drawSignal(int n)
{
	FILE *fptr;
	char fname[256];

/*	movecursor(1,1);*/
/*	printf("%d", n);*/
	
	switch(n)
	{
		case 0:
			movecursor(9,23);
			printf("|--");
			break;
		case 1:
			movecursor(7,23);
			printf("|--");
			break;
		case 2:
			movecursor(9,52);
			printf("-");
			movecursor(9,51);
			break;
		case 3:
			movecursor(9,54);
			printf("--|");
			movecursor(9,53);
			break;
		case 4:
			movecursor(4,54);
			printf("--|");
			movecursor(4,53);
			break;
		case 5:
			movecursor(6,54);
			printf("--|");
			movecursor(6,53);
			break;
		case 6:
			movecursor(14,27);
			printf("-");
			break;
		case 7:
			movecursor(14,23);
			printf("|--");
			break;
	}
	
	snprintf(fname, sizeof(fname), "%s/signalhead_%d", MRBFS_PATH, n);
	fptr = fopen(fname, "r");
	switch(fgetc(fptr))
	{
		case 'R':
			textcolor(RESET, RED, BLACK);
			break;
		case 'Y':
			textcolor(RESET, YELLOW, BLACK);
			break;
		case 'G':
			textcolor(RESET, GREEN, BLACK);
			break;
	}
	fclose(fptr);

	printf("@");
	textcolor(RESET, WHITE, BLACK);
}

int getOccupancy(char *fname)
{
	int c;
	FILE *fptr;
	fptr = fopen(fname, "r");
	c = fgetc(fptr);
	fclose(fptr);
	return(c);
}

void drawOccupancy()
{
	char fname[256];

	snprintf(fname, sizeof(fname), "%s/occ_dir0_main", MRBFS_PATH);
	if(getOccupancy(fname) == '1')
		textcolor(RESET, BLUE, BLACK);
	else
		textcolor(RESET, WHITE, BLACK);
	movecursor(8,1);
	printf("_____________________________");
	textcolor(RESET, WHITE, BLACK);

	snprintf(fname, sizeof(fname), "%s/occ_dir0_siding", MRBFS_PATH);
	if(getOccupancy(fname) == '1')
		textcolor(RESET, BLUE, BLACK);
	else
		textcolor(RESET, WHITE, BLACK);
	movecursor(6,1);
	printf("_____________________________");
	movecursor(7,30);
	printf("\\");
	textcolor(RESET, WHITE, BLACK);

	snprintf(fname, sizeof(fname), "%s/occ_dir1_main", MRBFS_PATH);
	if(getOccupancy(fname) == '1')
		textcolor(RESET, BLUE, BLACK);
	else
		textcolor(RESET, WHITE, BLACK);
	movecursor(8,49);
	printf("_____________________________");
	textcolor(RESET, WHITE, BLACK);

	snprintf(fname, sizeof(fname), "%s/occ_dir2_main", MRBFS_PATH);
	if(getOccupancy(fname) == '1')
		textcolor(RESET, BLUE, BLACK);
	else
		textcolor(RESET, WHITE, BLACK);
	movecursor(3,44);
	printf("__________________________________");
	movecursor(4,43);
	printf("/");
	textcolor(RESET, WHITE, BLACK);

	snprintf(fname, sizeof(fname), "%s/occ_dir2_siding", MRBFS_PATH);
	if(getOccupancy(fname) == '1')
		textcolor(RESET, BLUE, BLACK);
	else
		textcolor(RESET, WHITE, BLACK);
	movecursor(5,44);
	printf("__________________________________");
	textcolor(RESET, WHITE, BLACK);

	snprintf(fname, sizeof(fname), "%s/occ_dir3_main", MRBFS_PATH);
	if(getOccupancy(fname) == '1')
		textcolor(RESET, BLUE, BLACK);
	else
		textcolor(RESET, WHITE, BLACK);
	movecursor(13,1);
	printf("_________________________________/");
	textcolor(RESET, WHITE, BLACK);

	snprintf(fname, sizeof(fname), "%s/occ_interlocking_plant", MRBFS_PATH);
	if(getOccupancy(fname) == '1')
		textcolor(RESET, BLUE, BLACK);
	else
		textcolor(RESET, WHITE, BLACK);
	movecursor(8,32);
	printf("_______/_______");

	movecursor(11,36);
	printf("/");
	movecursor(10,37);
	printf("/");
	movecursor(9,38);
	printf("/");
	movecursor(8,39);
	printf("/");
	movecursor(7,40);
	printf("/");
	movecursor(6,41);
	printf("/");
	textcolor(RESET, WHITE, BLACK);
}

void drawTurnout(int n)
{
	FILE *fptr;
	char fname[256];
	int state;

	snprintf(fname, sizeof(fname), "%s/turnout_dir%d", MRBFS_PATH, n);
	fptr = fopen(fname, "r");
	if(fgetc(fptr) == 'd')
		state = 1;
	else
		state = 0;
	fclose(fptr);

	switch(n)
	{
		case 0:
			if(state)
			{
				// Diverging
				textcolor(RESET, YELLOW, BLACK);
				movecursor(8,30);
				printf(" \\");
				textcolor(RESET, WHITE, BLACK);
			}
			else
			{
				// Main
				textcolor(RESET, YELLOW, BLACK);
				movecursor(8,30);
				printf("__");
				textcolor(RESET, WHITE, BLACK);
			}
			break;
		case 1:
			if(0)
			{
				// Diverging
				textcolor(RESET, YELLOW, BLACK);
				movecursor(8,47);
				printf("  ");
				movecursor(9,47);
				printf("\\");
				textcolor(RESET, WHITE, BLACK);
			}
			else
			{
				// Main
				textcolor(RESET, YELLOW, BLACK);
				movecursor(8,47);
				printf("__");
				movecursor(9,47);
				printf(" ");
				textcolor(RESET, WHITE, BLACK);
			}
			break;
		case 2:
			if(state)
			{
				// Diverging
				textcolor(RESET, YELLOW, BLACK);
				movecursor(5,42);
				printf("__");
				textcolor(RESET, WHITE, BLACK);
			}
			else
			{
				// Main
				textcolor(RESET, YELLOW, BLACK);
				movecursor(5,42);
				printf("/ ");
				textcolor(RESET, WHITE, BLACK);
			}
			break;
		case 3:
			if(0)
			{
				// Diverging
				textcolor(RESET, YELLOW, BLACK);
				movecursor(12,35);
				printf(" ");
				movecursor(11,34);
				printf("__");
				textcolor(RESET, WHITE, BLACK);
			}
			else
			{
				// Main
				textcolor(RESET, YELLOW, BLACK);
				movecursor(12,35);
				printf("/");
				movecursor(11,34);
				printf("  ");
				textcolor(RESET, WHITE, BLACK);
			}
			break;
	}
}

void drawTimers(void)
{
	FILE *fptr;
	char fname[256], data[16];

	movecursor(12,40);
	snprintf(fname, sizeof(fname), "%s/timelockTimer", MRBFS_PATH);
	fptr = fopen(fname, "r");
	if(fgets(data, sizeof(data), fptr))
		printf("Timelock: %d", atoi(data));
	else
		printf("Timelock: Err");
	clearline();
	fclose(fptr);

	movecursor(9,40);
	snprintf(fname, sizeof(fname), "%s/debounceTimer", MRBFS_PATH);
	fptr = fopen(fname, "r");
	if(fgets(data, sizeof(data), fptr))
		printf("%-3d", atoi(data));
	else
		printf("Err");
	fclose(fptr);
}

void drawState(int n)
{
	FILE *fptr;
	char fname[256], data[16], stateString[32], stateStringTemp[32];
	int state = 0;
	int line=0, right=0, lockout=0;

	snprintf(fname, sizeof(fname), "%s/stateMachine%d", MRBFS_PATH, n);
	fptr = fopen(fname, "r");
	if(fgets(data, sizeof(data), fptr))
		state = atoi(data);
	fclose(fptr);

	snprintf(fname, sizeof(fname), "%s/lockoutStatus%d", MRBFS_PATH, n);
	fptr = fopen(fname, "r");
	if(fgets(data, sizeof(data), fptr))
		lockout = atoi(data);
	fclose(fptr);

	switch(n)
	{
		case 0:
			line = 7;
			right = 0;
			break;
		case 1:
			line = 9;
			right = 1;
			break;
		case 2:
			line = 4;
			right = 1;
			break;
		case 3:
			line = 12;
			right = 0;
			break;
	}
	
	switch(state)
	{
		case 0:
			strncpy(stateStringTemp, "IDLE", sizeof(stateStringTemp));
			break;
		case 1:
			strncpy(stateStringTemp, "CLEARANCE", sizeof(stateStringTemp));
			break;
		case 2:
			strncpy(stateStringTemp, "TIMEOUT", sizeof(stateStringTemp));
			break;
		case 3:
			strncpy(stateStringTemp, "TIMER", sizeof(stateStringTemp));
			break;
		case 4:
			strncpy(stateStringTemp, "OCCUPIED", sizeof(stateStringTemp));
			break;
		case 5:
			strncpy(stateStringTemp, "LOCKOUT", sizeof(stateStringTemp));
			break;
		case 6:
			strncpy(stateStringTemp, "CLEAR", sizeof(stateStringTemp));
			break;
	}
	
	if(right)
		sprintf(stateString, "%9s", stateStringTemp);
	else
		sprintf(stateString, "%-9s", stateStringTemp);
	
	movecursor(line, right?77-strlen(stateString):2);
	printf("%s", stateString);

	if(lockout)
		sprintf(stateString, "LOCKED");
	else
		sprintf(stateString, "      ");

	movecursor(line, right?59:14);
	if(lockout)
		printf("LOCKED");
	else
		printf("      ");
}

void drawSounds(void)
{
	FILE *fptr;
	char fname[256], data[16];
	int i;

	for(i=0; i<2; i++)
	{
		movecursor(4,7+(i*9));
		snprintf(fname, sizeof(fname), "%s/soundOut%d", MRBFS_PATH, i);
		fptr = fopen(fname, "r");
		if(fgets(data, sizeof(data), fptr))
		{
			if(data[0]-'0')
				textcolor(RESET, GREEN, BLACK);
			printf("Sound%d", i);
			textcolor(RESET, WHITE, BLACK);
		}
		else
			printf("Err");
		fclose(fptr);
	}
}

int main(void)
{
	int i;
	
	hidecursor();
	clearscreen();
	movecursor(1,1);
	textcolor(RESET, WHITE, BLACK);
/*	printf("\n");*/
/*	printf("\n");*/
/*	printf("                                           __________________________________\n");*/
/*	printf("                                          /       @--|                       \n");*/
/*	printf("                                         /___________________________________\n");*/
/*	printf("_____________________________           /         @--|                       \n");*/
/*	printf("                      |--@   \\         /                                     \n");*/
/*	printf("______________________________\\_______/______________________________________\n");*/
/*	printf("                      |--@           /          @-@--|                       \n");*/
/*	printf("                                    /                                        \n");*/
/*	printf("                                   /                                         \n");*/
/*	printf("                                  /                                          \n");*/
/*	printf("_________________________________/                                           \n");*/
/*	printf("                      |--@-@                                                 \n");*/
/*	printf("                                                                             \n");*/
/*	printf("                                                                             \n");*/

	while(1)
	{
		for(i=0; i<8; i++)
			drawSignal(i);
		for(i=0; i<4; i++)
			drawOccupancy(i);
		for(i=0; i<4; i++)
			drawTurnout(i);
		for(i=0; i<4; i++)
			drawState(i);

		drawTimers();
		drawSounds();

		movecursor(1,1);
		
//		sleep(1);	
	}
	return 0;
}

