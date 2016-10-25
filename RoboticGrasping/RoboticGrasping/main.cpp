//#include<iostream>
//#include"iarmInterface.h"
//
//
//
//int main()
//{
//	float x, y, z, roll, pitch, yaw;
//	IARMInterface iarm;
//	iarm.init();
//	iarm.stockUnfold();
//	iarm.stockLiftDown();
//	iarm.stockMoveHomePosition();
//
//	while (true) {
//		if (iarm.check() == IARMInterface::CHECK_RESULT_FINISH) {
//			printf("x y z\n");
//			scanf("%f %f %f", &x, &y, &z);
//			if (x == 1000 || y == 1000 || z == 1000) break;
//			iarm.moveTo(x, y, z);
//		}
//	}
//	return 0;
//}

/*
* Copyright (c) 2010 by Exact Dynamics B.V.
*
* All rights reserved. No part of this software or
* source code may be used, reproduced, stored in a retrieval system, or
* transmitted, in any form or by any means, without prior written permission
* from Exact Dynamics B.V. Except as expressly provided in writing by Exact
* Dynamics B.V., Exact Dynamics B.V. does not grant any express or implied
* rights to use (any part of) this software & source code.
*
*  Exact Dynamics B.V. This software may only be used in combination
*  with a valid Transparent Mode Software License from Exact Dynamics B.V.
*  Exact Dynamics B.V. does not take any responsibility for the consequences
*  of the usage of this software and the iARM, this includes damage to the iARM
*  and/or its suroundings (including people).
*
*  By using this software, you agree to this agreement. If you do not agree
*  please contact Exact Dynamics B.V. and do not use this software or the iARM.
*
*  This Copyright notice may not be removed or modified without prior
*  written consent of esd gmbh.
*
*  Exact Dynamics B.V., reserves the right to modify this software
*  without notice.
*
*  For support, questions and/or suggestions do not hesitate to contact
*  use at research@exactdynamics.nl, after reading the manual.
*
*  Have fun!
*
*  Exact Dynamics B.V.
*
*  Chamber of Commerce: Arnhem, the Netherlands, 090705840000.
*/

#ifdef WIN32
#include <windows.h>
#include <conio.h>
#endif
#ifdef UNIX
#include <termios.h>
#include <unistd.h>
#include <string.h>

struct termios ots; /* copy of initial termios of tty */

#endif

#include <signal.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>


#include "iarm.h"

					/* CONSTANTS */

typedef enum IARM_CARTESIAN_DIMENSION { X, Y, Z, YAW, PITCH, ROLL } IARM_CARTESIAN_DIMENSION;
typedef enum IARM_JOINT { J1, J2, J3, J4, J5, J6, GRIPPER } IARM_JOINT;

/* DEFINES */

#define BOOL int
#define FALSE 0
#define TRUE 1

#define EXIT_ERROR				-1
#define EXIT_SUCCESS			0
#define IARM_NR_JOINTS			6

#define DEFAULT_CAN_PORT		0
#define STATUS_CHECK_INTERVAL	100

#define VELOCITY_STEP_LINEAR		10.0f
#define VELOCITY_STEP_ORIENTATION	0.1f
#define VELOCITY_STEP_JOINT			0.1f
#define VELOCITY_STEP_GRIPPER		0.01f

/* FORWARD DECLARATIONS */
void update_status(void);
void print_help(void);
void print_separator(void);
void ShowAllHelp(void);
void print_error(void);
BOOL read_arguments(int argc, char *argv[]);
void process_key_press(char key);
void mssleep(int ms);
BOOL keyboard_read_key(char *key);
void print_application_title(void);
void ttyset(void);
void ttyreset(int);

/* GLOBALS */

IARM_HANDLE g_hRobot = IARM_INVALID_HANDLE;
int			g_can_port = DEFAULT_CAN_PORT;
IARM_STATUS	g_status;

float linearVelocity[6] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
float jointVelocity[IARM_NR_JOINTS] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
float gripperVelocity = 0.0f;
float joint_position[IARM_NR_JOINTS] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
float gripper_position = 0.0f;
float positionHome[IARM_NR_JOINTS] = { 129.0f, -420.0f, 7.0f, 1.57f, 0.0f, -1.57f };
float positionZero[IARM_NR_JOINTS] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

int main(int argc, char *argv[])
{
	int status_counter = 0;
	char key;

	print_application_title();

	if (!read_arguments(argc, argv))
		return EXIT_ERROR;

	printf("Waiting for iARM to connect on CAN-bus %d...\n", g_can_port);

	g_hRobot = iarm_connect(g_can_port);
	if (IARM_INVALID_HANDLE == g_hRobot)
	{
		printf("Could not open CAN device on CAN-bus %d\n", g_can_port);
		return EXIT_ERROR;
	}
	ttyset();
	print_help();

	printf("!Robot connected\n");
	iarm_get_status(g_hRobot, &g_status);

	/* Main loop for example application.
	* Every <STATUS_CHECK_INTERVAL> cycles the status of the iARM is checked. */
	while (iarm_is_connected(g_hRobot) == IARM_SUCCESS)
	{
		/* read and process keyboard input */
		if (keyboard_read_key(&key))
			process_key_press(key);

		if (status_counter++ == STATUS_CHECK_INTERVAL)
		{
			update_status();
			status_counter = 0;
		}

		/* yield for a short time (required) */
		mssleep(10);
	}

	/* clean up */
	iarm_disconnect(g_hRobot);
	printf("iARM terminated.\nPress any key to quit...\n");

#ifdef WIN32
	_getch();
#endif
	ttyreset(0);

	return 0;
}

/* Retrieve actual status of the iARM and print a few messages related
* to the actual status, e.g. notification of movement finished and mechanical limits. */
void update_status()
{
	IARM_STATUS current_status;
	int i;

	iarm_get_status(g_hRobot, &current_status);

	/* Check if the movement_status has been changed and report the new state if this is the case */
	if (current_status.movement_status != g_status.movement_status)
	{
		switch (current_status.movement_status)
		{
		case IARM_MOVEMENT_FINISHED:
			printf("!Movement finished\n");
			break;
		case IARM_MOVEMENT_LIMIT:
			printf("!Mechanical limit\n");
			break;
		default:
			break;
		}
	}

	/* Check if we have a (resolved) joint block and did not report it already */
	for (i = 0; i < IARM_NR_JOINTS; ++i)
	{
		if ((current_status.blocked_status[i] != g_status.blocked_status[i]) &&
			(current_status.blocked_status[i] != IARM_BLOCK_NONE))
			printf("!Block on joint %d\n", i + 1);
		else if ((current_status.blocked_status[i] != g_status.blocked_status[i]) &&
			(current_status.blocked_status[i] == IARM_BLOCK_NONE))
			printf("!Block on joint %d resolved\n", i + 1);
	}

	/* Check if we have a gripper block and did not report it already */
	if ((current_status.blocked_status[GRIPPER] != g_status.blocked_status[GRIPPER]) &&
		(current_status.blocked_status[GRIPPER] != IARM_BLOCK_NONE))
		printf("!Gripper closed\n");

	memcpy(&g_status, &current_status, sizeof(IARM_STATUS));
}

void process_key_press(char key)
{
	IARM_RESULT result = IARM_SUCCESS;
	int i;

	switch (key)
	{
		/* XYZ */
	case 'a':
		linearVelocity[X] += VELOCITY_STEP_LINEAR;
		printf(" > X-Axis+: %f\n", linearVelocity[X]);
		result = iarm_move_direction_linear(g_hRobot, linearVelocity);
		break;
	case 'z':
		linearVelocity[X] -= VELOCITY_STEP_LINEAR;
		printf(" > X-Axis-: %f\n", linearVelocity[X]);
		result = iarm_move_direction_linear(g_hRobot, linearVelocity);
		break;
	case 's':
		linearVelocity[Y] += VELOCITY_STEP_LINEAR;
		printf(" > Y-Axis+: %f\n", linearVelocity[Y]);
		result = iarm_move_direction_linear(g_hRobot, linearVelocity);
		break;
	case 'x':
		linearVelocity[Y] -= VELOCITY_STEP_LINEAR;
		printf(" > Y-Axis-: %f\n", linearVelocity[Y]);
		result = iarm_move_direction_linear(g_hRobot, linearVelocity);
		break;
	case 'd':
		linearVelocity[Z] += VELOCITY_STEP_LINEAR;
		printf(" > Z-Axis+: %f\n", linearVelocity[Z]);
		result = iarm_move_direction_linear(g_hRobot, linearVelocity);
		break;
	case 'c':
		linearVelocity[Z] -= VELOCITY_STEP_LINEAR;
		printf(" > Z-Axis-: %f\n", linearVelocity[Z]);
		result = iarm_move_direction_linear(g_hRobot, linearVelocity);
		break;
	case 'f':
		linearVelocity[YAW] += VELOCITY_STEP_ORIENTATION;
		printf(" > Yaw+   : %f\n", linearVelocity[YAW]);
		result = iarm_move_direction_linear(g_hRobot, linearVelocity);
		break;
	case 'v':
		linearVelocity[YAW] -= VELOCITY_STEP_ORIENTATION;
		printf(" > Yaw-   : %f\n", linearVelocity[YAW]);
		result = iarm_move_direction_linear(g_hRobot, linearVelocity);
		break;
	case 'g':
		linearVelocity[PITCH] += VELOCITY_STEP_ORIENTATION;
		printf(" > Pitch+ : %f\n", linearVelocity[PITCH]);
		result = iarm_move_direction_linear(g_hRobot, linearVelocity);
		break;
	case 'b':
		linearVelocity[PITCH] -= VELOCITY_STEP_ORIENTATION;
		printf(" > Pitch- : %f\n", linearVelocity[PITCH]);
		result = iarm_move_direction_linear(g_hRobot, linearVelocity);
		break;
	case 'h':
		linearVelocity[ROLL] += VELOCITY_STEP_ORIENTATION;
		printf(" > Roll+  : %f\n", linearVelocity[ROLL]);
		result = iarm_move_direction_linear(g_hRobot, linearVelocity);
		break;
	case 'n':
		linearVelocity[ROLL] -= VELOCITY_STEP_ORIENTATION;
		printf(" > Roll-  : %f\n", linearVelocity[ROLL]);
		result = iarm_move_direction_linear(g_hRobot, linearVelocity);
		break;
	case 'o':
		printf(" > Unfolding\n");
		result = iarm_unfold(g_hRobot);
		break;
	case 'i':
		printf(" > Folding\n");
		result = iarm_fold(g_hRobot);
		break;
	case 27: /* <ESCAPE> key */
		printf(" > Terminating...\n");
		iarm_disconnect(g_hRobot);
		break;
	case ';': /* GO TO HOME POSITION */
	{
		printf(" > Home   : cart (fold-out)\n");
		result = iarm_move_position_linear(g_hRobot, positionHome);
	}
	break;
	case '0': // zero-position
	{
		printf(" > Home   : zero position\n");
		result = iarm_move_position_joint(g_hRobot, positionZero, 0.0f, IARM_LIFT_KEEP_POS);
	}
	break;
	case '1':
		jointVelocity[J1] += VELOCITY_STEP_JOINT;
		printf(" > Joint1+: %f\n", jointVelocity[J1]);
		result = iarm_move_direction_joint(g_hRobot, jointVelocity, gripperVelocity);
		break;
	case '2':
		jointVelocity[J2] += VELOCITY_STEP_JOINT;
		printf(" > Joint2+:  %f\n", jointVelocity[J2]);
		result = iarm_move_direction_joint(g_hRobot, jointVelocity, gripperVelocity);
		break;
	case '3':
		jointVelocity[J3] += VELOCITY_STEP_JOINT;
		printf(" > Joint3+:  %f\n", jointVelocity[J3]);
		result = iarm_move_direction_joint(g_hRobot, jointVelocity, gripperVelocity);
		break;
	case '4':
		jointVelocity[J4] += VELOCITY_STEP_JOINT;
		printf(" > Joint4+:  %f\n", jointVelocity[J4]);
		result = iarm_move_direction_joint(g_hRobot, jointVelocity, gripperVelocity);
		break;
	case '5':
		jointVelocity[J5] += VELOCITY_STEP_JOINT;
		printf(" > Joint5+:  %f\n", jointVelocity[J5]);
		result = iarm_move_direction_joint(g_hRobot, jointVelocity, gripperVelocity);
		break;
	case '6':
		jointVelocity[J6] += VELOCITY_STEP_JOINT;
		printf(" > Joint6+:  %f\n", jointVelocity[J6]);
		result = iarm_move_direction_joint(g_hRobot, jointVelocity, gripperVelocity);
		break;
	case '7':
		gripperVelocity += VELOCITY_STEP_GRIPPER;
		printf(" > Gripper+:  %f\n", gripperVelocity);
		result = iarm_move_direction_joint(g_hRobot, jointVelocity, gripperVelocity);
		break;
	case 'q':
		jointVelocity[J1] -= VELOCITY_STEP_JOINT;
		printf(" > Joint1-:  %f\n", jointVelocity[J1]);
		result = iarm_move_direction_joint(g_hRobot, jointVelocity, gripperVelocity);
		break;
	case 'w':
		jointVelocity[J2] -= VELOCITY_STEP_JOINT;
		printf(" > Joint2-:  %f\n", jointVelocity[J2]);
		result = iarm_move_direction_joint(g_hRobot, jointVelocity, gripperVelocity);
		break;
	case 'e':
		jointVelocity[J3] -= VELOCITY_STEP_JOINT;
		printf(" > Joint3-: %f\n", jointVelocity[J3]);
		result = iarm_move_direction_joint(g_hRobot, jointVelocity, gripperVelocity);
		break;
	case 'r':
		jointVelocity[J4] -= VELOCITY_STEP_JOINT;
		printf(" > Joint4-: %f\n", jointVelocity[J4]);
		result = iarm_move_direction_joint(g_hRobot, jointVelocity, gripperVelocity);
		break;
	case 't':
		jointVelocity[J5] -= VELOCITY_STEP_JOINT;
		printf(" > Joint5-: %f\n", jointVelocity[J5]);
		result = iarm_move_direction_joint(g_hRobot, jointVelocity, gripperVelocity);
		break;
	case 'y':
		jointVelocity[J6] -= VELOCITY_STEP_JOINT;
		printf(" > Joint6-: %f\n", jointVelocity[J6]);
		result = iarm_move_direction_joint(g_hRobot, jointVelocity, gripperVelocity);
		break;
	case 'u':
		gripperVelocity -= VELOCITY_STEP_GRIPPER;
		printf(" > Gripper-: %f\n", gripperVelocity);
		result = iarm_move_direction_joint(g_hRobot, jointVelocity, gripperVelocity);
		break;
	case 'p':
		if (iarm_get_status(g_hRobot, &g_status) == IARM_FAILED)
		{
			print_error();
			break;
		}
		printf(" * Showing position information:\n");
		printf(" * ");
		for (i = 0; i < 6; i++)
			printf("%f ", g_status.cartesian_position[i]);
		printf("\n");
		printf(" * ");
		for (i = 0; i < IARM_NR_JOINTS; ++i)
			printf("%f ", g_status.joint_position[i]);
		printf("\n");
		printf(" * Gripper opening: %f%%\n", g_status.gripper_opening*100.0f);
		break;
	case 'l':
		printf(" > Lift   : UP\n");
		result = iarm_get_status(g_hRobot, &g_status);
		for (i = 0; i < IARM_NR_JOINTS; ++i)
			joint_position[i] = g_status.joint_position[i];
		result = iarm_move_position_joint(g_hRobot, joint_position, g_status.gripper_opening, IARM_LIFT_UP);
		break;
	case 'k':
		printf(" > Lift   : STOP\n");
		result = iarm_move_stop(g_hRobot);
		break;
	case 'j':
		printf(" > Lift   : DOWN\n");
		result = iarm_get_status(g_hRobot, &g_status);
		for (i = 0; i < IARM_NR_JOINTS; i++)
			joint_position[i] = g_status.joint_position[i];
		result = iarm_move_position_joint(g_hRobot, joint_position, g_status.gripper_opening, IARM_LIFT_DOWN);
		break;
	case '?':
		print_help();
		break;
	default:
		printf(" > Stop\n");
		result = iarm_move_stop(g_hRobot);
		for (i = 0; i < IARM_NR_JOINTS; i++)
			jointVelocity[i] = 0.0f;
		for (i = 0; i < 6; i++)
			linearVelocity[i] = 0.0f;
		gripperVelocity = 0.0f;
		break;
	}
	if (result == IARM_FAILED)
		print_error();
}

void print_help(void)
{
	printf("'?'   Help                 \n");
	printf("'Esc' Quit                 \n");
	printf("'p'   Show positions       \n");
	printf("'m'   Setting a mark line  \n");
	print_separator();
	printf(" Linear movements:          Joint movements:\n");
	printf(" 'd'  Move Up         '1'/'q' Joint 1 +/-\n");
	printf(" 'c'  Move Down       '2'/'w' Joint 2 +/-\n");
	printf(" 'a'  Move Forward    '3'/'e' Joint 3 +/-\n");
	printf(" 'z'  Move Backward   '4'/'r' Joint 4 +/-\n");
	printf(" 's'  Move Left       '5'/'t' Joint 5 +/-\n");
	printf(" 'x'  Move Right      '6'/'y' Joint 6 +/-\n");
	printf("  Orientation:        '7'/'u' Joint 7 +/-\n");
	printf(" 'f'  Yaw up          '7'     close gripper\n");
	printf(" 'v'  Yaw down        'u'     open gripper\n");
	printf("                      'o'     fold-out\n");
	printf(" 'g'  Pitch xx        'i'     fold-in\n");
	printf(" 'b'  Pitch xx        '0'     goto home (Joint)\n");
	printf(" 'h'  Roll Left       ';'     goto home (cart)\n");
	printf(" 'n'  Roll Right      'l'/'j' lift UP/DOWN\n");
	printf("                      'k'     lift stop\n");
	printf(" press any other key to stop all movements!\n");
	print_separator();
}

void print_application_title(void)
{
	int versionMajor, versionMinor, versionBuild;
	iarm_get_library_version(&versionMajor, &versionMinor, &versionBuild);
	printf("i A R M (library version %d.%d.%d)\n\n", versionMajor, versionMinor, versionBuild);
}

void print_separator(void)
{
	printf("------------------------------------------------------------\n");
}

void ShowAllHelp(void)
{
	printf("iARM HELP\n");
	printf(" commandline parameters:\n");
	print_separator();
	printf(" -p=<port>    -- CAN port used to connect to the ARM.\n");
	printf(" -g_hRobot           -- Show this message.\n");
	print_separator();
	printf(" iARM runtime commands:\n");
	print_separator();
	print_help();
	print_separator();
}

void print_error(void)
{
	printf("Error %d reported by iARM: %s\n", iarm_errno(g_hRobot), iarm_error(g_hRobot));
	iarm_disconnect(g_hRobot);
}

int read_arguments(int argc, char *argv[])
{
	int i;
	char prm;
	char *ptr;
	BOOL arg;

	for (i = 1; i < argc; i++)
	{
		ptr = argv[i];
		while (*ptr == '-')
			ptr++;
		prm = *(ptr++);
		arg = (*ptr == '=');
		if (arg) ptr++;

		switch (prm)
		{
		case 'p':
			if (arg)
				g_can_port = strtoul(ptr, NULL, 10);
			else
			{
				printf("please supply a port number with -p (e.g. '-p=1')\n");
				return FALSE;
			}
			break;
		case '?':
		case 'h':
			ShowAllHelp();
			return FALSE;
		default:
			printf("\"%s\": unknown commandline argument: %d\n", argv[0], prm);
			return FALSE;
		}
	}
	return TRUE;
}

/* sleep for the provided amount of milliseconds */
void mssleep(int ms)
{
#ifdef WIN32
	Sleep(ms);
#endif // WIN32
#ifdef UNIX
	struct timeval tv;
	tv.tv_sec = 0;
	tv.tv_usec = (ms * 1000);
	select(0, NULL, NULL, NULL, &tv);
#endif // UNIX
}

/* returns TRUE if a key has been pressed, FALSE otherwise. If a key has been pressed, the
* corresponding ASCII code for that key is copied to the provided key address */
BOOL keyboard_read_key(char *key)
{
#ifdef UNIX
	if (read(STDIN_FILENO, key, 1) < 1)
		return FALSE;
	return TRUE;
#endif
#ifdef WIN32
	if (_kbhit() == 0)
		return FALSE;
	*key = _getch();
	return TRUE;
#endif
}

/* configures the TTY (only in UNIX) for reading keyboard input */
void ttyset(void)
{
#ifdef UNIX
	struct termios ts;
	struct sigaction sact;
	tcgetattr(STDIN_FILENO, &ts);
	ots = ts;
	ts.c_lflag &= ~ICANON; /* raw data mode */
	ts.c_lflag &= ~(ECHO | ECHOCTL | ECHONL); /* no echo */
	ts.c_lflag |= IEXTEN;

	/* restore tty after these signals */
	sact.sa_handler = ttyreset;
	sigaction(SIGHUP, &sact, NULL);
	sigaction(SIGINT, &sact, NULL);
	sigaction(SIGPIPE, &sact, NULL);
	sigaction(SIGTERM, &sact, NULL);
	tcsetattr(STDIN_FILENO, TCSANOW, &ts); /* set raw data mode */
	fcntl(STDIN_FILENO, F_SETFL, fcntl(STDIN_FILENO, F_GETFL, 0) | O_NONBLOCK); /*make stdin non blocking */
	fcntl(STDOUT_FILENO, F_SETFL, fcntl(STDOUT_FILENO, F_GETFL, 0) | O_NONBLOCK); /*make stdout non blocking*/
#endif
}

/* resets the TTY configuration (only in UNIX) that was changed in the ttyset function */
void ttyreset(int signal)
{
#ifdef UNIX
	tcsetattr(STDIN_FILENO, TCSANOW, &ots);
	tcsetattr(STDOUT_FILENO, TCSANOW, &ots);
#endif
	exit(signal);
}

