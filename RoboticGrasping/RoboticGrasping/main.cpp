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

#ifdef WIN32
#include <WinSock2.h>
#include <WS2tcpip.h>
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

#include <Eigen/Dense>


					/* CONSTANTS */
					// Enumeration for catesian dimension system
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

IARM_HANDLE g_hRobot = IARM_INVALID_HANDLE;

// variable for a can port number (default value is 0)
int			g_can_port = DEFAULT_CAN_PORT;

// Variable for status (global)
IARM_STATUS	g_status;

float linearVelocity[6] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
float jointVelocity[IARM_NR_JOINTS] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
float gripperVelocity = 0.0f;
float joint_position[IARM_NR_JOINTS] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
float gripper_position = 0.0f;
// Home position
float positionHome[IARM_NR_JOINTS] = { 129.0f, -420.0f, 7.0f, 1.57f, 0.0f, -1.57f };
// Zero position
float positionZero[IARM_NR_JOINTS] = { 240.0f, 0.0f, 0.0f, 1.57f, 1.57f, 0.0f };

// Variable for socket communication
SOCKET sock;
struct sockaddr_in dest1;
char buf[1024];
WSADATA wsaData;

// Cheack if position is valid
bool isPositionValid(Eigen::Vector4f& position)
{
	float limitPosiX = 0.0f;
	float limitNegaX = 0.0f;
	float limitPosiY = 0.0f;
	float limitNegaY = 0.0f;
	float limitPosiZ = 0.0f;
	float limitNegaZ = 0.0f;

	return true;
}


// MAIN FUNCTION
int main(int argc, char *argv[])
{
	// Initialize status_counter 0
	int status_counter = 0;
	// Variable for keyboard input
	char key;

	//	Print library version 
	print_application_title();

	// Terminate the program when arguments is inaccurate 
	if (!read_arguments(argc, argv))
		return EXIT_ERROR;

	// Print a can port to connect iARM 
	printf("Waiting for iARM to connect on CAN-bus %d...\n", g_can_port);

	// Make connections between iARM and PC 
	g_hRobot = iarm_connect(g_can_port);

	// Initialize socket 
	WSAStartup(MAKEWORD(2, 0), &wsaData);
	sock = socket(AF_INET, SOCK_DGRAM, 0);
	dest1.sin_family = AF_INET;
	dest1.sin_addr.S_un.S_addr = inet_addr("127.0.0.1");
	dest1.sin_port = htons(11111);

	// Terminate the program when connection is failed
	if (IARM_INVALID_HANDLE == g_hRobot)
	{
		printf("Send\n");

		// Write text to socket
		memset(buf, 0, sizeof(buf));
		_snprintf(buf, sizeof(buf), "data to port 11111");
		// Send socket
		sendto(sock,
			buf, strlen(buf), 0, (struct sockaddr *)&dest1, sizeof(dest1));

		// Print error messeage and terminate program
		printf("Could not open CAN device on CAN-bus %d\n", g_can_port);
		return EXIT_ERROR;
	}

	// Configure TTY for keyboard input (only in UNIX)
	ttyset();

	// Print runtime parameters
	print_help();

	// Get the current iarm status
	printf("!Robot connected\n");
	iarm_get_status(g_hRobot, &g_status);

	// Main loop for example application.
	// Do loops until connection between iARM and PC is fine 
	while (iarm_is_connected(g_hRobot) == IARM_SUCCESS)
	{
		// Read keyboard input 
		if (keyboard_read_key(&key))
			// Execute commands
			process_key_press(key);

		// Every <STATUS_CHECK_INTERVAL> cycles the status of the iARM is checked. 
		if (status_counter++ == STATUS_CHECK_INTERVAL)
		{
			// Update status
			update_status();
			// Set counter back to 0
			status_counter = 0;
		}
		
		// Sleep 10 milliseconds
		mssleep(10);
	}
	
	// Clean up iARM
	iarm_disconnect(g_hRobot);
	printf("iARM terminated.\nPress any key to quit...\n");

	// Clean up socket
	closesocket(sock);

	WSACleanup();

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
	// Variable for the current status
	IARM_STATUS current_status;
	// Variable for the loop counter
	int i;

	// Get the current status
	iarm_get_status(g_hRobot, &current_status);

	// Cheack whether status is changed or not
	if (current_status.movement_status != g_status.movement_status)
	{
		// Print the new state 
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

	// Print whether joint is blocked or resolved
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

	// Update the current status to global status
	memcpy(&g_status, &current_status, sizeof(IARM_STATUS));
}

// Function to process keyinput
void process_key_press(char key)
{
	// 
	IARM_RESULT result = IARM_SUCCESS;
	int i;

	switch (key)
	{
		/* XYZ */
		// Move forward in X dimention according to linearVelocity[0]
	case 'a':
		linearVelocity[X] += VELOCITY_STEP_LINEAR;
		// Print the current X velocity
		printf(" > X-Axis+: %f\n", linearVelocity[X]);
		// Move and change orientation with a given velocity
		result = iarm_move_direction_linear(g_hRobot, linearVelocity);
		break;
		// Move backward in X dimention according to linearVelocity[0]
	case 'z':
		linearVelocity[X] -= VELOCITY_STEP_LINEAR;
		printf(" > X-Axis-: %f\n", linearVelocity[X]);
		result = iarm_move_direction_linear(g_hRobot, linearVelocity);
		break;
		// Move forward in Y dimention according to linearVelocity[1]
	case 's':
		linearVelocity[Y] += VELOCITY_STEP_LINEAR;
		printf(" > Y-Axis+: %f\n", linearVelocity[Y]);
		result = iarm_move_direction_linear(g_hRobot, linearVelocity);
		break;
		// Move backward in Y dimention according to linearVelocity[1]
	case 'x':
		linearVelocity[Y] -= VELOCITY_STEP_LINEAR;
		printf(" > Y-Axis-: %f\n", linearVelocity[Y]);
		result = iarm_move_direction_linear(g_hRobot, linearVelocity);
		break;
		// Move forward in Z dimention according to linearVelocity[2]
	case 'd':
		linearVelocity[Z] += VELOCITY_STEP_LINEAR;
		printf(" > Z-Axis+: %f\n", linearVelocity[Z]);
		result = iarm_move_direction_linear(g_hRobot, linearVelocity);
		break;
		// Move backward in Z dimention according to linearVelocity[2]
	case 'c':
		linearVelocity[Z] -= VELOCITY_STEP_LINEAR;
		printf(" > Z-Axis-: %f\n", linearVelocity[Z]);
		result = iarm_move_direction_linear(g_hRobot, linearVelocity);
		break;

		// 
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

		// Unfold iARM
	case 'o':
		printf(" > Unfolding\n");
		result = iarm_unfold(g_hRobot);
		break;
		// Fold iARM
	case 'i':
		printf(" > Folding\n");
		result = iarm_fold(g_hRobot);
		break;

		// Terminate the program
	case 27: /* <ESCAPE> key */
		printf(" > Terminating...\n");
		iarm_disconnect(g_hRobot);
		break;

		// REQUEST OBJECT RECOGNIZER
		// GET POSITION AND POSE OF OBJECT TO GRASP
	case ';':
	{
		printf(" Requesting object vector to the recognizer...");
		// Variable for the position of the object 
		Eigen::Vector4f positionOfobject;
		// Write text to socket
		memset(buf, 0, sizeof(buf));
		_snprintf(buf, sizeof(buf), "Request for object vector");
		// Send socket
		sendto(sock,
			buf, strlen(buf), 0, (struct sockaddr *)&dest1, sizeof(dest1));

		if (isPositionValid(positionOfobject)) {
			break;
		}
	}
	break;
	// Move to zero-position
	case '0': 
	{
		printf(" > Home   : zero position\n");
		result = iarm_move_position_joint(g_hRobot, positionZero, 0.0f, IARM_LIFT_KEEP_POS);
	}
	break;

	// Joint control
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

		// Print help
	case '?':
		print_help();
		break;

		// Stop any movements by pressing non-command key
	default:
		printf(" > Stop\n");
		// Get the current status
		result = iarm_move_stop(g_hRobot);

		// Set all velocity 0 
		for (i = 0; i < IARM_NR_JOINTS; i++)
			jointVelocity[i] = 0.0f;
		for (i = 0; i < 6; i++)
			linearVelocity[i] = 0.0f;
		// Set gripperVelocity 0
		gripperVelocity = 0.0f;
		break;

	}

	// Command is not processed accurately
	if (result == IARM_FAILED)
		print_error();
}

// Function to print all runtime parameters 
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


// Function to print iarm library information
void print_application_title(void)
{
	int versionMajor, versionMinor, versionBuild;
	// API function to get library version
	iarm_get_library_version(&versionMajor, &versionMinor, &versionBuild);
	printf("i A R M (library version %d.%d.%d)\n\n", versionMajor, versionMinor, versionBuild);
}

// Function to print separator
void print_separator(void)
{
	printf("------------------------------------------------------------\n");
}

// Function to print all parameters
void ShowAllHelp(void)
{
	printf("iARM HELP\n");
	printf(" commandline parameters:\n");
	print_separator();

	// Parameter to show CAN port used to connect to the iARM
	// In <port> input a port number that you cheack
	printf(" -p=<port>    -- CAN port used to connect to the ARM.\n");

	// 
	printf(" -g_hRobot           -- Show this message.\n");
	print_separator();

	// Print runtime commands
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


// Function to read program arguments
// Return TURE when read an accurate argument
// Retrun FALSE when read an inaccurate argument 
int read_arguments(int argc, char *argv[])
{
	// Variable for a loop counter
	int i;
	// Variable for an argument(one character)
	char prm;
	// Variable for an argument string 
	char *ptr;
	BOOL arg;
	// Cheack all arguments
	for (i = 1; i < argc; i++)
	{
		// Get a i-th argument
		ptr = argv[i];
		// Skip a argument indicator '-' 
		while (*ptr == '-')
			ptr++;
		// Get one character from an argument
		prm = *(ptr++);
		// Make arg True if a character is '='
		arg = (*ptr == '=');
		if (arg) ptr++;
		// Parse an argument
		switch (prm)
		{
			// When an argument is 'p'
		case 'p':
			if (arg)
				g_can_port = strtoul(ptr, NULL, 10);
			else
			{
				printf("please supply a port number with -p (e.g. '-p=1')\n");
				return FALSE;
			}
			break;
			// When an argument is '?' or 'h'
			// Show help
		case '?':
		case 'h':
			ShowAllHelp();
			return FALSE;
			// When an argument is undefined
		default:
			printf("\"%s\": unknown commandline argument: %d\n", argv[0], prm);
			return FALSE;
		}
	}
	return TRUE;
}

// Function to sleep for the provided amount of milliseconds
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

// Function to read keyinput
// Returns TRUE if a key has been pressed, FALSE otherwise. 
// If a key has been pressed, the corresponding ASCII code for that key is copied to the provided key address 
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
