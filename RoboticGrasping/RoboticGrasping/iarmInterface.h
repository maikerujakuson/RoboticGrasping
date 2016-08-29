#ifndef __INCLUDED_IARMINTERFACE_H__
#define __INCLUDED_IARMINTERFACE_H__

#include <iostream>
#include <queue>

#include <windows.h>
// #include <NuiApi.h>

#include <iarm.h>

class IARMInterface {
private:
	// �t�B�[���h�ϐ�
	IARM_HANDLE hRobot;
	int can_port;
	IARM_STATUS status;

	// �萔
	const int IARM_NR_JOINTS;
	const int DEFAULT_CAN_PORT;
	float homePosition[6];

	//enum
	typedef enum IARM_JOINT {
		J1,
		J2,
		J3,
		J4,
		J5,
		J6,
		GRIPPER
	} IARM_JOINT;

	typedef enum IARM_FUNCTION {
		NONE,
		STOP,
		HOME,
		FOLD,
		UNFOLD,
		LIFTDOWN,
		MOVETO,
		MOVEBY,
		ROTTO,
		ROTBY,
		ROTEATO,
		ROTEABY,
		MOVEJOINT,
		MOVEGRIPPER,
		OPENGRIPPER,
		CLOSEGRIPPER
	} IARM_FUNCTION;

	typedef struct {
		IARM_FUNCTION func;
		float param1;
		float param2;
		float param3;
	} Command;

	IARM_FUNCTION currentCommand;
	std::queue<Command> commandQueue;

	// �v���C�x�[�g�֐�
	int update_status(void);
	void print_error(void);

public:
	typedef enum IARM_CARTESIAN_DIMENSION {	X, Y, Z, YAW, PITCH, ROLL} IARM_CARTESIAN_DIMENSION;

	// �萔
	static const int MOVE_FINISHED = 0x01;
	static const int MOVE_LIMIT = 0x02;
	static const int BLOCK_JOINT = 0x04;
	static const int BLOCK_JOINT2 = 0x08;
	static const int GRIPPER_CLOSED = 0x10;

	static const int CHECK_RESULT_ERR = -1;
	static const int CHECK_RESULT_NONE = 0;
	static const int CHECK_RESULT_FINISH = 1;

	// �R���X�g���N�^
	IARMInterface();
	~IARMInterface();

	// ������
	int init();
	// �A�C�A�[���̏�Ԃ��`�F�b�N����
	int check();
	// �X�g�b�v
	int stop();

	// �X�e�[�^�X�̕\��
	int printStatus(void);

	// ���̈ړ�
	int moveTo(float x, float y, float z);
	int moveBy(float x, float y, float z);
	int moveTo(float pos[3]);
	int moveBy(float pos[3]);
	int moveTo(Vector3f pos);
	int moveBy(Vector3f pos);

	// �z�[���|�W�V�����ւ̈ړ�
	int moveHomePosition(void);

	// ���̕��s�ړ�
	int slowDown(void);

	// ���̉�]
	int rotationTo(float angle);
	int rotationBy(float angle);

	// �I�C���[�p��]
	int rotEulerAngleTo(float roll, float pitch, float yaw);
	int rotEulerAngleBy(float roll, float pitch, float yaw);
	int rotEulerAngleTo(float eulerAngle[3]);
	int rotEulerAngleBy(float eulerAngle[3]);

	// �܂���
	int unfold();
	int fold();

	// ���t�g��������
	int liftDown();

	// �O���b�p�[����
	int openGripper(void);
	int closeGripper(void);
	int closeGripper(float gripperOpening);

	// �L���[�֖��߂��X�g�b�N����
	// ���̈ړ�
	int stockMoveTo(float x, float y, float z);
	int stockMoveBy(float x, float y, float z);
	int stockMoveTo(float pos[3]);
	int stockMoveBy(float pos[3]);
	int stockMoveTo(Vector3f pos);
	int stockMoveBy(Vector3f pos);

	// �z�[���|�W�V�����ւ̈ړ�
	int stockMoveHomePosition(void);

	// ���̕��s�ړ�
	int stockSlowDown(void);

	// ���̉�]
	int stockRotationTo(float angle);
	int stockRotationBy(float angle);

	// �܂���
	int stockUnfold();
	int stockFold();

	// ���t�g��������
	int stockLiftDown();

	// �O���b�p�[����
	int stockOpenGripper(void);
	int stockCloseGripper(void);

	// �֐߉�]
	int moveJoint(int id, float angle);
	int moveGripper(float opening);
};

#endif