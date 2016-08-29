#ifndef __INCLUDED_IARMINTERFACE_H__
#define __INCLUDED_IARMINTERFACE_H__

#include <iostream>
#include <queue>

#include <windows.h>
// #include <NuiApi.h>

#include <iarm.h>
#include "vector3f.h"

class IARMInterface {
private:
	// フィールド変数
	IARM_HANDLE hRobot;
	int can_port;
	IARM_STATUS status;

	// 定数
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

	// プライベート関数
	int update_status(void);
	void print_error(void);

public:
	typedef enum IARM_CARTESIAN_DIMENSION {	X, Y, Z, YAW, PITCH, ROLL} IARM_CARTESIAN_DIMENSION;

	// 定数
	static const int MOVE_FINISHED = 0x01;
	static const int MOVE_LIMIT = 0x02;
	static const int BLOCK_JOINT = 0x04;
	static const int BLOCK_JOINT2 = 0x08;
	static const int GRIPPER_CLOSED = 0x10;

	static const int CHECK_RESULT_ERR = -1;
	static const int CHECK_RESULT_NONE = 0;
	static const int CHECK_RESULT_FINISH = 1;

	// コンストラクタ
	IARMInterface();
	~IARMInterface();

	// 初期化
	int init();
	// アイアームの状態をチェックする
	int check();
	// ストップ
	int stop();

	// ステータスの表示
	int printStatus(void);

	// 手首の移動
	int moveTo(float x, float y, float z);
	int moveBy(float x, float y, float z);
	int moveTo(float pos[3]);
	int moveBy(float pos[3]);
	int moveTo(Vector3f pos);
	int moveBy(Vector3f pos);

	// ホームポジションへの移動
	int moveHomePosition(void);

	// 手首の平行移動
	int slowDown(void);

	// 手首の回転
	int rotationTo(float angle);
	int rotationBy(float angle);

	// オイラー角回転
	int rotEulerAngleTo(float roll, float pitch, float yaw);
	int rotEulerAngleBy(float roll, float pitch, float yaw);
	int rotEulerAngleTo(float eulerAngle[3]);
	int rotEulerAngleBy(float eulerAngle[3]);

	// 折り畳み
	int unfold();
	int fold();

	// リフトを下げる
	int liftDown();

	// グリッパー操作
	int openGripper(void);
	int closeGripper(void);
	int closeGripper(float gripperOpening);

	// キューへ命令をストックする
	// 手首の移動
	int stockMoveTo(float x, float y, float z);
	int stockMoveBy(float x, float y, float z);
	int stockMoveTo(float pos[3]);
	int stockMoveBy(float pos[3]);
	int stockMoveTo(Vector3f pos);
	int stockMoveBy(Vector3f pos);

	// ホームポジションへの移動
	int stockMoveHomePosition(void);

	// 手首の平行移動
	int stockSlowDown(void);

	// 手首の回転
	int stockRotationTo(float angle);
	int stockRotationBy(float angle);

	// 折り畳み
	int stockUnfold();
	int stockFold();

	// リフトを下げる
	int stockLiftDown();

	// グリッパー操作
	int stockOpenGripper(void);
	int stockCloseGripper(void);

	// 関節回転
	int moveJoint(int id, float angle);
	int moveGripper(float opening);
};

#endif