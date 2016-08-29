#include "iarmInterface.h"

// コンストラクタ
IARMInterface::IARMInterface() : hRobot(IARM_INVALID_HANDLE),
								IARM_NR_JOINTS(6),
								DEFAULT_CAN_PORT(0)
{
	can_port = DEFAULT_CAN_PORT;

	// 初期位置・姿勢
	homePosition[0] = 395.0f;
	homePosition[1] = 0.0f;
	homePosition[2] = -100.0f;
	homePosition[3] = 1.57f;
	homePosition[4] = -3.14f;
	homePosition[5] = 0.0f;

	currentCommand = NONE;
}

// デストラクタ
IARMInterface::~IARMInterface() {
	// clean up
	if(hRobot != IARM_INVALID_HANDLE) {
		fold();
		// 動作が終了するまでループ
		while(check() != CHECK_RESULT_FINISH) { Sleep(100); }
		std::cout << "iARM terminated." << std::endl;
	}
	iarm_disconnect(hRobot);
}

// 初期化
int IARMInterface::init() {
	std::cout << "Waiting for iARM to connect on CAN-bus " <<  can_port << std::endl;

	// iarmにコネクト
	hRobot = iarm_connect(can_port);

	// コネクトできなかったとき
	if(hRobot == IARM_INVALID_HANDLE)
	{
		std::cout << "CANデバイスのオープンに失敗しました(port:" <<  can_port << ")" << std::endl;
		return -1;
	}

	// ステータスの取得
	iarm_get_status(hRobot, &status);

	Sleep(5000);

	return 0;
}

int IARMInterface::check() {
	// iarmとの接続確認
	if(iarm_is_connected(hRobot) != IARM_SUCCESS) {
		return CHECK_RESULT_ERR;
	}

	// ステータスをアップデート
	int result = update_status();

	// 現在のコマンドがないとき・移動が終わったとき・グリッパーが閉じられたとき
	if(currentCommand == NONE || (result & MOVE_FINISHED) || (currentCommand == CLOSEGRIPPER && (result & GRIPPER_CLOSED))) {
		// コマンドがストックされているとき
		if(!commandQueue.empty()) {
			Command front = commandQueue.front();
			// コマンドのタイプをチェック
			switch(front.func) {
				case STOP: {
					stop();
					break;
						   }
				case HOME: {
					moveHomePosition();
					break;
						   }
				case UNFOLD: {
					unfold();
					break;
							 }
				case LIFTDOWN: {
					liftDown();
					break;
							   }
				case MOVETO: {
					moveTo(front.param1, front.param2, front.param3);
					break;
							 }
				case MOVEBY: {
					moveBy(front.param1, front.param2, front.param3);
					break;
							 }
				case ROTTO: {
					rotationTo(front.param1);
					break;
							}
				case ROTBY: {
					rotEulerAngleBy(0,0,front.param1);
					break;
							}
				case OPENGRIPPER: {
					openGripper();
					break;
								  }
				case CLOSEGRIPPER: {
					closeGripper();
					break;
								   }
			}
			// 現在のコマンドとキューを更新
			commandQueue.pop();
		}
		// ストックが空の時
		else{
			currentCommand = NONE;
			return CHECK_RESULT_FINISH;
		}
	}

	return CHECK_RESULT_NONE;
}

// 動きの停止
int IARMInterface::stop() {
	return iarm_move_stop(hRobot);
}

// ステータスのアップデート
int IARMInterface::update_status()
{
	IARM_STATUS current_status;
	int result = 0;

	// 状態を取得
	iarm_get_status(hRobot, &current_status);

	// movement_statusが前回と変わっているとき
	switch(current_status.movement_status) {
		case IARM_MOVEMENT_FINISHED: {
			//std::cout << "!Movement finished" << std::endl;
			result |= MOVE_FINISHED;
			break;
		}
		case IARM_MOVEMENT_LIMIT: {
			//std::cout << "!Mechanical limit" << std::endl;
			result |= MOVE_LIMIT;
			break;
		}
		default: {
			break;
		}
	}

	// 動かすことができない関節がないかチェック
	for(int i = 0; i < IARM_NR_JOINTS; ++i) {
		if((current_status.blocked_status[i] != status.blocked_status[i]) && (current_status.blocked_status[i] != IARM_BLOCK_NONE)) {
			// std::cout << "!Block on joint " << i+1 << std::endl;
			result |= BLOCK_JOINT;
		}
		else if((current_status.blocked_status[i] != status.blocked_status[i]) && (current_status.blocked_status[i] == IARM_BLOCK_NONE)) {
			// std::cout << "!Block on joint " << i+1 << " resolved" << std::endl;
			result |= BLOCK_JOINT2;
		}
	}

	// グリッパーが閉じているかチェック
	if((current_status.blocked_status[GRIPPER] != status.blocked_status[GRIPPER]) && (current_status.blocked_status[GRIPPER] != IARM_BLOCK_NONE)) {
		// std::cout << "!Gripper closed" << std::endl;
		result |= GRIPPER_CLOSED;
	}

	// ステータスを上書き
	memcpy(&status, &current_status, sizeof(IARM_STATUS));

	return result;
}

// エラー表示
void IARMInterface::print_error(void) {
	std::cerr << "Error " << iarm_errno(hRobot) << " reported by iARM: " << iarm_error(hRobot) << std::endl;
}

// 手首の位置をx,y,zに移動する
int IARMInterface::moveTo(float x, float y, float z) {

	IARM_STATUS status;
	IARM_RESULT result;

	// 現在の状態を取得する
	result = iarm_get_status(hRobot, &status);
	if(result == IARM_FAILED) {
		print_error();
		return -1;
	}

	// 移動させるポジションを設定
	float position[6] = { homePosition[X] + x, homePosition[Y] + y, homePosition[Z] + z, status.cartesian_position[YAW], status.cartesian_position[PITCH], status.cartesian_position[ROLL] };
	
	// 移動させる
	result = iarm_move_position_linear(hRobot, position);
	if(result == IARM_FAILED) {
		print_error();
		return -1;
	}

	currentCommand = MOVETO;

	return 0;
}

// 手首の位置をx,y,zに移動する
int IARMInterface::moveTo(float pos[3]) {
	return moveTo(pos[0], pos[1], pos[2]);
}

// 手首の位置をx,y,zに移動する
int IARMInterface::moveTo(Vector3f pos) {
	return moveTo(pos.x, pos.y, pos.z);
}

// 手首の位置を現在からx,y,zだけ移動させる
int IARMInterface::moveBy(float x, float y, float z) {

	IARM_STATUS status;
	IARM_RESULT result;

	// 現在のiarm状態を取得する
	result = iarm_get_status(hRobot, &status);
	if(result == IARM_FAILED) {
		print_error();
		return -1;
	}

	// 移動させるポジションを設定(位置は現在の位置からx,y,z)
	float position[6] = {status.cartesian_position[X] + x, status.cartesian_position[Y] + y, status.cartesian_position[Z] + z, status.cartesian_position[YAW], status.cartesian_position[PITCH], status.cartesian_position[ROLL]};
	// 移動させる
	result = iarm_move_position_linear(hRobot, position);
	if(result == IARM_FAILED) {
		print_error();
		return -1;
	}

	currentCommand = MOVEBY;

	return 0;
}

// 手首の位置を現在からx,y,zだけ移動させる
int IARMInterface::moveBy(float pos[3]) {
	return moveBy(pos[0], pos[1], pos[2]);
}

// アームをゆっくり下ろす
int IARMInterface::slowDown(void) {
	IARM_RESULT result;

	float velocity[6] = {0};
	velocity[Y] = 15.0f;

	// 速度の設定
	result = iarm_move_direction_linear(hRobot, velocity);
	if(result == IARM_FAILED) {
		print_error();
		return -1;
	}

	return 0;
}

// 手首の回転
int IARMInterface::rotationTo(float angle) {
	IARM_RESULT result;
	IARM_STATUS status;

	if(iarm_get_status(hRobot, &status) == IARM_FAILED) {
		print_error();
		return -1;
	}

	float jointPosition[6];
	for(int i=0; i<6; i++) {
		jointPosition[i] = status.joint_position[i];
	}
	jointPosition[J6] = angle;

	result = iarm_move_position_joint(hRobot, jointPosition, status.gripper_opening, IARM_LIFT_KEEP_POS);

	if(result == IARM_FAILED) {
		print_error();
		return -1;
	}

	currentCommand = ROTTO;

	return 0;
}

// 手首の回転
int IARMInterface::rotationBy(float angle) {
	IARM_RESULT result;
	IARM_STATUS status;

	if(iarm_get_status(hRobot, &status) == IARM_FAILED) {
		print_error();
		return -1;
	}

	float jointPosition[6];
	for(int i=0; i<6; i++) {
		jointPosition[i] = status.joint_position[i];
	}
	jointPosition[J6] += angle;

	result = iarm_move_position_joint(hRobot, jointPosition, status.gripper_opening, IARM_LIFT_KEEP_POS);
	if(result == IARM_FAILED) {
		print_error();
		return -1;
	}

	currentCommand = ROTBY;

	return 0;
}

// オイラー角で手首の回転（絶対角度）
int IARMInterface::rotEulerAngleTo(float roll, float pitch, float yaw) {
	IARM_STATUS status;
	IARM_RESULT result;

	// 現在のiarm状態を取得する
	result = iarm_get_status(hRobot, &status);
	if(result == IARM_FAILED) {
		print_error();
		return -1;
	}

	// iarmを移動させるポジションを設定(位置は現在の位置からx,y,z 姿勢はそのまま)
	float position[6] = {status.cartesian_position[X], status.cartesian_position[Y], status.cartesian_position[Z], homePosition[YAW] + yaw, homePosition[PITCH] + pitch, homePosition[ROLL] + roll};
	
	// ポジションを移動させる
	result = iarm_move_position_linear(hRobot, position);
	if(result == IARM_FAILED) {
		print_error();
		return -1;
	}

	currentCommand = ROTEATO;

	return 0;
}

// オイラー角で手首の回転（絶対角度）
int IARMInterface::rotEulerAngleTo(float eulerAngle[3]) {
	return rotEulerAngleTo(eulerAngle[2], eulerAngle[1], eulerAngle[0]);
}

// オイラー角で手首の回転（相対角度）
int IARMInterface::rotEulerAngleBy(float roll, float pitch, float yaw) {
	IARM_STATUS status;
	IARM_RESULT result;

	// 現在のiarm状態を取得する
	result = iarm_get_status(hRobot, &status);
	if(result == IARM_FAILED) {
		print_error();
		return -1;
	}

	// iarmを移動させるポジションを設定(位置は現在の位置からx,y,z 姿勢はそのまま)
	float position[6] = {status.cartesian_position[X], status.cartesian_position[Y], status.cartesian_position[Z], status.cartesian_position[YAW] + yaw, status.cartesian_position[PITCH] + pitch, status.cartesian_position[ROLL] + roll};
	
	// 位置の移動
	result = iarm_move_position_linear(hRobot, position);
	if(result == IARM_FAILED) {
		print_error();
		return -1;
	}

	currentCommand = ROTEABY;

	return 0;
}

// オイラー角で手首の回転（相対角度）
int IARMInterface::rotEulerAngleBy(float eulerAngle[3]) {
	return rotEulerAngleBy(eulerAngle[2], eulerAngle[1], eulerAngle[0]);
}

// 関節回転
int IARMInterface::moveJoint(int id, float angle) {
	IARM_STATUS status;
	IARM_RESULT result;

	// IDが適正な値かチェック
	if(id < 0 || id >= IARM_NR_JOINTS) {
		std::cerr << "idの値が適正ではありません(id:" << id << ")" << std::endl;
		return -1;
	}

	// 現在のiarm状態を取得する
	result = iarm_get_status(hRobot, &status);
	if(result == IARM_FAILED) {
		print_error();
		return -1;
	}

	// 現在のiarmのジョイント角度を代入
	float position[6];
	for(int i=0; i < IARM_NR_JOINTS; i++) {
		position[i] = status.joint_position[i];
		if(i == id) {
			position[i] += angle;
		}
	}

	// グリッパー
	float gripper_opening = status.gripper_opening;
	if(id == IARM_NR_JOINTS) {
		gripper_opening += angle;
	}

	// 関節の回転
	result = iarm_move_position_joint(hRobot, position, gripper_opening, IARM_LIFT_KEEP_POS);
	if(result == IARM_FAILED) {
		print_error();
		return -1;
	}

	currentCommand = MOVEJOINT;

	return 0;
}

// グリッパ操作
int IARMInterface::moveGripper(float opening) {
	IARM_STATUS status;
	IARM_RESULT result;

	// 現在のiarm状態を取得する
	result = iarm_get_status(hRobot, &status);
	if(result == IARM_FAILED) {
		print_error();
		return -1;
	}

	// 関節角度を現在の角度に設定
	float position[6];
	for(int i=0; i < IARM_NR_JOINTS; i++) {
		position[i] = status.cartesian_position[i];
	}

	// グリッパー
	float gripper_opening = status.gripper_opening + opening;

	// iArmに命令
	result = iarm_move_position_joint(hRobot, position, gripper_opening, IARM_LIFT_KEEP_POS);
	if(result == IARM_FAILED) {
		print_error();
		return -1;
	}

	currentCommand = MOVEGRIPPER;

	return 0;
}

// ホームポジション
int IARMInterface::moveHomePosition() {
	IARM_RESULT result;

	// 移動させるポジションを設定(位置は現在の位置からx,y,z)
	float position[6];
	for(int i=0; i<IARM_NR_JOINTS; i++) {
		position[i] = homePosition[i];
	}
	// 移動させる
	result = iarm_move_position_linear(hRobot, position);
	if(result == IARM_FAILED) {
		print_error();
		return -1;
	}

	currentCommand = HOME;

	return 0;
}

// 折り畳み解除
int IARMInterface::unfold() {
	currentCommand = UNFOLD;
	return iarm_unfold(hRobot);
}

// 折り畳み
int IARMInterface::fold() {
	currentCommand = FOLD;
	return iarm_fold(hRobot);
}

// リフト下げる
int IARMInterface::liftDown(void) {
	IARM_RESULT result;

	// ステータスの取得
	result = iarm_get_status(hRobot, &status);
	if(result == IARM_FAILED) {
		print_error();
		return -1;
	}

	// 関節角度を現在の角度に設定
	float jointPosition[6];
	for(int i=0; i<6; i++) {
		jointPosition[i] = status.joint_position[i];
	}

	// リフトを下げる
	result = iarm_move_position_joint(hRobot, jointPosition, status.gripper_opening, IARM_LIFT_DOWN);
	if(result == IARM_FAILED) {
		print_error();
		return -1;
	}

	currentCommand = LIFTDOWN;

	return 0;
}

// ステータスの表示
int IARMInterface::printStatus(void) {
	// ステータスが取得できなかった場合
	if(iarm_get_status(hRobot, &status) == IARM_FAILED) {
		print_error();
		return -1;
	}

	// ブロックステータス
	std::cout << "blocked status:" << std::endl;
	for(int i=0; i<6; i++)
		std::cout << status.blocked_status[i] << " ";
	std::cout << std::endl;

	// 3次元座標　オイラー角
	std::cout << "cartesian position:" << std::endl;
	for(int i = 0; i < 6; i++)
		std::cout << status.cartesian_position[i] << " ";
	std::cout << std::endl;

	// 関節位置
	for(int i = 0; i < IARM_NR_JOINTS; ++i)
		std::cout << "joint[" << i << "]" << " position:" << status.joint_position[i] << " velocity:" << status.joint_velocities[i];
	std::cout << std::endl;

	// グリッパー
	std::cout << "gripper opening: " << status.gripper_opening*100.0f << "% velocity:" << status.gripper_velocity << std::endl;

	// 移動ステータス
	std::cout << "movement status:" << status.movement_status << std::endl;


	return 0;
}

// グリッパー開く
int IARMInterface::openGripper() {
	IARM_RESULT result;

	// ステータスの取得
	result = iarm_get_status(hRobot, &status);
	if(result == IARM_FAILED) {
		print_error();
		return -1;
	}

	// 関節角度を現在の角度に設定
	float jointPosition[6];
	for(int i=0; i<6; i++) {
		jointPosition[i] = status.joint_position[i];
	}

	// グリッパー開く
	if(status.gripper_opening < 0.9f) {
		result = iarm_move_position_joint(hRobot, jointPosition, 1.0f, IARM_LIFT_KEEP_POS);
		if(result == IARM_FAILED) {
			print_error();
			return -1;
		}
		currentCommand = OPENGRIPPER;
	}

	return 0;
}

// グリッパー閉じる
int IARMInterface::closeGripper() {
	IARM_RESULT result;

	// ステータスの取得
	result = iarm_get_status(hRobot, &status);
	if(result == IARM_FAILED) {
		print_error();
		return -1;
	}

	// 関節角度を現在の角度に設定
	float jointPosition[6];
	for(int i=0; i<6; i++) {
		jointPosition[i] = status.joint_position[i];
	}

	// リフトを下げる
	result = iarm_move_position_joint(hRobot, jointPosition, 0.0f, IARM_LIFT_KEEP_POS);
	if(result == IARM_FAILED) {
		print_error();
		return -1;
	}

	currentCommand = CLOSEGRIPPER;

	return 0;
}

int IARMInterface::stockMoveHomePosition() {
	Command com = { HOME, 0, 0, 0 };
	commandQueue.push(com);
	return 0;
}

int IARMInterface::stockOpenGripper() {
	Command com = { OPENGRIPPER, 0, 0, 0 };
	commandQueue.push(com);
	return 0;
}

int IARMInterface::stockCloseGripper() {
	Command com = { CLOSEGRIPPER, 0, 0, 0 };
	commandQueue.push(com);
	return 0;
}

int IARMInterface::stockFold() {
	Command com = { FOLD, 0, 0, 0 };
	commandQueue.push(com);
	return 0;
}

int IARMInterface::stockUnfold() {
	Command com = { UNFOLD, 0, 0, 0 };
	commandQueue.push(com);
	return 0;
}

int IARMInterface::stockLiftDown() {
	Command com = { LIFTDOWN, 0, 0, 0 };
	commandQueue.push(com);
	return 0;
}

int IARMInterface::stockMoveBy(float x, float y, float z) {
	Command com = { MOVEBY, x, y, z };
	commandQueue.push(com);
	return 0;
}

int IARMInterface::stockMoveTo(float x, float y, float z) {
	Command com = { MOVETO, x, y, z };
	commandQueue.push(com);
	return 0;
}

int IARMInterface::stockMoveBy(float pos[3]) {
	return stockMoveBy(pos[0], pos[1], pos[2]);
}

int IARMInterface::stockMoveTo(float pos[3]) {
	return stockMoveTo(pos[0], pos[1], pos[2]);
}

int IARMInterface::stockMoveBy(Vector3f pos) {
	return stockMoveBy(pos.x, pos.y, pos.z);
}

int IARMInterface::stockMoveTo(Vector3f pos) {
	return stockMoveTo(pos.x, pos.y, pos.z);
}

int IARMInterface::stockRotationBy(float angle) {
	Command com = { ROTBY, angle, 0, 0 };
	commandQueue.push(com);
	return 0;
}

int IARMInterface::stockRotationTo(float angle) {
	Command com = { ROTTO, angle, 0, 0 };
	commandQueue.push(com);
	return 0;
}