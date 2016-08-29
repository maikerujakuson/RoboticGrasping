#include "iarmInterface.h"

// �R���X�g���N�^
IARMInterface::IARMInterface() : hRobot(IARM_INVALID_HANDLE),
								IARM_NR_JOINTS(6),
								DEFAULT_CAN_PORT(0)
{
	can_port = DEFAULT_CAN_PORT;

	// �����ʒu�E�p��
	homePosition[0] = 395.0f;
	homePosition[1] = 0.0f;
	homePosition[2] = -100.0f;
	homePosition[3] = 1.57f;
	homePosition[4] = -3.14f;
	homePosition[5] = 0.0f;

	currentCommand = NONE;
}

// �f�X�g���N�^
IARMInterface::~IARMInterface() {
	// clean up
	if(hRobot != IARM_INVALID_HANDLE) {
		fold();
		// ���삪�I������܂Ń��[�v
		while(check() != CHECK_RESULT_FINISH) { Sleep(100); }
		std::cout << "iARM terminated." << std::endl;
	}
	iarm_disconnect(hRobot);
}

// ������
int IARMInterface::init() {
	std::cout << "Waiting for iARM to connect on CAN-bus " <<  can_port << std::endl;

	// iarm�ɃR�l�N�g
	hRobot = iarm_connect(can_port);

	// �R�l�N�g�ł��Ȃ������Ƃ�
	if(hRobot == IARM_INVALID_HANDLE)
	{
		std::cout << "CAN�f�o�C�X�̃I�[�v���Ɏ��s���܂���(port:" <<  can_port << ")" << std::endl;
		return -1;
	}

	// �X�e�[�^�X�̎擾
	iarm_get_status(hRobot, &status);

	Sleep(5000);

	return 0;
}

int IARMInterface::check() {
	// iarm�Ƃ̐ڑ��m�F
	if(iarm_is_connected(hRobot) != IARM_SUCCESS) {
		return CHECK_RESULT_ERR;
	}

	// �X�e�[�^�X���A�b�v�f�[�g
	int result = update_status();

	// ���݂̃R�}���h���Ȃ��Ƃ��E�ړ����I������Ƃ��E�O���b�p�[������ꂽ�Ƃ�
	if(currentCommand == NONE || (result & MOVE_FINISHED) || (currentCommand == CLOSEGRIPPER && (result & GRIPPER_CLOSED))) {
		// �R�}���h���X�g�b�N����Ă���Ƃ�
		if(!commandQueue.empty()) {
			Command front = commandQueue.front();
			// �R�}���h�̃^�C�v���`�F�b�N
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
			// ���݂̃R�}���h�ƃL���[���X�V
			commandQueue.pop();
		}
		// �X�g�b�N����̎�
		else{
			currentCommand = NONE;
			return CHECK_RESULT_FINISH;
		}
	}

	return CHECK_RESULT_NONE;
}

// �����̒�~
int IARMInterface::stop() {
	return iarm_move_stop(hRobot);
}

// �X�e�[�^�X�̃A�b�v�f�[�g
int IARMInterface::update_status()
{
	IARM_STATUS current_status;
	int result = 0;

	// ��Ԃ��擾
	iarm_get_status(hRobot, &current_status);

	// movement_status���O��ƕς���Ă���Ƃ�
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

	// ���������Ƃ��ł��Ȃ��֐߂��Ȃ����`�F�b�N
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

	// �O���b�p�[�����Ă��邩�`�F�b�N
	if((current_status.blocked_status[GRIPPER] != status.blocked_status[GRIPPER]) && (current_status.blocked_status[GRIPPER] != IARM_BLOCK_NONE)) {
		// std::cout << "!Gripper closed" << std::endl;
		result |= GRIPPER_CLOSED;
	}

	// �X�e�[�^�X���㏑��
	memcpy(&status, &current_status, sizeof(IARM_STATUS));

	return result;
}

// �G���[�\��
void IARMInterface::print_error(void) {
	std::cerr << "Error " << iarm_errno(hRobot) << " reported by iARM: " << iarm_error(hRobot) << std::endl;
}

// ���̈ʒu��x,y,z�Ɉړ�����
int IARMInterface::moveTo(float x, float y, float z) {

	IARM_STATUS status;
	IARM_RESULT result;

	// ���݂̏�Ԃ��擾����
	result = iarm_get_status(hRobot, &status);
	if(result == IARM_FAILED) {
		print_error();
		return -1;
	}

	// �ړ�������|�W�V������ݒ�
	float position[6] = { homePosition[X] + x, homePosition[Y] + y, homePosition[Z] + z, status.cartesian_position[YAW], status.cartesian_position[PITCH], status.cartesian_position[ROLL] };
	
	// �ړ�������
	result = iarm_move_position_linear(hRobot, position);
	if(result == IARM_FAILED) {
		print_error();
		return -1;
	}

	currentCommand = MOVETO;

	return 0;
}

// ���̈ʒu��x,y,z�Ɉړ�����
int IARMInterface::moveTo(float pos[3]) {
	return moveTo(pos[0], pos[1], pos[2]);
}

// ���̈ʒu��x,y,z�Ɉړ�����
int IARMInterface::moveTo(Vector3f pos) {
	return moveTo(pos.x, pos.y, pos.z);
}

// ���̈ʒu�����݂���x,y,z�����ړ�������
int IARMInterface::moveBy(float x, float y, float z) {

	IARM_STATUS status;
	IARM_RESULT result;

	// ���݂�iarm��Ԃ��擾����
	result = iarm_get_status(hRobot, &status);
	if(result == IARM_FAILED) {
		print_error();
		return -1;
	}

	// �ړ�������|�W�V������ݒ�(�ʒu�͌��݂̈ʒu����x,y,z)
	float position[6] = {status.cartesian_position[X] + x, status.cartesian_position[Y] + y, status.cartesian_position[Z] + z, status.cartesian_position[YAW], status.cartesian_position[PITCH], status.cartesian_position[ROLL]};
	// �ړ�������
	result = iarm_move_position_linear(hRobot, position);
	if(result == IARM_FAILED) {
		print_error();
		return -1;
	}

	currentCommand = MOVEBY;

	return 0;
}

// ���̈ʒu�����݂���x,y,z�����ړ�������
int IARMInterface::moveBy(float pos[3]) {
	return moveBy(pos[0], pos[1], pos[2]);
}

// �A�[����������艺�낷
int IARMInterface::slowDown(void) {
	IARM_RESULT result;

	float velocity[6] = {0};
	velocity[Y] = 15.0f;

	// ���x�̐ݒ�
	result = iarm_move_direction_linear(hRobot, velocity);
	if(result == IARM_FAILED) {
		print_error();
		return -1;
	}

	return 0;
}

// ���̉�]
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

// ���̉�]
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

// �I�C���[�p�Ŏ��̉�]�i��Ίp�x�j
int IARMInterface::rotEulerAngleTo(float roll, float pitch, float yaw) {
	IARM_STATUS status;
	IARM_RESULT result;

	// ���݂�iarm��Ԃ��擾����
	result = iarm_get_status(hRobot, &status);
	if(result == IARM_FAILED) {
		print_error();
		return -1;
	}

	// iarm���ړ�������|�W�V������ݒ�(�ʒu�͌��݂̈ʒu����x,y,z �p���͂��̂܂�)
	float position[6] = {status.cartesian_position[X], status.cartesian_position[Y], status.cartesian_position[Z], homePosition[YAW] + yaw, homePosition[PITCH] + pitch, homePosition[ROLL] + roll};
	
	// �|�W�V�������ړ�������
	result = iarm_move_position_linear(hRobot, position);
	if(result == IARM_FAILED) {
		print_error();
		return -1;
	}

	currentCommand = ROTEATO;

	return 0;
}

// �I�C���[�p�Ŏ��̉�]�i��Ίp�x�j
int IARMInterface::rotEulerAngleTo(float eulerAngle[3]) {
	return rotEulerAngleTo(eulerAngle[2], eulerAngle[1], eulerAngle[0]);
}

// �I�C���[�p�Ŏ��̉�]�i���Ίp�x�j
int IARMInterface::rotEulerAngleBy(float roll, float pitch, float yaw) {
	IARM_STATUS status;
	IARM_RESULT result;

	// ���݂�iarm��Ԃ��擾����
	result = iarm_get_status(hRobot, &status);
	if(result == IARM_FAILED) {
		print_error();
		return -1;
	}

	// iarm���ړ�������|�W�V������ݒ�(�ʒu�͌��݂̈ʒu����x,y,z �p���͂��̂܂�)
	float position[6] = {status.cartesian_position[X], status.cartesian_position[Y], status.cartesian_position[Z], status.cartesian_position[YAW] + yaw, status.cartesian_position[PITCH] + pitch, status.cartesian_position[ROLL] + roll};
	
	// �ʒu�̈ړ�
	result = iarm_move_position_linear(hRobot, position);
	if(result == IARM_FAILED) {
		print_error();
		return -1;
	}

	currentCommand = ROTEABY;

	return 0;
}

// �I�C���[�p�Ŏ��̉�]�i���Ίp�x�j
int IARMInterface::rotEulerAngleBy(float eulerAngle[3]) {
	return rotEulerAngleBy(eulerAngle[2], eulerAngle[1], eulerAngle[0]);
}

// �֐߉�]
int IARMInterface::moveJoint(int id, float angle) {
	IARM_STATUS status;
	IARM_RESULT result;

	// ID���K���Ȓl���`�F�b�N
	if(id < 0 || id >= IARM_NR_JOINTS) {
		std::cerr << "id�̒l���K���ł͂���܂���(id:" << id << ")" << std::endl;
		return -1;
	}

	// ���݂�iarm��Ԃ��擾����
	result = iarm_get_status(hRobot, &status);
	if(result == IARM_FAILED) {
		print_error();
		return -1;
	}

	// ���݂�iarm�̃W���C���g�p�x����
	float position[6];
	for(int i=0; i < IARM_NR_JOINTS; i++) {
		position[i] = status.joint_position[i];
		if(i == id) {
			position[i] += angle;
		}
	}

	// �O���b�p�[
	float gripper_opening = status.gripper_opening;
	if(id == IARM_NR_JOINTS) {
		gripper_opening += angle;
	}

	// �֐߂̉�]
	result = iarm_move_position_joint(hRobot, position, gripper_opening, IARM_LIFT_KEEP_POS);
	if(result == IARM_FAILED) {
		print_error();
		return -1;
	}

	currentCommand = MOVEJOINT;

	return 0;
}

// �O���b�p����
int IARMInterface::moveGripper(float opening) {
	IARM_STATUS status;
	IARM_RESULT result;

	// ���݂�iarm��Ԃ��擾����
	result = iarm_get_status(hRobot, &status);
	if(result == IARM_FAILED) {
		print_error();
		return -1;
	}

	// �֐ߊp�x�����݂̊p�x�ɐݒ�
	float position[6];
	for(int i=0; i < IARM_NR_JOINTS; i++) {
		position[i] = status.cartesian_position[i];
	}

	// �O���b�p�[
	float gripper_opening = status.gripper_opening + opening;

	// iArm�ɖ���
	result = iarm_move_position_joint(hRobot, position, gripper_opening, IARM_LIFT_KEEP_POS);
	if(result == IARM_FAILED) {
		print_error();
		return -1;
	}

	currentCommand = MOVEGRIPPER;

	return 0;
}

// �z�[���|�W�V����
int IARMInterface::moveHomePosition() {
	IARM_RESULT result;

	// �ړ�������|�W�V������ݒ�(�ʒu�͌��݂̈ʒu����x,y,z)
	float position[6];
	for(int i=0; i<IARM_NR_JOINTS; i++) {
		position[i] = homePosition[i];
	}
	// �ړ�������
	result = iarm_move_position_linear(hRobot, position);
	if(result == IARM_FAILED) {
		print_error();
		return -1;
	}

	currentCommand = HOME;

	return 0;
}

// �܂��݉���
int IARMInterface::unfold() {
	currentCommand = UNFOLD;
	return iarm_unfold(hRobot);
}

// �܂���
int IARMInterface::fold() {
	currentCommand = FOLD;
	return iarm_fold(hRobot);
}

// ���t�g������
int IARMInterface::liftDown(void) {
	IARM_RESULT result;

	// �X�e�[�^�X�̎擾
	result = iarm_get_status(hRobot, &status);
	if(result == IARM_FAILED) {
		print_error();
		return -1;
	}

	// �֐ߊp�x�����݂̊p�x�ɐݒ�
	float jointPosition[6];
	for(int i=0; i<6; i++) {
		jointPosition[i] = status.joint_position[i];
	}

	// ���t�g��������
	result = iarm_move_position_joint(hRobot, jointPosition, status.gripper_opening, IARM_LIFT_DOWN);
	if(result == IARM_FAILED) {
		print_error();
		return -1;
	}

	currentCommand = LIFTDOWN;

	return 0;
}

// �X�e�[�^�X�̕\��
int IARMInterface::printStatus(void) {
	// �X�e�[�^�X���擾�ł��Ȃ������ꍇ
	if(iarm_get_status(hRobot, &status) == IARM_FAILED) {
		print_error();
		return -1;
	}

	// �u���b�N�X�e�[�^�X
	std::cout << "blocked status:" << std::endl;
	for(int i=0; i<6; i++)
		std::cout << status.blocked_status[i] << " ";
	std::cout << std::endl;

	// 3�������W�@�I�C���[�p
	std::cout << "cartesian position:" << std::endl;
	for(int i = 0; i < 6; i++)
		std::cout << status.cartesian_position[i] << " ";
	std::cout << std::endl;

	// �֐߈ʒu
	for(int i = 0; i < IARM_NR_JOINTS; ++i)
		std::cout << "joint[" << i << "]" << " position:" << status.joint_position[i] << " velocity:" << status.joint_velocities[i];
	std::cout << std::endl;

	// �O���b�p�[
	std::cout << "gripper opening: " << status.gripper_opening*100.0f << "% velocity:" << status.gripper_velocity << std::endl;

	// �ړ��X�e�[�^�X
	std::cout << "movement status:" << status.movement_status << std::endl;


	return 0;
}

// �O���b�p�[�J��
int IARMInterface::openGripper() {
	IARM_RESULT result;

	// �X�e�[�^�X�̎擾
	result = iarm_get_status(hRobot, &status);
	if(result == IARM_FAILED) {
		print_error();
		return -1;
	}

	// �֐ߊp�x�����݂̊p�x�ɐݒ�
	float jointPosition[6];
	for(int i=0; i<6; i++) {
		jointPosition[i] = status.joint_position[i];
	}

	// �O���b�p�[�J��
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

// �O���b�p�[����
int IARMInterface::closeGripper() {
	IARM_RESULT result;

	// �X�e�[�^�X�̎擾
	result = iarm_get_status(hRobot, &status);
	if(result == IARM_FAILED) {
		print_error();
		return -1;
	}

	// �֐ߊp�x�����݂̊p�x�ɐݒ�
	float jointPosition[6];
	for(int i=0; i<6; i++) {
		jointPosition[i] = status.joint_position[i];
	}

	// ���t�g��������
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