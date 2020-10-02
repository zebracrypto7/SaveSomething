#pragma once

namespace IcKinematics
{
	public enum class IcIKResult
	{
		IC_IK_SUCCEED = 0,	// ����
		IC_IK_FAILED = -1,	// �ėp���s
		IC_IK_NOW_JOINT_ERROR = -2,	// ���݊֐ߊp�̎擾�Ɏ��s���܂����I�I
		IC_IK_FILE_NOTFOUND = -3,	// �t�@�C���̓ǂݍ��݂Ɏ��s
		IC_IK_INCORRECT_DH_PARAMETER,	// �z�肵�Ă���DH�p�����[�^�ł͂���܂���I�I
		IC_IK_OUT_OF_ARM_RANGE = -4,	//�A�[���̓��B�̈�͈͊O�ł��I�I
		IC_IK_OUT_OF_ANGLE_RANGE = -5,	//�֐ߊp�x�̉ғ��͈͊O�ł��I�I
	};

	public enum class IcIKConfig
	{
		FRONT_UPPER_FLIP = 0,
		FRONT_UPPER_NO_FLIP,
		FRONT_LOWER_FLIP,
		FRONT_LOWER_NO_FLIP,
		BACK_UPPER_FLIP,
		BACK_UPPER_NO_FLIP,
		BACK_LOWER_FLIP,
		BACK_LOWER_NO_FLIP,
	};

	public enum class IcIKRotationType
	{
		XYZ=0,
		XZY,
		YXZ,
		YZX,
		ZXY,
		ZYX,
		ZYZ,
	};
}