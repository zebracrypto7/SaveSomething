#pragma once

namespace IcKinematics
{
	public enum class IcIKResult
	{
		IC_IK_SUCCEED = 0,	// 成功
		IC_IK_FAILED = -1,	// 汎用失敗
		IC_IK_NOW_JOINT_ERROR = -2,	// 現在関節角の取得に失敗しました！！
		IC_IK_FILE_NOTFOUND = -3,	// ファイルの読み込みに失敗
		IC_IK_INCORRECT_DH_PARAMETER,	// 想定しているDHパラメータではありません！！
		IC_IK_OUT_OF_ARM_RANGE = -4,	//アームの到達領域範囲外です！！
		IC_IK_OUT_OF_ANGLE_RANGE = -5,	//関節角度の稼働範囲外です！！
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