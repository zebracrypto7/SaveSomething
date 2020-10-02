#pragma once

#include"IcIKConst.h"
#include "XmlDhParam.h"

using namespace System;
using namespace System::Collections::Generic;
using namespace IcCalcUtil;

namespace IcKinematics
{
	public ref class IcInverseKinematics
	{
	public:
		IcInverseKinematics();
		IcInverseKinematics(String^ aPath);

	private:
		IcMatrix4D^ roboBaceMat;
		IcMatrix4D^ toolTfmMat;
		double robotCoordOriginHeight;	// 高さ0の位置

		//コンフィグ値(enumで管理)
		IcIKConfig m_Config;

		//DHパラメータ
		XmlDhParam^ m_DhParam;

		//同次変換行列(z軸まわり回転、z軸方向移動)
		IcMatrix4D^ tfmMatZ_1;
		IcMatrix4D^ tfmMatZ_2;
		IcMatrix4D^ tfmMatZ_3;
		IcMatrix4D^ tfmMatZ_4;
		IcMatrix4D^ tfmMatZ_5;
		IcMatrix4D^ tfmMatZ_6;

		//同次変換行列(x軸周り回転、x軸方向移動)
		IcMatrix4D^ tfmMatX_1;
		IcMatrix4D^ tfmMatX_2;
		IcMatrix4D^ tfmMatX_3;
		IcMatrix4D^ tfmMatX_4;
		IcMatrix4D^ tfmMatX_5;
		IcMatrix4D^ tfmMatX_6;

		//同次変換行列（軸から軸への）
		IcMatrix4D^ tfmMat_1;
		IcMatrix4D^ tfmMat_2;
		IcMatrix4D^ tfmMat_3;
		IcMatrix4D^ tfmMat_4;
		IcMatrix4D^ tfmMat_5;
		IcMatrix4D^ tfmMat_6;
		IcMatrix4D^ tfmMat_1_to_6;

		//角度最大値
		double angle_1_max;
		double angle_2_max;
		double angle_3_max;
		double angle_4_max;
		double angle_5_max;
		double angle_6_max;

		//角度最小値
		double angle_1_min;
		double angle_2_min;
		double angle_3_min;
		double angle_4_min;
		double angle_5_min;
		double angle_6_min;

	public:
		IcIKResult Init(String^ aPath);
		IcIKResult SetTool(IcMatrix4D^ aToolMat);
		IcIKResult SetTool(double aX, double aY, double aZ, double aRx, double aRy, double aRz, IcIKRotationType aRotationType);
		IcIKResult SetBase(IcMatrix4D^ aBaseMat);
		IcIKResult SetBase(IcCoordinates^ aBaseCoord);
		IcIKResult SetBase(double aX, double aY, double aZ, double aRx, double aRy, double aRz, IcIKRotationType aRotationType);
		IcIKResult CalcInv(IcMatrix4D^ aTgtMat, List<double>^% aResJointList, bool aIsRobotCoord);
		IcIKResult CalcInv(IcCoordinates^ aTgtCoordinate, List<double>^% aResJointList, bool aIsRobotCoord);
		IcIKResult CalcInv(List<double>^ aTgtCoordinate, List<double>^% aResJointList, bool aIsRobotCoord);
		IcIKResult CalcFwd(List<double>^ aNowJointList, IcMatrix4D^% aResTgtMat);
		IcIKResult CalcFwd(List<double>^ aNowJointList, IcCoordinates^% aResTgtCoord);
		IcIKResult CalcFwd(List<double>^ aNowJointList, List<double>^% aResTgtCoord);
		IcIKResult CalcFwd(List<double>^ aNowJointList, double% x, double% y, double% z, double% rx, double% ry, double% rz, IcIKRotationType aRotationType);
		void SetConfig(bool, bool, bool);

		// Yaskawa motmanジョイント角度計算
		IcIKResult CalcInvMotomanGp8(IcCoordinates^ aTgtCoord, List<double>^% aResJointList);
		IcIKResult CalcInvMotomanGp8(IcMatrix4D^ aTgtMat, List<double>^% aResJointList);

	private:
		IcIKResult SetNowJoint(List<double>^ aNowJointList);
		IcMatrix4D^ MakeInverseTfmMatZ(double aDeg, double aDhParam_D, bool isDeg);

	private:
		// デバッグ用　ロボットプログラム出力前に検算
		IcIKResult CalcFwdMotomanGp8(List<double>^ aJointList, IcMatrix4D^% aResMat);
		IcIKResult SetNowJointMotomanGp8(List<double>^ aNowJointList);
	};

}
