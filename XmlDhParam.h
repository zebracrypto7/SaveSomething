
#pragma once

using namespace System;
using namespace System::Collections::Generic;

public ref class XmlDhLinkParam
{
public:
	XmlDhLinkParam()
	{
		JointType = -1;
		DhA = 0.0;
		DhD = 0.0;
		DhTheta = 0.0;
		DhAlpha = 0.0;
		VecX = 0.0;
		VecY = 0.0;
		VecZ = 0.0;
		AngX = 0.0;
		AngY = 0.0;
		AngZ = 0.0;
		Max = 0.0;
		Min = 0.0;
	}
	int JointType;
	double DhA;
	double DhD;
	double DhTheta;
	double DhAlpha;
	double VecX;
	double VecY;
	double VecZ;
	double AngX;
	double AngY;
	double AngZ;
	double Max;
	double Min;
};

// ジョイント情報
public ref class XmlDhParam
{
public:
	XmlDhParam()
	{
		RobotName = "";	// 名前（識別用）
		JointCount = -1;		// ジョイント数
		CalcType = -1;		// 計算方法
		DecimalRound = 3;	// 小数点以下丸め
		DecimalRoundType = 0;

		VecTcpX = 0.0;
		VecTcpY = 0.0;
		VecTcpZ = 0.0;
		AngTcpX = 0.0;
		AngTcpY = 0.0;
		AngTcpZ = 0.0;
		VecTcpXsub = 0.0;	// 不明
		VecTcpYsub = 0.0;
		VecTcpZsub = 0.0;
		AngTcpXsub = 0.0;
		AngTcpYsub = 0.0;
		AngTcpZsub = 0.0;

		BaseHight = 0.0;
		RobotCoordOriginHeight = 0.0;

		DhLinkParam = gcnew List<XmlDhLinkParam^>();
	}
	~XmlDhParam(){}

	// 汎用
	String^ RobotName;	// 名前（識別用）
	int JointCount;		// ジョイント数
	int CalcType;		// 計算方法
	int DecimalRound;		// 小数点以下丸め
	int DecimalRoundType;	// 小数点以下丸め

	// TCP
	double VecTcpX;
	double VecTcpY;
	double VecTcpZ;
	double AngTcpX;
	double AngTcpY;
	double AngTcpZ;
	double VecTcpXsub;	// 不明
	double VecTcpYsub;
	double VecTcpZsub;
	double AngTcpXsub;
	double AngTcpYsub;
	double AngTcpZsub;

	double BaseHight;	// 床から１軸原点までの高さ
	double RobotCoordOriginHeight;	// ロボット座標にてどの高さを0とするか

	// リンクパラメータ
	List<XmlDhLinkParam^>^ DhLinkParam;
};