
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

// �W���C���g���
public ref class XmlDhParam
{
public:
	XmlDhParam()
	{
		RobotName = "";	// ���O�i���ʗp�j
		JointCount = -1;		// �W���C���g��
		CalcType = -1;		// �v�Z���@
		DecimalRound = 3;	// �����_�ȉ��ۂ�
		DecimalRoundType = 0;

		VecTcpX = 0.0;
		VecTcpY = 0.0;
		VecTcpZ = 0.0;
		AngTcpX = 0.0;
		AngTcpY = 0.0;
		AngTcpZ = 0.0;
		VecTcpXsub = 0.0;	// �s��
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

	// �ėp
	String^ RobotName;	// ���O�i���ʗp�j
	int JointCount;		// �W���C���g��
	int CalcType;		// �v�Z���@
	int DecimalRound;		// �����_�ȉ��ۂ�
	int DecimalRoundType;	// �����_�ȉ��ۂ�

	// TCP
	double VecTcpX;
	double VecTcpY;
	double VecTcpZ;
	double AngTcpX;
	double AngTcpY;
	double AngTcpZ;
	double VecTcpXsub;	// �s��
	double VecTcpYsub;
	double VecTcpZsub;
	double AngTcpXsub;
	double AngTcpYsub;
	double AngTcpZsub;

	double BaseHight;	// ������P�����_�܂ł̍���
	double RobotCoordOriginHeight;	// ���{�b�g���W�ɂĂǂ̍�����0�Ƃ��邩

	// �����N�p�����[�^
	List<XmlDhLinkParam^>^ DhLinkParam;
};