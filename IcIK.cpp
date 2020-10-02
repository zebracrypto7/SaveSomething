#include "IcIK.h"
#include "XmlDhParam.h"

using namespace IcCalcUtil;
using namespace IcUtil;
using namespace IcKinematics;

// �R���X�g���N�^
IcInverseKinematics::IcInverseKinematics()
{
	roboBaceMat = gcnew IcMatrix4D();
	toolTfmMat = gcnew IcMatrix4D();

	//�R���t�B�O�l(enum�ŊǗ�)
	m_Config = IcIKConfig::FRONT_UPPER_FLIP;

	//�����ϊ��s��(z���܂���]�Az�������ړ�)
	tfmMatZ_1 = gcnew IcMatrix4D();
	tfmMatZ_2 = gcnew IcMatrix4D();
	tfmMatZ_3 = gcnew IcMatrix4D();
	tfmMatZ_4 = gcnew IcMatrix4D();
	tfmMatZ_5 = gcnew IcMatrix4D();
	tfmMatZ_6 = gcnew IcMatrix4D();

	//�����ϊ��s��(x�������]�Ax�������ړ�)
	tfmMatX_1 = gcnew IcMatrix4D();
	tfmMatX_2 = gcnew IcMatrix4D();
	tfmMatX_3 = gcnew IcMatrix4D();
	tfmMatX_4 = gcnew IcMatrix4D();
	tfmMatX_5 = gcnew IcMatrix4D();
	tfmMatX_6 = gcnew IcMatrix4D();

	//�����ϊ��s��i�����玲�ւ́j
	tfmMat_1 = gcnew IcMatrix4D();
	tfmMat_2 = gcnew IcMatrix4D();
	tfmMat_3 = gcnew IcMatrix4D();
	tfmMat_4 = gcnew IcMatrix4D();
	tfmMat_5 = gcnew IcMatrix4D();
	tfmMat_6 = gcnew IcMatrix4D();
	tfmMat_1_to_6 = gcnew IcMatrix4D();
}
IcInverseKinematics::IcInverseKinematics(String^ aPath)
{
	roboBaceMat = gcnew IcMatrix4D();
	toolTfmMat = gcnew IcMatrix4D();

	//�R���t�B�O�l(enum�ŊǗ�)
	m_Config = IcIKConfig::FRONT_UPPER_FLIP;

	//DH�p�����[�^�ǂݍ���
	Init(aPath);
}

//! @brief �������֐�
//! @param aPath�FDH�p�����[�^�t�@�C���p�X
//! @retval IC_IK_RESULT_SUCCEED�F����
//! @terbal IC_IK_INCORRECT_DH_PARAMETER : DH�p�����[�^�̒l���s�K��
IcIKResult IcInverseKinematics::Init(String^ aPath)
{
	bool failedReadXml = false;
	System::IO::StreamReader^ sr = nullptr;
	m_DhParam = gcnew XmlDhParam();

	// XML�ǂݍ���
	try
	{
		if (!System::IO::File::Exists(aPath)){ return IcIKResult::IC_IK_FILE_NOTFOUND; }	// �t�@�C�����Ȃ���Ύ��s
		System::Xml::Serialization::XmlSerializer^ serializer = gcnew System::Xml::Serialization::XmlSerializer(XmlDhParam::typeid);
		sr = gcnew System::IO::StreamReader(aPath, gcnew System::Text::UTF8Encoding(false));
		m_DhParam = (XmlDhParam^)serializer->Deserialize(sr);
	}
	catch (Exception^ e)
	{
		// �G���[
		failedReadXml = true;
		Console::Write(e->Message);
	}
	finally
	{
		// �t�@�C��������
		if (sr != nullptr)
		{
			sr->Close();
		}
	}
	if (failedReadXml){ return IcIKResult::IC_IK_FILE_NOTFOUND; }	// �ǂݍ��ݎ��s

	roboBaceMat->SetOffsetZ(m_DhParam->BaseHight);
	robotCoordOriginHeight = m_DhParam->RobotCoordOriginHeight;

	// DH�p�����[�^���s�K�؂łȂ����m�F
	//if (m_DhParam->DhLinkParam[1]->DhAlpha != 90.0 || m_DhParam->DhLinkParam[1]->DhAlpha != -90.0)
	//{
	//	// ��P�֐ߎ��Ƒ�Q�֐ߎ����������Ă��Ȃ�
	//	return IcIKResult::IC_IK_INCORRECT_DH_PARAMETER;
	//}
	//if (m_DhParam->DhLinkParam[2]->DhAlpha != 0.0)
	//{
	//	// ��Q�֐ߎ��Ƒ�R�֐ߎ������s�łȂ�
	//	return IcIKResult::IC_IK_INCORRECT_DH_PARAMETER;
	//}
	//if (m_DhParam->DhLinkParam[3]->DhAlpha != 90.0 || m_DhParam->DhLinkParam[3]->DhAlpha != -90.0)
	//{
	//	// ��R�֐ߎ��Ƒ�S�֐ߎ����������Ă��Ȃ�
	//	return IcIKResult::IC_IK_INCORRECT_DH_PARAMETER;
	//}
	//if (m_DhParam->DhLinkParam[4]->DhAlpha != 90.0 || m_DhParam->DhLinkParam[4]->DhAlpha != -90.0)
	//{
	//	// ��S�֐ߎ��Ƒ�T�֐ߎ����������Ă��Ȃ�
	//	return IcIKResult::IC_IK_INCORRECT_DH_PARAMETER;
	//}
	//if (m_DhParam->DhLinkParam[5]->DhAlpha != 90.0 || m_DhParam->DhLinkParam[5]->DhAlpha != -90.0)
	//{
	//	// ��T�֐ߎ��Ƒ�U�֐ߎ����������Ă��Ȃ�
	//	return IcIKResult::IC_IK_INCORRECT_DH_PARAMETER;
	//}
	//if (m_DhParam->DhLinkParam[4]->DhA != 0.0 ||
	//	m_DhParam->DhLinkParam[4]->DhD != 0.0 ||
	//	m_DhParam->DhLinkParam[5]->DhA != 0.0)
	//{
	//	// �s�v�ȃ����N�������Ȃ�
	//	return IcIKResult::IC_IK_INCORRECT_DH_PARAMETER;
	//}
	// DH�p�����[�^�m�F�����܂�

	tfmMatX_2 = gcnew IcMatrix4D(1.0, 0.0, 0.0, m_DhParam->DhLinkParam[1]->DhA,
		0.0, 0.0, -1.0, 0.0,
		0.0, 1.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 1.0);

	tfmMatX_3 = gcnew IcMatrix4D(1.0, 0.0, 0.0, m_DhParam->DhLinkParam[2]->DhA,
		0.0, 1.0, 0.0, 0.0,
		0.0, 0.0, 1.0, 0.0,
		0.0, 0.0, 0.0, 1.0);

	// dhParam_Alpha_3 = 90���ƌ��ߑł�
	tfmMatX_4 = gcnew IcMatrix4D(1.0, 0.0, 0.0, m_DhParam->DhLinkParam[3]->DhA,
		0.0, 0.0, -1.0, 0.0,
		0.0, 1.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 1.0);
	
	// dhParam_Alpha_4 = -90���ƌ��ߑł�
	tfmMatX_5 = gcnew IcMatrix4D(1.0, 0.0, 0.0, m_DhParam->DhLinkParam[4]->DhA,
		0.0, 0.0, 1.0, 0.0,
		0.0, -1.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 1.0);

	// dhParam_Alpha_5 = 90���ƌ��ߑł�
	tfmMatX_6 = gcnew IcMatrix4D(1.0, 0.0, 0.0, m_DhParam->DhLinkParam[5]->DhA,
		0.0, 0.0, -1.0, 0.0,
		0.0, 1.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 1.0);

	return IcIKResult::IC_IK_SUCCEED;
}

//! @brief �c�[���ݒ�
//! @param aToolMat�F�t�����W�ʂ���TCP�ւ̓����ϊ��s��
//! @retval 0�i��IC_IK_RESULT_SUCCEED�j�F����
IcIKResult IcInverseKinematics::SetTool(IcMatrix4D^ aToolMat)
{
	toolTfmMat = aToolMat;
	return IcIKResult::IC_IK_SUCCEED;
}
//! @brief �c�[���ݒ�
//! @param aX,aY,aZ,aRx,aRy,aRz:�t�����W�ʒ��S����݂�TCP���W�p��
//! @param aRotationType 0:�p����ZYX�Ŏ擾
//! @retval 0�i��IC_IK_RESULT_SUCCEED�j�F����
IcIKResult IcInverseKinematics::SetTool(double aX, double aY, double aZ, double aRx, double aRy, double aRz, IcIKRotationType aRotationType)
{
	IcIKResult res = IcIKResult::IC_IK_FAILED;
	if (aRotationType == IcIKRotationType::ZYX)
	{
		// ZYX
		IcCoordinates^ coord = gcnew IcCoordinates(aX, aY, aZ, aRx, aRy, aRz);
		IcMatrix4D^ mat = IcMatrix4D::ConvertFromCoord(coord);
		res = SetTool(mat);
	}
	return res;
}

IcIKResult IcInverseKinematics::SetBase(IcMatrix4D^ aBaseMat)
{
	roboBaceMat = aBaseMat;
	roboBaceMat->SetOffsetZ(m_DhParam->BaseHight);
	return IcIKResult::IC_IK_SUCCEED;
}

IcIKResult IcInverseKinematics::SetBase(IcCoordinates^ aBaseCoord)
{
	IcMatrix4D^ mat = IcMatrix4D::ConvertFromCoord(aBaseCoord);
	IcIKResult res = SetBase(mat);
	return res;
}

//! @brief �c�[���ݒ�
//! @param aX,aY,aZ,aRx,aRy,aRz:��΍��W���猩�����{�b�g�x�[�X���W
//! @param aRotationType 0:�p����ZYX�Ŏ擾
//! @retval 0�i��IC_IK_RESULT_SUCCEED�j�F����
IcIKResult IcInverseKinematics::SetBase(double aX, double aY, double aZ, double aRx, double aRy, double aRz, IcIKRotationType aRotationType)
{
	IcIKResult res = IcIKResult::IC_IK_FAILED;
	if (aRotationType == IcIKRotationType::ZYX)
	{
		// ZYX
		IcCoordinates^ coord = gcnew IcCoordinates(aX, aY, aZ, aRx, aRy, aRz);
		IcMatrix4D^ mat = IcMatrix4D::ConvertFromCoord(coord);
		res = SetBase(mat);
	}
	return res;
}

//! @brief �t�^���w�v�Z
//! @param aTgtMat�F�ڕWTCP�ʒu�p��
//! @param aResJointList�F�v�Z���ʁiDeg�j
//! @param aIsRobotCoord : ture:z=0�̓��{�b�g�ŗL�̂��̂Ƃ��� false:z=0�͒�ʁi���j�Ɠ����������w��
//! @retval IC_IK_RESULT_SUCCEE�F����  IC_IK_OUT_OF_RANGE : �p�x���ғ��͈͊O
IcIKResult IcInverseKinematics::CalcInv(IcMatrix4D^ aTgtMat, List<double>^% aResJointList, bool aIsRobotCoord)
{
	//DH�p�����[�^D
	double dhParam_D_1 = m_DhParam->DhLinkParam[0]->DhD;
	double dhParam_D_2 = m_DhParam->DhLinkParam[1]->DhD;
	double dhParam_D_3 = m_DhParam->DhLinkParam[2]->DhD;
	double dhParam_D_4 = m_DhParam->DhLinkParam[3]->DhD;
	double dhParam_D_5 = m_DhParam->DhLinkParam[4]->DhD;
	double dhParam_D_6 = m_DhParam->DhLinkParam[5]->DhD;

	//DH�p�����[�^A
	double dhParam_A_1 = m_DhParam->DhLinkParam[0]->DhA;
	double dhParam_A_2 = m_DhParam->DhLinkParam[1]->DhA;
	double dhParam_A_3 = m_DhParam->DhLinkParam[2]->DhA;
	double dhParam_A_4 = m_DhParam->DhLinkParam[3]->DhA;
	double dhParam_A_5 = m_DhParam->DhLinkParam[4]->DhA;
	double dhParam_A_6 = m_DhParam->DhLinkParam[5]->DhA;

	//DH�p�����[�^��(deg)
	double dhParam_Alpha_1 = m_DhParam->DhLinkParam[0]->DhAlpha;
	double dhParam_Alpha_2 = m_DhParam->DhLinkParam[1]->DhAlpha;
	double dhParam_Alpha_3 = m_DhParam->DhLinkParam[2]->DhAlpha;
	double dhParam_Alpha_4 = m_DhParam->DhLinkParam[3]->DhAlpha;
	double dhParam_Alpha_5 = m_DhParam->DhLinkParam[4]->DhAlpha;
	double dhParam_Alpha_6 = m_DhParam->DhLinkParam[5]->DhAlpha;

	//dH�p�����[�^��(deg)
	double dhParam_Theta_1 = m_DhParam->DhLinkParam[0]->DhTheta;
	double dhParam_Theta_2 = m_DhParam->DhLinkParam[1]->DhTheta;
	double dhParam_Theta_3 = m_DhParam->DhLinkParam[2]->DhTheta;
	double dhParam_Theta_4 = m_DhParam->DhLinkParam[3]->DhTheta;
	double dhParam_Theta_5 = m_DhParam->DhLinkParam[4]->DhTheta;
	double dhParam_Theta_6 = m_DhParam->DhLinkParam[5]->DhTheta;
	/////////////////////////////////
	// �^�[�Q�b�g�}�g���N�X�Ƀc�[���t�}�g���N�X���E����|���Z�B
	aTgtMat = aTgtMat*toolTfmMat->Inversed();
	// �^�[�Q�b�g�}�g���N�X�ɓy��t�}�g���N�X��������|���Z�B
	aTgtMat = roboBaceMat->Inversed() * aTgtMat;

	IcVector3D^ tgtCoord = gcnew IcVector3D(aTgtMat->GetXyz());
	IcVector3D^ tgtZVec = gcnew IcVector3D(aTgtMat->GetZVec());
	//��5�֐ߎ����_���W(preX, preY, preZ)�擾
	double preX = tgtCoord->x - dhParam_D_6 * tgtZVec->x;
	double preY = tgtCoord->y - dhParam_D_6 * tgtZVec->y;
	double preZ = tgtCoord->z - dhParam_D_6 * tgtZVec->z;
	preZ -= dhParam_D_1;
	if (aIsRobotCoord){ preZ += robotCoordOriginHeight; }

	// �A�[���̓͂�����������
	if (preX*preX + preY*preY + preZ*preZ >
		dhParam_D_2 * dhParam_D_2 + (dhParam_A_3 + dhParam_D_4)*(dhParam_A_3 + dhParam_D_4))
	{
		// �t�^���v�Z�s�i�A�[�����͂��܂���j
		return IcIKResult::IC_IK_OUT_OF_ARM_RANGE;
	}

	//////�t�^���w�v�Z////////////////////////////////////////////

	double Arm = 0.0;
	if (m_Config == IcIKConfig::FRONT_LOWER_FLIP ||
		m_Config == IcIKConfig::FRONT_LOWER_NO_FLIP ||
		m_Config == IcIKConfig::FRONT_UPPER_FLIP ||
		m_Config == IcIKConfig::FRONT_UPPER_NO_FLIP)
	{
		Arm = -dhParam_A_2 + Math::Sqrt(preX * preX + preY * preY - dhParam_D_2 * dhParam_D_2);
	}
	else
	{
		Arm = -dhParam_A_2 - Math::Sqrt(preX * preX + preY * preY - dhParam_D_2 * dhParam_D_2);
	}

	//��1�W���C���g�p
	double cos_1 = ((Arm + dhParam_A_2)*preX - dhParam_D_2*preY) /
		((Arm + dhParam_A_2)*(Arm + dhParam_A_2) + dhParam_D_2*dhParam_D_2);
	double sin_1 = (dhParam_D_2 * preX + (Arm + dhParam_A_2) * preY) /
		((Arm + dhParam_A_2)*(Arm + dhParam_A_2) + dhParam_D_2*dhParam_D_2);
	double theta_1 = Math::Atan2(sin_1, cos_1);

	//��3�W���C���g�p
	double sin_Beta = dhParam_A_4 / Math::Sqrt(dhParam_A_4*dhParam_A_4 + dhParam_D_4*dhParam_D_4);
	double cos_Beta = dhParam_D_4 / Math::Sqrt(dhParam_A_4*dhParam_A_4 + dhParam_D_4*dhParam_D_4);
	double theta_Beta = Math::Atan2(sin_Beta, cos_Beta);
	double sin_Gamma = (preZ*preZ + Arm*Arm - dhParam_D_4*dhParam_D_4 - dhParam_A_4*dhParam_A_4 - dhParam_A_3*dhParam_A_3) /
		2 / Math::Sqrt(dhParam_A_3 * dhParam_A_3 * dhParam_A_4*dhParam_A_4 + dhParam_A_3 * dhParam_A_3 *dhParam_D_4*dhParam_D_4);
	double cos_Gamma = 0.0;
	if (m_Config == IcIKConfig::FRONT_UPPER_FLIP ||
		m_Config == IcIKConfig::FRONT_UPPER_NO_FLIP ||
		m_Config == IcIKConfig::BACK_LOWER_FLIP ||
		m_Config == IcIKConfig::BACK_UPPER_NO_FLIP)
	{
		cos_Gamma = Math::Sqrt(1 - sin_Gamma*sin_Gamma);
	}
	else
	{
		cos_Gamma = -Math::Sqrt(1 - sin_Gamma*sin_Gamma);
	}
	double theta_Gamma = Math::Atan2(sin_Gamma, cos_Gamma);
	double theta_3 = theta_Gamma - theta_Beta;

	//��2�W���C���g�p
	double E = dhParam_D_4 * Math::Cos(theta_3) - dhParam_A_4 * Math::Sin(theta_3);	//�����ȗ��������邽�߂̒萔
	double F = dhParam_D_4 * Math::Sin(theta_3) + dhParam_A_4 * Math::Cos(theta_3); //�����ȗ��������邽�߂̒萔
	double sin_2 = (E * Arm + (F + dhParam_A_3) * preZ) / ((F + dhParam_A_3)*(F + dhParam_A_3) + E*E);
	double cos_2 = ((F + dhParam_A_3)*Arm - E * preZ) / ((F + dhParam_A_3)*(F + dhParam_A_3) + E*E);
	double theta_2 = Math::Atan2(sin_2, cos_2);

	//��1�A��Q�A��3�W���C���g�p��aTgtMat����p����������3�~3���s��posMat���擾
	IcMatrix4D^ posMat = gcnew IcMatrix4D();
	posMat = MakeInverseTfmMatZ(theta_1, dhParam_D_1, false) * aTgtMat;
	posMat = tfmMatX_2->Inversed() * posMat;
	posMat = MakeInverseTfmMatZ(theta_2, dhParam_D_2, false) * posMat;
	posMat = tfmMatX_3->Inversed() * posMat;
	posMat = MakeInverseTfmMatZ(theta_3, dhParam_D_3, false) * posMat;
	posMat = tfmMatX_4->Inversed() * posMat;

	double posMat_11 = posMat[0][0];
	double posMat_12 = posMat[0][1];
	double posMat_13 = posMat[0][2];
	double posMat_21 = posMat[1][0];
	double posMat_22 = posMat[1][1];
	double posMat_23 = posMat[1][2];
	double posMat_31 = posMat[2][0];
	double posMat_32 = posMat[2][1];
	double posMat_33 = posMat[2][2];

	//�p��////////////////////////////

	//��5�W���C���g�p
	double sin_5 = 0.0;
	if (m_Config == IcIKConfig::FRONT_LOWER_NO_FLIP ||
		m_Config == IcIKConfig::FRONT_UPPER_NO_FLIP ||
		m_Config == IcIKConfig::BACK_LOWER_NO_FLIP ||
		m_Config == IcIKConfig::BACK_UPPER_NO_FLIP)
	{
		sin_5 = Math::Sqrt(posMat_13 * posMat_13 + posMat_23 * posMat_23);
	}
	else
	{
		sin_5 = -Math::Sqrt(posMat_13 * posMat_13 + posMat_23 * posMat_23);
	}
	double cos_5 = posMat_33;
	double theta_5 = Math::Atan2(sin_5, cos_5);


	//��4�W���C���g�p
	double theta_4 = 0.0;
	if (sin_5 != 0.0)
	{
		double sin_4 = posMat_23 / sin_5;
		double cos_4 = posMat_13 / sin_5;
		theta_4 = Math::Atan2(sin_4, cos_4);
	}
	else
	{
		double tan_4 = -(posMat_11 - posMat_22) / (posMat_12 + posMat_21);
		theta_4 = Math::Atan(tan_4);
	}

	//��6�W���C���g�p
	double theta_6 = 0.0;
	if (sin_5 != 0.0)
	{
		double sin_6 = posMat_32 / sin_5;
		double cos_6 = -posMat_31 / sin_5;
		theta_6 = Math::Atan2(sin_6, cos_6);
	}
	else
	{
		double sin_6 = Math::Cos(theta_4) * posMat_21 - Math::Sin(theta_4) * posMat_11;
		double cos_6 = Math::Sin(theta_4) * posMat_11 + Math::Cos(theta_4) * posMat_21;
		theta_6 = Math::Atan2(sin_6, cos_6);
	}

	double jointDeg_1 = IcCalcUtilCppCli::RadToDeg(theta_1);
	double jointDeg_2 = IcCalcUtilCppCli::RadToDeg(theta_2);
	double jointDeg_3 = IcCalcUtilCppCli::RadToDeg(theta_3);
	double jointDeg_4 = IcCalcUtilCppCli::RadToDeg(theta_4);
	double jointDeg_5 = IcCalcUtilCppCli::RadToDeg(theta_5);
	double jointDeg_6 = IcCalcUtilCppCli::RadToDeg(theta_6);
	aResJointList->Add(jointDeg_1 + m_DhParam->DhLinkParam[0]->DhTheta);
	aResJointList->Add(jointDeg_2 - 90.0 + m_DhParam->DhLinkParam[1]->DhTheta);
	aResJointList->Add(jointDeg_3 + m_DhParam->DhLinkParam[2]->DhTheta);
	aResJointList->Add(jointDeg_4 + m_DhParam->DhLinkParam[3]->DhTheta);
	aResJointList->Add(jointDeg_5 + m_DhParam->DhLinkParam[4]->DhTheta);
	aResJointList->Add(jointDeg_6 + m_DhParam->DhLinkParam[5]->DhTheta);

	//�֐ߊp�x�͈̓`�F�b�N
	/*if (aResJointList[0] > angle_1_max) return IcIKResult::IC_IK_OUT_OF_ANGLE_RANGE;
	if (aResJointList[0] < angle_1_min) return IcIKResult::IC_IK_OUT_OF_ANGLE_RANGE;
	if (aResJointList[1] > angle_2_max) return IcIKResult::IC_IK_OUT_OF_ANGLE_RANGE;
	if (aResJointList[1] < angle_2_min) return IcIKResult::IC_IK_OUT_OF_ANGLE_RANGE;
	if (aResJointList[2] > angle_3_max) return IcIKResult::IC_IK_OUT_OF_ANGLE_RANGE;
	if (aResJointList[2] < angle_3_min) return IcIKResult::IC_IK_OUT_OF_ANGLE_RANGE;
	if (aResJointList[3] > angle_4_max) return IcIKResult::IC_IK_OUT_OF_ANGLE_RANGE;
	if (aResJointList[3] < angle_4_min) return IcIKResult::IC_IK_OUT_OF_ANGLE_RANGE;
	if (aResJointList[4] > angle_5_max) return IcIKResult::IC_IK_OUT_OF_ANGLE_RANGE;
	if (aResJointList[4] < angle_5_min) return IcIKResult::IC_IK_OUT_OF_ANGLE_RANGE;
	if (aResJointList[5] > angle_6_max) return IcIKResult::IC_IK_OUT_OF_ANGLE_RANGE;
	if (aResJointList[5] < angle_6_min) return IcIKResult::IC_IK_OUT_OF_ANGLE_RANGE;*/

	return IcIKResult::IC_IK_SUCCEED;
}

//! @brief �t�^���w�v�Z
//! @param aTgtCoordinate�F�ڕWTCP�ʒu�p��
//! @param aResJointList�F�v�Z���ʁiDeg�j
//! @param aIsRobotCoord : ture:z=0�̓��{�b�g�ŗL�̂��̂Ƃ��� false:z=0�͒�ʁi���j�Ɠ����������w��
//! @retval IC_IK_RESULT_SUCCEE�F����  IC_IK_OUT_OF_RANGE : �p�x���ғ��͈͊O
IcIKResult IcInverseKinematics::CalcInv(IcCoordinates^ aTgtCoordinate, List<double>^% aResJointList, bool aIsRobotCoord)
{
	IcMatrix4D^ TgtMat = IcCalcUtil::IcMatrix4D::ConvertFromCoord(aTgtCoordinate);
	IcIKResult res = CalcInv(TgtMat, aResJointList, aIsRobotCoord);
	return res;
}

//! @brief �t�^���w�v�Z
//! @param aTgtCoordinate�F�ڕWTCP�ʒu�p��
//! @param aResJointList�F�v�Z���ʁiDeg�j
//! @param aIsRobotCoord : ture:z=0�̓��{�b�g�ŗL�̂��̂Ƃ��� false:z=0�͒�ʁi���j�Ɠ����������w��
//! @retval IC_IK_RESULT_SUCCEE�F����  IC_IK_OUT_OF_RANGE : �p�x���ғ��͈͊O
IcIKResult IcInverseKinematics::CalcInv(List<double>^ aTgtCoordinate, List<double>^% aResJointList, bool aIsRobotCoord)
{
	IcCoordinates^ coord = gcnew IcCoordinates(aTgtCoordinate);
	IcMatrix4D^ TgtMat = IcCalcUtil::IcMatrix4D::ConvertFromCoord(coord);
	IcIKResult res = CalcInv(TgtMat, aResJointList, aIsRobotCoord);
	return res;
}

//! @brief ���^���w�v�Z
//! @oaran aNowJointList : ���݂̊e���W���C���g�p�iDeg�j
//! @param aResTgtMat : ����TCP�ʒu�p��
//! @param retval  0�i��IC_IK_RESULT_SUCCEED�j�F����
IcIKResult IcInverseKinematics::CalcFwd(List<double>^ aNowJointList, IcMatrix4D^% aResTgtMat)
{
	IcIKResult result;
	IcMatrix4D^ calcResMat; //�ꎞ�v�Z���ʊi�[�p

	//���݃W���C���g�p�ݒ�
	result = SetNowJoint(aNowJointList);
	if (result != IcIKResult::IC_IK_SUCCEED)
	{
		//�G���[
		return IcIKResult::IC_IK_NOW_JOINT_ERROR;
	}
	//���^���w�v�Z
	calcResMat = roboBaceMat*tfmMat_1;
	calcResMat *= tfmMat_2;
	calcResMat *= tfmMat_3;
	calcResMat *= tfmMat_4;
	calcResMat *= tfmMat_5;
	calcResMat *= tfmMat_6;
	calcResMat *= toolTfmMat;

	aResTgtMat = calcResMat;

	// z���W��������̍���
	// �������Ȃ�

	// z���W�����{�b�g���_����̍���
	//aResTgtMat[2][3] -= m_DhParam->RobotCoordOriginHeight;

	return IcIKResult::IC_IK_SUCCEED;
}

//! @brief ���^���w�v�Z
//! @oaran aNowJointList : ���݂̊e���W���C���g�p�iDeg�j
//! @param aResTgtCoord : ����TCP�ʒu�p��
//! @param retval  0�i��IC_IK_RESULT_SUCCEED�j�F����
IcIKResult IcInverseKinematics::CalcFwd(List<double>^ aNowJointList, IcCoordinates^% aResTgtCoord)
{
	IcIKResult result = IcIKResult::IC_IK_FAILED;
	IcMatrix4D^ aResMat = gcnew IcMatrix4D();
	result = CalcFwd(aNowJointList, aResMat);
	aResTgtCoord->x = aResMat[0][3];
	aResTgtCoord->y = aResMat[1][3];
	aResTgtCoord->z = aResMat[2][3];

	double aResMat_11 = aResMat[0][0];
	double aResMat_12 = aResMat[0][1];
	double aResMat_13 = aResMat[0][2];
	double aResMat_21 = aResMat[1][0];
	double aResMat_22 = aResMat[1][1];
	double aResMat_23 = aResMat[1][2];
	double aResMat_31 = aResMat[2][0];
	double aResMat_32 = aResMat[2][1];
	double aResMat_33 = aResMat[2][2];

	// ZYX //////////////////////////////////////////////
	double cos_ry = Math::Sqrt(aResMat_11*aResMat_11 + aResMat_21 * aResMat_21);
	double sin_ry = aResMat_31;
	double theta_ry = Math::Atan2(sin_ry, cos_ry);

	double theta_rx = 0.0;
	if (cos_ry != 0.0)
	{
		double cos_rx = aResMat_33 / cos_ry;
		double sin_rx = aResMat_32 / cos_ry;
		theta_rx = Math::Atan2(sin_rx, cos_rx);
	}
	else
	{

	}

	double theta_rz = 0.0;
	if (cos_ry != 0.0)
	{
		double cos_rz = aResMat_11 / cos_ry;
		double sin_rz = aResMat_21 / cos_ry;
		theta_rz = Math::Atan2(sin_rz, cos_rz);
	}
	else
	{

	}

	double jointDeg_rx = IcCalcUtil::IcCalcUtilCppCli::RadToDeg(theta_rx);
	double jointDeg_ry = IcCalcUtil::IcCalcUtilCppCli::RadToDeg(-theta_ry);
	//double jointDeg_ry = IcCalcUtil::IcCalcUtilCppCli::RadToDeg(theta_ry);
	double jointDeg_rz = IcCalcUtil::IcCalcUtilCppCli::RadToDeg(theta_rz);

	aResTgtCoord->rx = jointDeg_rx;
	aResTgtCoord->ry = jointDeg_ry;
	aResTgtCoord->rz = jointDeg_rz;

	return result;
}

//! @brief ���^���w�v�Z
//! @oaran aNowJointList : ���݂̊e���W���C���g�p�iDeg�j
//! @param aResTgtCoord : ����TCP�ʒu�p��
//! @param retval  0�i��IC_IK_RESULT_SUCCEED�j�F����
IcIKResult IcInverseKinematics::CalcFwd(List<double>^ aNowJointList, List<double>^% aResTgtCoord)
{
	IcCoordinates^ resTgtCoord = gcnew IcCoordinates();
	IcIKResult result = CalcFwd(aNowJointList, resTgtCoord);
	if (result == IcIKResult::IC_IK_SUCCEED)
	{
		aResTgtCoord = gcnew List<double>;
		aResTgtCoord->Add(resTgtCoord->x);
		aResTgtCoord->Add(resTgtCoord->z);
		aResTgtCoord->Add(resTgtCoord->rx);
		aResTgtCoord->Add(resTgtCoord->ry);
		aResTgtCoord->Add(resTgtCoord->rz);
	}

	return result;
}

//! @brief ���^���w�v�Z
//! @oaran aNowJointList : ���݂̊e���W���C���g�p�iDeg�j
//! @param x, y, z, rx, ry, rz : ����TCP�ʒu�p��
//! @param retval  0�i��IC_IK_RESULT_SUCCEED�j�F����
IcIKResult IcInverseKinematics::CalcFwd(List<double>^ aNowJointList, double% x, double% y, double% z, double% rx, double% ry, double% rz, IcIKRotationType aRotationType)
{
	IcIKResult result = IcIKResult::IC_IK_FAILED;

	if (aRotationType == IcIKRotationType::ZYX)
	{
		// ZYX
		IcCoordinates^ coord = gcnew IcCoordinates();
		result = CalcFwd(aNowJointList, coord);
		x = coord->x;
		y = coord->y;
		z = coord->z;
		rx = coord->rx;
		ry = coord->ry;
		rz = coord->rz;
	}

	return result;
}

//! @brief ���݃W���C���g�p�ݒ�
//! @param aNowJointList�F���݂̊e���W���C���g�p�iDeg�j
//! @retval  0�i��IC_IK_RESULT_SUCCEED�j�F����
IcIKResult IcInverseKinematics::SetNowJoint(List<double>^ aNowJointList)
{
	double jointRad_1 = IcCalcUtilCppCli::DegToRad(aNowJointList[0] - m_DhParam->DhLinkParam[0]->DhTheta);
	double jointRad_2 = IcCalcUtilCppCli::DegToRad(aNowJointList[1] + 90.0 - m_DhParam->DhLinkParam[1]->DhTheta);
	double jointRad_3 = IcCalcUtilCppCli::DegToRad(aNowJointList[2] - m_DhParam->DhLinkParam[2]->DhTheta);
	double jointRad_4 = IcCalcUtilCppCli::DegToRad(aNowJointList[3] - m_DhParam->DhLinkParam[3]->DhTheta);
	double jointRad_5 = IcCalcUtilCppCli::DegToRad(aNowJointList[4] - m_DhParam->DhLinkParam[4]->DhTheta);
	double jointRad_6 = IcCalcUtilCppCli::DegToRad(aNowJointList[5] - m_DhParam->DhLinkParam[5]->DhTheta);

	tfmMatZ_1 = gcnew IcMatrix4D(Math::Cos(jointRad_1), -Math::Sin(jointRad_1), 0.0, 0.0,
		Math::Sin(jointRad_1), Math::Cos(jointRad_1), 0.0, 0.0,
		0.0, 0.0, 1.0, m_DhParam->DhLinkParam[0]->DhD,
		0.0, 0.0, 0.0, 1.0);

	tfmMatZ_2 = gcnew IcMatrix4D(Math::Cos(jointRad_2), -Math::Sin(jointRad_2), 0.0, 0.0,
		Math::Sin(jointRad_2), Math::Cos(jointRad_2), 0.0, 0.0,
		0.0, 0.0, 1.0, m_DhParam->DhLinkParam[1]->DhD,
		0.0, 0.0, 0.0, 1.0);

	tfmMatZ_3 = gcnew IcMatrix4D(Math::Cos(jointRad_3), -Math::Sin(jointRad_3), 0.0, 0.0,
		Math::Sin(jointRad_3), Math::Cos(jointRad_3), 0.0, 0.0,
		0.0, 0.0, 1.0, m_DhParam->DhLinkParam[2]->DhD,
		0.0, 0.0, 0.0, 1.0);

	tfmMatZ_4 = gcnew IcMatrix4D(Math::Cos(jointRad_4), -Math::Sin(jointRad_4), 0.0, 0.0,
		Math::Sin(jointRad_4), Math::Cos(jointRad_4), 0.0, 0.0,
		0.0, 0.0, 1.0, m_DhParam->DhLinkParam[3]->DhD,
		0.0, 0.0, 0.0, 1.0);

	tfmMatZ_5 = gcnew IcMatrix4D(Math::Cos(jointRad_5), -Math::Sin(jointRad_5), 0.0, 0.0,
		Math::Sin(jointRad_5), Math::Cos(jointRad_5), 0.0, 0.0,
		0.0, 0.0, 1.0, m_DhParam->DhLinkParam[4]->DhD,
		0.0, 0.0, 0.0, 1.0);

	tfmMatZ_6 = gcnew IcMatrix4D(Math::Cos(jointRad_6), -Math::Sin(jointRad_6), 0.0, 0.0,
		Math::Sin(jointRad_6), Math::Cos(jointRad_6), 0.0, 0.0,
		0.0, 0.0, 1.0, m_DhParam->DhLinkParam[5]->DhD,
		0.0, 0.0, 0.0, 1.0);

	tfmMatX_2 = gcnew IcMatrix4D(1.0, 0.0, 0.0, m_DhParam->DhLinkParam[1]->DhA,
		0.0, 0.0, -1.0, 0.0,
		0.0, 1.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 1.0);

	tfmMatX_3 = gcnew IcMatrix4D(1.0, 0.0, 0.0, m_DhParam->DhLinkParam[2]->DhA,
		0.0, 1.0, 0.0, 0.0,
		0.0, 0.0, 1.0, 0.0,
		0.0, 0., 0.0, 1.0);

	// dhParam_Alpha_3 = 90���ƌ��ߑł�
	tfmMatX_4 = gcnew IcMatrix4D(1.0, 0.0, 0.0, m_DhParam->DhLinkParam[3]->DhA,
		0.0, 0.0, -1.0, 0.0,
		0.0, 1.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 1.0);

	// dhParam_Alpha_4 = -90���ƌ��ߑł�
	tfmMatX_5 = gcnew IcMatrix4D(1.0, 0.0, 0.0, m_DhParam->DhLinkParam[4]->DhA,
		0.0, 0.0, 1.0, 0.0,
		0.0, -1.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 1.0);

	// dhParam_Alpha_5 = 90���ƌ��ߑł�
	tfmMatX_6 = gcnew IcMatrix4D(1.0, 0.0, 0.0, m_DhParam->DhLinkParam[5]->DhA,
		0.0, 0.0, -1.0, 0.0,
		0.0, 1.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 1.0);


	//�e�֐ߖ��ϊ��s��쐬
	tfmMat_1 = tfmMatZ_1 * tfmMatX_2;
	tfmMat_2 = tfmMatZ_2 * tfmMatX_3;
	tfmMat_3 = tfmMatZ_3 * tfmMatX_4;
	tfmMat_4 = tfmMatZ_4 * tfmMatX_5;
	tfmMat_5 = tfmMatZ_5 * tfmMatX_6;
	tfmMat_6 = tfmMatZ_6;

	return IcIKResult::IC_IK_SUCCEED;
}

//! @brief z������t�ϊ��s��擾
//! @param aTheta z�������]�p�x
//! @param aDhParam_D z���������s�ړ���
//! @param isDeg true:aTheta���x���P�ʁ@false:aTheta�����W�A���P��
//! @retval  0�i��IC_IK_RESULT_SUCCEED�j�F����
IcMatrix4D^ IcInverseKinematics::MakeInverseTfmMatZ(double aTheta, double aDhParam_D, bool isDeg)
{
	double theta = aTheta;
	if (isDeg) theta = IcCalcUtilCppCli::DegToRad(aTheta);
	IcMatrix4D^ invMat = gcnew IcMatrix4D(Math::Cos(-theta), -Math::Sin(-theta), 0.0, 0.0,
		Math::Sin(-theta), Math::Cos(-theta), 0.0, 0.0,
		0.0, 0.0, 1.0, -aDhParam_D,
		0.0, 0.0, 0.0, 1.0);
	return invMat;
}

//! @brief �R���t�B�O�l�Z�b�g
//! @param isFront true:Front false:Back
//! @param isUpper true:Upper false:Lower
//! @param isNoFlip true:NoFlip false:Flip
void IcInverseKinematics::SetConfig(bool isFront, bool isUpper, bool isNoFlip)
{
	if (isFront)
	{
		if (isUpper)
		{
			if (isNoFlip) m_Config = IcIKConfig::FRONT_UPPER_NO_FLIP;
			else m_Config = IcIKConfig::FRONT_UPPER_FLIP;
		}
		else
		{
			if (isNoFlip) m_Config = IcIKConfig::FRONT_LOWER_NO_FLIP;
			else m_Config = IcIKConfig::FRONT_LOWER_FLIP;
		}
	}
	else
	{
		if (isUpper)
		{
			if (isNoFlip) m_Config = IcIKConfig::BACK_UPPER_NO_FLIP;
			else m_Config = IcIKConfig::BACK_UPPER_FLIP;
		}
		else
		{
			if (isNoFlip) m_Config = IcIKConfig::BACK_LOWER_NO_FLIP;
			else m_Config = IcIKConfig::BACK_LOWER_FLIP;
		}
	}
}