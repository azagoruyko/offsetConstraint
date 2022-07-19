#include <string>
#include <vector>

#include <maya/MFnMatrixData.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnCompoundAttribute.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MFnEnumAttribute.h>
#include <maya/MTransformationMatrix.h>
#include <maya/MEulerRotation.h>
#include <maya/MSelectionList.h>
#include <maya/MDGModifier.h>

#include "offsetConstraint.h"

#define MSTR(v) MString(to_string(v).c_str())

MTypeId OffsetConstraint::typeId(1274449);
MObject OffsetConstraint::attr_target;
MObject OffsetConstraint::attr_targetWeight;
MObject OffsetConstraint::attr_targetMatrix;
MObject OffsetConstraint::attr_targetMatrixBase;
MObject OffsetConstraint::attr_targetsParentInverseMatrix;
MObject OffsetConstraint::attr_constraintParentInverseMatrix;
MObject OffsetConstraint::attr_constraintMatrixBase;
MObject OffsetConstraint::attr_constraintJointOrientX;
MObject OffsetConstraint::attr_constraintJointOrientY;
MObject OffsetConstraint::attr_constraintJointOrientZ;
MObject OffsetConstraint::attr_constraintJointOrient;
MObject OffsetConstraint::attr_constraintRotateOrder;
MObject OffsetConstraint::attr_offsetOrParentBlend;
MObject OffsetConstraint::attr_constraintTranslateX;
MObject OffsetConstraint::attr_constraintTranslateY;
MObject OffsetConstraint::attr_constraintTranslateZ;
MObject OffsetConstraint::attr_constraintTranslate;
MObject OffsetConstraint::attr_constraintRotateX;
MObject OffsetConstraint::attr_constraintRotateY;
MObject OffsetConstraint::attr_constraintRotateZ;
MObject OffsetConstraint::attr_constraintRotate;

inline MVector maxis(const MMatrix& mat, unsigned int a)
{
	return MVector(mat[a][0], mat[a][1], mat[a][2]);
}

inline MVector xaxis(const MMatrix& mat) { return maxis(mat, 0); }
inline MVector yaxis(const MMatrix& mat) { return maxis(mat, 1); }
inline MVector zaxis(const MMatrix& mat) { return maxis(mat, 2); }
inline MPoint  taxis(const MMatrix& mat) { return maxis(mat, 3); }

inline void set_maxis(MMatrix& mat, unsigned int a, const MVector& v)
{
	mat[a][0] = v.x;
	mat[a][1] = v.y;
	mat[a][2] = v.z;
}

inline MVector mscale(const MMatrix& mat)
{
	return MVector(maxis(mat, 0).length(), maxis(mat, 1).length(), maxis(mat, 2).length());
}

inline void set_mscale(MMatrix& mat, const MVector& scale)
{
	set_maxis(mat, 0, maxis(mat, 0).normal() * scale.x);
	set_maxis(mat, 1, maxis(mat, 1).normal() * scale.y);
	set_maxis(mat, 2, maxis(mat, 2).normal() * scale.z);
}

inline MQuaternion mat2quat(const MMatrix& inputMat)
{
	MMatrix mat(inputMat);
	set_mscale(mat, MVector(1, 1, 1)); // reset scale

	float tr, s;
	float q[4];
	int i, j, k;
	MQuaternion quat;

	int nxt[3] = { 1, 2, 0 };
	tr = mat[0][0] + mat[1][1] + mat[2][2];

	// check the diagonal
	if (tr > 0.0)
	{
		s = sqrt(tr + float(1.0));
		quat.w = s / float(2.0);
		s = float(0.5) / s;

		quat.x = (mat[1][2] - mat[2][1]) * s;
		quat.y = (mat[2][0] - mat[0][2]) * s;
		quat.z = (mat[0][1] - mat[1][0]) * s;
	}
	else
	{
		// diagonal is negative
		i = 0;
		if (mat[1][1] > mat[0][0])
			i = 1;
		if (mat[2][2] > mat[i][i])
			i = 2;

		j = nxt[i];
		k = nxt[j];
		s = sqrt((mat[i][i] - (mat[j][j] + mat[k][k])) + float(1.0));

		q[i] = s * float(0.5);
		if (s != float(0.0))
			s = float(0.5) / s;

		q[3] = (mat[j][k] - mat[k][j]) * s;
		q[j] = (mat[i][j] + mat[j][i]) * s;
		q[k] = (mat[i][k] + mat[k][i]) * s;

		quat.x = q[0];
		quat.y = q[1];
		quat.z = q[2];
		quat.w = q[3];
	}

	return quat;
}

inline MMatrix blendMatrices(const MMatrix& m1, const MMatrix& m2, double w)
{
	const MQuaternion q = slerp(mat2quat(m1), mat2quat(m2), w);
	MMatrix m = q.asMatrix();
	set_maxis(m, 3, taxis(m1)*(1-w) + taxis(m2)*w);
	set_mscale(m, mscale(m1)*(1-w) + mscale(m2)*w);
	return m;
}

struct TargetData
{
	MMatrix matrix;
	MMatrix matrixBase;
	double weight;
};

MStatus OffsetConstraint::compute(const MPlug& plug, MDataBlock& dataBlock)
{
	if (plug == attr_constraintTranslate || plug == attr_constraintRotate || plug.parent() == attr_constraintTranslate || plug.parent() == attr_constraintRotate)
	{
		const MMatrix targetsParentInverseMatrix = dataBlock.inputValue(attr_targetsParentInverseMatrix).asMatrix();
		const MMatrix constraintParentInverseMatrix = dataBlock.inputValue(attr_constraintParentInverseMatrix).asMatrix();
		const MMatrix constraintMatrixBase = dataBlock.inputValue(attr_constraintMatrixBase).asMatrix();
		const MVector constraintJointOrient = dataBlock.inputValue(attr_constraintJointOrient).asVector();
		const short constraintRotateOrder = dataBlock.inputValue(attr_constraintRotateOrder).asShort();
		const double offsetOrParentBlend = dataBlock.inputValue(attr_offsetOrParentBlend).asDouble();

		MArrayDataHandle targetArrayHandle = dataBlock.inputArrayValue(attr_target);
		if (targetArrayHandle.elementCount() == 0)
		{
			MGlobal::displayError("Targets are empty");
			return MS::kFailure;
		}

		MTransformationMatrix joTrm;
		joTrm.rotateBy(MEulerRotation(constraintJointOrient, (MEulerRotation::RotationOrder)constraintRotateOrder), MSpace::kWorld);
		const MMatrix jomatInverse = joTrm.asMatrixInverse();

		double weightSum = 1e-5;
		vector<TargetData> targetDataList(targetArrayHandle.elementCount());
		for (int i = 0; i < targetArrayHandle.elementCount(); i++)
		{
			targetArrayHandle.jumpToArrayElement(i);
			MDataHandle targetHandle = targetArrayHandle.inputValue();
			targetDataList[i].weight = targetHandle.child(attr_targetWeight).asDouble();
			targetDataList[i].matrix = targetHandle.child(attr_targetMatrix).asMatrix();
			targetDataList[i].matrixBase = targetHandle.child(attr_targetMatrixBase).asMatrix();

			weightSum += targetDataList[i].weight;
		}

		MVector outputTranslate;
		MQuaternion outputQuat;

		for (int i = 0; i < targetDataList.size(); i++)
		{
			const double& targetWeight = targetDataList[i].weight / weightSum;
			const MMatrix &targetMatrix = targetDataList[i].matrix;
			const MMatrix &targetMatrixBase = targetDataList[i].matrixBase;
			const MMatrix targetMatrixBaseInverse = targetMatrixBase.inverse();

			// Move constraint matrix to target matrix, then parentConstraint it. Otherwise offset won't work unless constraint/target axes are equal.
			// Then calculate and apply offset.
			// parentConstraintMatrix = destBase * srcBase.inverse() * src
			// offsetConstraintMatrix = src * srcBase.inverse() * destBase

			MMatrix movedConstraintBase(constraintMatrixBase);
			set_maxis(movedConstraintBase, 3, taxis(targetMatrixBase));

			const MMatrix offsetInTarget = movedConstraintBase * targetMatrixBaseInverse;
			const MMatrix offsetConstraintMatrix =
				(offsetInTarget * targetMatrix * targetsParentInverseMatrix) *
				(offsetInTarget * targetMatrixBase).inverse() *
				constraintMatrixBase *
				constraintParentInverseMatrix;

			const MMatrix parentConstraintMatrix = (constraintMatrixBase * targetMatrixBaseInverse) * (targetMatrix * targetsParentInverseMatrix) * constraintParentInverseMatrix;

			const MMatrix blendedMatrix = blendMatrices(offsetConstraintMatrix, parentConstraintMatrix, offsetOrParentBlend);

			MMatrix finalMat = blendedMatrix * jomatInverse;
			set_maxis(finalMat, 3, taxis(blendedMatrix));

			outputTranslate += taxis(finalMat) * targetWeight;
			
			if (i == 0) // like orientConstraint with shortest
				outputQuat = mat2quat(finalMat);
			else
				outputQuat = slerp(outputQuat, mat2quat(finalMat), targetWeight);
		}

		dataBlock.outputValue(attr_constraintTranslate).set(outputTranslate);

		MEulerRotation euler = outputQuat.asEulerRotation();
		euler.reorderIt((MEulerRotation::RotationOrder)constraintRotateOrder);
		dataBlock.outputValue(attr_constraintRotate).setMVector(euler.asVector());

		dataBlock.setClean(attr_constraintTranslate);
		dataBlock.setClean(attr_constraintRotate);

		return MS::kSuccess;
	}

	return MPxConstraint::compute(plug, dataBlock);
}

const MObject OffsetConstraint::weightAttribute() const
{
	return OffsetConstraint::attr_targetWeight;
}

const MObject OffsetConstraint::targetAttribute() const
{
	return OffsetConstraint::attr_target;
}

void OffsetConstraint::getOutputAttributes(MObjectArray& attributeArray)
{
	attributeArray.clear();
	attributeArray.append(OffsetConstraint::attr_constraintTranslate);
	attributeArray.append(OffsetConstraint::attr_constraintRotate);
}

MStatus OffsetConstraint::initialize()
{
	MFnTypedAttribute tAttr;
	MFnNumericAttribute nAttr;
	MFnMatrixAttribute mAttr;
	MFnCompoundAttribute cAttr;
	MFnUnitAttribute uAttr;

	attr_targetWeight = nAttr.create("targetWeight", "tw", MFnNumericData::kDouble, 1.0);
	nAttr.setMin(0.0);
	nAttr.setMax(1.0);
	nAttr.setKeyable(true);

	attr_targetMatrix = mAttr.create("targetMatrix", "tm");
	mAttr.setHidden(true);

	attr_targetMatrixBase = mAttr.create("targetMatrixBase", "tmb");
	mAttr.setHidden(true);

	attr_target = cAttr.create("target", "tgt");
	cAttr.addChild(attr_targetWeight);
	cAttr.addChild(attr_targetMatrix);
	cAttr.addChild(attr_targetMatrixBase);
	cAttr.setArray(true);
	cAttr.setHidden(true);
	cAttr.setDisconnectBehavior(MFnAttribute::kDelete);
	addAttribute(attr_target);

	
	attr_targetsParentInverseMatrix = mAttr.create("targetsParentInverseMatrix", "tpim");
	mAttr.setHidden(true);
	addAttribute(attr_targetsParentInverseMatrix);


	attr_constraintParentInverseMatrix = mAttr.create("constraintParentInverseMatrix", "cpim");
	mAttr.setHidden(true);
	addAttribute(attr_constraintParentInverseMatrix);

	attr_constraintMatrixBase = mAttr.create("constraintMatrixBase", "cmb");
	mAttr.setHidden(true);
	addAttribute(attr_constraintMatrixBase);

	attr_constraintJointOrientX = uAttr.create("constraintJointOrientX", "cjox", MFnUnitAttribute::kAngle);
	attr_constraintJointOrientY = uAttr.create("constraintJointOrientY", "cjoy", MFnUnitAttribute::kAngle);
	attr_constraintJointOrientZ = uAttr.create("constraintJointOrientZ", "cjoz", MFnUnitAttribute::kAngle);
	attr_constraintJointOrient = nAttr.create("constraintJointOrient", "cjo", attr_constraintJointOrientX, attr_constraintJointOrientY, attr_constraintJointOrientZ);
	nAttr.setHidden(true);
	addAttribute(attr_constraintJointOrient);

	attr_constraintRotateOrder = nAttr.create("constraintRotateOrder", "cro", MFnNumericData::kShort, 0);
	nAttr.setHidden(true);
	addAttribute(attr_constraintRotateOrder);

	attr_offsetOrParentBlend = nAttr.create("offsetOrParentBlend", "opb", MFnNumericData::kDouble, 0);
	nAttr.setMin(0.0);
	nAttr.setMax(1.0);
	nAttr.setKeyable(true);
	addAttribute(attr_offsetOrParentBlend);

	attr_constraintTranslateX = nAttr.create("constraintTranslateX", "ctx", MFnNumericData::kDouble);
	attr_constraintTranslateY = nAttr.create("constraintTranslateY", "cty", MFnNumericData::kDouble);
	attr_constraintTranslateZ = nAttr.create("constraintTranslateZ", "ctz", MFnNumericData::kDouble);
	attr_constraintTranslate = nAttr.create("constraintTranslate", "ct", attr_constraintTranslateX, attr_constraintTranslateY, attr_constraintTranslateZ);
	nAttr.setHidden(true);
	nAttr.setReadable(true);
	nAttr.setWritable(false);
	addAttribute(attr_constraintTranslate);

	attr_constraintRotateX = uAttr.create("constraintRotateX", "crx", MFnUnitAttribute::kAngle);
	attr_constraintRotateY = uAttr.create("constraintRotateY", "cry", MFnUnitAttribute::kAngle);
	attr_constraintRotateZ = uAttr.create("constraintRotatez", "crz", MFnUnitAttribute::kAngle);
	attr_constraintRotate = nAttr.create("constraintRotate", "cr", attr_constraintRotateX, attr_constraintRotateY, attr_constraintRotateZ);
	nAttr.setHidden(true);
	nAttr.setReadable(true);
	nAttr.setWritable(false);
	addAttribute(attr_constraintRotate);

	attributeAffects(attr_target, attr_constraintTranslate);
	attributeAffects(attr_targetWeight, attr_constraintTranslate);
	attributeAffects(attr_targetMatrix, attr_constraintTranslate);
	attributeAffects(attr_targetMatrixBase, attr_constraintTranslate);
	attributeAffects(attr_targetsParentInverseMatrix, attr_constraintTranslate);
	attributeAffects(attr_constraintMatrixBase, attr_constraintTranslate);
	attributeAffects(attr_constraintParentInverseMatrix, attr_constraintTranslate);
	attributeAffects(attr_constraintJointOrient, attr_constraintTranslate);
	attributeAffects(attr_constraintRotateOrder, attr_constraintTranslate);
	attributeAffects(attr_offsetOrParentBlend, attr_constraintTranslate);

	attributeAffects(attr_target, attr_constraintRotate);
	attributeAffects(attr_targetWeight, attr_constraintRotate);
	attributeAffects(attr_targetMatrix, attr_constraintRotate);
	attributeAffects(attr_targetMatrixBase, attr_constraintRotate);
	attributeAffects(attr_targetsParentInverseMatrix, attr_constraintRotate);
	attributeAffects(attr_constraintParentInverseMatrix, attr_constraintRotate);
	attributeAffects(attr_constraintMatrixBase, attr_constraintRotate);
	attributeAffects(attr_constraintJointOrient, attr_constraintRotate);
	attributeAffects(attr_constraintRotateOrder, attr_constraintRotate);
	attributeAffects(attr_offsetOrParentBlend, attr_constraintRotate);

	return MS::kSuccess;
}
/////////////////////////////////////////////////////////////////////////////
void OffsetConstraintCommand::createdConstraint(MPxConstraint* constraint)
{
	if (constraint)
	{
		OffsetConstraint* c = (OffsetConstraint*)constraint;
	}
	else
		MGlobal::displayError("Failed to get created constraint.");
}

MStatus OffsetConstraintCommand::connectTarget(MDagPath& targetPath, int index)
{
	MStatus status = connectTargetAttribute(targetPath, index, MPxTransform::worldMatrix, OffsetConstraint::attr_targetMatrix, true);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	
	MFnMatrixData matrixFn;
	const MObject matrixObject = matrixFn.create(targetPath.inclusiveMatrix(), &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	MFnDependencyNode constraintFn(constraintNode()->thisMObject());
	MPlug targetMatrixBasePlug = 
		constraintFn.findPlug(OffsetConstraint::attr_target)
		.elementByLogicalIndex(index)
		.child(OffsetConstraint::attr_targetMatrixBase);

	status = targetMatrixBasePlug.setMObject(matrixObject);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	
	return MS::kSuccess;
}

MStatus OffsetConstraintCommand::connectObjectAndConstraint(MDGModifier& modifier)
{
	MStatus status;

	MObject transform = transformObject();
	if (transform.isNull())
		return MS::kFailure;

	MFnTransform transformFn(transform);

	MPlug wmPlug = transformFn.findPlug(MPxTransform::worldMatrix).elementByLogicalIndex(0);
	MFnDependencyNode constraintFn(constraintNode()->thisMObject());
	status = modifier.newPlugValue(constraintFn.findPlug(OffsetConstraint::attr_constraintMatrixBase), wmPlug.asMObject());
	CHECK_MSTATUS_AND_RETURN_IT(status);

	MPlug jo = transformFn.findPlug("jointOrient");
	if (!jo.isNull()) // for joints
	{
		status = connectObjectAttribute(jo.attribute(), OffsetConstraint::attr_constraintJointOrient, true);
		CHECK_MSTATUS_AND_RETURN_IT(status);
	}

	status = connectObjectAttribute(MPxTransform::rotateOrder, OffsetConstraint::attr_constraintRotateOrder, true);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	status = connectObjectAttribute(MPxTransform::parentInverseMatrix, OffsetConstraint::attr_constraintParentInverseMatrix, true, true);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	// outputs
	if (!transformFn.findPlug(MPxTransform::translateX).isLocked())
		CHECK_MSTATUS_AND_RETURN_IT(connectObjectAttribute(MPxTransform::translateX, OffsetConstraint::attr_constraintTranslateX, false, false));

	if (!transformFn.findPlug(MPxTransform::translateY).isLocked())
		CHECK_MSTATUS_AND_RETURN_IT(connectObjectAttribute(MPxTransform::translateY, OffsetConstraint::attr_constraintTranslateY, false, false));

	if (!transformFn.findPlug(MPxTransform::translateZ).isLocked())
		CHECK_MSTATUS_AND_RETURN_IT(connectObjectAttribute(MPxTransform::translateZ, OffsetConstraint::attr_constraintTranslateZ, false, false));

	if (!transformFn.findPlug(MPxTransform::rotateX).isLocked())
		CHECK_MSTATUS_AND_RETURN_IT(connectObjectAttribute(MPxTransform::rotateX, OffsetConstraint::attr_constraintRotateX, false, false));

	if (!transformFn.findPlug(MPxTransform::rotateY).isLocked())
		CHECK_MSTATUS_AND_RETURN_IT(connectObjectAttribute(MPxTransform::rotateY, OffsetConstraint::attr_constraintRotateY, false, false));

	if (!transformFn.findPlug(MPxTransform::rotateZ).isLocked())
		CHECK_MSTATUS_AND_RETURN_IT(connectObjectAttribute(MPxTransform::rotateZ, OffsetConstraint::attr_constraintRotateZ, false, false));

	return MS::kSuccess;
}

const MObject& OffsetConstraintCommand::constraintInstancedAttribute() const
{
	return OffsetConstraint::attr_constraintParentInverseMatrix;
}
const MObject& OffsetConstraintCommand::constraintOutputAttribute() const
{
	return OffsetConstraint::attr_constraintTranslate;
}
const MObject& OffsetConstraintCommand::constraintTargetInstancedAttribute() const
{
	return OffsetConstraint::attr_targetMatrix;
}

const MObject& OffsetConstraintCommand::constraintTargetAttribute() const
{
	return OffsetConstraint::attr_target;
}
const MObject& OffsetConstraintCommand::constraintTargetWeightAttribute() const
{
	return OffsetConstraint::attr_targetWeight;
}

MTypeId OffsetConstraintCommand::constraintTypeId() const
{
	return OffsetConstraint::typeId;
}
MPxConstraintCommand::TargetType OffsetConstraintCommand::targetType() const
{
	return MPxConstraintCommand::kTransform;
}