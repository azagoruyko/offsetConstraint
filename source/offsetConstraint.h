#pragma	once

#include <maya/MPxNode.h>
#include <maya/MFnPluginData.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MGlobal.h>
#include <maya/MPlug.h>
#include <maya/MPxConstraint.h>
#include <maya/MPxConstraintCommand.h>

using namespace std;

class OffsetConstraintCommand : public MPxConstraintCommand
{
public:
	static void* creator() { return new OffsetConstraintCommand; }

	MTypeId constraintTypeId() const override;
	MPxConstraintCommand::TargetType targetType() const override;

	const MObject& constraintInstancedAttribute() const override;
	const MObject& constraintOutputAttribute() const override;
	const MObject& constraintTargetInstancedAttribute() const override;
	const MObject& constraintTargetAttribute() const override;
	const MObject& constraintTargetWeightAttribute() const override;
	//const MObject& objectAttribute() const override;
	MStatus connectTarget(MDagPath& targetPath, int index);
	MStatus connectObjectAndConstraint(MDGModifier& modifier) override;
	void createdConstraint(MPxConstraint* constraint) override;

};

class OffsetConstraint : public MPxConstraint
{
public:
	static MTypeId typeId;
	static MObject attr_target;
	static MObject attr_targetWeight;
	static MObject attr_targetMatrix;
	static MObject attr_targetMatrixBase;
	static MObject attr_constraintParentInverseMatrix;
	static MObject attr_constraintMatrixBase;
	static MObject attr_constraintJointOrientX;
	static MObject attr_constraintJointOrientY;
	static MObject attr_constraintJointOrientZ;
	static MObject attr_constraintJointOrient;
	static MObject attr_constraintRotateOrder;
	static MObject attr_offsetOrParentBlend;
	static MObject attr_constraintTranslateX;
	static MObject attr_constraintTranslateY;
	static MObject attr_constraintTranslateZ;
	static MObject attr_constraintTranslate;
	static MObject attr_constraintRotateX;
	static MObject attr_constraintRotateY;
	static MObject attr_constraintRotateZ;
	static MObject attr_constraintRotate;

	static void* creator() { return new OffsetConstraint; }
	static MStatus initialize();

	MStatus compute(const MPlug& plug, MDataBlock& dataBlock);

	const MObject weightAttribute() const;
	const MObject targetAttribute() const;
	void getOutputAttributes(MObjectArray& attributeArray);	
};

