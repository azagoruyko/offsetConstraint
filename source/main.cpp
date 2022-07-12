#include <maya/MFnPlugin.h>
#include <maya/MDrawRegistry.h>

#include "offsetConstraint.h"

MStatus initializePlugin(MObject plugin)
{
	MStatus stat;
	MFnPlugin pluginFn(plugin);

	stat = pluginFn.registerNode("offsetConstraint", OffsetConstraint::typeId, OffsetConstraint::creator, OffsetConstraint::initialize, MPxNode::kConstraintNode);
	CHECK_MSTATUS_AND_RETURN_IT(stat);

	stat = pluginFn.registerConstraintCommand("offsetConstraint", OffsetConstraintCommand::creator);
	CHECK_MSTATUS_AND_RETURN_IT(stat);

	return MS::kSuccess;
}

MStatus uninitializePlugin(MObject plugin)
{
	MStatus stat;
	MFnPlugin pluginFn(plugin);

	stat = pluginFn.deregisterNode(OffsetConstraint::typeId);
	CHECK_MSTATUS_AND_RETURN_IT(stat);

	stat = pluginFn.deregisterConstraintCommand("offsetConstraint");
	CHECK_MSTATUS_AND_RETURN_IT(stat);

	return MS::kSuccess;
}
