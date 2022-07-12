**Offset Constraint** is a constraint for Maya that can be used in retargeting. It works very simple but it's quite strange Maya doesn't have a such one.

Compare formulas:<br>
```python
parentConstraint = destBase * srcBase.inverse() * src
offsetConstraint = src * srcBase.inverse() * destBase # src and dest should have the same axes here
```

Here is the difference between pointConstraint (with maintainOffset) + orientConstraint (right) and offsetConstraint (left, selected). <br>
See 'foot' sliding and too much bend in 'knee' on the right retarget.

![offsetConstraint vs pointOrientConstraint_persp](https://user-images.githubusercontent.com/9614751/178586684-0a8b4d6a-2989-4dc4-a278-3ac26475281e.gif)
![offsetConstraint vs pointOrientConstraint_side](https://user-images.githubusercontent.com/9614751/178586689-4fe9c97e-f77c-4f3d-ad08-f7c8432a06b0.gif)

## How to install
Use Visual Studio and cmake. No dependencies required.

Then load `offsetConstraint.mll` in Maya and use it as a usual constraint.
1. Select source and destination transforms.
2. Run `offsetConstraint` command.<br>
It supports multiple targets. Rotation is interpolated via slerp (like shortest in orientConstraint) .
