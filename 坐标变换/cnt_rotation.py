import transforms3d as tfs
import numpy as np 
# 四元数转旋轴角
x = tfs.quaternions.quat2axangle([1,0,0,0])
# 轴角转四元数
y = tfs.quaternions.axangle2quat([1,0,0],0.5)


print(x)
print(y)