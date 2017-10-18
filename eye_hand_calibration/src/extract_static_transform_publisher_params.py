# coding: utf-8
import tf.transformations as transf
import numpy as np

# Paste transform here
Tww = np.array([
	[0.9976904246283719,-0.03753080186398116,-0.05661497607793935,0.01499234094312587],
	[0.03798475945949661,0.9992540573192651,0.006963259279318835,-0.003649473856551957],
	[0.0563114078465744,-0.009097683355306741,0.9983718032396064,0.008261644863923801],
	[0,0,0,1]
])

# Get necessary values
xyz = transf.translation_from_matrix(Tww)
rpy = transf.euler_from_matrix(Tww, 'sxyz')

# Put the angles in the right order!
ypr = rpy[::-1]

# Print the values
for v in np.concatenate([xyz, ypr]):
    print v,
