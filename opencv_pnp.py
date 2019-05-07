import numpy as np
import cv2
import math

world = np.array(\
[\
( 0.000,  0.000,  0.000), \
( -0.055, 0.000,  0.000), \
( 0.000, 0.022,  0.000), \
( -0.055, 0.022,  0.000), \
( 0.000,  0.000,  0.088), \
( 0.000,  0.022,  0.088), \
#( -0.055,  0.022,  0.088), \
])

img_pnts = np.array(\
[\
(1666.        , 2297), \
(424.        , 2189), \
(1695., 1849), \
(395., 1753), \
(1980., 1681), \
(1903., 1345), \
#(1075.        , 1269), \
])


# Camera Intrinsic Paramter
dist_coeffs = np.zeros((5, 1))
width = 4606
height = 3456
focal_length = 3791
center = (width / 2, height / 2)

camera_matrix = np.array(
						[[focal_length, 0, center[0]], 
						[0, focal_length, center[1]],
						[0, 0, 1]], dtype = "double"
						)


print("Solve Perspective N Point Problem")

print("\nCamera Matrix : ")
print(camera_matrix)

print("\nDistortion Coefficient")
print(dist_coeffs)
  
(success, rot_vec, trans_vec) = cv2.solvePnP(world, img_pnts, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)
  
print("\nTranslation Vector")
print(trans_vec)
  
print("\nRotation Vector")
print(rot_vec)
  
print("\nRotation Matrix")
R, jacob = cv2.Rodrigues(rot_vec)
print(R)