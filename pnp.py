import numpy as np

world = np.array(\
[\
( 0.000,  0.000,  0.000), \
( 0.000, 0.055,  0.000), \
( 0.000, 0.000,  0.022), \
( 0.000, 0.055,  0.022), \
( 0.088,  0.000,  0.000), \
( 0.088,  0.000,  0.022), \
( 0.088,  0.055,  0.022), \
])

img_pnt = np.array(\
[\
(2940.        , 2297), \
(4182.        , 2189), \
(2911., 1849), \
(4211., 1753), \
(2719., 1681), \
(2703., 1345), \
(3531.        , 1269), \
])

f = 3791
# f = 160 # P20 lite camera 3.8mm sensor pixel size 1.12um
Cu = 2303 # 320 
Cv = 1728 # 240

## Calculate Normalize Point
def calculateNormailzePnt(img_pnt):

  normalized_img = np.zeros((0, 2))
  for i in range(img_pnt.shape[0]):
    vec = np.zeros((1, 2))
    vec[0][0] = (img_pnt[i][0] - Cu) / f
    vec[0][1] = (img_pnt[i][1] - Cv) / f
    normalized_img = np.append(normalized_img, vec, axis = 0)

  return normalized_img

## Create M Matrix that will be solved by SVD.
def createMMatrix(imgPnt, worldPnt):
    
  pntNum = imgPnt.shape[0]
  M = np.empty((0, 12 + pntNum))
  
  for i in range(imgPnt.shape[0]):
    Xw = worldPnt[i][0]
    Yw = worldPnt[i][1]
    Zw = worldPnt[i][2]
    x = imgPnt[i][0]
    y = imgPnt[i][1]
    
    vec = np.zeros((3, 12 + pntNum))
    # For X
    vec[0, 0] = Xw
    vec[0, 1] = Yw
    vec[0, 2] = Zw
    vec[0, 3] = 1.0
    vec[0, 12 + i] = -x
    
    # For Y
    vec[1, 4] = Xw
    vec[1, 5] = Yw
    vec[1, 6] = Zw
    vec[1, 7] = 1.0
    vec[1, 12 + i] = -y
    
    # For Z
    vec[2, 8] = Xw
    vec[2, 9] = Yw
    vec[2, 10] = Zw
    vec[2, 11] = 1.0
    vec[2, 12 + i] = -1

    M  = np.append(M, vec, axis=0)

  return M


def calculateProjectionMatrix(M):

  U, S, V = np.linalg.svd(M, full_matrices=True)

  num = 1
  v = V[V.shape[0] - num]
  
  #print()
  #print("Eigen Value Matrix S : ")
  #print(S)
  
  #print()
  #print("Eigen Vector Matrix V : ")
  #print(V)
  
  #print()
  #print("Eigen Vector with Minimum Eigen Value : ")
  #print(v)
  
  lamda = v[V.shape[0] - num]
  sign = 1
  #print()
  #print("Lamda with minimum Eigen Value : ")
  print(lamda)
  # lamda has to be positive!
  if (lamda < 0):
    sign = -1
  
  #print()
  #print("Projection Matrix P : ")
  P = sign *  np.array([[v[0], v[1], v[2], v[3]], [v[4], v[5], v[6], v[7]], [v[8], v[9], v[10], v[11]]])
  #print(P)      
  return P
    
def calculateRotationMatrixViaQRFactorization(P):
    
  A1 = np.matrix(P[0, 0:3])
  A2 = np.matrix(P[1, 0:3])
  A3 = np.matrix(P[2, 0:3]) 
  
  f = np.linalg.norm(A3)
  R3 = A3 / f
  
  e = A2 * R3.T

  d = np.linalg.norm(A2 - e * R3)
  R2 = (A2 - e * R3) / d

  c = A1 * R3.T
  b = A1 * R2.T
  
  a = np.linalg.norm(A1 - b * R2 - c * R3)
  R1 = (A1 - b * R2 - c * R3) / a
  
  #print()
  #print("Rotation Matrix R : ")
  R = np.zeros((0, 3))
  R = np.append(R, R1, axis=0)
  R = np.append(R, R2, axis=0)
  R = np.append(R, R3, axis=0)
  #print(R)
   
  K = np.matrix([[a, b, c], [0, d, e], [0, 0, f]])
  
  return R, K

# Rotation matrix to Eular https://www.learnopencv.com/rotation-matrix-to-euler-angles/

def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

def rotationMatrixToEulerAngles(R) :
 
  assert(isRotationMatrix(R))
     
  sy = np.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
     
  singular = sy < 1e-6
 
  if  not singular :
        x = np.arctan2(R[2,1] , R[2,2])
        y = np.arctan2(-R[2,0], sy)
        z = np.arctan2(R[1,0], R[0,0])
  else :
        x = np.arctan2(-R[1,2], R[1,1])
        y = np.arctan2(-R[2,0], sy)
        z = 0
 
  return np.array([x, y, z])
  

print("Solve PNP Problem!")

# 1. Calculate Normalize Coordinate.
normalized_img = calculateNormailzePnt(img_pnt)

#M = createMMatrix(normalized_img, world)
# 2. Create M Matrix that will be solved by SVD.
M = createMMatrix(img_pnt, world)

# 3. Calculate Projection Matrix
P = calculateProjectionMatrix(M)

# 4. Calculate R and K matrix via QR factorization.
R, K = calculateRotationMatrixViaQRFactorization(P)

Eular_Angles = rotationMatrixToEulerAngles(R)


print("Camera Intrisic Matrix")
print(K / K[2, 2])

print()
# 5. Translation found by P = [KR KT] -> T = inv(K) * P[:, 3]
print("Camera Translation : ")
print(np.linalg.inv(K) * np.matrix(P[:, 3]).T)

print()
print("Camera Rotation : ")
print(R)

print()
print("Eular Angles : ")
print(Eular_Angles)