import numpy as np
import cv2 as cv
import glob

#follows tutorial by https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html to calibrate a fish-eye lens back to a normal distribution. Need to store at least 10 photographs of a chessboard in the same file location as the script
#the pattern size is the m-1 x n-1 with m and n being the rows and columns of the board

pattern_size = (8, 11)  # inner corners for 9x12 squares
square_size = 30.0      # mm, size of each side of the chest board

criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

objp = np.zeros((pattern_size[0]*pattern_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
objp[:, :2] *= square_size

objpoints = []
imgpoints = []

images = glob.glob('*.jpg')
print(f"Found {len(images)} images.")

for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    ret, corners = cv.findChessboardCorners(gray, pattern_size, None)

    if ret:
        corners2 = cv.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)
        objpoints.append(objp)

        cv.drawChessboardCorners(img, pattern_size, corners2, ret)
        cv.imshow('img', img)
        cv.waitKey(300)
    else:
        print(f"Chessboard not found in {fname}")

if len(objpoints) > 0:
    ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)



img = cv.imread(r"C:\Users\15405\OneDrive\Desktop\Career\ETHZ\ETHZ Work\HatBottle.jpg")
h,  w = img.shape[:2]
newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))

"""
# undistort
mapx, mapy = cv.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (w,h), 5)
dst = cv.remap(img, mapx, mapy, cv.INTER_LINEAR)
 
# crop the image
x, y, w, h = roi
dst = dst[y:y+h, x:x+w]
cv.imwrite('calibresult_2.png', dst)
"""


# undistort
dst = cv.undistort(img, mtx, dist, None, newcameramtx)

# crop the image
x, y, w, h = roi
dst = dst[y:y+h, x:x+w]
cv.imwrite('calibresult_1.png', dst)


cv.destroyAllWindows()

"""
Settings for our particular camera at the distance the fish will be from the camera 

Found 11 images.
MTX [[1.81568134e+04 0.00000000e+00 9.56161742e+02]
 [0.00000000e+00 1.32721393e+04 5.42345423e+02]
 [0.00000000e+00 0.00000000e+00 1.00000000e+00]]


DIST [[-8.42669229e+01  1.67955964e+04 -2.19710745e-02 -3.01774375e-01
   1.78927085e+01]]


DST [[[48 36 36]
  [46 36 36]
  [44 40 39]
  ...
  [ 0  0  0]
  [ 0  0  0]
  [ 0  0  0]]

 [[46 35 35]
  [44 37 37]
  [47 39 39]
  ...
  [ 0  0  0]
  [ 0  0  0]
  [ 0  0  0]]

 [[43 35 35]
  [45 37 37]
  [49 40 38]
  ...
  [ 0  0  0]
  [ 0  0  0]
  [ 0  0  0]]

 ...

 [[28 19 24]
  [27 18 25]
  [29 22 27]
  ...
  [24 28 17]
  [27 30 19]
  [ 2  3  2]]

 [[30 18 24]
  [27 16 23]
  [27 18 25]
  ...
  [23 30 18]
  [23 31 19]
  [ 0  0  0]]

 [[31 18 23]
  [30 18 24]
  [28 17 25]
  ...
  [22 31 18]
  [12 17 10]
  [ 0  0  0]]]

RET 0.4249369251351803

RVECS (array([[ 0.5895107 ],
       [-0.57719099],
       [-1.47901223]]), array([[ 0.58951013],
       [-0.57749148],
       [-1.47889114]]), array([[ 0.58147816],
       [-0.58617926],
       [-1.47654078]]), array([[ 0.58109033],
       [-0.58569294],
       [-1.47674032]]), array([[ 0.58883795],
       [-0.59641857],
       [-1.47149378]]), array([[ 0.58832388],
       [-0.59624867],
       [-1.47167891]]), array([[-0.00179468],
       [-0.76037598],
       [ 0.01925017]]), array([[-0.00182056],
       [-0.76024474],
       [ 0.01927602]]), array([[-0.00483241],
       [-0.76249411],
       [ 0.01958751]]), array([[-0.00491668],
       [-0.76253942],
       [ 0.01951434]]), array([[ 0.003601  ],
       [-0.75352749],
       [ 0.01599649]]))

TVECS (array([[  -59.34418572],
       [  -33.28582945],
       [12988.32016243]]), array([[  -60.25328961],
       [  -33.3599577 ],
       [12992.45247328]]), array([[-9.28416127e+00],
       [-3.32265263e+01],
       [ 1.30466841e+04]]), array([[-9.61815350e+00],
       [-3.32034810e+01],
       [ 1.30525242e+04]]), array([[  -77.37292598],
       [  -34.51757393],
       [12853.57393317]]), array([[  -77.64262181],
       [  -34.54043854],
       [12856.16733583]]), array([[  -28.16871195],
       [ -338.84332683],
       [12922.05679205]]), array([[  -28.19334579],
       [ -338.78739211],
       [12921.6509094 ]]), array([[  -58.99494721],
       [ -339.32467026],
       [12769.69490817]]), array([[  -59.01300913],
       [ -339.29905574],
       [12768.89881924]]), array([[  -33.38620919],
       [ -338.45637179],
       [12687.58587936]]))
       """
