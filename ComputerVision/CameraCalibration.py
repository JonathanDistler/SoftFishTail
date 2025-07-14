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
Settings for our particular camera


Distance= [[-2.75729559e-01  1.04389635e-01 -4.49111933e-04 -7.43910505e-05
  -2.53113830e-02]]
RET = 0.1465472828049153
RVECS= (array([[-0.05162181],
       [ 0.02308008],
       [-3.11600805]]), array([[-0.05171959],
       [ 0.02323851],
       [-3.11601676]]), array([[-0.05177829],
       [ 0.02349962],
       [-3.11597496]]), array([[ 0.01126001],
       [ 0.00987828],
       [-1.53670164]]), array([[ 0.01136053],
       [ 0.01007103],
       [-1.53670373]]), array([[-0.00718577],
       [ 0.03049837],
       [-1.560244  ]]), array([[-0.00716393],
       [ 0.03063802],
       [-1.56019821]]), array([[ 0.00160078],
       [ 0.03146436],
       [-1.13447463]]), array([[-0.01842049],
       [ 0.0277032 ],
       [-2.08961508]]), array([[ 0.01566063],
       [ 0.01008768],
       [-1.0660054 ]]), array([[ 0.0155862 ],
       [ 0.01003329],
       [-1.08792327]]), array([[ 0.01208424],
       [ 0.01794524],
       [-2.0911354 ]]), array([[ 0.0122321 ],
       [ 0.01796599],
       [-2.09109835]]))
TVECS= (array([[104.12318512],
       [158.26247529],
       [350.55470546]]), array([[104.12982637],
       [158.26190204],
       [350.54491119]]), array([[104.12242707],
       [158.26543162],
       [350.54963632]]), array([[-57.02214174],
       [107.04713389],
       [352.24353161]]), array([[-57.01891092],
       [107.05100412],
       [352.28138586]]), array([[-245.29462333],
       [ 112.03529063],
       [ 358.52394954]]), array([[-245.30196707],
       [ 112.03428534],
       [ 358.55671967]]), array([[-275.20148001],
       [  19.96631224],
       [ 358.02940985]]), array([[-188.45823194],
       [ 211.70600596],
       [ 358.15929987]]), array([[-73.43679964],
       [ 85.68785931],
       [352.23318858]]), array([[-75.76220577],
       [ 88.79038257],
       [352.27971784]]), array([[ 60.74097148],
       [131.02599016],
       [352.98009235]]), array([[ 60.74422265],
       [131.02035768],
       [352.98655823]]))
"""
