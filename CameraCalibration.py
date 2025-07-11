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
