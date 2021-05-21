#%%
import cv2
import numpy as np

# Read input
img = cv2.imread('/home/surface/Downloads/K9YLm.png', cv2.IMREAD_GRAYSCALE)

# Initialize output
out = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

# Median blurring to get rid of the noise; invert image
img = 255 - cv2.medianBlur(img, 3)

# Detect and draw lines
lines = cv2.HoughLinesP(img, 1, np.pi/180, 10, minLineLength=20, maxLineGap=30)
for line in lines:
    for x1, y1, x2, y2 in line:
        cv2.line(out, (x1, y1), (x2, y2), (0, 0, 255), 2)
#%%
cv2.imshow('out', out)

cv2.waitKey(0)
cv2.destroyAllWindows()