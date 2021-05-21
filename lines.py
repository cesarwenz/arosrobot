#%%
import cv2
import numpy as np

# Read input
img = cv2.imread('/home/surface/agx/src/icerobot/icerobot_navigation/maps/mymap.pgm', cv2.IMREAD_GRAYSCALE)

# Initialize output
out = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
img = cv2.Canny(img, 1000, 1000, None, 3)

# Detect and draw lines
lines = cv2.HoughLinesP(img, 40, np.pi/180, 100, minLineLength=910, maxLineGap=240)

for line in lines:
    for x1, y1, x2, y2 in line:
        cv2.line(out, (x1, y1), (x2, y2), (0, 0, 255), 1)
#%%
cv2.imshow('out', out)
cv2.waitKey(0)
cv2.destroyAllWindows()
