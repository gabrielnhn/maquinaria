import cv2
import numpy as np


# BGR values to filter only the selected color range
# lower_bgr_values = np.array([185,  190,  191])
lower_bgr_values = np.array([192,  193,  193])

upper_bgr_values = np.array([255, 255, 255])



## User-defined parameters: (Update these values to your liking)
# Minimum size for a contour to be considered anything
MIN_AREA = 10000

# Minimum size for a contour to be considered part of the track
# MIN_AREA_TRACK = 60000
MIN_AREA_TRACK = 30000




def get_contour_data(mask, out):
    """
    Return the centroid of the largest contour in
    the binary image 'mask' (the line)
    and return the side in which the smaller contour is (the track mark)
    (If there are any of these contours),
    and draw all contours on 'out' image
    """
    # get a list of contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    mark = {}
    line = {}


    for contour in contours:
        contour_vertices = len(cv2.approxPolyDP(contour, 1.0, True))

        M = cv2.moments(contour)
        # Search more about Image Moments on Wikipedia :)

        if M['m00'] > MIN_AREA:
        # if countor.area > MIN_AREA

            if (M['m00'] > MIN_AREA_TRACK):
                # Contour is part of the track
                line['x'] = int(M["m10"]/M["m00"])
                line['y'] = int(M["m01"]/M["m00"])

                # plot the area in light blue
                cv2.drawContours(out, contour, -1, (255,255,0), 1)
                # cv2.putText(out, str(M['m00']), (int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"])),
                #     cv2.FONT_HERSHEY_PLAIN, 2, (255,255,0), 2)
                cv2.putText(out, f"LEN: {contour_vertices}", (int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"])),
                    cv2.FONT_HERSHEY_PLAIN, 2, (255,255,0), 2)

            else:
                # Contour is a track mark
                if (not mark) or (mark['y'] > int(M["m01"]/M["m00"])):
                    # if there are more than one mark, consider only
                    # the one closest to the robot
                    mark['y'] = int(M["m01"]/M["m00"])
                    mark['x'] = int(M["m10"]/M["m00"])

                    # plot the area in pink
                    cv2.drawContours(out, contour, -1, (255,0,255), 1)
                    cv2.putText(out, str(M['m00']), (int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"])),
                        cv2.FONT_HERSHEY_PLAIN, 2, (255,0,255), 2)


    if mark and line:
    # if both contours exist
        if mark['x'] > line['x']:
            mark_side = "right"
        else:
            mark_side = "left"
    else:
        mark_side = None


    return (line, mark_side)




image_path = "bosta.png"

image = cv2.imread(image_path)

mask = cv2.inRange(image, lower_bgr_values, upper_bgr_values)

kernel = np.ones((5, 5), np.uint8)
mask = cv2.erode(mask, kernel, iterations=1)


data = get_contour_data(mask, image)



cv2.imshow("image", image)
cv2.imshow("mask", mask)

cv2.waitKey(0)


# cv2.imwrite("BRUH.png", image)
cv2.imwrite("BRUMH1.png", mask)
