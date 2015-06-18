import cv2
import numpy as np

__author__ = 'def'

def show_image(text, image):
    cv2.imshow(text, image)
    #cv2.waitKey(-1)

def process_mask(mask, kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))):
    """
        Input: binary mask, kernel size for closing
        Output: filtered contours contained in image
    """
    filtered_mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    contours, dummy = cv2.findContours(filtered_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    contours_filtered = []
    for contour in contours:
        if len(contour) > 20 and cv2.contourArea(contour) > 400:
            contours_filtered.append(contour)

    contours_filtered.sort(key=lambda x: cv2.contourArea(x))
    contours_filtered.reverse()

    return contours_filtered

def simplify_contour(contour):
    perimeter = cv2.arcLength(contour,True)
    approx = cv2.approxPolyDP(contour,0.12*perimeter,True)

    if len(approx) == 4 and cv2.isContourConvex(approx):
        return approx
    else:
        return None

def contour_center(contour):
    M = cv2.moments(contour)
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])
    return (cx, cy)

def main():
    # Load image
    image = cv2.imread("data/image2.png", cv2.CV_LOAD_IMAGE_GRAYSCALE)
    show_image("original", image)

    # Get clothes contour:
    ret, image_contours_src = cv2.threshold(image, 240, 255, cv2.THRESH_BINARY_INV)
    clothes_contours, dummy = cv2.findContours(image_contours_src, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    clothes_contour = clothes_contours[0]
    # Size filtering:
    maxArea = cv2.contourArea(clothes_contour)
    for contour in clothes_contours[1:]:
        currentArea = cv2.contourArea(contour)
        if  currentArea > maxArea:
            maxArea = currentArea
            clothes_contour = contour

    # Simplify contour:
    perimeter = cv2.arcLength(contour,True)
    approx = cv2.approxPolyDP(contour,0.012*perimeter,True)

    # Print clothes contour
    contour_show = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    cv2.drawContours(contour_show, [approx], -1, (0, 0,255))
    for point in approx:
        cv2.circle(contour_show, tuple(point[0]), 3, (0, 0, 255), 2)
    show_image("contour", contour_show)

    # Apply canny to find 'not cross' lines
    blur = cv2.GaussianBlur(image,(11,11),0)
    inverted = cv2.bitwise_not(blur)
    show_image("gauss", inverted)
    edges = cv2.Canny(inverted,80,160, apertureSize=3)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    show_image("canny", edges)
    edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel, iterations=5)
    show_image("canny2", edges)

    # Find highest zones
    ret, highest_zones = cv2.threshold(image, 20, 255, cv2.THRESH_BINARY_INV)
    highest_contours = process_mask(highest_zones)
    highest_points = []
    for contour in highest_contours:
        highest_points.append(contour_center(contour))

    # Print clothes contour
    hp_show = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    cv2.drawContours(hp_show, [approx], -1, (0, 0,255))
    for point in approx:
        cv2.circle(hp_show, tuple(point[0]), 3, (0, 0, 255), 2)
    for point in highest_points:
        cv2.circle(hp_show, tuple(point), 3, (255, 0, 0), 2)
    show_image("h point", hp_show)

    show_image("highest", highest_zones)
    cv2.waitKey(-1)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()