import cv2

# distance from camera to object measured(meter)
KNOWN_STICKER_DISTANCE = 0.4
# width of object in the real world or object plane(meter)
KNOWN_STICKER_WIDTH = 0.06
# colors
GREEN = (0, 255, 0)
RED = (0, 0, 255)
WHITE = (255, 255, 255)
fonts = cv2.FONT_HERSHEY_COMPLEX

# focal length finder function
def focal_length(width_in_rf_image, measured_distance, real_width):
    focal_length_value = (width_in_rf_image*measured_distance)/real_width
    # return focal_length_value
    return focal_length_value

# distance estimation function
def distance_finder(focal_length, object_width_in_frame, real_object_width):
    distance = (real_object_width*focal_length)/object_width_in_frame
    return distance
