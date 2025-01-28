import cv2
import math

angle = 45
def masking(frame, lower, upper, min_object_area):
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv_frame, lower, upper)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    output = cv2.bitwise_and(frame,frame, mask= mask)
    cv2.imshow("Masked Output",output)
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > min_object_area:
            x, y, w, h = cv2.boundingRect(contour)
            return [(x, y, w, h)]
    return []

def median(lst) -> int :
    if lst:
        n = len(lst)
        s = sorted(lst)
        return (s[n//2-1]/2.0+s[n//2]/2.0, s[n//2])[n % 2]
    return 0

def dashed_line(count, fixed_coordinate, start_coordinate, height, frame, color=(255, 0, 150)):
    line_size = (height / count) / 2
    coordinate_change = height / count
    for i in range(count):
        start_segment = start_coordinate + coordinate_change * i
        end_segment = start_segment + line_size
        cv2.line(frame, (fixed_coordinate, int(start_segment)), (fixed_coordinate, int(end_segment)), color, 1)

def dashed_horizontal_line(count, fixed_coordinate, start_coordinate, height, frame, color=(255, 0, 150)):
    line_size = (height / count) / 2
    coordinate_change = height / count
    for i in range(count):
        start_segment = start_coordinate + coordinate_change * i
        end_segment = start_segment + line_size
        cv2.line(frame, (int(start_segment), fixed_coordinate), (int(end_segment), fixed_coordinate), color, 1)

def draw(target_img, x, y, w, h, texts='', color=(200, 169,31)):
    x1 = int(x)
    y1 = int(y)
    x2 = int(x + w)
    y2 = int(y + h)
    cv2.line(target_img, (x1, y1), (x1 + 15, y1), color, 2)
    cv2.line(target_img, (x1, y1), (x1, y1 + 15), color, 2)

    cv2.line(target_img, (x2, y1), (x2 - 15, y1), color, 2)
    cv2.line(target_img, (x2, y1), (x2, y1 + 15), color, 2)

    cv2.line(target_img, (x2, y2), (x2 - 15, y2), color, 2)
    cv2.line(target_img, (x2, y2), (x2, y2 - 15), color, 2)

    cv2.line(target_img, (x1, y2), (x1 + 15, y2), color, 2)
    cv2.line(target_img, (x1, y2), (x1, y2 - 15), color, 2)

    if texts:
        for i, text in enumerate(texts):
            h2 = i * 25
            cv2.putText(target_img, str(text), (x1, y1 - h2), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

def detect_face(frame, cascade):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    detections = cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))
    return detections

def nothing(x):
    pass

def constrain(value, min_val=(-1), max_val=1):
    if(value <= min_val):
        return min_val

    elif(value >= max_val):
        return max_val

    return value

cv2.namedWindow('Frame')

class Distance:
    def __init__(self, *values):
        self.object_width = float(values[0])
        self.pixel_width = float(values[1])
        self.screen_width = values[2]
        self.camera_angle = math.radians(float(values[3]))
        self.pixel_height = values[4]
        self.coordinate = values[5]
        self.y = values[6]
        self.angle2 = values[7]

    def perpendicular_distance(self):
        ratio = self.screen_width / self.pixel_width
        base = ratio * self.object_width
        h1 = base / 2 * math.sqrt(3)
        return h1

    def camera_angle_calculation(self):
        angle2 = self.pixel_height / self.pixel_width
        angle2 = math.asin(constrain(angle2))
        print(f"Top angle: {90 - (math.degrees(angle2))}")
        return angle2

    def object_angle_and_distance(self):
        center_distance = (self.screen_width / 2) - self.coordinate
        left_side = center_distance < 0
        real_distance = self.object_width * (abs(center_distance) / self.pixel_width)
        h1 = self.perpendicular_distance()
        h2 = math.sqrt((h1 ** 2) + (real_distance ** 2))
        angle3 = math.atan(real_distance / h1)
        return left_side, h2, angle3, center_distance

    def final_distance_and_height(self):
        angle2 = self.camera_angle_calculation()
        h2 = self.perpendicular_distance()
        drone_height = h2 * math.sin(angle2)
        drone_distance = h2 * math.cos(angle2)
        return drone_distance, drone_height, angle2

if __name__ == "__main__":
    cam = cv2.VideoCapture(0)
    obj_list = []
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    object_real_width = 5
    camera_angle = 60

    while True:
        ret, frame = cam.read()
        if ret:
            frame = cv2.flip(frame, 1)

            detections = masking(frame, (0, 0, 0), (135, 220, 160), 150)
            if detections:
                for x, y, w, h in detections:
                    obj = Distance(object_real_width, w, 1028, camera_angle, h, (x + (w / 2)), y, 45)
                    perp_distance = obj.perpendicular_distance()
                    left_side, net_distance, angle3, center_offset = obj.object_angle_and_distance()
                    camera_distance, camera_height, servo_angle = obj.final_distance_and_height()
                    draw(frame, x, y, w, h,
                         [f"Width of object: {object_real_width:.2f}cm",
                          f"Pixel width of object: {h}",
                          f"Pixel height of object: {w}",
                          f"h/w: {h / w}",
                          f"angle: {math.degrees(math.asin(constrain(h / w)))}",
                          f"Perpendicular distance: {perp_distance:.2f}cm"])
                    end_point = int((67 - (center_offset / 10)))
                    cv2.line(frame, (67, 106), (end_point, 10), (255, 255, 0), 1)
                    dashed_horizontal_line(20, 360, 0, 1280, frame, color=(0, 150, 250))
                    cv2.putText(frame, f"Distance to ground: {camera_height}", (10, 126), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                    cv2.putText(frame, f"Bird's Eye Distance: {camera_distance}", (10, 146), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

            cv2.rectangle(frame, (10, 10), (138, 106), (255, 255, 0), 1)
            dashed_line(20, 640, 0, 960, frame)
            dashed_line(5, 74, 15, 96, frame)
            cv2.imshow("Frame", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    cam.release()
    cv2.destroyAllWindows()
