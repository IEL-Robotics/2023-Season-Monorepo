import cv2
from pupil_apriltags import Detector
det=Detector(families="tag16h5")
cap=cv2.VideoCapture(0)
def getDistance(p1,p2):
    return ((p1[0]-p2[0])**2+(p1[1]+p2[1])**2)**0.5
while True:
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    _,frame=cap.read()
    frame_siyah_beyaz=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    detection=det.detect(frame_siyah_beyaz,estimate_tag_pose=False,camera_params=None,tag_size=None)
    for dete in detection:
        corners=dete.corners
        corn1=(int(corners[0][0]),int(corners[0][1]))
        corn2=(int(corners[1][0]),int(corners[1][1]))
        corn3=(int(corners[2][0]),int(corners[2][1]))
        corn4=(int(corners[3][0]),int(corners[3][1]))
        points=[corn1,corn2,corn3,corn4]
        points_oriented=[(corn1,corn2),(corn1,corn3),(corn1,corn4)]
        distances=[getDistance(*i) for i in points_oriented]
        if max(distances)/min(distances)>1.4:
            cv2.line(frame, (corn1[0], corn1[1]),
                    (corn2[0], corn2[1]), (0, 255, 0), 1)
            cv2.line(frame, (corn2[0], corn2[1]),
                    (corn3[0], corn3[1]), (0, 255, 0), 1)
            cv2.line(frame, (corn3[0], corn3[1]),
                    (corn4[0], corn4[1]), (0,255, 0), 1)
            cv2.line(frame, (corn4[0], corn4[1]),
                    (corn1[0], corn1[1]), (0,255, 0), 1)
    cv2.imshow("video",frame)
cap.release()
cv2.destroyAllWindows()