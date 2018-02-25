import face_recognition
import cv2
import numpy as np
import math
import paho.mqtt.client as mqtt

video_capture = cv2.VideoCapture(0)

gonzalo_image = face_recognition.load_image_file("gonzalo.png")
gonzalo_face_encoding = face_recognition.face_encodings(gonzalo_image)[0]

known_face_encodings = [
    gonzalo_face_encoding
]
known_face_names = [
    "Gonzalo"
]

RED = (0, 0, 255)
GREEN = (0, 255, 0)
BLUE = (255, 0, 0)

face_locations = []
face_encodings = []
face_names = []
process_this_frame = True
status = ''
labelColor = GREEN

client = mqtt.Client()
client.connect("localhost", 1883, 60)

while True:
    ret, frame = video_capture.read()

    # Resize frame of video to 1/4 size for faster face recognition processing
    small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)

    # Convert the image from BGR color (which OpenCV uses) to RGB color (which face_recognition uses)
    rgb_small_frame = small_frame[:, :, ::-1]

    face_locations = face_recognition.face_locations(rgb_small_frame)
    face_encodings = face_recognition.face_encodings(rgb_small_frame, face_locations)
    face_landmarks_list = face_recognition.face_landmarks(rgb_small_frame, face_locations)

    face_names = []
    for face_encoding, face_landmarks in zip(face_encodings, face_landmarks_list):
        matches = face_recognition.compare_faces(known_face_encodings, face_encoding)
        name = "Unknown"

        if True in matches:
            first_match_index = matches.index(True)
            name = known_face_names[first_match_index]

            nose_tip = face_landmarks['nose_tip']
            maxLandmark = max(nose_tip)
            minLandmark = min(nose_tip)

            diff = math.fabs(maxLandmark[1] - minLandmark[1])
            if diff < 2:
                status = "center"
                labelColor = BLUE
                client.publish("/face/{}/center".format(name), "1")
            elif maxLandmark[1] > minLandmark[1]:
                status = ">>>>"
                labelColor = RED
                client.publish("/face/{}/left".format(name), "1")
            else:
                status = "<<<<"
                client.publish("/face/{}/right".format(name), "1")
                labelColor = RED

            shape = np.array(face_landmarks['nose_bridge'], np.int32)
            cv2.polylines(frame, [shape.reshape((-1, 1, 2)) * 4], True, (0, 255, 255))
            cv2.fillPoly(frame, [shape.reshape((-1, 1, 2)) * 4], GREEN)

        face_names.append("{} {}".format(name, status))

    for (top, right, bottom, left), name in zip(face_locations, face_names):
        # Scale back up face locations since the frame we detected in was scaled to 1/4 size
        top *= 4
        right *= 4
        bottom *= 4
        left *= 4

        if 'Unknown' not in name.split(' '):
            cv2.rectangle(frame, (left, top), (right, bottom), labelColor, 2)
            cv2.rectangle(frame, (left, bottom - 35), (right, bottom), labelColor, cv2.FILLED)
            cv2.putText(frame, name, (left + 6, bottom - 6), cv2.FONT_HERSHEY_DUPLEX, 1.0, (255, 255, 255), 1)
        else:
            cv2.rectangle(frame, (left, top), (right, bottom), BLUE, 2)

    cv2.imshow('Video', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

video_capture.release()
cv2.destroyAllWindows()
