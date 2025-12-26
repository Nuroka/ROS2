import cv2

face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

cap = cv2.VideoCapture(4)

if not cap.isOpened():
    print("Cannot open the camera.")
    exit()

print("Real-time face detection is running...")

while True:
    ret, frame = cap.read()
    if not ret:
        print("cannot read frame.")
        break

    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray_frame, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

    for (x, y, w, h) in faces:
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
        cv2.putText(frame, 'Face Detected', (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 2)

    num_faces = len(faces)
    cv2.putText(frame, f'Faces detected: {num_faces}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

    cv2.imshow('Face Detection', frame)

    key = cv2.waitKey(1) & 0xFF

    if key == ord('s') and num_faces > 0:
        cv2.imwrite('detected_face.png', frame)
        print("Frame with detected face saved as 'detected_face.png'.")

    if key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()