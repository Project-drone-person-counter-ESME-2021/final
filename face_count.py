import cv2

def face_count(img) :
    face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
    img = cv2.imread(img)

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, 1.05, 5)
    face_count = len(faces)

    return face_count

if __name__ == '__main__' :
    face_count_test = face_count('test.jpg')
    print "Face count :", face_count_test



