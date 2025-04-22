import cv2

def main():
    # Open the default camera (usually 0)
    cap = cv2.VideoCapture("http://192.168.4.1:81/stream")
    
    if not cap.isOpened():
        print("Error: Could not open camera")
        return
    
    print("Starting video stream...")
    print("Press 'q' to quit")
    
    try:
        while True:
            # Capture frame-by-frame
            ret, frame = cap.read()
            
            if not ret:
                print("Error: Can't receive frame")
                break
            
            # Display the frame
            cv2.imshow('Video Stream', frame)
            
            # Break the loop if 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
    except KeyboardInterrupt:
        print("\nStream terminated by user")
    finally:
        # Release everything when done
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
