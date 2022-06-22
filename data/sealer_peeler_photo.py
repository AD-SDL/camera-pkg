import cv2


cam = cv2.VideoCapture(0)

# reading the input using the camera
result, image = cam.read()
  
# If image will detected without any error, 
# show result
if result:
  
    # showing result, it take frame name and image 
    # output
    cv2.imshow("sealer_out_peeler_out", image)
  
    # saving image in local storage
    cv2.imwrite("sealer_out_peeler_out.png", image)
  
    # If keyboard interrupt occurs, destroy image 
    # window
    cv2.waitKey(0)
    cv2.destroyWindow("sealer_out_peeler__out")
  
# If captured image is corrupted, moving to else part
else:
    print("No image detected. Please! try again")