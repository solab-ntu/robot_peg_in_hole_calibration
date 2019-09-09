import numpy as np
import cv2
import yaml
import glob

def main(DEBUG):
    CONFIG = 'config.yaml'
    with open(CONFIG) as f:
        path = yaml.load(f)

    BASE = path['holeSearching'] if DEBUG else path['ROOT']

    IMAGE_PATH = BASE + 'img/hs*.bmp'

    images = sorted(glob.glob(IMAGE_PATH))
    # Load the left and right images in gray scale
    imgLeft = cv2.imread(images[0], 0)
    imgRight = cv2.imread(images[1], 0)

    # Initialize the stereo block matching object 
    stereo = cv2.StereoBM_create(numDisparities=32, blockSize=13)

    # Compute the disparity image
    disparity = stereo.compute(imgLeft, imgRight)

    # Normalize the image for representation
    min = disparity.min()
    max = disparity.max()
    disparity = np.uint8(255 * (disparity - min) / (max - min))
    x = disparity.nonzero()[0][0]
    y = disparity.nonzero()[1][0]
    print(disparity[x,y])
    # Display the result
    cv2.namedWindow('disparity', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('disparity', 1200, 800)
    cv2.imshow('disparity', np.hstack((imgLeft, imgRight, disparity)))
    cv2.waitKey(0)
    cv2.destroyAllWindows()
if __name__ == "__main__":
    DEBUG = True
    main(DEBUG)