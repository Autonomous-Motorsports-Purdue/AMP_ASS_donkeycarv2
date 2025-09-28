import numpy as np
import cv2
import onnxruntime as rt

class Onnx:
    def __init__(self):
        self.sess = rt.InferenceSession(
            "./model_proc.static_int8.onnx",
            providers=[('CUDAExecutionProvider', {"cudnn_conv_algo_search": "EXHAUSTIVE"})])
        print('model loaded')

    def run(self, img):
        img = img[4:-4, :, :]
        img_rs=img.copy()
        
        

        img = img[:, :, ::-1].transpose(2, 0, 1)
        img = np.ascontiguousarray(img)
        img = np.expand_dims(img, 0)  # add a batch dimension
        img = img.astype(np.float32)
        img=img / 255.0

        # Run segmentation model on inputted image
        img_out = self.sess.run(None, {'input': img})

        #end = time.time()

        x0=img_out[0]
        x1=img_out[1]

        # Detect driveable area and lane lines

        # da = driveable area
        # ll = lane lines
        da_predict=np.argmax(x0, 1)
        ll_predict=np.argmax(x1, 1)

        height, width, _ = img_rs.shape
        DA = da_predict.astype(np.uint8)[0]*255
        LL = ll_predict.astype(np.uint8)[0]*255
        # crop the very top 50 pixels. the new model has a weird band of segmentation
        DA[:50, :] = 0
        LL[:50, :] = 0
        img_rs[DA>100]=[255,0,0]
        img_rs[LL>100]=[0,255,0]
        #print(f"Processed image in: {end-st}")
        return LL, DA
        #cv2.namedWindow("img", cv2.WINDOW_NORMAL)
        #cv2.resizeWindow("img", (755, 490))
        #cv2.imshow('img', imgcopy)
        #cv2.waitKey(1)


        return lanes, drivable
