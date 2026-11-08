import numpy as np
import cv2
import onnxruntime as rt

class Onnx:
    def __init__(self):
        self.sess = rt.InferenceSession(
            "./model_v2.onnx",
            providers=[('CUDAExecutionProvider', {"cudnn_conv_algo_search": "EXHAUSTIVE"})])
        print('model loaded')

    def run(self, img):
        img_rs=img.copy()

        img = img[:, :, ::-1].transpose(2, 0, 1)
        img = np.ascontiguousarray(img)
        img = np.expand_dims(img, 0)  # add a batch dimension
        img = img.astype(np.float32)
        img=img / 255.0

        # Run segmentation model on inputted image
        img_out = self.sess.run(None, {'input': img})
        x0=img_out[0]

        # Detect driveable area and lane lines

        # da = driveable area
        da_predict=np.argmax(x0, 1)

        height, width, _ = img_rs.shape
        DA = da_predict.astype(np.uint8)[0]*255
        # crop the very top 50 pixels. the model has a weird band of segmentation
        DA[:50, :] = 0
        #LL[:50, :] = 0

        img_rs[DA>100]=[255,0,0]

        return DA