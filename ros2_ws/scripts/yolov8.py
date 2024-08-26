import torch
from ultralytics import YOLO
from kas_utils.visualization import draw_objects
import numpy as np
import cv2

class YOLOv8_wrapper(YOLO):
    def __init__(self, model_file, weights_file, min_score=0.7):
        super().__init__(model_file)
        self.model_file = model_file
        self.weights_file = weights_file
        self.min_score = min_score

        # Load model weights
        weights = torch.load(self.weights_file)
        print(f"Weights loaded. Keys in weights: {weights.keys()}")
        
        model_weights = weights['model']
        self.model.load(model_weights)
        print(f"Model weights loaded: {self.model}")
        
        self.model.eval()  # Ensure the model is in evaluation mode
        self.warmup()
        print("Model warmed up and ready for inference.")

    def segment(self, image):
        if isinstance(image, str):
            image = cv2.imread(image)
            if image is None:
                print(f"Error: Unable to load image from path: {image}")
                return None
            else:
                print(f"Image loaded from path: {image}")

        height, width = image.shape[:2]
        print(f"Image dimensions: {height}x{width}")

        # Определяем метод predict для удобства
        predict = self.__call__

        # Run inference using predict instead of self
        results = predict(image, save=False, show=False, verbose=False, conf=self.min_score, imgsz=640)
        print(f"Inference results: {results}")

        # Check if results are empty
        if not results or len(results) == 0:
            print("No detections made by the model.")
            return None

        # Preprocess results
        try:
            scores, classes_ids, boxes, masks = preprocess_results(results, (height, width))
          
        except Exception as e:
            print(f"Error during preprocessing: {e}")
            raise

        return scores, classes_ids, boxes, masks


def preprocess_results(results, orig_shape):
    assert len(results) == 1  # only single image supported
    result = results[0]
    print("Processing results...")
    print(f"Boxes tensor shape: {result.boxes.boxes.shape}")

    if result.masks is None:
        assert result.boxes.boxes.numel() == 0
        scores = np.empty((0,), dtype=float)
        print(f"Scores extracted: {scores}")
        classes_ids = np.empty((0,), dtype=int)
        boxes = np.empty((0, 4), dtype=int)
        print(f"Class IDs extracted: {classes_ids}")
        print(f"Boxes extracted: {boxes}")
        scaled_masks = np.empty((0, *orig_shape), dtype=np.uint8)
        return scores, classes_ids, boxes, scaled_masks

    boxes = result.boxes.boxes.cpu().numpy()
    masks = result.masks.masks.cpu().numpy()
    
    assert len(masks) == len(boxes)

    scores = boxes[:, 4].astype(float)
    classes_ids = boxes[:, 5].astype(int)
    boxes = boxes[:, :4].astype(int)
    masks = masks.astype(np.uint8)

    height, width = orig_shape
    mask_height, mask_width = masks.shape[1:]
    masks = masks.transpose(1, 2, 0)
    scaled_masks = scale_image((mask_height, mask_width), masks, (height, width))
    scaled_masks = scaled_masks.transpose(2, 0, 1)

    return scores, classes_ids, boxes, scaled_masks
    
    


def scale_image(im1_shape, masks, im0_shape, ratio_pad=None):
    """
    Takes a mask, and resizes it to the original image size

    Args:
      im1_shape (tuple): model input shape, [h, w]
      masks (torch.Tensor): [h, w, num]
      im0_shape (tuple): the original image shape
      ratio_pad (tuple): the ratio of the padding to the original image.

    Returns:
      masks (torch.Tensor): The masks that are being returned.
    """
    # Rescale coordinates (xyxy) from im1_shape to im0_shape
    if ratio_pad is None:  # calculate from im0_shape
        gain = min(im1_shape[0] / im0_shape[0], im1_shape[1] / im0_shape[1])  # gain  = old / new
        pad = (im1_shape[1] - im0_shape[1] * gain) / 2, (im1_shape[0] - im0_shape[0] * gain) / 2  # wh padding
    else:
        pad = ratio_pad[1]
    top, left = int(pad[1]), int(pad[0])  # y, x
    bottom, right = int(im1_shape[0] - pad[1]), int(im1_shape[1] - pad[0])

    if len(masks.shape) < 2:
        raise ValueError(f'"len of masks shape" should be 2 or 3, but got {len(masks.shape)}')
    masks = masks[top:bottom, left:right]
    # masks = masks.permute(2, 0, 1).contiguous()
    # masks = F.interpolate(masks[None], im0_shape[:2], mode='bilinear', align_corners=False)[0]
    # masks = masks.permute(1, 2, 0).contiguous()
    masks = cv2.resize(masks, (im0_shape[1], im0_shape[0]))

    if len(masks.shape) == 2:
        masks = masks[:, :, None]
    return masks
    
    
    



