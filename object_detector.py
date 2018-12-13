
class ObjectDetector:
    """
    A class to help facilitate using different object detectors in the future.
    """

    def __init__(self, args):
        """
        Arguments
          args: a parser object from the argparse library.
        """
        raise NotImplementedError("Please implement me!")

    def launch(self, queue_in_size=2, queue_out_size=2, wait_time=0.05, max_wait=0.5):
        """
        Must communicate through the below queues:

        self.image_in_q = queue.Queue(queue_in_size)
            - queue of (image_np, frame_time) tuples
        self.detections_out_q = queue.Queue(queue_out_size)
            - queue of (image_np, net_out, frame_time) tuples
            - net_out is a dictionary of boxes, classes, and scores, for now.
                TODO: make net_out a class of some sort.
        """
        raise NotImplementedError("Please implement me!")
