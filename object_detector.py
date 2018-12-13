
class ObjectDetector:
    """
    A class to help facilitate using different object detectors in the future.
    It is an abstract class and must be extended. See object_detectors/
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

        Additionally, must maintain a "done" boolean and a threading Event
          called output_1 (thats a one, not the letter l). This is used to signal
          when a detection is completed, which is needed when forcing a certain fps
          (aka reading a video).
        """
        self.done = False
        self.image_in_q = queue.Queue(queue_in_size)
        self.detections_out_q = queue.Queue(queue_out_size)
        self.output_1 = threading.Event()
        raise NotImplementedError("Please implement me!")
