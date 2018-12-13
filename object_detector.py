
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
        """
        raise NotImplementedError("Please implement me!")
