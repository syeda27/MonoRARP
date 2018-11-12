"""
This file contains some helper methods for dealing with trackers.
"""

def raise_undefined_tracker_type(self, tracker_type):
    msg = """
Invalid tracker type: {}
Supported tracker types:
  KCF
  Particles
""".format(tracker_type)
    raise ValueError(msg)
