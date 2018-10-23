"""
This is similar to vehicle.py, but is a more lightweight class.
The only things stored are the information needed to interact with the state
class.
To that effect, the vehicle_state does not even include a key. If you lose the
key to the object, you're out of luck.
It is basically a wrapper to a dictionary, but supports future extensions.
"""

class vehicle_state:
    def __init__(self):
        self.quantities = {}
        self.update_time = None
