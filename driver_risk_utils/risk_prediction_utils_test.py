# TODO Use google unit tests
# Currently tests are run by running the python script

import time

import defaults
import risk_prediction_utils

# TODO test each of the function

def test_default_args():
    start = time.time()

    print("Test completed successfully in {:.2} seconds".format(time.time() - start))

if __name__ == "__main__":
    test_default_args()
