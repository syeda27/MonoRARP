# TODO Use google unit tests
# Currently tests are run by running the python script

import time

import defaults
import argument_utils

def test_default_args():
    start = time.time()
    args = argument_utils.parse_args()
    assert args.focal == defaults.FOCAL
    print("Test completed successfully in {:.2} seconds".format(time.time() - start))

if __name__ == "__main__":
    test_default_args()
