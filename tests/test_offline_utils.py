
import sys
import os
sys.path.append("..")
sys.path.append("../driver_risk_utils/")

import offline_utils
import unittest

class TestSaveMethods(unittest.TestCase):
    def setUp(self):
        # test save_output
        data0 = {"Test": 100}
        data1 = {"Test": (1,2,3)}
        data2 = {"Test": [1,2,3,4]}
        data3 = {"1": {"Test": [1,2,3]}}
        self.data = [data0, data1, data2, data3]

    def test_all_options(self):
        # Test that save_output works as expected."
        for i,d in enumerate(self.data):
            offline_utils.save_output(d, "test", i, RESULTS, verbose=True)

        # test that save_output does not overwrite on default.
        for i,d in enumerate(self.data):
            with self.assertRaises(ValueError):
                offline_utils.save_output(d, "test", i, RESULTS, verbose=True)

        # test that save_output does overwrite on command.
        for i,d in enumerate(self.data):
            offline_utils.save_output(d, "test", i, RESULTS,
                overwrite=True, verbose=True)

        # test the load options
        data0 = offline_utils.load_input("test", 0, RESULTS, verbose=True)
        self.assertEqual(data0, {"Test": 100})
        data1 = offline_utils.load_input("test", 1, RESULTS, verbose=True)
        self.assertEqual(data1, {"Test": (1,2,3)})
        data2 = offline_utils.load_input("test", 2, RESULTS, verbose=True)
        self.assertEqual(data2, {"Test": [1,2,3,4]})
        data3 = offline_utils.load_input("test", 3, RESULTS, verbose=True)
        self.assertEqual(data3, {"1": {"Test": [1,2,3]}})

RESULTS = "/tmp/test_results"

def check_user_remove(path):
    print("All items to remove:")
    files = os.listdir(path)
    for f in files:
        print(f)
    do_delete = input("Delete all files and directories listed above? [Y/n]: ")
    if do_delete.lower() == "y":
        print("Deleting.")
        import shutil
        shutil.rmtree(path)
        print("Deleted.")
    else:
        print("Not deleting.")

if __name__ == '__main__':
    if os.path.exists(RESULTS):
        check_user_remove(RESULTS)
    offline_utils.check_make_directory(RESULTS)
    unittest.main()
