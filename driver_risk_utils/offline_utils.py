"""
This file contains some utiltiy methods for functionality specific to
the offline version of the system.

For example, general save and load functions.
"""

import numpy as np
import os
import pickle

def check_make_directory(directory_path):
    """ Can be relative or absolute path """
    if not os.path.exists(directory_path):
        os.makedirs(directory_path)

def save_output(
        data,
        component_name,
        frame_id,
        results_path="results",
        extension=".pkl",
        overwrite=False,
        verbose=False):
    """
    Save the given data for future use.
    Arguments:
      data - the data. Must be pickleable.
      component_name - the name of the folder within results to save to.
      frame_id - a unique identifier for this file.
      folder_path - where do we want to save the data to? Can be relative.
      extension - what extension do we want to give the file.
      overwrite - boolean, should we overwrite existing data of this name or
        raise a ValueError if existing data is encountered?
      verbose - boolean, True will print information.
    """
    file_name = str(frame_id) + extension
    folder_path = os.path.join(results_path, component_name)
    check_make_directory(folder_path)
    file_path = os.path.join(folder_path, file_name)
    if not overwrite:
        if os.path.exists(file_path):
            raise ValueError("ERROR: Path <{}> already exists, not overwriting.".format(file_path))
    if verbose:
        if os.path.exists(file_path):
            print("WARNING: Overwriting data at: {}".format(file_path))
        else:
            print("INFO: Saving data to: {}".format(file_path))
    with open(file_path, "wb") as f:
        pickle.dump(data, f)
