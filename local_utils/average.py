class timed_obj:
    def __init__(self, key):
        self.key = key
        self.total_time = 0
        self.wrapped_num = 0 # this is incremented one per update
        self.individual_num = 0 # this is incremented by some x >= 1 per update

    def update(self, time, num):
        self.total_time += time
        self.wrapped_num += 1
        self.individual_num += num

    def print_profile(self, total_object):
        Output = """
Key: {}
Total time: {}
Total # aggregate calls: {}
Total # individual calls: {}
Seconds per aggregate call: {}
Seconds per individual call: {}
Inidividual calls per second: {}
Portion of total time: {}
""".format(
            self.key,
            self.total_time,
            self.wrapped_num,
            self.individual_num,
            self.total_time / self.wrapped_num,
            self.total_time / self.individual_num,
            self.individual_num / self.total_time,
            self.total_time / total_object.total_time
        )
        print(Output)


def check_valid_sim(tokens):
    return len(tokens) > 2

def average_logs(file_name,
                 token_idx_num=None,
                 token_idx_time=-1,
                 is_valid_fn=None,
                 split_on=" ",
                 token_idx_key=0,
                 keys = {"Simulating"},
                 total_key = "Simulating"):
    """
    Give a filename, the
    """
    timed_objects = {}
    for key in keys:
        timed_objects[key] = timed_obj(key)
    if total_key not in keys:
        raise ValueError("total_key must be one of the keys to parse.")

    with open(file_name, "r") as f:
        for line in f.readlines():
            tokens = line.split(split_on)
            if is_valid_fn is not None and not is_valid_fn(tokens):
                print("Invalid line: {}".format(tokens))
                continue
            key = tokens[token_idx_key]
            if key not in keys:
                print("No valid key: {}".format(tokens))
                continue
            time = float(tokens[token_idx_time])
            num = 1
            if token_idx_num is not None:
                num = float(tokens[token_idx_num])
            timed_objects[key].update(time, num)

    for key in keys:
        timed_objects[key].print_profile(timed_objects[total_key])

f = "simulated_times.log"
keys = {"SimForward", "Deepcopies", "GetAction", "SceneUpdate", "Simulating"}
average_logs(f, 1, -1, check_valid_sim, keys=keys)
