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

    def print_profile(self, total_object=None):
        Output = """
Key: {}
Total time: {}
Total # aggregate calls: {}
Total # individual calls: {}
Seconds per aggregate call: {}
Seconds per individual call: {}
Inidividual calls per second: {}
""".format(
            self.key,
            self.total_time,
            self.wrapped_num,
            self.individual_num,
            self.total_time / self.wrapped_num,
            self.total_time / self.individual_num,
            self.individual_num / self.total_time
        )
        print(Output)
        if total_object is not None:
            print("Portion of total time: {}".format(
                self.total_time / total_object.total_time))

def check_valid_sim(tokens):
    return len(tokens) > 2

def average_logs(file_name,
                 token_idx_num=None,
                 token_idx_time=-1,
                 is_valid_fn=None,
                 split_on=" ",
                 token_idx_key=0,
                 keys={"Simulating"},
                 total_key=None,
                 all_total=False):
    """
    Give a filename, the... TODO
    """
    timed_objects = {}
    for key in keys:
        timed_objects[key] = timed_obj(key)
    if total_key is not None and total_key not in keys:
        raise ValueError("total_key must be one of the keys to parse.")
    all_total = False
    if total_key is None:
        print("Aggregating all times as total")
        total_key = "Total"
        all_total = True
        timed_objects[total_key] = timed_obj(total_key)
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
            if ":" == tokens[token_idx_time][0]:
                tokens[token_idx_time] = tokens[token_idx_time][1:]
            time = float(tokens[token_idx_time])
            num = 1
            if token_idx_num is not None:
                num = float(tokens[token_idx_num])
            timed_objects[key].update(time, num)
            if all_total:
                timed_objects[total_key].update(time, num)

    for key in keys:
        timed_objects[key].print_profile(timed_objects[total_key])

f = "simulated_times.log"
keys = {"SimForward", "Deepcopies", "GetAction", "SceneUpdate", "Simulating"}
average_logs(f, 1, -1, check_valid_sim, keys=keys, total_key="Simulating")

print("\n\n\n")
f = "risk_sim_times.log"
keys = {"SceneInit", "RiskSim", "CalculateRisk"}
average_logs(f, None, -1, check_valid_sim, keys=keys)

print("\n\n\n")
f = "get_action_times.log"
keys = {"GetFore", "GetLatAceel", "PropLongA"}
average_logs(f, None, -1, check_valid_sim, keys=keys)

print("\n\n\n")
f = "queue_processing.log"
keys = {"NeuralNet", "DetectObjects", "GetRisk", "Display"}
average_logs(f, 1, -1, check_valid_sim, keys=keys)
