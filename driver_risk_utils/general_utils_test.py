from driver_risk_utils import general_utils


"""
Tests to make sure timers are passed by reference.
"""

timer = general_utils.Timing()

def test_fn(timer):
    timer.update_start("Test")
    for i in range(1000):
        a = i * i
    timer.update_end("Test")

timer.print_stats()
print("Before function call")
test_fn(timer)
timer.print_stats()
