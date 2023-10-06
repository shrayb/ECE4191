from multiprocessing import Process, Manager, Value
import time

class Motor:
    def __init__(self):
        self.ticks = Value('i', 0)

    def update(self):
        with self.ticks.get_lock():
            self.ticks.value += 1

def update_encoder(left_motor, right_motor):
    while True:
        left_motor.update()
        right_motor.update()

if __name__ == "__main__":
    with Manager() as manager:
        left_motor = Motor()
        right_motor = Motor()

        writer_process = Process(target=update_encoder, args=(left_motor, right_motor))
        writer_process.start()

        while True:
            print("Waiting for process to initialise...")
            time.sleep(1)
            # print(f"Value from writer process (Left Motor): {left_motor.ticks.value}")
            # print(f"Value from writer process (Right Motor): {right_motor.ticks.value}")
            tick_sum = 0
            old_ticks = (left_motor.ticks.value + right_motor.ticks.value) / 2
            counts = 15
            time_diff = 0.5
            for index in range(counts):
                time.sleep(time_diff)
                new_ticks = (left_motor.ticks.value + right_motor.ticks.value) / 2
                tick_sum += new_ticks - old_ticks
                old_ticks = new_ticks
                print(index)

            tick_diff_average = tick_sum / counts / time_diff
            print("Tick diff:", tick_diff_average)
            print("Safe from aliasing:", tick_diff_average / 2 - 0.1 * tick_diff_average)
            break
