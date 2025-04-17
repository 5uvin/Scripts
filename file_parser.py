import csv
import matplotlib.pyplot as plt

def parse_file(file_path):
    with open(file_path, 'r') as file:
        reader = csv.reader(file)
        rows = list(reader)
        ros_stats = []

        for idx, row in enumerate(rows):
            if row[1] == "kdvisual_ros2_node":
                ros_stats.append(row)
        timestamps = [float(row[0]) for row in ros_stats ]
        cpu_percent = [float(row[3]) for row in ros_stats ]
        ram_use = [float(row[4]) for row in ros_stats ]
        topic_hz = [float(row[5]) for row in ros_stats ]
        start_time = float(timestamps[0])
        timestamps = [float(ts) - start_time for ts in timestamps]

        fig, axs = plt.subplots(1, 3, figsize=(15, 5), sharex=True)
        

        axs[0].plot(timestamps, cpu_percent, label='CPU %', color='blue')
        axs[0].set_xlabel('Time')
        axs[0].set_ylabel('CPU %')
        axs[0].set_title('CPU Usage Over Time')
        axs[0].grid(True)
        axs[0].legend()


        axs[1].plot(timestamps, ram_use, label='RAM Usage (MB)', color='green')
        axs[1].set_xlabel('Time')
        axs[1].set_ylabel('RAM Usage (MB)')
        axs[1].set_title('RAM Usage Over Time')
        axs[1].grid(True)
        axs[1].legend()

        axs[2].plot(timestamps, topic_hz, label='Pose Topic Output (Hz)', color='orange')
        axs[2].set_xlabel('Time')
        axs[2].set_ylabel('Pose Topic Output (Hz)')
        axs[2].set_title('Pose Topic Output (Hz) over Time')
        axs[2].grid(True)
        axs[2].legend()

        plt.tight_layout()
        plt.show()

if __name__ == "__main__":
    parse_file("/home/user/Downloads/20250417_133848_resource_log.csv")