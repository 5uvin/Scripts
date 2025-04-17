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

        fig, axs = plt.subplots(2, 2, figsize=(8, 6), sharex=True)
        

        axs[0,0].plot(timestamps, cpu_percent, label='CPU %', color='blue')
        axs[0,0].set_ylabel('CPU %')
        axs[0,0].set_title('CPU Usage Over Time')
        axs[0,0].grid(True)
        axs[0,0].legend()


        axs[0,1].plot(timestamps, ram_use, label='RAM Usage (MB)', color='green')
        axs[0,1].set_xlabel('Time')
        axs[0,1].set_ylabel('RAM Usage (MB)')
        axs[0,1].set_title('RAM Usage Over Time')
        axs[0,1].grid(True)
        axs[0,1].legend()

        axs[1,0].plot(timestamps, topic_hz, label='Pose Topic Output (Hz)', color='orange')
        axs[1,0].set_xlabel('Time')
        axs[1,0].set_ylabel('Pose Topic Output (Hz)')
        axs[1,0].set_title('Pose Topic Output (Hz) over Time')
        axs[1,0].grid(True)
        axs[1,0].legend()

        plt.tight_layout()
        plt.show()

if __name__ == "__main__":
    parse_file("/home/user/Downloads/20250417_133848_resource_log.csv")