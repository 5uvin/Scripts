import pandas as pd
import matplotlib.pyplot as plt
import os


def plot(data, out_path, title):
    plt.figure(figsize=(20, 5), dpi=900)

    # scale and shift status values to plot them out of valid covariance ranges
    plt.plot(
        data["field.header.seq"],
        0.05 * data["field.status.status"] - 0.2,
        label="GNSS status (high is 'GBAS_FIX', low is 'STATUS_FIX')",
    )
    plt.plot(
        data["field.header.seq"],
        data["field.position_covariance0"],
        label="XX covariance",
    )
    plt.plot(
        data["field.header.seq"],
        data["field.position_covariance4"],
        label="YY covariance",
    )
    plt.plot(
        data["field.header.seq"],
        data["field.position_covariance8"],
        label="ZZ covariance",
    )
    plt.xlabel("Seq #")
    plt.title(title)
    plt.legend()
    plt.savefig(out_path)
    plt.close()


def main():
    # get all filenames of csv files, no extension
    names = [
        f[:-4] for f in os.listdir(".") if os.path.isfile(f) and f.endswith(".csv")
    ]
    names.sort()

    for name in names:
        csv_in = name + ".csv"
        png_out = name + ".png"
        title = "GNSS covariance plots for '" + name + "'"

        if not os.path.isfile(csv_in):
            print("Error, cannot find: " + csv_in)
            exit(1)

        if os.path.getsize(csv_in) > 0:
            data = pd.read_csv(csv_in)
        else:
            print("Error, cannot read: " + csv_in)
            exit(1)

        plot(data, png_out, title)


if __name__ == "__main__":
    main()
