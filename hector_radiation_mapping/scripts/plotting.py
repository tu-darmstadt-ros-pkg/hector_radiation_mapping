import matplotlib.pyplot as plt
import glob
import os
import rospkg


def get_data_from_file(name):
    data = []
    with open(name + '.txt', 'r') as file:
        for line in file:
            data.append(float(line.strip()))
    return data


def get_data_from_file_with_prefix(path, file_prefix):
    pattern = os.path.join(path, file_prefix + "*")
    data = []

    # Iterate over all files that match the pattern
    for file_path in glob.glob(pattern):
        if os.path.isfile(file_path):
            # Process the file here
            print(file_path)
            fileData = []
            with open(file_path, 'r') as file:
                for line in file:
                    fileData.append(float(line.strip()))

            data.append(fileData)
    return data


def create_average(datas):
    res = []
    for i in range(len(datas[0])):
        mean = 0.0
        for data in datas:
            mean += data[i]
        mean /= len(datas)
        res.append(mean)
    return res


export_folder = rospkg.RosPack().get_path('hector_radiation_mapping') + '/radiation_mapping_exports/'
file_folder = export_folder + "runtime (copy)/"

# Plot cps and dose rate over time
evaluationSizes = get_data_from_file(file_folder + 'evaluationSizes')
evaluationTimes = get_data_from_file(file_folder + 'evaluationTimes')

file_folder = export_folder + "runtime/"
evaluationSizesNo = get_data_from_file(file_folder + 'evaluationSizes')
evaluationTimesNo = get_data_from_file(file_folder + 'evaluationTimes')

def get_running_average(data):
    queue = []
    running_average = []
    for i in range(len(data)):
        queue.append(data[i])
        if len(queue) > 10:
            queue.pop(0)
        running_average.append(sum(queue) / len(queue))
    return running_average


evaluationTimes = get_running_average(evaluationTimes)
evaluationTimesNo = get_running_average(evaluationTimesNo)

# plot dots for each sample
plt.plot(evaluationSizes, evaluationTimes, label='Optimization On')
plt.plot(evaluationSizesNo, evaluationTimesNo, label='Optimization Off')
plt.xlabel('Model Size')
plt.ylabel('Time (ms)')
plt.title('Time for Model Evaluation - Optimization On/Off')
plt.legend()
plt.grid()
plt.show()


export_folder = rospkg.RosPack().get_path('hector_radiation_mapping') + '/radiation_mapping_exports/'
file_folder = export_folder + "samples/"

# Plot cps and dose rate over time
cps = get_data_from_file(file_folder + 'cps')
doseRate = get_data_from_file(file_folder + 'doseRate')
time = get_data_from_file(file_folder + 'time')

cps_processed = []
doseRate_processed = []
time_processed = []
for i in range(len(time)):
    # if cps[i] > 10.0:
    cps_processed.append(cps[i])
    doseRate_processed.append(doseRate[i])
    time_processed.append(time[i])

plt.plot(time_processed, doseRate_processed, 'o', label='cps')

plt.xlabel('Time in s')
plt.ylabel('cps / dose rate')
plt.title('CPS and dose rate over time')
plt.legend()
plt.show()
