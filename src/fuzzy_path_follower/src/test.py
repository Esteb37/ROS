import numpy as np
from wang_mendel import wang_mendel
import matplotlib.pyplot as plt

data = np.load(
    "C:/Users/Esteban/Documents/Escuela/Robotica/ROS/src/fuzzy_path_follower/src/datos.npy")[-3000:]
max_speed = 0.5

rules = np.load(
    "C:/Users/Esteban/Documents/Escuela/Robotica/ROS/src/fuzzy_path_follower/src/linear_rules.npy")

controller = wang_mendel([7, 5, 19],
                         (0, 4),
                         (-2.3, 3.7),
                         (0, 0.75),
                         rules)

results = np.empty(len(data), dtype=float)

for i, row in enumerate(data):

    distance = row[0]
    angle = row[1]

    results[i] = controller.get_output(distance, angle)

plt.figure(figsize=(10, 5))
plt.rcParams.update({'font.size': 15})

plt.gca().yaxis.grid(True)
plt.plot(data[0:, 2], label='MG')
plt.plot(results, 'r-.', label='Pred')
plt.legend(bbox_to_anchor=(1.05, 1.0), loc='upper left')
plt.tight_layout()
plt.show()
