import os
from collections import defaultdict

import numpy as np
import StringIO
from tabulate import tabulate

# Define garment categories
categories = {'hoodie': 0, 'jacket': 1, 'pants': 2, 'polo': 3, 'robe': 4, 'skirt': 5}


def generate_table(garment_data, garment_label=None, percentage=False, **kwargs):
    """
    Generates a nice table displaying results
    :param garment_data:
    :param percentage: False to show count, True to show count/sum(count)
    """
    if not garment_label:
        garment_label = [item[0] for item in categories.items() if item[1] == garment_data[0, 0]][0]
    txt = "Garment: {}\n".format(garment_label)
    table = []
    for row_index in range(1, 4):
        values, counts = np.unique(garment_data[:, row_index], return_counts=True)
        if percentage:
            counts = np.true_divide(counts*100, np.sum(counts))
        pairs = defaultdict(lambda: 0, dict(zip(values, counts)))
        table.append([row_index, pairs[0.0], pairs[0.5], pairs[1.0], pairs[-1.0]])
    txt += tabulate(table, headers=["stage", "0", "0.5", "1", "-1"], **kwargs)
    txt += '\n\n'
    return txt


def generate_table2(garment_data, garment_label=None, percentage=False, **kwargs):
    """
    Generates a nice table displaying results
    :param garment_data:
    :param percentage: False to show count, True to show count/sum(count)
    """
    if not garment_label:
        garment_label = [item[0] for item in categories.items() if item[1] == garment_data[0, 0]][0]
    txt = "Garment: {}\n".format(garment_label)
    table = []
    for row_index in range(1, 4):
        values, counts = np.unique(garment_data[:, row_index], return_counts=True)
        if percentage:
            percentage = 100
        else:
            percentage = 1
        pairs = defaultdict(lambda: 0, dict(zip(values, counts)))
        table.append([row_index, np.true_divide((pairs[0.5]+pairs[1])*percentage,pairs[0.5]+pairs[1]+pairs[0])])
    txt += tabulate(table, headers=["stage", "percent"], **kwargs)
    txt += '\n\n'
    return txt


if __name__ == '__main__':
    # Load file and modify categories to be loaded in numpy
    input_file = os.path.expanduser('~/Research/garments-birdsEye-flat-results/output.txt')
    with open(input_file, 'r') as f:
        raw_data_txt = f.readlines()
    data_txt = []
    for line in raw_data_txt:
        line_split = line.rstrip().split(' ')
        line_split[0] = str(categories[line_split[0].rstrip('1234567890')])
        data_txt.append(line_split)
    data_txt = ''.join([' '.join(line)+'\n' for line in data_txt])

    # Load data array with numpy
    data = np.genfromtxt(StringIO.StringIO(data_txt))

    # Calculate indices of each category block
    indices = [0] + (np.where(np.diff(data[:, 0]))[0]+1).tolist() + [data.shape[0]]
    blocks = []
    for i, j in zip(indices, indices[1:]):
        blocks.append(data[i:j, :])

    # Find stats for each block
    for block in blocks:
        print(generate_table(block, percentage=False, floatfmt=".2f"))

    print(generate_table(data, garment_label="All", percentage=False, floatfmt=".2f"))

    with open('aux.txt', 'w') as f:
        for block in blocks:
            f.write(generate_table(block, percentage=True, tablefmt='latex'))

        f.write(generate_table(data, garment_label="All", percentage=True, tablefmt='latex'))
