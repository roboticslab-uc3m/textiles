import os
import StringIO
from collections import defaultdict
import numpy as np

# Define garment categories
categories = {'hoodie':0, 'jacket':1, 'pants':2, 'polo':3, 'robe':4, 'skirt':5}


def print_table(garment_data):
    print "Garment:", [item[0] for item in categories.items() if item[1] == garment_data[0, 0]][0]
    print "----------------------------------------------"
    print " stage | 0 | 0.5 | 1 | -1 |"
    for row_index in range(1, 4):
        values, counts = np.unique(garment_data[:, row_index], return_counts=True)
        pairs = defaultdict(lambda: 0, dict(zip(values, counts)))
        print "   {}   | {} | {} | {} | {} |".format(row_index, pairs[0.0], pairs[0.5], pairs[1.0], pairs[-1.0])
    print '\n'

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
    indices = [0] + (np.where(np.diff(data[:,0]))[0]+1).tolist() + [data.shape[0]]
    blocks = []
    for i, j in zip(indices, indices[1:]):
        blocks.append(data[i:j, :])

    # Find stats for each block
    for block in blocks:
        print_table(block)

