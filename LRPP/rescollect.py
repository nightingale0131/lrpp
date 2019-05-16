# extract results from .dat files and dump it all in a csv file
# Creation date: 2019-01-30

import os
import csv
import re

numbers = re.compile(r'(\d+)')
def numericalSort(value):
    # this function and above line courtesy of:
    # https://stackoverflow.com/questions/21289201/read-files-sequentially-in-order
    parts = numbers.split(value)
    parts[1::2] = map(int, parts[1::2])
    return parts

def average(alist):
    # take average of a list, assuming they are numbers
    n = len(alist)
    sm = float(sum(alist))
    return sm/n

if __name__ == '__main__':
    # get directory
    # resdir = raw_input('Which directory are the results in?\n')
    resdir = "../results/LRPP1_0/maze_Ah"
    # resdir = "../results/2019_09_results/maze_Bh"
    os.chdir(resdir)
    print(os.getcwd())

    start = (20,20) # where robot starts task, used for finding #supermaps

    with open('summary.csv', 'wb') as csvfile:
        writer = csv.writer(csvfile, delimiter=' ')
        writer.writerow(["T", "% saved pol", "% saved opt", "# supermaps", "Total runtime"])

        # go through files in the directory, print each file being analyzed
        for filename in sorted(os.listdir(os.getcwd()), key=numericalSort):
            if filename == 'summary.csv': continue
            # get number of tasks
            parts = numbers.split(filename)
            print(parts)
            T = int(parts[1])

            with open(filename, 'rb') as datafile:
                print("Reading {}...".format(filename))

                islasttask = False
                isfinal = False
                savedP = []
                savedOpt = []

                # extract all needed data
                for line in datafile:
                    # get number of supermaps
                    if "Start of task {}".format(T) in line: islasttask = True
                    if islasttask ==  True and str(start) in line:
                        eb = line.find('[')
                        lb = line.find(']')
                        supermaps = line[eb + 1:lb].split(', ') # extract list of supermaps
                        nmaps = len(supermaps)

                    # get costP average
                    if "average of costP" in line:
                        isfinal = False
                        eq = line.find('=')
                        costP = float(line[eq + 2:])

                    # get costR average
                    if "average of costR" in line:
                        eq = line.find('=')
                        costR = float(line[eq + 2:])

                    # get costOpt average
                    if "average of costOpt" in line:
                        eq = line.find('=')
                        costOpt = float(line[eq + 2:])

                    # get runtime
                    if "Finished tasks" in line:
                    # if "End Program" in line:
                        raw_time = line[0:14]

                    # get savedP and savedOpt
                    if line.startswith("   1"): isfinal = True
                    if isfinal == True:
                        result = line.split()
                        P = float(result[1])
                        R = float(result[2])
                        O = float(result[3])
                        savedP.append((R - P)/R*100)
                        savedOpt.append((R - O)/R*100)

            # analyze results [#, costP, costR, costOpt]
            ave_savedP = average(savedP)
            ave_savedOpt = average(savedOpt)

            data= [T, costP, costR, costOpt, nmaps, raw_time]
            # data = [T, ave_savedP, ave_savedOpt, nmaps, raw_time]
            writer.writerow(data)


