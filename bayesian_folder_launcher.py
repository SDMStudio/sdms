import os
import sys

"""
    How to use
        run python3 bayesian_folder_launcher.py <input_folder> 
"""

#from progressbar import progressbar

# def showProgressBar(count, total):
#     bar_len = 60
#     filled_len = int(round(bar_len * count / float(total)))

#     percents = round(100.0 * count / float(total), 1)
#     bar = '=' * filled_len + '-' * (bar_len - filled_len)

#     sys.stdout.write('[%s] %s%s\r' % (bar, percents, '%'))
#     sys.stdout.flush()

if __name__=="__main__":
    folder = sys.argv[1]
    filenames = [f"{folder}/{f}" for f in os.listdir(folder) if (f[-4:] == ".byg" or f[-4:] == ".BYG")]
    #for i in progressbar(range(len(filenames)), redirect_stdout=True):

    for i, oneInput in enumerate(filenames):
        #showProgressBar(i, len(filenames))
        for agent in range(2):
            os.system(f"./build/src/examples/ex-bayesian {filenames[i]} {agent} {folder}/results.txt")
