from random import random, randint
import sys

"""
    How to use 
        run python3 bayesian_input_gen.py <nb_action_agent1> <nb_action_agent2> <nb_types_agent1> <nb_types_agent2> <nb_instances> <output_folder>
    
    Note : the output folder must already exist 
"""

def generateOneBYG(a1, a2, t1, t2, filename):
    content = f"{t1} {t2}\n{a1} {a2}\n"

    # generate and add payoffs
    payoffs = [[[random()*30 - 15 for _ in range(a2)] for _ in range(a1)] for _ in range(t1*t2)]
    playerFactor = -1
    for i in range(t1*t2):
        for _ in range(2): #for each player
            playerFactor *= -1
            for j in range(a1):
                for k in range(a2):
                    content += f"{payoffs[i][j][k]*playerFactor} "
                content += "\n"

    # generate and add joint type probas
    probas = [[random() for _ in range(t2)] for _ in range(t1)]
    for i in range(t1):
        total = sum(probas[i])
        tmp = [x / total for x in probas[i]]
        for x in tmp:
            content += f"{x} "
        content +=  "\n"

    with open(filename, "w") as f:
        f.write(content)

def generateData(a1, a2, t1, t2, nbGames, filenamePrefix, rand=False):
    for i in range(nbGames):
        if not rand:
            generateOneBYG(a1, a2, t1, t2, f"{filenamePrefix}/byg_{i}.byg")
        else:
            generateOneBYG(randint(1,a1), randint(1, a2), randint(1,t1), randint(1,t2), f"{filenamePrefix}/byg_{i}.byg")


if __name__=="__main__":
    if len(sys.argv) != 7:
        exit("need 4 args")
    a1, a2, t1, t2, nbGames = [int(x) for x in sys.argv[1:6]]
    prefix = sys.argv[6]
    generateData(a1,a2,t1,t2, nbGames, prefix)