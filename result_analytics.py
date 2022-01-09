import sys
from statistics import mean

"""
    How to use
        run python3 result_analytics.py <input_file> 
"""

if __name__=="__main__":
    if len(sys.argv) < 2:
        exit()
    with open(sys.argv[1]) as f:
        lines = f.read().split('\n')[:-1]
        for i,_ in enumerate(lines):
            lines[i] = lines[i].split(':')[-1].replace(' ','').split('|')
        lpTimesAgent1 = [int(lines[i][0]) for i in range(0, len(lines), 2)]
        solveTimesAgent1 = [int(lines[i][1]) for i in range(0, len(lines), 2)]
        lpTimesAgent2 = [int(lines[i][0]) for i in range(1, len(lines), 2)]
        solveTimesAgent2 = [int(lines[i][1]) for i in range(1, len(lines), 2)]

        metricsAgent1 = [mean(lpTimesAgent1), mean(solveTimesAgent1)]
        metricsAgent2 = [mean(lpTimesAgent2), mean(solveTimesAgent2)]

        print(f"Agent 1 - [lp, solve] : {metricsAgent1}")
        print(f"Agent 2 - [lp, solve] : {metricsAgent2}")
