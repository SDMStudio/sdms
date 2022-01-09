import os

all_params = [
    [10,50,10,10],
    [20,40,10,10],
    [30,30,10,10],
    [10,10,10,50],
    [10,10,20,40],
    [10,10,30,30]
]

for params in all_params:
    folderName = f"{params[0]}_{params[1]}_{params[2]}_{params[3]}"
    print(f"Dealing with {folderName}")
    # os.system(f"sudo rm -rf byg_data/{folderName}")
    # os.system(f"mkdir byg_data/{folderName}")
    # os.system(f"python3 bayesian_input_gen.py {params[0]} {params[1]} {params[2]} {params[3]} 50 byg_data/{folderName} > /dev/null 2>&1")
    os.system(f"python3 bayesian_folder_launcher.py byg_data/{folderName} > /dev/null 2>&1")
