import argparse
import os
import shutil
import subprocess
from threading import Lock, Thread
import yaml

# Global variables to synchronize the planner_runner threads
lock = Lock()
n = 0

def planner_runner(args, outfile_base, batchsize):
    """ Run planner processes until we hit the batch size, creating a
        uniqely named output file for each """
    global n
    lock.acquire()
    next_n = n
    n += 1
    lock.release()

    while (next_n < batchsize):
        outfile = outfile_base + str(next_n) + ".out"
        if (not os.path.exists(outfile)):
            args = args + ["--output", outfile]
            subprocess.run(args)

        lock.acquire()
        next_n = n
        n += 1
        lock.release()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Run a batch of planning operations and save the results.")
    parser.add_argument('savedir', help="Directory to save plans into")
    parser.add_argument('--problem', metavar='PROB',
        help="Problem description file")
    parser.add_argument('--batchsize', '-n', metavar='N', type=int,
        required=True, help="Number of plans to generate")
    parser.add_argument('--timelimit', '-t', metavar='T', type=float,
        help="Planner time limit")
    parser.add_argument('--threads', metavar='N', type=int, default=1,
        help="Number of planner processes to run simultaneously")
    args = parser.parse_args()


    # Verify or create our output directory
    if (os.path.isdir(args.savedir)):
        print("\"", args.savedir, "\" is already a directory, continue? (y/N)")
        resp = input().lower()
        if (not (resp == 'y' or resp == 'yes')):
            exit()
    else:
        os.mkdir(args.savedir)
        os.mkdir(os.path.join(args.savedir, "plans"))

    # Find or create the config file
    config_file = os.path.join(args.savedir, "prob.yaml")
    timelimit = args.timelimit
    if (os.path.exists(config_file)):
        # use existing config file
        if (args.problem):
            print("WARNING: --problem was provided but problem config already"
                " exists at \"",
                config_file,
                "\", ignoring argument")

        # Open config
        with open(config_file, 'r') as cfh:
            config_data = yaml.load(cfh)

        # Copy timelimit
        timelimit = config_data["time_limit"]
    else:
        # create config file based on args.problem
        if (not args.problem):
            print("ERROR: Please provide a problem description file")
            exit()

        # Open args.problem
        with open(args.problem, 'r') as pfh:
            config_data = yaml.load(pfh)

        # Add timelimit
        if (timelimit):
            config_data["time_limit"] = timelimit
        else:
            print("ERROR: Must provide a timelimit when createing a config")
            exit()

        # Copy mujoco config
        new_mj_conf = os.path.join(
            os.path.dirname(config_file),
            config_data["mujoco_config"])
        if (not os.path.exists(new_mj_conf)):
            shutil.copyfile(
                os.path.join(os.path.dirname(args.problem),
                             config_data["mujoco_config"]),
                new_mj_conf)
        else:
            print("ERROR: MJ file already exists !")
            exit()

        # Save
        with open(config_file, 'w') as cfh:
            yaml.dump(config_data, cfh)

    outfile_base = os.path.join(args.savedir, "plans/plan_")
    planner_bin = os.path.join(os.getcwd(), "plan")
    planner_args = [
        planner_bin,
        config_file,
        "--randomstart",
        "--timelimit", str(timelimit)]

    # Create all the threads and wait for them to finish
    threads = []
    for _ in range(args.threads):
        threads.append(Thread(
            target=planner_runner,
            args=(planner_args, outfile_base, args.batchsize)))
        threads[-1].start()

    for thread in threads:
        thread.join()
