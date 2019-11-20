import argparse
from multiprocessing import Pool
import os
import subprocess


def smooth_plan(plan_fn, smoother_bin, mjxml, dest_dir, overwrite):
    """ Smooth a plan using the smoother_bin binary """

    plan_bn = os.path.basename(plan_fn)
    # Drop the last file extension if it exists and add .json
    out_bn = '.'.join(
        plan_bn.split('.')[0:1]
        + plan_bn.split('.')[1:-1]
        + ["json"])
    out_fn = os.path.join(dest_dir, out_bn)

    args = [
        smoother_bin,
        "--mjxml", mjxml,
        "--output", out_fn, 
        "--json",
        plan_fn,
        ]

    if (not os.path.exists(out_fn)) or overwrite:
        print("Do smoothing to \'{}\' with \'{}\' and put the results in \'{}\'."
            .format(plan_fn, smoother_bin, out_fn))
        subprocess.run(args)


class Smoother:
    # If pool.map would just take a lambda function I wouldn't need to write
    # this class :(
    def __init__(self, smoother_bin, mjxml, dest_dir, overwrite):
        self.smoother_bin = smoother_bin
        self.mjxml = mjxml
        self.dest_dir = dest_dir
        self.overwrite = overwrite

    def __call__(self, plan_fn):
        return smooth_plan(plan_fn,
            smoother_bin = self.smoother_bin,
            mjxml = self.mjxml,
            dest_dir = self.dest_dir,
            overwrite = self.overwrite)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Generate smoothed plans for all plans in a directory")
    parser.add_argument('plan_dir', help="Directory with plans to smooth")
    parser.add_argument('--mjxml', '-m',  metavar='FILE', type=str, required=True,
        help="MuJoCo XML environment description file")
    parser.add_argument('--threads', metavar='N', type=int, default=1,
        help="Number of smoothing processes to start simultaneously")
    parser.add_argument('--overwrite', action='store_true',
        help="Overwrite existing files")
    args = parser.parse_args()

    # Check if the plan dir is valid
    if (not os.path.isdir(args.plan_dir)):
        print("\'{}\' is not a valid directory".format(args.plan_dir))
        exit()

    # Get a directory to save the smoothed plans
    parent = os.path.dirname(args.plan_dir)
    dest_dir = os.path.join(parent, "smoothed")
    if (not os.path.isdir(dest_dir)):
        os.mkdir(dest_dir)
    elif not args.overwrite:
        print("Destination directory \'{}\' already exists, continue? (y/N):"
            .format(dest_dir))
        resp = input().lower()
        if (not (resp == 'y' or resp == 'yes')):
            exit()

    # Locate the binary for smooth_plan
    smoother_bin = os.path.join(os.getcwd(), "smooth_plan")

    # List all files in the plan directory
    files = [os.path.join(args.plan_dir, x) for x in os.listdir(args.plan_dir)]

    # Feed them to smooth_plan in threads
    pool = Pool(args.threads)
    # TODO: this lambda function might work if we switch to pathos
    #   multiprocessing
    # pool.map(
    #     lambda x: smooth_plan(
    #         x,
    #         smoother_bin=smoother_bin,
    #         mjxml=args.mjxml,
    #         dest_dir=dest_dir,
    #         overwrite=args.overwrite),
    #     files)
    pool.map(
        Smoother(
            smoother_bin=smoother_bin,
            mjxml=args.mjxml,
            dest_dir=dest_dir,
            overwrite=args.overwrite),
               # smosmoother_bin,
               #   args.mjxml,
               #   dest_dir,
               #   args.overwrite),
        files)

