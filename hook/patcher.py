import json
import subprocess
import argparse


def load_patch_info(patch_filename):
    with open(patch_filename) as fd:
        return json.load(fd)


def main(binary, patch_filename, patcher_configname, args, output):
    # load patchers from file
    with open(patcher_configname) as f:
        patcher_config = json.load(f)

    patchers = patcher_config["patchers"]
    # replace placeholders {BINARY} and {ARGS} in all commands
    for patcher in patchers:
        cmds = []
        for cmd in patchers[patcher]:
            tmp_cmd = []
            for el in cmd:
                el = el.replace("{BINARY}", binary)
                el = el.replace("{ARGS}", args)
                el = el.replace("{OUTPUT}", output)
                tmp_cmd.append(el)
            cmds.append(tmp_cmd)

        patchers[patcher] = cmds

    # load the patch manifest
    patch_info = load_patch_info(patch_filename)

    # collect patch steps that can be executed in a combined step.
    collected_info = ""
    last_patcher = ""
    for patcher, info in patch_info:
        # combine the different oh asserts
        if patcher == "oh_assert_" or patcher == "oh_assert" or patcher == "sroh_assert":
            patcher = "oh"

        if patcher != last_patcher:
            if last_patcher != "":
                # patcher changed, run the combined step
                run_patcher(binary, collected_info, last_patcher, patchers[last_patcher], output)
                collected_info = ""
            last_patcher = patcher

        # aggregate the information
        collected_info += info

    # run the patcher for the last iteration
    run_patcher(binary, collected_info, last_patcher, patchers[last_patcher], output)

    # finalize
    finalizers = patcher_config["finalizers"]
    # replace placeholders {BINARY} and {ARGS} in all commands
    tmp_finalizers = []
    for finalizer in finalizers:
        cmds = []
        for el in finalizer:
            el = el.replace("{BINARY}", binary)
            el = el.replace("{ARGS}", args)
            el = el.replace("{OUTPUT}", output)
            cmds.append(el)
        tmp_finalizers.append(cmds)

    for finalizer in tmp_finalizers:
        subprocess.call(finalizer)


def run_patcher(binary, collected_info, last_patcher, commands, output):
    # wite the patch guide information
    with open(output + "/" + last_patcher + "_guide.txt", 'w') as fd:
        fd.write(collected_info)

    # run the patcher commands
    for cmd in commands:
        subprocess.call(cmd)

    # chmod +x the binary ??
    subprocess.call(["chmod", "+x", binary])


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Process step-wise post-patching.')
    parser.add_argument('binary', metavar='BINARY', type=str, help='a binary that receives post-patching')
    parser.add_argument('-m', dest='manifest', type=str, help='the manifest to run post-patching', required=True)
    parser.add_argument('-p', dest='patchers', type=str, help='file containing the patchers', required=True)
    parser.add_argument('--args', dest='args', type=str, default='',
                        help='arguments that are passed to the patchers for running the program')
    parser.add_argument('-o', dest='output', type=str, help='the output directory', required=True)
    args = parser.parse_args()
    main(args.binary, args.manifest, args.patchers, args.args, args.output)
