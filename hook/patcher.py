import json
import sys

import subprocess


def load_patch_info(patch_filename):
    with open(patch_filename) as fd:
        return json.load(fd)


def main(binary, patch_filename):
    patch_info = load_patch_info(patch_filename)
    patchers = {
        "sc": [["python", "/home/dennis/self-checksumming/patcher/dump_pipe.py", binary, "sc_guide.txt",
                "/home/dennis/build/cfbuild/patch_guide"]],
        "oh_assert_": [["python", "/home/dennis/sip-oblivious-hashing/patcher/patchAsserts.py", "-b", binary, "-n",
                        binary + "_patched", "-s", "/home/dennis/build/cfbuild/oh.stats", "-p",
                        "/home/dennis/sip-oblivious-hashing/assertions/gdb_script_for_do_assert.txt", "-m",
                        "oh_assert__guide.txt"],
                       ["mv", binary + "_patched", binary]],
        "sroh_assert": [["python", "/home/dennis/sip-oblivious-hashing/patcher/patchAsserts.py", "-b", binary, "-n",
                         binary + "_patched", "-s", "/home/dennis/build/cfbuild/oh.stats", "-p",
                         "/home/dennis/sip-oblivious-hashing/assertions/gdb_script_for_do_assert.txt", "-m",
                         "sroh_assert_guide.txt"],
                        ["mv", binary + "_patched", binary]],
    }

    collected_info = ""
    last_patcher = ""
    for patcher, info in patch_info:
        if patcher != last_patcher:
            if last_patcher != "":
                run_patcher(binary, collected_info, last_patcher, patchers[last_patcher])
                collected_info = ""

            last_patcher = patcher

        collected_info += info

    run_patcher(binary, collected_info, last_patcher, patchers[last_patcher])

    with open("oh_assert_" + "_guide.txt", 'w') as fd:
        fd.write("")
    subprocess.call(["python", "/home/dennis/sip-oblivious-hashing/patcher/patchAsserts.py", "-b", binary, "-n",
                     binary + "_patched", "-s", "/home/dennis/build/cfbuild/oh.stats", "-p",
                     "/home/dennis/sip-oblivious-hashing/assertions/gdb_script_for_do_assert.txt", "-f", "True",
                     "-m", "oh_assert__guide.txt"])


def run_patcher(binary, collected_info, last_patcher, commands):
    with open(last_patcher + "_guide.txt", 'w') as fd:
        fd.write(collected_info)
    for cmd in commands:
        subprocess.call(cmd)
    subprocess.call(["chmod", "+x", binary])


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
