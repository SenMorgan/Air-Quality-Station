# PlatformIO post-script for ESP8266 builds.
#
# The ESP8266 linker script places code (.text) of any static library that is
# not explicitly listed in the .irom0.text section into IRAM. The precompiled
# BSEC library (libalgobsec.a) is not listed there, so it overflows the IRAM
# (`section .text1 will not fit in region iram1_0_seg`) once
# bsec_get_state()/bsec_set_state() are linked in.
#
# This script patches the generated linker script so that libalgobsec.a is
# executed from flash (the same approach is used by Tasmota).

from os.path import join

Import("env")  # noqa: F821  (provided by PlatformIO/SCons)

LD_SCRIPT = join(env.subst("$BUILD_DIR"), "ld", "local.eagle.app.v6.common.ld")  # noqa: F821
ANCHOR = "*libc.a:(.literal .text .literal.* .text.*)"
RULE = "*libalgobsec.a:(.literal .text .literal.* .text.*)"


def patch_ld_script(target, source, env):
    path = str(target[0])
    with open(path, encoding="utf-8") as f:
        content = f.read()

    if RULE in content:
        return
    if ANCHOR not in content:
        raise RuntimeError("bsec_irom.py: anchor not found in " + path)

    content = content.replace(ANCHOR, RULE + "\n    " + ANCHOR, 1)
    with open(path, "w", encoding="utf-8") as f:
        f.write(content)
    print("bsec_irom.py: libalgobsec.a moved to flash (irom0.text)")


env.AddPostAction(LD_SCRIPT, patch_ld_script)  # noqa: F821
