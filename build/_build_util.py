#!/usr/bin/env python3
import sys
import shlex
import argparse
import subprocess

# ---- Known flags tables (extend if you need more) ----

# Build flags that require the next token as a value
BUILD_NEEDS_VALUE = {"--config", "--target", "--preset", "--toolset", "-t"}

# Build flags with optional numeric next token
BUILD_OPTIONAL_NUM = {"--parallel"}

# Build flags that are simple switches (no value)
BUILD_SWITCHES = {"--clean-first", "--verbose"}

# Configure flags that require the next token
CONFIG_NEEDS_VALUE = {"-C"}

# Configure flags that are simple switches (no value)
CONFIG_SWITCHES = {"-Wdev", "-Wno-dev", "-Werror=dev", "-Wno-error=dev"}

# If a token matches none of the above, which bucket should it default to?
DEFAULT_BUCKET = "config"   # change to "build" if you prefer the opposite


def is_int(s: str) -> bool:
    try:
        int(s)
        return True
    except ValueError:
        return False


def split_args(mixed):
    """
    Split a mixed list of tokens into (config_args, build_args).
    The function:
      * does not assume any order,
      * supports '--' semantics:
          - the first '--' (encountered while splitting) switches routing to BUILD bucket,
          - any later '--' are forwarded into build args (so native tool flags work).
    """
    cfg, bld = [], []
    route = "auto"  # 'auto' until we see the first '--'; afterwards we are in 'build'
    expect = None   # ("config"/"build", flag) -> next token is the value

    i = 0
    while i < len(mixed):
        tok = mixed[i]

        # Handle expectation from the previous flag
        if expect is not None:
            bucket, flag = expect
            if flag in BUILD_OPTIONAL_NUM:
                # consume only if numeric; otherwise leave token for normal routing
                if is_int(tok):
                    bld.append(tok)
                    i += 1
                expect = None
                continue
            # regular "next value"
            if bucket == "build":
                bld.append(tok)
            else:
                cfg.append(tok)
            expect = None
            i += 1
            continue

        # First separator behavior:
        if tok == "--":
            if route == "auto":
                # Switch all subsequent tokens to BUILD bucket (this '--' is not kept)
                route = "build"
                i += 1
                continue
            else:
                # Already routing to build; keep this '--' to pass to native tool
                bld.append(tok)
                i += 1
                continue

        # --------- Try to classify as BUILD ----------
        # Long flags that need a value next
        if tok in BUILD_NEEDS_VALUE:
            bld.append(tok)
            expect = ("build", tok)
            i += 1
            continue

        # --parallel [N]
        if tok in BUILD_OPTIONAL_NUM:
            bld.append(tok)
            expect = ("build", tok)
            i += 1
            continue

        # -jN or -j N
        if tok.startswith("-j"):
            if tok == "-j":
                bld.append(tok)
                expect = ("build", tok)
            else:
                bld.append(tok)  # -j8 form
            i += 1
            continue

        # Simple build switches
        if tok in BUILD_SWITCHES:
            bld.append(tok)
            i += 1
            continue

        # If we've already switched route to build, push everything else there
        if route == "build":
            bld.append(tok)
            i += 1
            continue

        # --------- Try to classify as CONFIGURE ----------
        # Cache and unset forms (you said you use combined -DNAME=VALUE; we pass as-is)
        if tok.startswith("-D") or tok.startswith("-U"):
            cfg.append(tok)
            i += 1
            continue

        # Configure switches
        if tok in CONFIG_SWITCHES:
            cfg.append(tok)
            i += 1
            continue

        # Configure flags that need a value
        if tok in CONFIG_NEEDS_VALUE:
            cfg.append(tok)
            expect = ("config", tok)
            i += 1
            continue

        # --------- Fallback ----------
        if DEFAULT_BUCKET == "build":
            bld.append(tok)
        else:
            cfg.append(tok)
        i += 1

    return cfg, bld


def main(argv=None):
    argv = argv or sys.argv

    # Wrapper-level args only
    p = argparse.ArgumentParser(
        description="Split mixed CMake args into configure/build and run both steps."
    )
    p.add_argument("generator", help='CMake generator, e.g. "Visual Studio 18 2026" or "Ninja"')
    p.add_argument("build_dir", help="Binary/build directory")
    p.add_argument("arch", nargs="?", default="", help='Arch for VS generators, e.g. "x64"')
    p.add_argument(
        "rest",
        nargs=argparse.REMAINDER,
        help="Mixed CMake args (configure and build). Use -- to switch the rest to build args; a later -- is passed to the native tool."
    )
    ns = p.parse_args(argv[1:])

    # Strip a leading '--' that argparse keeps when using REMAINDER
    rest = ns.rest
    if rest and rest[0] == "--":
        rest = rest[1:]

    config_args, build_args = split_args(rest)

    build_config_is_specified = "--config" in build_args

    # Logs
    print("Running CMake configure...")
    print(f"      Generator: {ns.generator}")
    print(f"   Architecture: {ns.arch}")
    print(f"      Build Dir: {ns.build_dir}")
    print(" Configure Args:", " ".join(shlex.quote(a) for a in config_args))
    if len(build_args) == 0:
        print("     Build Args: <none>")
    else:
        print("     Build Args:", " ".join(shlex.quote(a) for a in build_args))
    print()

    # Build the configure command
    cfg_cmd = ["cmake", "-S", ".", "-B", ns.build_dir, "-G", ns.generator]
    if ns.arch:
        cfg_cmd += ["-A", ns.arch]
    cfg_cmd += config_args

    rc = subprocess.run(cfg_cmd).returncode
    if rc != 0:
        print("\nCMake configure failed.")
        return rc

    if build_config_is_specified:
        print("\nRunning CMake build...")
        build_cmd = ["cmake", "--build", ns.build_dir] + build_args
        return subprocess.run(build_cmd).returncode
    else:
        print("\nCMake build skipped (no --config specified).")
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
