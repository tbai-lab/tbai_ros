#!/usr/bin/env python3
import argparse
import os
import subprocess
import sys


def run_bash(command: str, shell: bool = True, check: bool = True):
  
  ## wayland hack
  env = os.environ.copy()
  env["WAYLAND_DISPLAY"] = ""
  env["XDG_SESSION_TYPE"] = "x11"

  result = subprocess.run(
    command,
    shell=shell,
    capture_output=True,
    text=True,
    check=check,
    env=env,
  )
  return result


def main():
  parser = argparse.ArgumentParser(description="Run a bash command")
  parser.add_argument("command", nargs="+")
  args = parser.parse_args()

  command = " ".join(args.command)
  print(f"Running command: {command}")
  try:
    result = run_bash(command, check=True)
    sys.exit(result.returncode)
  except subprocess.CalledProcessError as e:
    sys.exit(e.returncode)


if __name__ == "__main__":
  main()
